#!/usr/bin/env python
"""闸门式数据采集,PiPER 主从链路版(观测走 topic,主臂固件直通从臂)。

从 rebot 的 ``record_rebot_gated_ros2.py`` 移植闸门**节奏**,去掉 rebot 特有的
teleop_map 硬件闸门(冻结/回零/ramp/对齐 —— PiPER 是固件直通,从臂只听主臂,
软件没有也不该有这些控制)。保留的人工闸门节奏:

  1. **整轮第一条**:回车确认开录(提示把主臂摆到起始位,仅提示不判定);
  2. **3 2 1 倒计时**(START_COUNTDOWN,可关)→ 倒完开录;
  3. 每条固定时长(EP_TIME,录制中 →/Esc 提前结束),最后 END_COUNTDOWN 秒终端倒计时;
  4. 录完**当场选**:回车/k=保留  d=丢弃重录  q=保存已录退出;
  5. 回车**直接开下一条**(无 0 位确认、无回零 —— 你自己把主臂摆回起始位再回车);
  6. 退出/结束/^C:落盘已录,丢弃未完成的当前条,不甩 traceback。

与 rebot gated 的差异(为什么没有回零/冻结):
  rebot 的从臂由 teleop_map 软件驱动,需要"开录前冻结防跟手、录完回零归位"。
  PiPER 主臂固件直通从臂(MasterSlaveConfig 0xFA),从臂位姿=主臂位姿,
  "摆回起始位"就是"把主臂摆回去",无需软件干预。action 源是主臂位姿
  (/piper/leader/joint_state),观测是从臂反馈(/piper/joint_state)。

节奏环境变量(os.environ 直读,不进 CLI):
  START_COUNTDOWN   开录前倒计时秒数(默认 3,0=关)
  END_COUNTDOWN     结束前倒计时秒数(默认 5,0=关)
  JOINT_STALE_MS    从臂存活探测新鲜度(默认 300ms)
  SAVE_ASYNC        1=保留后保存放后台(默认):闸门提示/摆位立即进入下一轮,
                    开录前自动等上一条落盘(与 add_frame 串行,缓冲区安全);
                    后台线程内视频串行编码(parallel_encoding=False,避免线程内
                    fork ProcessPool 死锁);0=同步保存(主线程,视频可并行,最快)

前提:已跑 start_piper_teleop.sh(起了 arm + leader + 相机节点)。
用法见 record_piper_gated.sh。
"""

import logging
import os
import threading
import time
from dataclasses import asdict
from pprint import pformat

from lerobot.configs import parser
from lerobot.datasets import (
    LeRobotDataset,
    VideoEncodingManager,
    aggregate_pipeline_dataset_features,
    create_initial_features,
)
from lerobot.processor import make_default_processors
from lerobot.robots import make_robot_from_config
from lerobot.teleoperators import make_teleoperator_from_config
from lerobot.utils.feature_utils import combine_feature_dicts
from lerobot.utils.keyboard_input import init_keyboard_listener
from lerobot.utils.utils import init_logging, log_say
from lerobot.utils.visualization_utils import init_visualization, shutdown_visualization

# 复用官方 record 的配置 dataclass 与录制循环(import 该模块顺带触发插件注册)
from lerobot.scripts.lerobot_record import RecordConfig, record_loop

logger = logging.getLogger(__name__)


def _quiet_noisy_encoders() -> None:
    """压掉保存时刷屏的 x265/libx264 banner 与 datasets 的 'Map' 进度条。

    不改任何功能:先尝试把 ffmpeg 库日志级别提到 ERROR;真正的兜底是
    save 期间把进程 stderr(fd 2)重定向到调试文件 —— x265/libx264 的
    banner 是 C 级直写 fd 2,只有 dup2 级别的重定向能拦干净。
    """
    try:
        import av

        av.logging.set_level(av.logging.ERROR)
    except Exception:
        pass
    try:
        from datasets import disable_progress_bar

        disable_progress_bar()
    except Exception:
        pass


_SAVE_STDERR_LOG = "/tmp/piper_save_stderr.log"


class _stderr_silenced:
    """把进程 stderr(fd 2)重定向到调试文件;退出自动恢复。线程级不安全但
    save 与主线程互斥(异步保存期间主线程只等 _ask),可安全使用。"""

    def __enter__(self):
        import os

        self._saved = os.dup(2)
        f = os.open(_SAVE_STDERR_LOG, os.O_WRONLY | os.O_CREAT | os.O_APPEND, 0o644)
        os.dup2(f, 2)
        os.close(f)
        return self

    def __exit__(self, *exc):
        import os

        os.dup2(self._saved, 2)
        os.close(self._saved)
        return False


def _clean_tmp_dirs(root) -> None:
    """删掉数据集目录里编码/保存失败留下的 tmp* 临时目录(orphan 视频块)。"""
    import glob
    import shutil

    try:
        for p in glob.glob(os.path.join(str(root), "tmp*")):
            if os.path.isdir(p):
                shutil.rmtree(p, ignore_errors=True)
    except Exception:
        pass


def _ask(prompt: str) -> str:
    try:
        return input(prompt).strip().lower()
    except EOFError:
        return "q"
    except KeyboardInterrupt:
        print()
        return "q"


def _countdown(n: int, label: str, stop: threading.Event | None = None) -> None:
    for i in range(n, 0, -1):
        if stop is not None and stop.is_set():
            return
        print(f"\r{label} {i}   ", end="", flush=True)
        t_end = time.monotonic() + 1.0
        while time.monotonic() < t_end:
            if stop is not None and stop.is_set():
                return
            time.sleep(0.05)
    print()


def _start_end_countdown(ep_time_s: float, n: int):
    """后台线程:睡到本条剩余 n 秒,然后倒计时。stop 置位即静默退出。"""
    stop = threading.Event()

    def _run() -> None:
        lead = ep_time_s - n
        t_end = time.monotonic() + max(lead, 0.0)
        while time.monotonic() < t_end:
            if stop.is_set():
                return
            time.sleep(0.1)
        print()
        _countdown(n, "⏳ 本条剩余", stop)

    th = threading.Thread(target=_run, daemon=True)
    th.start()
    return stop, th


def _follower_alive(robot, stale_ms: float = 300.0) -> bool:
    """从臂存活探测 = joint_state 新鲜度。arm 节点 100Hz 发 /piper/joint_state。"""
    got = robot._bus.latest_joint_state(robot.config.state_topic)
    if got is None:
        logger.warning(f"{robot.config.state_topic} 从未收到消息")
        return False
    age_ms = (time.monotonic() - got[1]) * 1000.0
    if age_ms > stale_ms:
        logger.warning(f"{robot.config.state_topic} 已 {age_ms:.0f}ms 无新帧!")
        return False
    return True


@parser.wrap()
def main(cfg: RecordConfig) -> None:
    init_logging()
    logging.info(pformat(asdict(cfg)))
    _quiet_noisy_encoders()  # 压掉每段编码的 x265/libx264 banner 与 'Map' 进度条

    if cfg.teleop is None:
        raise ValueError("闸门式采集需要 --teleop.*(用 ros2_piper_teleop)")

    if cfg.display_data:
        init_visualization(
            cfg.display_mode, session_name="recording", ip=cfg.display_ip, port=cfg.display_port
        )
        import atexit
        import subprocess

        def _close_viewer_on_crash():
            try:
                shutdown_visualization(cfg.display_mode)
            except Exception:
                pass
            try:
                subprocess.run(["pkill", "-f", "rerun --port=9876"], timeout=3, capture_output=True)
            except Exception:
                pass

        atexit.register(_close_viewer_on_crash)

    robot = make_robot_from_config(cfg.robot)
    teleop = make_teleoperator_from_config(cfg.teleop)
    t_proc, r_proc, o_proc = make_default_processors()

    dataset_features = combine_feature_dicts(
        aggregate_pipeline_dataset_features(
            pipeline=t_proc,
            initial_features=create_initial_features(action=robot.action_features),
            use_videos=cfg.dataset.video,
        ),
        aggregate_pipeline_dataset_features(
            pipeline=o_proc,
            initial_features=create_initial_features(observation=robot.observation_features),
            use_videos=cfg.dataset.video,
        ),
    )

    num_cams = len(robot.cameras) if hasattr(robot, "cameras") else 0
    if cfg.resume:
        dataset = LeRobotDataset.resume(
            cfg.dataset.repo_id,
            root=cfg.dataset.root,
            batch_encoding_size=cfg.dataset.video_encoding_batch_size,
            rgb_encoder=cfg.dataset.rgb_encoder,
            depth_encoder=cfg.dataset.depth_encoder,
            encoder_threads=cfg.dataset.encoder_threads,
            streaming_encoding=cfg.dataset.streaming_encoding,
            encoder_queue_maxsize=cfg.dataset.encoder_queue_maxsize,
            image_writer_processes=cfg.dataset.num_image_writer_processes if num_cams else 0,
            image_writer_threads=(cfg.dataset.num_image_writer_threads_per_camera * num_cams) if num_cams else 0,
        )
        logger.info(f"RESUME: 续录 {dataset.repo_id},已有 {dataset.num_episodes} 条")
    else:
        cfg.dataset.stamp_repo_id()
        dataset = LeRobotDataset.create(
            cfg.dataset.repo_id,
            cfg.dataset.fps,
            root=cfg.dataset.root,
            robot_type=robot.name,
            features=dataset_features,
            use_videos=cfg.dataset.video,
            image_writer_processes=cfg.dataset.num_image_writer_processes,
            image_writer_threads=cfg.dataset.num_image_writer_threads_per_camera * num_cams,
            batch_encoding_size=cfg.dataset.video_encoding_batch_size,
            rgb_encoder=cfg.dataset.rgb_encoder,
            depth_encoder=cfg.dataset.depth_encoder,
            encoder_threads=cfg.dataset.encoder_threads,
            streaming_encoding=cfg.dataset.streaming_encoding,
            encoder_queue_maxsize=cfg.dataset.encoder_queue_maxsize,
        )

    robot.connect()
    teleop.connect()

    # —— 采集前校准预检:静默检测标定状态,缺失则弹出交互式引导 ——
    try:
        from tools.calibration.calibration_preflight import calibration_preflight

        if not calibration_preflight(
            robot, teleop,
            leader_type="ros2_piper_teleop",
            follower_type="ros2_piper_follower",
        ):
            log_say("Calibration check failed", cfg.play_sounds, blocking=True)
            raise RuntimeError("校准预检未通过,已退出。请根据以上提示完成标定后重试。")
        logger.info("校准预检通过,开始采集")
    except ImportError as e:
        logger.warning(f"校准预检模块不可用({e}),跳过。")
    except RuntimeError:
        raise

    listener, events = init_keyboard_listener()

    target = cfg.dataset.num_episodes
    ep_time = cfg.dataset.episode_time_s
    stale_ms = float(os.environ.get("JOINT_STALE_MS", "300"))
    start_cd = int(os.environ.get("START_COUNTDOWN", "3"))
    end_cd = int(os.environ.get("END_COUNTDOWN", "5"))
    kept = dataset.num_episodes  # 已确认保存成功的条数(resume 时从已录条数接着数)
    submitted = dataset.num_episodes  # 已接受「保留」提交保存的条数 —— 用它判停,避免异步保存未确认时多问一轮
    save_async = os.environ.get("SAVE_ASYNC", "1") != "0"  # 1=快照后台流水线(默认);0=同步保存(旧行为)
    pending: dict = {"th": None, "ok": False, "err": None}  # 至多一个后台保存线程
    next_ep = dataset.num_episodes  # 下一个要分发的 episode_index(单调,避免与快照线程冲突)
    first = True

    def _fresh_buffer(idx: int):
        """用显式 idx 建新缓冲区,防止 writer 默认(meta.total)与快照线程的 idx 撞车。"""
        w = dataset.writer
        w.episode_buffer = w._create_episode_buffer(int(idx))

    def _join_pending_save() -> bool:
        """等后台快照保存结束;成功则 kept+1 返回 True,失败返回 False(该条丢失未计入)。"""
        nonlocal kept
        th = pending["th"]
        if th is None:
            return True
        th.join()
        pending["th"] = None
        if pending["ok"]:
            kept += 1
            log_say("Saved", cfg.play_sounds)
            print(f"  ✓ 保存完成:{kept}/{target} 条", flush=True)
            return True
        logger.error(f"后台保存失败: {pending['err']}")
        print("⚠️  上一条保存失败(未计入),重开一条或退出。")
        return False

    def _discard_current():
        """丢弃当前未完成的条(record 出错 / d / q 共用):先确保缓冲区 index 指向当前条,
        再清;之后恢复单调 index。不会误删快照线程(上一轮)正在编码的暂存目录。"""
        nonlocal next_ep
        _fresh_buffer(next_ep)
        dataset.clear_episode_buffer()
        _fresh_buffer(next_ep)

    def _snapshot_and_save_async():
        """快照当前条 → 立刻把缓冲区切到下一个 index → 后台保存快照(录 N+1 与存 N 并行)。
        保存严格串行:调用前必须已 join 上一个 pending(见 keep 路径)。"""
        nonlocal next_ep
        w = dataset.writer
        buf = w.episode_buffer
        snap = {k: (list(v) if isinstance(v, list) else v) for k, v in buf.items()}
        idx = int(snap["episode_index"])
        assert idx == next_ep, f"buffer idx {idx} != next_ep {next_ep}"
        _fresh_buffer(next_ep + 1)
        next_ep += 1

        def _run_save():
            try:
                # 保存期把 stderr 重定向到调试文件:x265/libx264 banner、
                # 'Map' 进度条全是写 stderr,终端保持干净。
                # 线程内不能用并行视频编码(ProcessPool fork 死锁,实测踩中)→ 串行。
                with _stderr_silenced():
                    dataset.save_episode(episode_data=snap, parallel_encoding=False)
                pending["ok"] = True
            except Exception as e:
                pending["ok"] = False
                pending["err"] = e
                logger.error(f"episode {idx} 后台保存失败: {e}")

        pending["ok"] = False
        pending["err"] = None
        pending["th"] = threading.Thread(target=_run_save, daemon=True)
        pending["th"].start()
        return idx

    try:
        with VideoEncodingManager(dataset):
            try:
                while submitted < target:
                    # —— 闸门 1:提示立即可见;开录不等待(上一条保存跑在后台快照线程)——
                    wait_hint = " (上一条保存中,后台)" if pending["th"] is not None else ""
                    if first:
                        if _ask(f"\n▶ 共录 {target} 条,每条 {ep_time:g}s(录制中 →/Esc 提前结束)。\n"
                                f"  请把主臂摆到起始位,摆好后回车开始整轮录制 | q 退出: ") == "q":
                            break
                    elif _ask(f"\n▶ 已保留 {kept}/{target}。回车开始下一条"
                              f"(每条 {ep_time:g}s,→/Esc 提前结束;主臂摆回起始位){wait_hint} | q 退出: ") == "q":
                        break

                    # —— 从臂存活探测:joint_state 断流 = arm 节点/CAN 侧出事 ——
                    if not _follower_alive(robot, stale_ms):
                        print("⚠️  /piper/joint_state 断流 —— piper_arm_node/CAN 侧出事!")
                        if _ask("回车重新探测 / q 退出并保存已录: ") == "q":
                            break
                        continue

                    # —— 开录倒计时 ——
                    # (上一条的保存由后台快照线程负责:快照双缓冲让 录 N+1 与 存 N 并行,
                    #  这里不再等它;开录即录,保存串行完成,见 gate2 keep 的 _snapshot_and_save_async)
                    if start_cd > 0:
                        _countdown(start_cd, "▶ 开录倒计时")

                    first = False
                    events["exit_early"] = False
                    log_say(f"Recording episode {kept}", cfg.play_sounds)
                    cd_stop, cd_th = (
                        _start_end_countdown(ep_time, end_cd) if 0 < end_cd < ep_time else (None, None)
                    )
                    rec_error = None
                    try:
                        record_loop(
                            robot=robot,
                            events=events,
                            fps=cfg.dataset.fps,
                            teleop_action_processor=t_proc,
                            robot_action_processor=r_proc,
                            robot_observation_processor=o_proc,
                            teleop=teleop,
                            dataset=dataset,
                            control_time_s=ep_time,
                            single_task=cfg.dataset.single_task,
                            display_data=cfg.display_data,
                            display_mode=cfg.display_mode,
                        )
                    except Exception as e:
                        rec_error = e
                    finally:
                        if cd_stop is not None:
                            cd_stop.set()
                            cd_th.join(timeout=2)

                    if rec_error is not None:
                        logger.error(f"episode {kept} 录制中出错: {rec_error}")
                        try:
                            _discard_current()
                        except Exception:
                            pass
                        if _ask("本条已丢弃。回车重试本条 / q 退出并保存已录: ") == "q":
                            break
                        continue

                    events["exit_early"] = False
                    events["stop_recording"] = False
                    events["rerecord_episode"] = False

                    # —— 闸门 2:保留 / 丢弃(此处无后台保存,缓冲区可安全操作)——
                    dec = _ask("■ 录完:回车/k=保留   d=丢弃重录   q=保存已录并退出: ")
                    if dec == "d":
                        _discard_current()
                        log_say("Discarded", cfg.play_sounds)
                        continue
                    if dec == "q":
                        _discard_current()
                        break

                    submitted += 1  # 接受这条:同步/异步保存都算已提交(判停用)
                    if not save_async:
                        # 同步保存(旧行为):不并发,逐条等完;保存期 stderr 重定向防刷屏
                        try:
                            with _stderr_silenced():
                                dataset.save_episode()
                        except Exception as e:
                            logger.error(f"episode {kept} 保存失败: {e}")
                            try:
                                dataset.clear_episode_buffer()
                            except Exception:
                                pass
                            _clean_tmp_dirs(dataset.root)
                            if _ask("保存失败,本条丢弃。回车继续 / q 退出: ") == "q":
                                break
                            continue
                        kept += 1
                        log_say("Saved", cfg.play_sounds)
                        print(f"  ✓ 保存完成:{kept}/{target} 条", flush=True)
                    else:
                        # —— 快照流水线(默认):录 N+1 与存 N 并行 ——
                        # 1) 先 join 上一轮保存(它已在本条 15s 录制期间跑完,通常 ≈0 等待;
                        #    只在它异常慢时才需要等——保存严格串行,保证 meta/index 顺序)。
                        if not _join_pending_save():
                            submitted -= 1  # 上一轮保存失败,名额退回
                            print("⚠️  上一条保存失败未计入;本条仍保留。")
                        # 2) 快照本条 → 立刻切到下一缓冲区 → 后台保存(线程内串行编码,不 fork)
                        try:
                            _snapshot_and_save_async()
                        except Exception as e:
                            logger.error(f"快照流水线失败,本条按同步保存兜底: {e}")
                            try:
                                with _stderr_silenced():
                                    dataset.save_episode()
                            except Exception as e2:
                                logger.error(f"episode {kept} 兜底保存失败: {e2}")
                                try:
                                    dataset.clear_episode_buffer()
                                except Exception:
                                    pass
                                _clean_tmp_dirs(dataset.root)
                                if _ask("保存失败,本条丢弃。回车继续 / q 退出: ") == "q":
                                    break
                                continue
                            kept += 1
                            log_say("Saved", cfg.play_sounds)
                            print(f"  ✓ 保存完成:{kept}/{target} 条", flush=True)
            finally:
                # 收尾(正常跑完 / q / ^C 都走这里):等最后一个后台保存落盘再退出
                _join_pending_save()
    except KeyboardInterrupt:
        print("\n⚑ 键盘中断 —— 本条未完成已丢弃,正在收尾(已保留的不丢)...")
        try:
            _discard_current()
        except Exception:
            pass
    finally:
        if cfg.display_data:
            shutdown_visualization(cfg.display_mode)
        try:
            teleop.disconnect()
        except Exception:
            pass
        try:
            robot.disconnect()
        except Exception:
            pass
        log_say("Exiting", cfg.play_sounds)
        print(f"\n== 采集结束。已保留 {kept} 条 → {dataset.root}")


if __name__ == "__main__":
    main()
