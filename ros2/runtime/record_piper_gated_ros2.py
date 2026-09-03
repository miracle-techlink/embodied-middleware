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
    listener, events = init_keyboard_listener()

    target = cfg.dataset.num_episodes
    ep_time = cfg.dataset.episode_time_s
    stale_ms = float(os.environ.get("JOINT_STALE_MS", "300"))
    start_cd = int(os.environ.get("START_COUNTDOWN", "3"))
    end_cd = int(os.environ.get("END_COUNTDOWN", "5"))
    kept = dataset.num_episodes  # resume 时从已录条数接着数
    first = True
    try:
        with VideoEncodingManager(dataset):
            while kept < target:
                # —— 闸门 1:整轮第一条提示摆起始位;后续每条只回车开录 ——
                if first:
                    if _ask(f"\n▶ 共录 {target} 条,每条 {ep_time:g}s(录制中 →/Esc 提前结束)。\n"
                            f"  请把主臂摆到起始位,摆好后回车开始整轮录制 | q 退出: ") == "q":
                        break
                elif _ask(f"\n▶ 已保留 {kept}/{target}。回车开始下一条"
                          f"(每条 {ep_time:g}s,→/Esc 提前结束;主臂摆回起始位) | q 退出: ") == "q":
                    break

                # —— 从臂存活探测:joint_state 断流 = arm 节点/CAN 侧出事 ——
                if not _follower_alive(robot, stale_ms):
                    print("⚠️  /piper/joint_state 断流 —— piper_arm_node/CAN 侧出事!")
                    if _ask("回车重新探测 / q 退出并保存已录: ") == "q":
                        break
                    continue

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
                        dataset.clear_episode_buffer()
                    except Exception:
                        pass
                    if _ask("本条已丢弃。回车重试本条 / q 退出并保存已录: ") == "q":
                        break
                    continue

                events["exit_early"] = False
                events["stop_recording"] = False
                events["rerecord_episode"] = False

                # —— 闸门 2:保留 / 丢弃 ——
                dec = _ask("■ 录完:回车/k=保留   d=丢弃重录   q=保存已录并退出: ")
                if dec == "d":
                    dataset.clear_episode_buffer()
                    log_say("Discarded", cfg.play_sounds)
                    continue
                if dec == "q":
                    dataset.clear_episode_buffer()
                    break
                try:
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
    except KeyboardInterrupt:
        print("\n⚑ 键盘中断 —— 本条未完成已丢弃,正在收尾(已保留的不丢)...")
        try:
            dataset.clear_episode_buffer()
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
