#!/usr/bin/env python
"""闸门式数据采集(单臂 reBot + 腕部深度 + front 第二视角)。

与 ``lerobot-record`` 的区别:把"到点自动连录下一条"换成**人工闸门**——
  1. 回车后:**从臂存活探测**(断电/CAN 静默提前拦截)→ 从臂**回零位** → **3 2 1** 倒计时
     → **慢速对齐主臂**(回零位 → 主臂当前位姿,限速 ~30°/s,不跳变)→ 才开始录;
  2. 每条固定时长(默认 ``--dataset.episode_time_s=15`` 秒;录制中按方向键→可提前结束),
     **最后 5 秒终端给 5 4 3 2 1 倒计时**;录完**自动回零**(兼作断电探测器)再问你;
  3. 录完当场选择**保留 / 丢弃重录**;任意阶段 Ctrl-C 优雅收尾,不甩 traceback。

节奏环境变量(``os.environ`` 直读,不进 CLI):
  ``ZERO_BEFORE_EPISODE=0``  关掉每条开录前的从臂回零(默认开)
  ``ZERO_AFTER_EPISODE=0``   关掉每条结束后的自动回零(默认开)
  ``ALIGN_LEADER=0``         关掉开录前的慢速对齐主臂(默认开)
  ``ALIGN_STEP_DEG``         对齐限速(度/帧,默认 1.0 ≈ 30°/s @30fps)
  ``ALIGN_TIMEOUT_S``        对齐超时秒数(默认 20)
  ``START_COUNTDOWN``        开录前倒计时秒数(默认 3,0=关)
  ``END_COUNTDOWN``          结束前倒计时秒数(默认 5,0=关)

其余全部复用 lerobot 官方栈:同一套 config 解析(CLI 与 lerobot-record 完全一致:
``--robot.* --teleop.* --dataset.* --display_data`` ...)、同一个 ``record_loop`` 录制循环、
同一个 ``LeRobotDataset``(标准格式,可照常传 HF / 之后转 ModelScope)。

用法示例见 scripts/record_rebot_gated.sh(封装好相机/编码/CAN 覆盖)。
"""

import logging
import math
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

# 复用官方 record 的配置 dataclass 与录制循环(import 该模块也顺带触发插件注册)
from lerobot.scripts.lerobot_record import RecordConfig, record_loop

logger = logging.getLogger(__name__)


def _clean_tmp_dirs(root) -> None:
    """删掉数据集目录里编码/保存失败留下的 tmp* 临时目录(orphan 视频块)。"""
    import glob
    import os
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
        # 闸门提示符前 Ctrl+C = 优雅退出(落盘已录),不要甩一屏 traceback
        print()
        return "q"


def _countdown(n: int, label: str, stop: threading.Event | None = None) -> None:
    """同行刷新打印 n..1,每秒一拍;给了 stop 事件则可被即时打断(提前结束本条的场景)。"""
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


def _start_end_countdown(ep_time_s: float, n: int) -> tuple[threading.Event, threading.Thread]:
    """后台线程:睡到本条剩余 n 秒,然后开始 5 4 3 2 1。stop 置位即静默退出
    (录制被 →/Esc 提前结束、或 record_loop 出错时,残留倒计时不该继续吵)。"""
    stop = threading.Event()

    def _run() -> None:
        lead = ep_time_s - n
        t_end = time.monotonic() + max(lead, 0.0)
        while time.monotonic() < t_end:
            if stop.is_set():
                return
            time.sleep(0.1)
        print()  # 让出 record_loop 可能正在刷新的行
        _countdown(n, "⏳ 本条剩余", stop)

    th = threading.Thread(target=_run, daemon=True)
    th.start()
    return stop, th


def _arm_alive(robot, attempts: int = 2) -> bool:
    """从臂存活探测:逐台 robstride_ping(与 motorbridge-cli scan 同机制),有应答才算在线。
    电机断电/CAN 静默时 socketcan 写入并不报错(没人 ACK 而已),会录出整段废数据。
    别用 request_feedback + get_state 探:空闲臂不回反馈包,会误报全静默(踩过 2026-08-15)。"""
    motors = getattr(robot, "motors", None)
    if not motors:
        return True  # 纯相机模式(allow_missing_arm):无从臂可探
    for _ in range(attempts):
        alive = 0
        for m in motors.values():
            try:
                m.robstride_ping()
                alive += 1
            except Exception:
                pass
        if alive == len(motors):
            return True
        if alive:
            logger.warning(f"从臂部分电机无应答: {alive}/{len(motors)} 在线")
            return True  # 有回包说明总线通,单台问题交给录制内错误隔离
        time.sleep(0.1)
    return False


def _try_zero(robot, when: str) -> bool:
    """safe_zero 回零 + 失联告警。返回 False = 回零失败(疑似断电/CAN 失联)。"""
    print(f"↺ 从臂回零中({when})...")
    try:
        robot.safe_zero(exit_on_complete=False)
        return True
    except Exception as e:
        logger.error(f"{when}回零失败: {e}")
        print("⚠️  从臂无响应 —— 疑似断电 / 急停拍下 / CAN 失联!请检查臂电源和 can0 状态。")
        return False


# 与插件 mapping.py 的 REBOT_ARM_MOTORS 同序
_ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_yaw", "wrist_roll"]


def _read_arm_action_deg(robot) -> list[float] | None:
    """读从臂 6 关节**实际当前位**,换算成 action 空间(send_action 输入 = 电机原始度 / direction)。
    用作对齐 ramp 的起点 —— safe_zero 把臂物理挪到零位后,teleop map 里记的保持位
    还是上一条的输出,不喂实际位第一帧就跳变(踩过)。读不到返回 None(ramp 退回记忆起点)。"""
    motors = getattr(robot, "motors", None)
    bus = getattr(robot, "bus", None)
    if not motors or bus is None:
        return None
    try:
        for m in motors.values():
            m.request_feedback()
        bus.poll_feedback_once()
    except Exception:
        pass  # 反馈抖动就用缓存状态,ramp 起点只是参考
    dirs = getattr(robot.config, "joint_directions", {})
    out = []
    for name in _ARM_JOINTS:
        m = motors.get(name)
        st = m.get_state() if m is not None else None
        if st is None:
            return None
        out.append(math.degrees(st.pos) / (dirs.get(name, 1.0) or 1.0))
    return out


def _align_to_leader(robot, teleop, hz: float = 30.0, step_deg: float = 1.0,
                     timeout_s: float = 20.0) -> bool:
    """开录前对齐:回零后从臂在零位、主臂在手里,直接开录会弹射。先把 ramp 起点
    喂成从臂**实际当前位**,再用 teleop 内置 startup ramp 的**慢速**(step_deg/帧,
    默认 1°@30Hz ≈ 30°/s)把从臂滑到主臂当前位姿,ramp 收敛(输出已直通)= 到位。
    超时未收敛返回 False(由调用方探测臂是否还活着)。"""
    cur = _read_arm_action_deg(robot)
    if hasattr(teleop, "rearm_ramp"):
        try:
            teleop.rearm_ramp(step_deg_per_step=step_deg, current_deg=cur)
        except TypeError:
            teleop.rearm_ramp()  # 旧版插件没有慢速/起点参数,退回默认限速
    converged = getattr(teleop, "ramp_converged", None)
    t0 = time.monotonic()
    dt = 1.0 / hz
    while time.monotonic() - t0 < timeout_s:
        action = teleop.get_action()
        robot.send_action(action)
        if converged is None:
            # 插件没有 ramp_converged(旧版):没有收敛信号可等,直接退(默认 ramp 会在录制里继续限速)
            return True
        if teleop.ramp_converged:
            return True
        time.sleep(dt)
    return teleop.ramp_converged


@parser.wrap()
def main(cfg: RecordConfig) -> None:
    init_logging()
    logging.info(pformat(asdict(cfg)))

    if cfg.teleop is None:
        raise ValueError("闸门式采集需要 --teleop.*(用 starai_to_rebot_leader)")

    if cfg.display_data:
        init_visualization(
            cfg.display_mode, session_name="recording", ip=cfg.display_ip, port=cfg.display_port
        )
        # 看门狗:启动阶段(resume/建数据集/connect 任一步)崩了,主 try/finally 还没挂上,
        # 孤儿 rerun 窗口会定格在最后一帧,看着像"卡死"(踩过)。atexit 在正常退出和
        # 未捕获异常/KeyboardInterrupt 时都会跑:断流 + 收掉本次 spawn 的 viewer。
        import atexit
        import subprocess

        def _close_viewer_on_crash():
            try:
                shutdown_visualization(cfg.display_mode)
            except Exception:
                pass
            try:
                # 模式要同时命中 wrapper(.../bin/rerun --port=9876)和真正的 Rust
                # viewer(.../rerun_cli/rerun --port=9876)——只杀 wrapper 窗口会留着
                subprocess.run(["pkill", "-f", "rerun --port=9876"],
                               timeout=3, capture_output=True)
            except Exception:
                pass

        atexit.register(_close_viewer_on_crash)

    robot = make_robot_from_config(cfg.robot)
    teleop = make_teleoperator_from_config(cfg.teleop)
    t_proc, r_proc, o_proc = make_default_processors()

    # 与 lerobot-record 完全一致的特征聚合(action 来自 robot 动作空间,observation 来自 robot 观测)
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
        # 续录进已有数据集(会话中途死了不丢已录的)。--resume=true 时 REPO_ID 需给**完整已存在**的
        # 数据集名(含时间戳),或用 --dataset.root 指到该目录。EPISODES 是「总目标条数」。
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
        # schema 兼容检查:续录库的图像特征必须与本次相机配置完全一致 —— 否则录第一条才
        # 在 record_loop 里炸 KeyError(踩过:旧库带 wrist_depth,本次 NO_DEPTH=1)。
        ds_imgs = {k for k in dataset.features if k.startswith("observation.images.")}
        new_imgs = {k for k in dataset_features if k.startswith("observation.images.")}
        missing, extra = ds_imgs - new_imgs, new_imgs - ds_imgs
        if missing or extra:
            raise ValueError(
                f"续录数据集 schema 与本次相机配置不一致:库里有而本次没有 {sorted(missing)},"
                f"本次新增 {sorted(extra)}。请保持相机/深度配置与建库时一致"
                f"(如旧库带深度就去掉 NO_DEPTH=1),或换新 REPO_ID 开录。"
            )
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
    zero_before = os.environ.get("ZERO_BEFORE_EPISODE", "1") != "0"
    zero_after = os.environ.get("ZERO_AFTER_EPISODE", "1") != "0"
    align_on = os.environ.get("ALIGN_LEADER", "1") != "0"
    align_step = float(os.environ.get("ALIGN_STEP_DEG", "1.0"))
    align_timeout = float(os.environ.get("ALIGN_TIMEOUT_S", "20"))
    start_cd = int(os.environ.get("START_COUNTDOWN", "3"))
    end_cd = int(os.environ.get("END_COUNTDOWN", "5"))
    kept = dataset.num_episodes  # resume 时从已录条数接着数(target 为总目标)
    try:
        with VideoEncodingManager(dataset):
            while kept < target:
                # —— 闸门 1:回车开始录制本条 ——
                if _ask(f"\n▶ 已保留 {kept}/{target}。回车开始录制(每条 {ep_time:g}s,录制中 →/Esc 可提前结束) | q 退出: ") == "q":
                    break

                # —— 从臂存活探测:电机断电/CAN 静默时 socketcan 写入不报错(没人 ACK),
                # 会录出整段手臂没动的废数据且毫无告警(踩过)。每条开录前先探一次 ——
                if not _arm_alive(robot):
                    print("⚠️  从臂无响应 —— 疑似断电 / 急停拍下 / CAN 失联!检查臂电源、急停、can0。")
                    if _ask("回车重新探测 / q 退出并保存已录: ") == "q":
                        break
                    continue

                # —— 从臂回零位:每条从同一初始状态开录(ZERO_BEFORE_EPISODE=0 可关)。
                # 失败(断电/CAN 失联等)不杀会话:告警回到闸门,与录制中出错同一容错策略 ——
                if zero_before and not _try_zero(robot, "开录前"):
                    if _ask("回车重试本条 / q 退出并保存已录: ") == "q":
                        break
                    continue

                # —— 3 2 1 开录倒计时(START_COUNTDOWN=0 可关)——
                if start_cd > 0:
                    _countdown(start_cd, "▶ 开录倒计时")

                # —— 对齐主臂(ALIGN_LEADER=0 可关):回零后从臂在零位、主臂在手里,
                # 直接开录会弹射。用慢速 ramp(默认 1°/帧 ≈ 30°/s,ALIGN_STEP_DEG 可调)
                # 把从臂滑到主臂当前位姿,收敛(ramp 直通)后才开录 —— 起步不跳变 ——
                if align_on:
                    print(f"🎯 对齐主臂中(限速 {align_step:g}°/帧,请扶稳主臂)...")
                    if not _align_to_leader(robot, teleop, hz=cfg.dataset.fps,
                                            step_deg=align_step, timeout_s=align_timeout):
                        if not _arm_alive(robot):
                            print("⚠️  对齐超时且从臂无响应 —— 疑似断电 / CAN 失联!")
                            if _ask("回车重试本条 / q 退出并保存已录: ") == "q":
                                break
                            continue
                        print("⚠️  对齐超时(主臂可能一直在动),本条起步段仍有限速 ramp 保护,必要时选 d 重录。")
                elif hasattr(teleop, "rearm_ramp"):
                    # 没开对齐:退回原行为,录制内的启动 ramp 从保持位限速滑向主臂
                    teleop.rearm_ramp()

                events["exit_early"] = False
                log_say(f"Recording episode {kept}", cfg.play_sounds)
                # 结束倒计时(END_COUNTDOWN=0 可关):后台线程,record_loop 返回即停
                cd_stop, cd_th = (
                    _start_end_countdown(ep_time, end_cd) if 0 < end_cd < ep_time else (None, None)
                )
                # 单条错误隔离:硬件抖动(CAN 掉线 socketcan write failed / 相机卡)不该杀掉整轮 ——
                # 捕获、丢弃本条、让用户重试或退出,而不是让异常炸掉 50 条会话。
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
                    # record_loop 一返回(正常到点/提前结束/出错)就停掉结束倒计时线程
                    if cd_stop is not None:
                        cd_stop.set()
                        cd_th.join(timeout=2)

                if rec_error is not None:
                    logger.error(f"episode {kept} 录制中出错(可能 CAN 掉线/相机抖动): {rec_error}")
                    try:
                        dataset.clear_episode_buffer()
                    except Exception:
                        pass
                    if zero_after:
                        _try_zero(robot, "异常后")  # 顺带探测:臂死了这里会告警
                    if _ask("本条已丢弃。回车重试本条 / q 退出并保存已录: ") == "q":
                        break
                    continue

                # 录制中按 →/Esc/← 触发的 lerobot 键盘标志,这里都只当作"提前结束本条";
                # 是否继续 / 退出整轮,完全交给下面的提示(避免 Esc 直接杀掉整个采集)。
                events["exit_early"] = False
                events["stop_recording"] = False
                events["rerecord_episode"] = False

                # —— 本条结束自动回零(ZERO_AFTER_EPISODE=0 可关):录完立刻回零,
                # 你斟酌保留/丢弃时臂已在归位。这也是断电探测器:臂死了会在此告警 ——
                if zero_after:
                    _try_zero(robot, "结束后")

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
                    _clean_tmp_dirs(dataset.root)  # 清掉失败留下的 tmp* 视频块
                    if _ask("保存失败,本条丢弃。回车继续 / q 退出: ") == "q":
                        break
                    continue
                kept += 1
                log_say("Saved", cfg.play_sounds)
    except KeyboardInterrupt:
        # 任意阶段 Ctrl-C(探测/回零/倒计时/对齐/录制中):丢掉本条残缓冲,优雅收尾,
        # 不甩一屏 traceback(踩过:录制中 Ctrl-C 从 record_loop 里炸出来)。
        print("\n⚑ 键盘中断 —— 本条未完成已丢弃,正在收尾(已保留的不丢)...")
        try:
            dataset.clear_episode_buffer()
        except Exception:
            pass
    finally:
        # 收尾容错:任一步失败(如 CAN 掉线时 robot.disconnect 回零报错)都不该跳过后续清理
        # (相机/键盘/rerun 仍要关掉),逐步 try/except。
        log_say("Stop recording", cfg.play_sounds, blocking=True)
        for name, fn in [
            ("finalize", lambda: dataset.finalize() if dataset else None),
            ("robot.disconnect", lambda: robot.disconnect() if robot.is_connected else None),
            ("teleop.disconnect", lambda: teleop.disconnect() if teleop.is_connected else None),
            ("listener.stop", lambda: listener.stop() if listener is not None else None),
            ("viewer", lambda: shutdown_visualization(cfg.display_mode) if cfg.display_data else None),
        ]:
            try:
                fn()
            except Exception as e:
                logger.error(f"收尾 {name} 失败(继续清理其余): {e}")
        if cfg.dataset.push_to_hub and dataset and dataset.num_episodes > 0:
            try:
                dataset.push_to_hub(tags=cfg.dataset.tags, private=cfg.dataset.private)
            except Exception as e:
                logger.error(f"push_to_hub 失败: {e}")

    print(f"\n完成:共保留 {kept} 条,数据集在 {dataset.root}")


if __name__ == "__main__":
    main()
