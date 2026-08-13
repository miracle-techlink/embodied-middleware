#!/usr/bin/env python
"""闸门式数据采集(单臂 reBot + 腕部深度 + front 第二视角)。

与 ``lerobot-record`` 的区别:把"到点自动连录下一条"换成**人工闸门**——
  1. 每条固定时长(默认 ``--dataset.episode_time_s=15`` 秒;录制中按方向键→可提前结束);
  2. 录完当场选择**保留 / 丢弃重录**;
  3. **回车**才开始录制下一条。

其余全部复用 lerobot 官方栈:同一套 config 解析(CLI 与 lerobot-record 完全一致:
``--robot.* --teleop.* --dataset.* --display_data`` ...)、同一个 ``record_loop`` 录制循环、
同一个 ``LeRobotDataset``(标准格式,可照常传 HF / 之后转 ModelScope)。

用法示例见 scripts/record_rebot_gated.sh(封装好相机/编码/CAN 覆盖)。
"""

import logging
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
                subprocess.run(["pkill", "-f", "bin/rerun --port=9876"],
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
    kept = dataset.num_episodes  # resume 时从已录条数接着数(target 为总目标)
    try:
        with VideoEncodingManager(dataset):
            while kept < target:
                # —— 闸门 1:回车开始录制本条 ——
                if _ask(f"\n▶ 已保留 {kept}/{target}。回车开始录制(每条 {ep_time:g}s,录制中 →/Esc 可提前结束) | q 退出: ") == "q":
                    break

                # 每条开录前重新武装启动 ramp:从当前保持位平滑滑到 leader 当前绝对位姿(见插件 rearm_ramp)
                if hasattr(teleop, "rearm_ramp"):
                    teleop.rearm_ramp()

                events["exit_early"] = False
                log_say(f"Recording episode {kept}", cfg.play_sounds)
                # 单条错误隔离:硬件抖动(CAN 掉线 socketcan write failed / 相机卡)不该杀掉整轮 ——
                # 捕获、丢弃本条、让用户重试或退出,而不是让异常炸掉 50 条会话。
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
                    logger.error(f"episode {kept} 录制中出错(可能 CAN 掉线/相机抖动): {e}")
                    try:
                        dataset.clear_episode_buffer()
                    except Exception:
                        pass
                    if _ask("本条已丢弃。回车重试本条 / q 退出并保存已录: ") == "q":
                        break
                    continue

                # 录制中按 →/Esc/← 触发的 lerobot 键盘标志,这里都只当作"提前结束本条";
                # 是否继续 / 退出整轮,完全交给下面的提示(避免 Esc 直接杀掉整个采集)。
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
                    _clean_tmp_dirs(dataset.root)  # 清掉失败留下的 tmp* 视频块
                    if _ask("保存失败,本条丢弃。回车继续 / q 退出: ") == "q":
                        break
                    continue
                kept += 1
                log_say("Saved", cfg.play_sounds)
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
