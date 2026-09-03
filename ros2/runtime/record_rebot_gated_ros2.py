#!/usr/bin/env python
"""闸门式数据采集,ROS2 消息中心版(观测/action 全走 topic,硬件由消息中心持有)。

从直连版 ``~/rebot_datacollect/scripts/record_rebot_gated.py`` 移植,人工闸门节奏不变:
  0. **会话开始即冻结 teleop**:connect 完成马上 enable=false,开录前从臂**不跟手**,
     你安心把主臂摆回 0 位(整轮开录时才恢复,map 自动 rearm 限速,不弹射);
  1. **主臂 0 位人工确认**(整轮开头一次):提示你把主臂摆回 0 位(leader 0° = 从臂
     home 映射,整轮同一初始状态起步),摆好回车即确认;回车后打印一行当前偏差
     **仅供参考**,不自动判定;
  2. 回车后:从臂存活探测 → **3 2 1** 倒计时(**期间从臂冻结不跟手**,主臂可从容
     就位)→ 倒计时结束**这一刻才 enable+rearm 恢复跟随**,与录制同帧起步;没有
     对齐等待(map 限速 ramp 兜底,起步不弹射);
  3. 每条固定时长(默认 15s,录制中按方向键可提前结束),最后 5 秒终端倒计时;
  4. 录完:从臂自动回零 → 当场选**保留 / 丢弃重录** → 回车**直接开下一条**(同样:
     倒计时期间冻结,结束才恢复跟随;没有 0 位确认、没有对齐);
  5. 退出/结束收尾(正常跑完、q、任意阶段 ^C 全一样):**冻结遥操作 → 从臂 go_home
     归零(等到位)→ 落盘**,不甩 traceback。消息中心不随录制退出(设计:连续采集
     免 18s 相机 warmup),收尾会打印全停命令。

直连版各钩子在 ROS2 侧的等价物(硬件不在本进程,全部换成 topic 语义):
  - 从臂存活探测(robstride_ping) → **joint_state 新鲜度**(arm 节点 200Hz 在发即活)
  - 开录前慢速对齐(safe_zero+rearm) → **取消对齐等待**:倒计时期间从臂保持冻结/
    钉在 home,倒计时结束才 enable+rearm(退出残留 go_home 模式+重置 ramp,限速
    不弹射),恢复跟随与录制同帧起步——Will 2026-08-20:倒计时后直接开始、不需要
    对齐,且倒计时结束之前主从不能动
  - 录完自动回零(safe_zero) → **/rebot/teleop/go_home**:map 节点把 leader 视作零,
    从当前位限速 ramp 滑回 home(2026-08-20 加);等 6 关节到位(≤2°,30s 超时)再进
    闸门 2。等待期新鲜度兼作断电探测;会话结束:冻结+go_home 归零(等到位)再落盘
    (Will 2026-08-20:退出/结束时从臂必须归零、停遥操作)
  - 夹爪手感参数(GRIP_KP/GRIP_CLAMP)不在录制侧 —— 映射参数在 map 节点启动参数里

节奏环境变量(``os.environ`` 直读,不进 CLI):
  ``ZERO_AFTER_EPISODE=0``    关掉每条结束后的自动回零 go_home(默认开)
  ``HOME_TOL_DEG``            回零到位判定阈值(度,默认 2.0)
  ``HOME_TIMEOUT_S``          回零等待超时(秒,默认 30)
  ``JOINT_STALE_MS``          存活探测新鲜度(默认 300ms)
  ``START_COUNTDOWN``         开录前倒计时秒数(默认 3,0=关)
  ``END_COUNTDOWN``           结束前倒计时秒数(默认 5,0=关)

前提:终端 1 已跑 ``~/middleware/start_msg_center.sh``;本进程已 source ROS(见 .sh 封装)。
用法示例见 record_rebot_gated_ros2.sh。
"""

import logging
import os
import threading
import time
from collections import deque
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
from lerobot.robots.ros2_rebot_follower.config_ros2_rebot_follower import REBOT_MOTORS
from lerobot.teleoperators import make_teleoperator_from_config
from lerobot.utils.feature_utils import combine_feature_dicts
from lerobot.utils.keyboard_input import init_keyboard_listener
from lerobot.utils.utils import init_logging, log_say
from lerobot.utils.visualization_utils import init_visualization, shutdown_visualization

# 复用官方 record 的配置 dataclass 与录制循环(import 该模块也顺带触发插件注册)
from lerobot.scripts.lerobot_record import RecordConfig, record_loop

logger = logging.getLogger(__name__)

ENABLE_TOPIC = "/rebot/teleop/enable"
LEADER_TOPIC = "/rebot/leader/joint_state"
GO_HOME_TOPIC = "/rebot/teleop/go_home"
_LEADER_JOINTS = ("joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6")

# home 位单源:与 map 节点/直连 teleop 同一份配置(leader=0 的映射目标)
from lerobot.teleoperators.starai_to_rebot_leader.config_starai_to_rebot_leader import (
    StaraiToRebotLeaderConfig,
)
from lerobot.teleoperators.starai_to_rebot_leader.mapping import REBOT_ARM_MOTORS

_HOME6 = dict(zip(REBOT_ARM_MOTORS, StaraiToRebotLeaderConfig().rebot_home_deg))


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
        # 闸门提示符前 Ctrl+C = 优雅退出(落盘已录),不要甩一屏 traceback
        print()
        return "q"


def _countdown(n: int, label: str, stop: threading.Event | None = None) -> None:
    """同行刷新打印 n..1,每秒一拍;给了 stop 事件则可被即时打断。"""
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
    """后台线程:睡到本条剩余 n 秒,然后 5 4 3 2 1。stop 置位即静默退出。"""
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


# ---------------- ROS2 侧钩子(替代直连版的电机探测/回零/对齐) ----------------

def _follower_alive(robot, stale_ms: float = 300.0) -> bool:
    """从臂存活探测 = joint_state 新鲜度。arm 节点 200Hz 发 /rebot/follower/joint_state,
    断流即:arm 节点死了或它那边的 CAN 静默(直连版的 robstride_ping 等价物,弱一档:
    只能看到 arm 节点视角的失效)。"""
    got = robot._bus.latest_joint_state(robot.config.state_topic)
    if got is None:
        logger.warning(f"{robot.config.state_topic} 从未收到消息")
        return False
    age_ms = (time.monotonic() - got[1]) * 1000.0
    if age_ms > stale_ms:
        logger.warning(f"{robot.config.state_topic} 已 {age_ms:.0f}ms 无新帧!")
        return False
    return True


def _leader_dev_deg(bus, stale_ms: float) -> list[tuple[str, float]] | None:
    """主臂 6 关节相对 0 位的当前读数(度,标定零位基准)。返回 None = 断流/无帧。
    gripper 是 [0,1] 开度比,不参与。"""
    got = bus.latest_joint_state(LEADER_TOPIC)
    if got is None or (time.monotonic() - got[1]) * 1000.0 > stale_ms:
        return None
    data = got[0]
    devs = [(n, data[n][0]) for n in _LEADER_JOINTS if n in data]
    return devs or None


def _set_enabled(bus, on: bool) -> None:
    """冻结/恢复 teleop。map 节点在 False→True 转换时会自动 re-arm 启动 ramp,
    所以「恢复」天然带限速,从臂不会跳变。"""
    from std_msgs.msg import Bool

    msg = Bool()
    msg.data = bool(on)
    bus.publish(Bool, ENABLE_TOPIC, msg)


def _go_home(bus) -> None:
    """发自主回零:map 节点把 leader 视作零,从当前输出限速 ramp 滑回 home。
    rearm / enable 恢复都会退出该模式(每条开录时的 rearm 天然接管)。"""
    from std_msgs.msg import Empty

    bus.publish(Empty, GO_HOME_TOPIC, Empty())


def _wait_at_home(robot, stale_ms: float, tol_deg: float = 2.0,
                  timeout_s: float = 30.0) -> bool:
    """等从臂实际回到 home(joint_state 的 6 臂关节 vs _HOME6,全 ≤tol)。"""
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_s:
        got = robot._bus.latest_joint_state(robot.config.state_topic)
        if got is not None and (time.monotonic() - got[1]) * 1000.0 <= stale_ms:
            data = got[0]
            errs = [abs(data[n][0] - h) for n, h in _HOME6.items() if n in data]
            if len(errs) == len(_HOME6) and max(errs) <= tol_deg:
                return True
        time.sleep(0.1)
    return False


def _wait_cmd_settled(teleop, timeout_s: float = 20.0, settle_deg: float = 0.5,
                      stale_ms: float = 500.0) -> bool:
    """(已停用,2026-08-20 Will 拍板取消对齐等待;保留以防回滚)对齐主臂:先 rearm,
    再盯 joint_cmd 0.6s 窗口内 7 关节全不动 = ramp 收敛。"""
    teleop.rearm_ramp()
    bus = teleop._bus
    hist: deque[tuple[float, tuple[float, ...]]] = deque()
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_s:
        got = bus.latest_joint_state(teleop.config.cmd_topic)
        if got is not None and (time.monotonic() - got[1]) * 1000.0 <= stale_ms:
            pos = tuple(got[0].get(m, (0.0,))[0] for m in REBOT_MOTORS)
            hist.append((time.monotonic(), pos))
        while hist and time.monotonic() - hist[0][0] > 0.6:
            hist.popleft()
        if len(hist) >= 5:
            span = max(abs(a - b) for _, p in hist for a, b in zip(p, hist[-1][1]))
            if span < settle_deg:
                return True
        time.sleep(0.1)
    return False


@parser.wrap()
def main(cfg: RecordConfig) -> None:
    init_logging()
    logging.info(pformat(asdict(cfg)))

    if cfg.teleop is None:
        raise ValueError("闸门式采集需要 --teleop.*(用 ros2_rebot_teleop)")

    if cfg.display_data:
        init_visualization(
            cfg.display_mode, session_name="recording", ip=cfg.display_ip, port=cfg.display_port
        )
        import atexit
        import subprocess

        def _close_viewer_on_crash():
            # 启动阶段(建数据集/connect)崩了主 try/finally 还没挂上,孤儿 rerun 窗口
            # 会定格看着像卡死;atexit 在任何退出路径都收掉(与直连版同坑同修)。
            try:
                shutdown_visualization(cfg.display_mode)
            except Exception:
                pass
            try:
                subprocess.run(["pkill", "-f", "rerun --port=9876"],
                               timeout=3, capture_output=True)
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
        ds_imgs = {k for k in dataset.features if k.startswith("observation.images.")}
        new_imgs = {k for k in dataset_features if k.startswith("observation.images.")}
        missing, extra = ds_imgs - new_imgs, new_imgs - ds_imgs
        if missing or extra:
            raise ValueError(
                f"续录数据集 schema 与本次相机配置不一致:库里有而本次没有 {sorted(missing)},"
                f"本次新增 {sorted(extra)}。请保持相机配置与建库时一致,或换新 REPO_ID 开录。"
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

    # connect 会等 joint_state / joint_cmd 首帧 —— 消息中心不在跑会在这里明确报错
    robot.connect()
    teleop.connect()
    bus = teleop._bus
    bus.sub_joint_state(LEADER_TOPIC)  # 主臂 0 位参考读数用(100Hz)
    listener, events = init_keyboard_listener()

    # 进闸门前先冻结 teleop:整个等待/斟酌期间从臂不跟手(Will 要求,2026-08-20);
    # 每条开录时 _set_enabled(True) 恢复 —— map 在 False→True 转换自动 rearm 限速,不弹射。
    _set_enabled(bus, False)
    print("❄️  teleop 已冻结(开录前从臂不跟手)。请把主臂摆回 0 位,到闸门处回车开录。")

    target = cfg.dataset.num_episodes
    ep_time = cfg.dataset.episode_time_s
    zero_after = os.environ.get("ZERO_AFTER_EPISODE", "1") != "0"
    home_tol = float(os.environ.get("HOME_TOL_DEG", "2.0"))
    home_timeout = float(os.environ.get("HOME_TIMEOUT_S", "30"))
    stale_ms = float(os.environ.get("JOINT_STALE_MS", "300"))
    start_cd = int(os.environ.get("START_COUNTDOWN", "3"))
    end_cd = int(os.environ.get("END_COUNTDOWN", "5"))
    kept = dataset.num_episodes  # resume 时从已录条数接着数(target 为总目标)
    first = True  # 整轮第一条多一个 0 位人工确认;对齐等待全流程都没有(Will 2026-08-20)
    try:
        with VideoEncodingManager(dataset):
            while kept < target:
                # —— 闸门 1:整轮第一条前人工确认主臂 0 位(回车即确认,偏差仅供参考,
                #    不自动判定不拦截);后续每条只回车开录,不再 0 位确认/不再对齐 ——
                if first:
                    if _ask(f"\n▶ 共录 {target} 条,每条 {ep_time:g}s(录制中 →/Esc 可提前结束)。\n"
                            f"  请把主臂摆回 0 位,摆好后回车开始整轮录制 | q 退出: ") == "q":
                        break
                    devs = _leader_dev_deg(bus, stale_ms)
                    if devs is None:
                        print("⚠️  /rebot/leader/joint_state 断流(leader 节点出事?查 logs/latest/leader_node.log)"
                              "—— 偏差读不到,继续按你的判断走。")
                    else:
                        mx = max(abs(v) for _, v in devs)
                        bad = [(n, v) for n, v in devs if abs(v) > 3.0]
                        detail = " ".join(f"{n.replace('joint_', 'j')}:{v:+.1f}" for n, v in bad)
                        note = f",偏差较大: {detail}" if bad else ""
                        print(f"ℹ️  主臂当前最大偏差 {mx:.1f}°{note}(仅供参考)")
                elif _ask(f"\n▶ 已保留 {kept}/{target}。回车开始下一条"
                          f"(每条 {ep_time:g}s,→/Esc 可提前结束;主臂摆回起始位) | q 退出: ") == "q":
                    break

                # —— 从臂存活探测:joint_state 断流 = arm 节点/CAN 侧出事 ——
                if not _follower_alive(robot, stale_ms):
                    print("⚠️  joint_state 断流 —— 消息中心/arm 节点/CAN 侧出事!"
                          f"查 /tmp/arm_node.log 和终端 1。")
                    if _ask("回车重新探测 / q 退出并保存已录: ") == "q":
                        break
                    continue

                # —— 3 2 1 开录倒计时(START_COUNTDOWN=0 可关)。倒计时期间从臂保持
                #    冻结/钉在 home,主臂怎么晃它都不动(Will 2026-08-20:倒计时结束
                #    之后主从才能动,不然不能动)——
                if start_cd > 0:
                    _countdown(start_cd, "▶ 开录倒计时")

                # —— 倒计时结束,此刻才恢复跟随(与录制同时起步):enable(True)+rearm。
                #    rearm 不只是重置 ramp——它还退出上一条录完残留的 go_home 模式
                #    (map 里 enable 已是 True 不会自退,没这步从臂会一直钉在 home
                #    不跟手)。False→True 转换 map 也会自动 rearm,双保险 ——
                _set_enabled(teleop._bus, True)
                teleop.rearm_ramp()

                first = False  # 从这起:第一条已真正开录,后续都走简式闸门
                events["exit_early"] = False
                log_say(f"Recording episode {kept}", cfg.play_sounds)
                cd_stop, cd_th = (
                    _start_end_countdown(ep_time, end_cd) if 0 < end_cd < ep_time else (None, None)
                )
                # 单条错误隔离:topic 抖动不该杀掉整轮,丢弃本条让用户重试
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
                    logger.error(f"episode {kept} 录制中出错(可能 topic 断流/相机抖动): {rec_error}")
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

                # —— 本条结束自动回零(ZERO_AFTER_EPISODE=0 可关):发 go_home,map 从当前
                #    位限速滑回 home(等价直连版 safe_zero 的"录完归零再斟酌");到位/超时
                #    才进闸门 2。等待期新鲜度兼作断电探测 ——
                if zero_after:
                    print("↺ 从臂回零中(go_home,map 限速滑回 home)...")
                    _go_home(bus)
                    if not _wait_at_home(robot, stale_ms, tol_deg=home_tol,
                                         timeout_s=home_timeout):
                        if not _follower_alive(robot, stale_ms):
                            print("⚠️  回零超时且 joint_state 断流 —— 消息中心/arm 侧出事!"
                                  "查 logs/latest/arm_node.log。")
                        else:
                            print(f"⚠️  回零超时({home_timeout:g}s),继续进闸门;必要时选 d 重录。")

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
        # 收尾不可再被打断:连按 ^C 时第二个 ^C 会杀掉 finally,冻结/落盘被跳过
        # (踩过 2026-08-20 11:35:三连 ^C 后 teleop 一直处于恢复态,臂持续跟手)
        import signal
        try:
            signal.signal(signal.SIGINT, signal.SIG_IGN)
        except (ValueError, OSError):
            pass  # 非主线程等场景,尽力而为
        log_say("Stop recording", cfg.play_sounds, blocking=True)

        def _exit_go_home() -> None:
            # 退出收尾:从臂必须归零(Will 2026-08-20:退出/结束时归零+停遥操作)。
            # freeze 之后照样能动——map 的 go_home 模式不看 enable,先冻结再滑回零。
            print(f"↺ 退出收尾:从臂回零中(go_home,最长等 {home_timeout:g}s)...")
            _go_home(teleop._bus)
            if _wait_at_home(robot, stale_ms, tol_deg=home_tol, timeout_s=home_timeout):
                print("✅ 从臂已回零。")
            elif not _follower_alive(robot, stale_ms):
                print("⚠️  退出回零失败且 joint_state 断流 —— arm/消息中心侧出事,查 logs/latest/arm_node.log。")
            else:
                print(f"⚠️  退出回零 {home_timeout:g}s 未到位(可能卡阻),继续收尾。")

        for name, fn in [
            # 顺序即语义:先停遥操作(冻结)→ 从臂归零 → 才轮到落盘/断开
            ("freeze", lambda: _set_enabled(teleop._bus, False)
                if teleop.is_connected else None),
            ("go_home", lambda: _exit_go_home() if teleop.is_connected else None),
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
    print("状态:从臂已归零、遥操作已停止(冻结,不跟手);消息中心仍在跑(设计如此:连续采集免 18s 相机 warmup)。")
    print("  继续采:重跑本命令;续录同一数据集:RESUME=1 + 带时间戳的完整 REPO_ID;")
    print("  全停消息中心:~/middleware/start_teleop.sh stop")


if __name__ == "__main__":
    main()
