#!/usr/bin/env python
# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License").
"""Config for `rebot_follower` — 单臂 Seeed reBot B601-RS 的原生 lerobot Robot(带深度相机)。

在官方 ``seeed_b601_rs_follower`` 基础上做两点改动,让它更适合"单臂 + 双视角带深度"的采集:

1) ``joint_directions`` 全部归一化为 1.0(含夹爪)。官方默认在 ``send_action`` 里按
   ``joint_directions`` 翻转部分关节(elbow_flex/wrist_flex/wrist_yaw = -1,gripper = 6),
   会导致**录进数据集的 action 与 observation.state 不在同一坐标系**(观测读的是原始电机角,
   不做翻转)。归一化后 action 与 observation 同帧 → 训练最干净。物理方向由 teleop
   ``starai_to_rebot_leader`` 的 ``flip`` 负责(默认已把这几个 -1 关节搬过去)。

2) ``max_relative_target`` 默认给一个步长上限(度/步),让上电时从当前姿态**平滑 ramp**
   到 teleop 的 home 目标,避免起步跳变。想解除改成 None(靠 MIT kp/kd 自身平滑)。

深度:相机里 ``type: orbbec`` 且 ``use_depth: true`` 的,会自动多出一个
``observation.images.<cam>_depth``(H,W,1 uint16 毫米)特征,lerobot 0.5.2 认它为深度图录制。
普通 USB 摄像头(``type: opencv``)只出彩色,做第二视角。
"""

from dataclasses import dataclass, field

from lerobot.robots.robot import RobotConfig
from lerobot_robot_seeed_b601.config_seeed_b601_rs_follower import SeeedB601RSFollowerConfig


@RobotConfig.register_subclass("rebot_follower")
@dataclass
class RebotFollowerConfig(SeeedB601RSFollowerConfig):
    # 实测行程(teleop_starai_to_rebot.py --sweep,2026-07-08)去 ~3° 余量,替换官方保守限位。
    # send_action 就是往这组限位夹取;低位硬止点(肩/肘)保持 0。
    joint_limits: dict[str, tuple[float, float]] = field(
        default_factory=lambda: {
            "shoulder_pan": (-147.0, 147.0),   # 实测 [-150.5, 149.5]
            "shoulder_lift": (0.0, 202.0),     # 实测 [ -0.1, 205.0]
            "elbow_flex": (0.0, 240.0),        # 实测 [ -0.2, 243.8]
            "wrist_flex": (-87.0, 86.0),       # 实测 [-90.9,  89.3]
            "wrist_yaw": (-107.0, 104.0),      # 实测 [-110.4, 107.3]
            "wrist_roll": (-172.0, 239.0),     # 实测 [-175.5, 242.6]
            "gripper": (0.0, 270.0),           # 实测 [ -7.4, 381.7](夹取只用子区间,保持保守)
        }
    )
    # action 与 observation 同帧:关闭官方方向翻转(物理方向交给 teleop 的 flip)。
    joint_directions: dict[str, float] = field(
        default_factory=lambda: {
            "shoulder_pan": 1.0,
            "shoulder_lift": 1.0,
            "elbow_flex": 1.0,
            "wrist_flex": 1.0,
            "wrist_yaw": 1.0,
            "wrist_roll": 1.0,
            "gripper": 1.0,
        }
    )

    # None=不限步(默认)。设了数值会开启官方的 ensure_safe_goal_position:每周期把目标夹到
    # present±N,而 present 带传感器噪声 → 运动时命令通道被注入噪声 → 叠加高 kp 会明显抖动。
    # home 已是坐姿、起步无跳变,故默认关掉。想要起步限速再设个较大值(如 20~30)。
    max_relative_target: float | dict[str, float] | None = None

    # ---------- 观测相机读取(性能) ----------
    # True(默认,已硬件验证)= get_observation 用 read_latest 非阻塞取最新缓存帧。官方 async_read
    #   会**阻塞等下一帧**(30fps→33ms),把控制循环死锁在相机帧率上(实测 get_observation 20ms、
    #   循环 29.9Hz,一加 dataset/rerun 就掉到 27-28Hz)。read_latest 瞬时返回,循环由 record 的
    #   precise_sleep 按 fps 节流,帧新鲜度不变但消除"超时一次就多等整帧"的断崖 →
    #   实测循环 33.5→13.0ms、可达 29.9→76.9Hz。
    #   取舍:相机 fps < 循环 fps 时会读到重复帧(30/30 基本 1:1);帧超过 stale_frame_ms 会回退
    #   上一帧并告警(不会静默录冻结帧)。设 False 回退官方阻塞 async_read。
    cameras_nonblocking: bool = True
    stale_frame_ms: int = 200  # nonblocking 时,最新帧超过此龄触发告警+回退(暴露相机卡死)

    # ---------- 设备容错接入 ----------
    # True(默认)= connect 时相机逐个尝试,掉线/故障的相机摘除后继续,其余设备正常接入,
    #   数据集特征随之少这一路(结构自洽,不会录进全黑帧);False 回退官方行为(任一相机失败
    #   整体中止)。无论何值,required_cameras 名单内的相机失败仍然 fatal。
    #   运行中相机死掉由 cameras_nonblocking 的 stale 回退兜住(告警+上一帧),不阻断循环;
    #   is_connected 只看臂总线,相机掉线不会挡住退出时的 safe_zero/卸力矩(安全)。
    # 注意:有的相机固件拒绝一切 SET 命令(如 1080P USB Camera 2bdf:0289,设 width/fourcc
    #   直接失败被摘除)——给它配 fps/width/height/fourcc 全部留空(None),lerobot 会跳过
    #   所有 SET 直接用相机原生默认(640x480 YUYV)出流,特征形状连接后从相机实际值取。
    allow_missing_cameras: bool = True
    required_cameras: list[str] = field(default_factory=list)

    # False(默认)= 臂 CAN 总线连不上直接报错(通常意味着真故障:can0 没起/线没插)。
    # True = 纯相机模式:臂连不上也继续,电机观测补零(数据集结构不变)、send_action
    #   透传不执行 —— 主臂 teleop 还在的话,可以照常录"操作员意图 + 多视角视频",
    #   适合臂送修/调试相机机位时不停工。录出的数据 observation.state 全零,训练前自行筛除。
    allow_missing_arm: bool = False

    def __post_init__(self):
        """放宽 RobotConfig.__post_init__ 的相机 width/height/fps 必填校验:允许留 None
        (= 不向相机发任何 SET 命令,用相机原生默认出流,迁就拒绝 SET 的坏固件相机),
        特征形状连接后由 RebotFollower._cam_dims 从相机实际协商值解析。
        RobotConfig.__post_init__ 只有这一个检查,覆盖它不跳过其他逻辑(已核对 0.6.1);
        lerobot 升级时需重新核对 robots/config.py。"""

    # ---------- 夹爪(直驱 7 号电机,绕开官方力矩前馈路径) ----------
    # 官方 RS 夹爪走 send_mit(0,0,kp=0,kd=1.5,tau_ff),纯力矩前馈(力矩上限 10)推不动,
    # 且每周期在 send_action 里塞一句阻塞 poll_feedback_once 拖乱机械臂节奏。这里改为把夹爪
    # 从 action 摘出、直驱 motor 7 做位置 MIT(kp/kd),臂发送里就不含夹爪 → 更顺、夹爪也能动。
    grip_follow: bool = True
    grip_kp: float = 9.0
    grip_kd: float = 0.3
    grip_max_step_deg: float = 25.0     # 夹爪每步设定点变化上限(越大越跟手)

    # ---------- 退出安全 ----------
    # 官方 follower 断开时直接卸力矩 → 中段姿态会受重力砸下来。这里先平滑回到零位(坐姿,
    # 官方标定定义的 sit-down),到位后再交给父类卸力矩(此时手臂是撑住的,安全)。
    return_home_on_exit: bool = True
    exit_home_deg: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    exit_speed_deg_s: float = 40.0
