#!/usr/bin/env python
# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License").
"""Config for `starai_to_rebot_leader` — StarAI Violin leader → reBot 关节空间 teleop。

内部包一个已标定的 ``starai_violin_leader``,把它输出的"相对 leader 零位的角度"映射成
**reBot 自己关节空间的绝对目标角**(``shoulder_pan.pos`` ... ``wrist_roll.pos`` + ``gripper.pos``),
直接喂给 ``rebot_follower``。因为映射发生在 teleop 里,``lerobot-record`` 录进数据集的 action
就是 reBot 空间的目标(与 observation.state 同帧)。

映射(绝对映射,leader 标定零位恒对应 rebot_home_deg):

    reBot[j] = rebot_home_deg[j] + sign[j] * scale * (leader_rel[j] - leader_ref[j])

``absolute=True``(默认): ``leader_ref = 0``(leader 标定零位)。leader 的**绝对**角恒定映射到
从臂绝对目标 —— 进入遥操作瞬间从臂即对上 leader 当前绝对位姿(不再把"进入那一刻"当零位)。
因 robot 侧 ``max_relative_target=None`` 不限步,启动这一跳由 teleop 端 ``startup_ramp_deg_per_step``
限速平滑滑过去(纯输出插值,不碰传感器 present → 不抖;收敛后切直通,稳态 1:1)。

``absolute=False``: 旧行为,``leader_ref`` = teleop 首帧 leader 读数(t0 目标==rebot_home,无起步跳变)。

``rebot_home_deg`` = leader 中立时从臂的绝对位姿(默认物理正直坐姿)。
"""

from dataclasses import dataclass, field

from lerobot.teleoperators.config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("starai_to_rebot_leader")
@dataclass
class StaraiToRebotLeaderConfig(TeleoperatorConfig):
    # ---------- StarAI Violin leader 串口(飞特 UART 舵机) ----------
    port: str = "/dev/ttyCH341USB0"
    baudrate: int = 1_000_000
    arm_servo_ids: list[int] = field(default_factory=lambda: [0, 1, 2, 3, 4, 5])
    gripper_servo_id: int = 6
    # 复用已标定的 starai_violin_leader 标定文件(.../teleoperators/starai_violin_leader/<leader_id>.json)
    leader_id: str = "leader1"

    # ---------- 臂映射 ----------
    # reBot 各关节 home(度,原始电机角)= 从臂**物理正直/中立位**的原始编码器角。
    # 2026-07-08 diag_align 卸力矩手摆到正直位实测 ≈ [0.2, 0, 0.4, -1.3, 1.5, 2.1] → 取整如下。
    # 开机(leader 中立)时从臂即命令到此 = 正直坐姿,不歪、不抬臂。sl/ef 的 0 恰是其单侧行程起点
    # (StarAI 肩只往+、肘只往−),故也是行程起点,无死区。旧值 [7,2,2,-10,1,0] 会让 sp 偏7°、wf 偏9°。
    rebot_home_deg: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, -1.0, 1.5, 2.0])
    # 需翻转方向的臂关节(1-6,逗号分隔)。默认 3,4,5 = 把官方 joint_directions 的 -1
    # (elbow_flex/wrist_flex/wrist_yaw)搬到这里(因为 robot 侧已把 directions 归一为 1)。
    flip: str = "3,4,5"
    scale: float = 1.0  # 主→从 增量增益(1.0 = 1:1)

    # True = 绝对映射:leader 标定零位恒对应 rebot_home_deg,进入遥操作即跳到 leader 当前绝对位姿。
    # False = 旧的"进入遥操作首帧锚定为零位"的相对映射(无起步跳变)。
    absolute: bool = True
    # 启动 ramp 限速(度/步 @teleop 频率):absolute=True 时把启动那一跳从 home 平滑滑到 leader
    # 绝对位姿的每步上限;仅在收敛前生效,之后直通(稳态 1:1 无滞后)。纯输出端插值,不夹传感器
    # present → 不会引入 max_relative_target 那种抖动。设 <=0 关闭 = 瞬时跳变(RobStride 高 kp 下
    # 大偏差会暴力弹射,不建议)。6 度/步 @30Hz ≈ 180°/s,90° 偏差约 0.5s 滑到位。
    startup_ramp_deg_per_step: float = 6.0

    # ---------- 夹爪(输出 reBot gripper.pos 目标角,官方 follower 做阻抗力控) ----------
    # reBot 夹爪原始电机行程约 [0,270]。ratio=0(主臂捏到底)→ close 端(含 clamp 过冲产生夹持力),
    # ratio=1(张开)→ open 端。若开合方向反了,把 close/open 两个值对调即可。
    grip_close_deg: float = 20.0
    grip_open_deg: float = 250.0
    grip_clamp_deg: float = 25.0  # 闭合端过冲(度):持续夹持力,越大夹越紧(过大易过热)
    # StarAI 主臂夹爪 frac 区间 → 映射到从臂开合。2026-07-08 probe_gripper_servo 实测:
    # 主臂夹爪闭合(捏到底)raw≈-2°→frac≈0(闭合端),张开 raw≈69.6°→frac≈1(张开端),满行程约 [0,1]。
    # (旧值 0.62 是更早一次标定的残留 → 会砍掉前 62% 行程、看着像"夹爪不动";已改。)
    # 两端各留 ~0.05 余量,保证满合/满开触底、抗噪。ratio_min=闭合端,ratio_max=张开端。
    grip_ratio_min: float = 0.05
    grip_ratio_max: float = 0.95
