#!/usr/bin/env python
# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License").
"""leader → reBot 关节空间映射的**纯数学核心**,lerobot teleop 与 ROS2 teleop_map_node 共用。

两边各自只负责 IO(lerobot: 内部 leader 对象;ROS2: JointState topic),映射、启动 ramp、
夹爪换算全部走这一个文件 —— **改映射只改这里,两条栈同时生效**。

映射(absolute 模式,leader 标定零位恒对应 rebot_home_deg):

    reBot[j] = rebot_home_deg[j] + sign[j] * scale * (leader[j] - leader_ref[j])

启动 ramp:上电/重新武装后从 home(或保持位)限速滑向目标,纯输出插值不夹传感器 → 不抖;
收敛后直通,稳态 1:1。夹爪:leader ratio [min,max] → [close_eff, open](close_eff 含 clamp
过冲,产生持续夹持力)。
"""

REBOT_ARM_MOTORS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_yaw", "wrist_roll"]


def parse_flip(s: str) -> list[float]:
    """"3,4,5" → [1,1,-1,-1,-1,1](1-基的关节序号转符号表)。"""
    sign = [1.0] * 6
    for tok in s.split(","):
        tok = tok.strip()
        if tok:
            sign[int(tok) - 1] = -1.0
    return sign


class LeaderToRebotMap:
    """无 IO 的映射状态机。

    输入:leader 6 关节角(度,相对其标定零位)+ 夹爪 ratio 原值;
    输出:reBot 7 维绝对目标角(6 臂关节 + 夹爪,度)。
    """

    def __init__(
        self,
        *,
        rebot_home_deg: list[float],
        flip: str,
        scale: float,
        absolute: bool = True,
        startup_ramp_deg_per_step: float = 6.0,
        grip_close_deg: float = 20.0,
        grip_open_deg: float = 250.0,
        grip_clamp_deg: float = 25.0,
        grip_ratio_min: float = 0.05,
        grip_ratio_max: float = 0.95,
    ):
        self._home = [float(v) for v in rebot_home_deg]
        self._sign = parse_flip(flip)
        self._scale = float(scale)
        self._absolute = bool(absolute)
        self._ramp_step = float(startup_ramp_deg_per_step)
        # 夹爪闭合端过冲:ratio=0 时目标压过闭合位,产生持续夹持力。
        close_dir = -1.0 if grip_close_deg <= grip_open_deg else 1.0
        self._grip_close_eff = float(grip_close_deg) + close_dir * float(grip_clamp_deg)
        self._grip_open = float(grip_open_deg)
        self._grip_ratio_min = float(grip_ratio_min)
        self._grip_ratio_max = float(grip_ratio_max)
        self.reset()

    def reset(self) -> None:
        """参考基准/ramp 全重置(等价 teleop 重新 connect)。"""
        self._leader_home: list[float] | None = None
        self._cmd_arm: list[float] | None = None  # 启动 ramp 用:当前臂输出目标
        self._ramped_in: bool = False             # 启动 ramp 是否已收敛(收敛后直通)

    def rearm_ramp(self) -> None:
        """重新武装启动限速 ramp:下一次 update 从**当前保持位**(``_cmd_arm``)平滑滑向目标,
        而不是直通。闸门式采集每条开录前 / 冻结恢复时调用 —— 等待期间主臂可能被挪动,
        不重新限速则下一段起步会无限速弹射。不重置 ``_cmd_arm`` 和 ``_leader_home``。"""
        self._ramped_in = False

    def update(self, leader: list[float], gripper_raw: float) -> list[float]:
        """一帧映射。``leader`` = 6 关节度(相对 leader 标定零位),``gripper_raw`` = 夹爪
        ratio 原值(未归一);返回 7 维绝对目标角(度):6 臂关节 + 夹爪。"""
        leader = [float(v) for v in leader]
        if self._leader_home is None:
            # absolute → leader 标定零位(全 0),绝对角恒定映射(启动即对上绝对位姿);
            # 非 absolute → 首帧 leader 读数(旧的进入即锚定,无起步跳变)。
            self._leader_home = [0.0] * 6 if self._absolute else list(leader)

        # 绝对目标(reBot 关节空间)
        target = [
            self._home[i] + self._sign[i] * self._scale * (leader[i] - self._leader_home[i])
            for i in range(6)
        ]

        # 启动 ramp:从 home/保持位限速滑向目标(纯输出插值,不夹传感器 → 不抖),收敛后直通
        if self._cmd_arm is None:
            self._cmd_arm = list(self._home)
        if not self._ramped_in and self._ramp_step > 0.0:
            residual = 0.0
            for i in range(6):
                d = max(-self._ramp_step, min(self._ramp_step, target[i] - self._cmd_arm[i]))
                self._cmd_arm[i] += d
                residual = max(residual, abs(target[i] - self._cmd_arm[i]))
            arm = list(self._cmd_arm)
            if residual < 0.5:
                self._ramped_in = True
        else:
            arm = target
            self._cmd_arm = list(target)

        # 夹爪:leader ratio [min, max] → [close_eff, open]
        denom = max(self._grip_ratio_max - self._grip_ratio_min, 1e-3)
        ratio = min(1.0, max(0.0, (float(gripper_raw) - self._grip_ratio_min) / denom))
        grip = self._grip_close_eff + ratio * (self._grip_open - self._grip_close_eff)

        return arm + [grip]
