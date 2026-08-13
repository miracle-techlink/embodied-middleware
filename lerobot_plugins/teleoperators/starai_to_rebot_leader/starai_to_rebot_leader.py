#!/usr/bin/env python
# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License").
"""`starai_to_rebot_leader` — StarAI Violin leader → reBot B601-RS 关节空间 teleop。

内部持有一个 ``StaraiViolinLeader``(复用其标定),``get_action`` 把 leader 输出映射成
reBot 关节空间的绝对目标角,键与 ``rebot_follower`` 的 action_features 完全一致
(``shoulder_pan.pos`` ... ``wrist_roll.pos`` + ``gripper.pos``)。见 config 里的映射说明。

映射/启动 ramp/夹爪换算的数学全部在 ``mapping.py``(与 ROS2 teleop_map_node 共用),
这里只剩 leader IO 和键名适配。
"""

import logging

from lerobot.teleoperators.teleoperator import Teleoperator

from .config_starai_to_rebot_leader import StaraiToRebotLeaderConfig
from .mapping import REBOT_ARM_MOTORS, LeaderToRebotMap

logger = logging.getLogger(__name__)


class StaraiToRebotLeader(Teleoperator):
    config_class = StaraiToRebotLeaderConfig
    name = "starai_to_rebot_leader"

    def __init__(self, config: StaraiToRebotLeaderConfig):
        super().__init__(config)
        self.config = config

        from lerobot.teleoperators.starai_violin_leader import (
            StaraiViolinLeader,
            StaraiViolinLeaderConfig,
        )

        self._leader = StaraiViolinLeader(
            StaraiViolinLeaderConfig(
                port=config.port,
                baudrate=config.baudrate,
                arm_servo_ids=config.arm_servo_ids,
                gripper_servo_id=config.gripper_servo_id,
                id=config.leader_id,
            )
        )
        self._map = LeaderToRebotMap(
            rebot_home_deg=config.rebot_home_deg,
            flip=config.flip,
            scale=config.scale,
            absolute=config.absolute,
            startup_ramp_deg_per_step=config.startup_ramp_deg_per_step,
            grip_close_deg=config.grip_close_deg,
            grip_open_deg=config.grip_open_deg,
            grip_clamp_deg=config.grip_clamp_deg,
            grip_ratio_min=config.grip_ratio_min,
            grip_ratio_max=config.grip_ratio_max,
        )

    # ---------------- features ----------------
    @property
    def action_features(self) -> dict[str, type]:
        ft = {f"{m}.pos": float for m in REBOT_ARM_MOTORS}
        ft["gripper.pos"] = float
        return ft

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    # ---------------- lifecycle(全部委托内部 leader) ----------------
    @property
    def is_connected(self) -> bool:
        return self._leader.is_connected

    @property
    def is_calibrated(self) -> bool:
        return self._leader.is_calibrated

    def connect(self, calibrate: bool = True) -> None:
        self._leader.connect(calibrate=calibrate)
        self._map.reset()  # 下一次 get_action 重新确定参考基准,启动 ramp 从 home 重新开始

    def calibrate(self) -> None:
        self._leader.calibrate()

    def configure(self) -> None:
        pass

    def rearm_ramp(self) -> None:
        """重新武装启动限速 ramp:下一次 get_action 从**当前保持位**平滑滑到 leader 当前
        绝对位姿,而不是直通。闸门式采集里每条开录前调用 —— 因为"回车等待"期间机械臂冻结、
        主臂可能被挪动,不重新限速则下一条起步会无限速弹射。"""
        self._map.rearm_ramp()

    # ---------------- action ----------------
    def get_action(self) -> dict[str, float]:
        la = self._leader.get_action()  # joint_1..6.pos(deg-from-home)+ gripper.pos([0,1])
        leader = [float(la[f"joint_{i + 1}.pos"]) for i in range(6)]
        out7 = self._map.update(leader, float(la.get("gripper.pos", 0.0)))
        out: dict[str, float] = {}
        for i, m in enumerate(REBOT_ARM_MOTORS):
            out[f"{m}.pos"] = out7[i]
        out["gripper.pos"] = out7[6]
        return out

    def send_feedback(self, feedback: dict[str, float]) -> None:
        pass

    def disconnect(self) -> None:
        self._leader.disconnect()
