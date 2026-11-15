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

悬停/离合(空格切换):
  进入 = 从臂输出冻结在最后目标(保持位姿),主臂自由 —— 可松手/重新摆放主臂;
  再按退出 = 从臂由 ``rearm_ramp`` 从冻结位**限速滑回** leader 实时位姿。
  注:不给 leader 施加锁力 —— 本型 leader(被动采集臂)力矩主权在臂盒 MCU,
  UART 锁力指令实测被立刻夺回(+8° 指令只走出 1.1°/0.2A 即被复位),官方悬停
  为 HD 型号的硬件按钮功能,软件侧无法复刻。
"""

import logging
import os
import sys
import threading

from lerobot.teleoperators.teleoperator import Teleoperator

from .config_starai_to_rebot_leader import StaraiToRebotLeaderConfig
from .mapping import REBOT_ARM_MOTORS, LeaderToRebotMap

logger = logging.getLogger(__name__)


class _SpacebarListener:
    """空格悬停开关:cbreak 读原始 stdin,独立守护线程,不进控制回路。

    lerobot-teleoperate 主循环不读键盘,空格独占;leader 标定阶段的 input()
    提示在本 listener 启动(connect 末尾)之前已完成,互不干扰。"""

    def __init__(self, on_toggle):
        self._on_toggle = on_toggle
        self._stop = threading.Event()
        self._termios = None
        self._old_termios = None
        self._thread: threading.Thread | None = None

    def start(self) -> bool:
        if not sys.stdin.isatty():
            logger.warning("stdin 不是 tty,空格悬停不可用(需终端前台运行)")
            return False
        import termios
        import tty

        self._termios = termios
        self._old_termios = termios.tcgetattr(sys.stdin.fileno())
        tty.setcbreak(sys.stdin.fileno())
        self._thread = threading.Thread(target=self._loop, daemon=True, name="hover-key")
        self._thread.start()
        return True

    def _loop(self) -> None:
        while not self._stop.is_set():
            try:
                ch = os.read(sys.stdin.fileno(), 1)
            except OSError:
                return
            if ch == b" ":
                self._on_toggle()

    def stop(self) -> None:
        self._stop.set()
        if self._old_termios is not None:
            self._termios.tcsetattr(sys.stdin, self._termios.TCSADRAIN, self._old_termios)
            self._old_termios = None


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
        # 悬停状态(_hover_req 由按键线程翻转,转换边沿在 get_action 里处理)
        self._hover_req = False
        self._hover = False
        self._frozen: dict[str, float] | None = None
        self._key_listener: _SpacebarListener | None = None

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
        self._hover_req = self._hover = False
        if self._key_listener is None:
            self._key_listener = _SpacebarListener(self._toggle_hover)
            if self._key_listener.start():
                logger.info(f"{self} 空格 = 悬停/恢复(从臂冻结)")

    def calibrate(self) -> None:
        self._leader.calibrate()

    def configure(self) -> None:
        pass

    def rearm_ramp(
        self,
        step_deg_per_step: float | None = None,
        current_deg: list[float] | None = None,
    ) -> None:
        """重新武装启动限速 ramp:下一次 get_action 从**当前保持位**平滑滑到 leader 当前
        绝对位姿,而不是直通。闸门式采集里每条开录前调用 —— 因为"回车等待"期间机械臂冻结、
        主臂可能被挪动,不重新限速则下一条起步会无限速弹射。
        ``step_deg_per_step`` 给定时本次 ramp 用慢速(开录前对齐用),收敛后自动恢复默认。
        ``current_deg`` = 从臂实际当前位(6 关节度,action 空间):臂被 safe_zero 等外部
        移动过后必须传,否则 ramp 从记忆位起步第一帧就跳变。"""
        self._map.rearm_ramp(step_deg_per_step, current_deg)

    @property
    def ramp_converged(self) -> bool:
        """启动 ramp 是否已收敛(从臂输出已跟上主臂当前位姿)。"""
        return self._map.ramp_converged

    # ---------------- 悬停 ----------------
    def _toggle_hover(self) -> None:
        self._hover_req = not self._hover_req

    # ---------------- action ----------------
    def get_action(self) -> dict[str, float]:
        # 悬停转换边沿(在控制回路里处理,保证 IO 顺序)
        if self._hover_req != self._hover:
            self._hover = self._hover_req
            if self._hover:
                logger.warning(f"{self} ■ 悬停:从臂冻结保持(空格恢复;主臂可自由摆放)")
            else:
                # 冻结期间从臂保持 frozen 位(map 未更新,_cmd_arm 即冻结位),限速滑回
                self._map.rearm_ramp()
                logger.warning(f"{self} ▶ 恢复:从臂限速滑回主臂实时位姿")

        if self._hover and self._frozen is not None:
            return dict(self._frozen)

        la = self._leader.get_action()  # joint_1..6.pos(deg-from-home)+ gripper.pos([0,1])
        leader = [float(la[f"joint_{i + 1}.pos"]) for i in range(6)]
        out7 = self._map.update(leader, float(la.get("gripper.pos", 0.0)))
        out: dict[str, float] = {}
        for i, m in enumerate(REBOT_ARM_MOTORS):
            out[f"{m}.pos"] = out7[i]
        out["gripper.pos"] = out7[6]
        self._frozen = dict(out)  # 正常帧持续刷新,悬停即冻结最新
        return out

    def send_feedback(self, feedback: dict[str, float]) -> None:
        pass

    def disconnect(self) -> None:
        if self._key_listener is not None:
            self._key_listener.stop()
            self._key_listener = None
        self._hover = False
        self._leader.disconnect()
