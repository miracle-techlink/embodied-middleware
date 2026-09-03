#!/usr/bin/env python
"""`ros2_piper_teleop` — 订阅主臂位姿 topic(/piper/leader/joint_state)作为 action 源。

配合 ``ros2_piper_follower`` 使用:主臂固件直通驱动从臂(无软件控制回路),
piper_leader_node 把主臂联动帧还原成主臂位姿发到 topic,本 teleop 订阅它当 action。

接口对齐 ros2_rebot_teleop:connect / get_action / is_connected / disconnect。
rearm_ramp 不存在(Piper 无 ramp 概念,固件直驱),提供空实现保底。
"""

import logging
import time
from functools import cached_property

from lerobot.teleoperators.teleoperator import Teleoperator

from ...robots.ros2_rebot_follower.ros2_bus import Ros2Bus
from .config_ros2_piper_teleop import PIPER_MOTORS, Ros2PiperTeleopConfig

logger = logging.getLogger(__name__)


class Ros2PiperTeleop(Teleoperator):
    config_class = Ros2PiperTeleopConfig
    name = "ros2_piper_teleop"

    def __init__(self, config: Ros2PiperTeleopConfig):
        super().__init__(config)
        self.config = config
        self._bus: Ros2Bus | None = None
        self._connected = False

    @cached_property
    def action_features(self) -> dict[str, type]:
        return {f"{m}.pos": float for m in PIPER_MOTORS}

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def is_calibrated(self) -> bool:
        return True  # 主臂标定/使能在固件侧,与本进程无关

    def connect(self, calibrate: bool = True) -> None:
        self._bus = Ros2Bus.instance()
        self._bus.sub_joint_state(self.config.state_topic)
        deadline = time.monotonic() + 8.0
        while self._bus.latest_joint_state(self.config.state_topic) is None:
            if time.monotonic() > deadline:
                raise ConnectionError(
                    f"{self.config.state_topic} 8s 无消息 —— piper_leader_node 在跑吗?"
                    f"(主臂需先掰动一下,让它发出首组联动帧,require_full_first 收满 7 关节才首发)"
                )
            time.sleep(0.05)
        self._connected = True
        logger.info(f"{self}: 已订阅 {self.config.state_topic}。")

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    def get_action(self) -> dict[str, float]:
        got = self._bus.latest_joint_state(self.config.state_topic)
        if got is None:
            raise ConnectionError(f"{self.config.state_topic} 尚无消息")
        # 主臂事件驱动:静置时帧会"陈旧",但那是合法的静止 —— 只在彻底超 stale_ms
        # (leader 节点死了/总线断了)时才判错。录到重复静止值是正确语义。
        if time.monotonic() - got[1] > self.config.stale_ms / 1000.0:
            raise ConnectionError(
                f"{self.config.state_topic} 断流(>{self.config.stale_ms}ms)— 检查 piper_leader_node"
            )
        data = got[0]
        return {f"{m}.pos": data[m][0] for m in PIPER_MOTORS}

    def rearm_ramp(self) -> None:
        """Piper 固件直驱,无 ramp 概念。空实现以对齐闸门录制的 hook 调用。"""
        pass

    def send_feedback(self, feedback: dict[str, float]) -> None:
        pass

    def disconnect(self) -> None:
        self._connected = False  # 总线是进程级单例,不动
