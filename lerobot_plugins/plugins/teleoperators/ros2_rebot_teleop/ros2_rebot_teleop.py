#!/usr/bin/env python
"""`ros2_rebot_teleop` — 镜像 /rebot/follower/joint_cmd 作为 action 源。

配合 ``ros2_rebot_follower`` 使用:teleop_map_node 以 100Hz 自持控制回路(映射+
ramp 都在它那边),record 循环通过本 teleop 读到的是**实际下发给从臂**的目标角,
所以数据集 action 与机械臂真实执行严格同帧。

record_rebot_gated 的 hook 对齐:connect / get_action / rearm_ramp / is_connected /
disconnect。rearm_ramp() 往 /rebot/teleop/rearm 发 Empty,map 节点收到后从当前
保持位重新限速滑向主臂位姿(等价原 starai_to_rebot_leader.rearm_ramp)。
"""

import logging
import time
from functools import cached_property

from lerobot.robots.ros2_rebot_follower.config_ros2_rebot_follower import REBOT_MOTORS
from lerobot.robots.ros2_rebot_follower.ros2_bus import Ros2Bus
from lerobot.teleoperators.teleoperator import Teleoperator

from .config_ros2_rebot_teleop import Ros2RebotTeleopConfig

logger = logging.getLogger(__name__)


class Ros2RebotTeleop(Teleoperator):
    config_class = Ros2RebotTeleopConfig
    name = "ros2_rebot_teleop"

    def __init__(self, config: Ros2RebotTeleopConfig):
        super().__init__(config)
        self.config = config
        self._bus: Ros2Bus | None = None
        self._connected = False

    @cached_property
    def action_features(self) -> dict[str, type]:
        return {f"{m}.pos": float for m in REBOT_MOTORS}

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def is_calibrated(self) -> bool:
        return True

    def connect(self, calibrate: bool = True) -> None:
        self._bus = Ros2Bus.instance()
        self._bus.sub_joint_state(self.config.cmd_topic)
        deadline = time.monotonic() + 5.0
        while self._bus.latest_joint_state(self.config.cmd_topic) is None:
            if time.monotonic() > deadline:
                raise ConnectionError(
                    f"{self.config.cmd_topic} 5s 无消息 — teleop_map_node 在跑吗?"
                )
            time.sleep(0.05)
        self._connected = True
        logger.info(f"{self}: 已订阅 {self.config.cmd_topic}。")

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    def get_action(self) -> dict[str, float]:
        got = self._bus.latest_joint_state(self.config.cmd_topic)
        if got is None or time.monotonic() - got[1] > self.config.stale_ms / 1000.0:
            raise ConnectionError(
                f"{self.config.cmd_topic} 断流(>{self.config.stale_ms}ms)— 检查 teleop_map_node"
            )
        data = got[0]
        return {f"{m}.pos": data[m][0] for m in REBOT_MOTORS}

    def rearm_ramp(self) -> None:
        """闸门录制每条开录前调用:通知 map 节点重新武装启动 ramp。"""
        from std_msgs.msg import Empty

        self._bus.publish(Empty, self.config.rearm_topic, Empty())

    def send_feedback(self, feedback: dict[str, float]) -> None:
        pass

    def disconnect(self) -> None:
        self._connected = False
