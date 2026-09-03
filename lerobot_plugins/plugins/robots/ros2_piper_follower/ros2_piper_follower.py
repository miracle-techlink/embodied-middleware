#!/usr/bin/env python
"""`ros2_piper_follower` — 观测来自 middleware piper_arm_node 的 PiPER X 从臂。

硬件由 piper_arm_node 持有(backend=mujoco 时即为仿真值),本类只做订阅与格式
还原;录进数据集的键:

    observation.state  ← /piper/joint_state 的 joint_1..joint_6.pos(度) + gripper.pos(毫米)
    action(send_action) ← own_cmd_topic=True 时发到 /piper/joint_cmd(7 关节绝对目标)

前置:middleware piper_arm_node 已在跑,本进程已 source ROS 环境(见 ros2_bus 报错提示)。
"""

import logging
import time
from functools import cached_property

import numpy as np
from lerobot.robots.robot import Robot

from ..ros2_rebot_follower.ros2_bus import Ros2Bus
from .config_ros2_piper_follower import PIPER_MOTORS, Ros2PiperFollowerConfig

logger = logging.getLogger(__name__)


class Ros2PiperFollower(Robot):
    config_class = Ros2PiperFollowerConfig
    name = "ros2_piper_follower"

    def __init__(self, config: Ros2PiperFollowerConfig):
        super().__init__(config)
        self.config = config
        self._bus: Ros2Bus | None = None
        self._connected = False
        self._last_img: dict = {}

    # ---------------- features ----------------
    @property
    def cameras(self) -> dict:
        """record 脚本取 len(robot.cameras) 算图像写线程数;相机实体在相机节点侧。"""
        return self.config.cameras

    @cached_property
    def observation_features(self) -> dict:
        ft: dict = {f"{m}.pos": float for m in PIPER_MOTORS}
        for cam_name, spec in self.config.cameras.items():
            ch = 1 if spec.kind == "depth" else 3
            ft[cam_name] = (spec.height, spec.width, ch)
        return ft

    @cached_property
    def action_features(self) -> dict[str, type]:
        return {f"{m}.pos": float for m in PIPER_MOTORS}

    # ---------------- lifecycle ----------------
    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def is_calibrated(self) -> bool:
        return True  # 标定/使能在 piper_arm_node 侧完成

    def connect(self, calibrate: bool = True) -> None:
        self._bus = Ros2Bus.instance()
        self._bus.sub_joint_state(self.config.state_topic)
        for spec in self.config.cameras.values():
            self._bus.sub_image(spec.topic, spec.kind)
        deadline = time.monotonic() + 5.0
        while self._bus.latest_joint_state(self.config.state_topic) is None:
            if time.monotonic() > deadline:
                raise ConnectionError(
                    f"{self.config.state_topic} 5s 无消息 — piper_arm_node 在跑吗?"
                )
            time.sleep(0.05)
        self._connected = True
        logger.info(f"{self}: 已订阅 {self.config.state_topic} + {len(self.config.cameras)} 路图像。")

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    def disconnect(self) -> None:
        self._connected = False  # 总线是进程级单例;使能/下电是 arm 节点的职责

    # ---------------- observation ----------------
    def get_observation(self) -> dict:
        if not self._connected:
            raise ConnectionError(f"{self}: 未 connect")
        stale_s = self.config.stale_frame_ms / 1000.0
        js = self._bus.latest_joint_state(self.config.state_topic)
        if js is None or time.monotonic() - js[1] > stale_s:
            logger.warning(f"{self.config.state_topic} 超过 {self.config.stale_frame_ms}ms 无新帧!")
            data = {} if js is None else js[0]
        else:
            data = js[0]
        obs: dict = {f"{m}.pos": data.get(m, (0.0, 0.0, 0.0))[0] for m in PIPER_MOTORS}

        for cam_name, spec in self.config.cameras.items():
            got = self._bus.latest_image(spec.topic)
            if got is not None and time.monotonic() - got[1] <= stale_s:
                obs[cam_name] = got[0]
                self._last_img[cam_name] = got[0]
            else:
                fallback = self._last_img.get(cam_name)
                if got is None and fallback is None:
                    ch = 1 if spec.kind == "depth" else 3
                    dt = np.uint16 if spec.kind == "depth" else np.uint8
                    fallback = np.zeros((spec.height, spec.width, ch), dtype=dt)
                else:
                    logger.warning(f"{cam_name} 帧龄超限,回退上一帧。")
                obs[cam_name] = fallback
        return obs

    # ---------------- action ----------------
    def send_action(self, action: dict) -> dict:
        if self.config.own_cmd_topic:
            from sensor_msgs.msg import JointState

            msg = JointState()
            msg.name = list(PIPER_MOTORS)
            msg.position = [float(action[f"{m}.pos"]) for m in PIPER_MOTORS]
            self._bus.publish(JointState, self.config.cmd_topic, msg)
        return action
