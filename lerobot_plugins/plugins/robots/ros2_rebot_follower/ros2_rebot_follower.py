#!/usr/bin/env python
"""`ros2_rebot_follower` — 观测来自 rebot_msg_center(ROS2)的 reBot 从臂。

硬件由 rebot_arm_node / orbbec_node / uvc_node 持有,本类只做订阅与格式还原,
录进数据集的键与 ``rebot_follower`` 逐一对齐:

    observation.state   ← /rebot/follower/joint_state 的 7 关节 .pos
    observation.images.wrist / front / wrist_depth  ← 压缩图 topic 解码
    action(send_action) ← own_cmd_topic=True 时才发到 /rebot/follower/joint_cmd

前置:消息中心各节点已在跑,且本进程已 source ROS 环境(见 ros2_bus 报错提示)。
"""

import logging
import time
from functools import cached_property

import numpy as np

from lerobot.robots.robot import Robot

from .config_ros2_rebot_follower import REBOT_MOTORS, Ros2RebotFollowerConfig
from .ros2_bus import Ros2Bus

logger = logging.getLogger(__name__)


class Ros2RebotFollower(Robot):
    config_class = Ros2RebotFollowerConfig
    name = "ros2_rebot_follower"

    def __init__(self, config: Ros2RebotFollowerConfig):
        super().__init__(config)
        self.config = config
        self._bus: Ros2Bus | None = None
        self._connected = False
        self._last_img: dict[str, np.ndarray] = {}

    # ---------------- features(与 rebot_follower 对齐) ----------------
    @property
    def cameras(self) -> dict:
        """record 脚本取 len(robot.cameras) 算图像写线程数;相机实体在相机节点侧。"""
        return self.config.cameras

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        ft: dict[str, type | tuple] = {f"{m}.pos": float for m in REBOT_MOTORS}
        for cam_name, spec in self.config.cameras.items():
            ch = 1 if spec.kind == "depth" else 3
            ft[cam_name] = (spec.height, spec.width, ch)
        return ft

    @cached_property
    def action_features(self) -> dict[str, type]:
        return {f"{m}.pos": float for m in REBOT_MOTORS}

    # ---------------- lifecycle ----------------
    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def is_calibrated(self) -> bool:
        return True  # 标定在 arm 节点侧完成

    def connect(self, calibrate: bool = True) -> None:
        self._bus = Ros2Bus.instance()
        self._bus.sub_joint_state(self.config.state_topic)
        for spec in self.config.cameras.values():
            self._bus.sub_image(spec.topic, spec.kind)
        # 等首帧关节状态(相机慢,不等)
        deadline = time.monotonic() + 5.0
        while self._bus.latest_joint_state(self.config.state_topic) is None:
            if time.monotonic() > deadline:
                raise ConnectionError(
                    f"{self.config.state_topic} 5s 无消息 — rebot_arm_node 在跑吗?"
                )
            time.sleep(0.05)
        self._connected = True
        logger.info(f"{self}: 已订阅 {self.config.state_topic} + {len(self.config.cameras)} 路图像。")

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    def disconnect(self) -> None:
        self._connected = False  # 总线是进程级单例,留给别的消费者;不卸力矩(arm 节点的职责)

    # ---------------- observation ----------------
    def get_observation(self) -> dict:
        if not self._connected:
            raise ConnectionError(f"{self}: 未 connect")
        obs: dict = {}
        stale_s = self.config.stale_frame_ms / 1000.0

        js = self._bus.latest_joint_state(self.config.state_topic)
        if js is None or time.monotonic() - js[1] > stale_s:
            logger.warning(f"{self.config.state_topic} 超过 {self.config.stale_frame_ms}ms 无新帧!")
            data = {} if js is None else js[0]
        else:
            data = js[0]
        for m in REBOT_MOTORS:
            pos, _vel, _eff = data.get(m, (0.0, 0.0, 0.0))
            obs[f"{m}.pos"] = pos

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
            msg.name = list(REBOT_MOTORS)
            msg.position = [float(action[f"{m}.pos"]) for m in REBOT_MOTORS]
            self._bus.publish(JointState, self.config.cmd_topic, msg)
        # own_cmd_topic=False:teleop_map_node 自持控制回路,这里只回显(record 录的是实际 cmd)
        return action
