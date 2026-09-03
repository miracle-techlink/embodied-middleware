#!/usr/bin/env python
"""PiPER X ROS2 producer/consumer with real-SDK or MuJoCo backend.

Default backend is ``mujoco`` so software can be integrated before hardware arrives.
Set ``backend:=sdk`` only after the PiPER CAN interface and SDK API have been verified.

Topics are selected from the registry JSON:
    publish /piper/joint_state      sensor_msgs/JointState @ state_fps
    subscribe /piper/joint_cmd      sensor_msgs/JointState @ command input
"""

from __future__ import annotations

import time
import signal

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

from middleware.backends.piper.piper_arm_backend import PiperMujocoBackend, PiperSdkBackend
from middleware.core.topic_registry import TopicRegistry

QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
)
JOINT_NAMES = [f"joint_{i}" for i in range(1, 7)] + ["gripper"]


class PiperArmNode(Node):
    def __init__(self):
        super().__init__("piper_arm_node")
        self.declare_parameter("backend", "mujoco")
        self.declare_parameter("can_name", "can_piper")
        self.declare_parameter("registry_json", "")
        self.declare_parameter("state_fps", 0.0)
        self.declare_parameter("enable_cmd", False)
        self.declare_parameter("command_timeout_ms", 200.0)

        reg = TopicRegistry(self.get_parameter("registry_json").value or None)
        state_spec = reg.require("/piper/joint_state")
        cmd_spec = reg.require("/piper/joint_cmd")
        fps = float(self.get_parameter("state_fps").value) or state_spec.default_fps
        backend_name = str(self.get_parameter("backend").value).lower()
        if backend_name == "mujoco":
            self._backend = PiperMujocoBackend()
        elif backend_name == "sdk":
            self._backend = PiperSdkBackend(str(self.get_parameter("can_name").value))
        else:
            raise ValueError(f"未知 backend={backend_name!r}, 只能是 mujoco 或 sdk")

        self._enable_cmd = bool(self.get_parameter("enable_cmd").value)
        self._command_timeout_s = float(self.get_parameter("command_timeout_ms").value) / 1000.0
        self._last_cmd = 0.0
        self._backend.connect()
        if self._enable_cmd:
            self._backend.enable()
        self._state_pub = self.create_publisher(JointState, state_spec.topic_name, QOS)
        self.create_subscription(JointState, cmd_spec.topic_name, self._on_cmd, QOS)
        self.create_timer(1.0 / fps, self._publish_state)
        self.get_logger().info(
            f"PiPER backend={backend_name}, publish={state_spec.topic_name}@{fps:g}Hz, "
            f"subscribe={cmd_spec.topic_name}, enable_cmd={self._enable_cmd}"
        )

    def _on_cmd(self, msg: JointState):
        if not self._enable_cmd:
            return
        values = {name: float(pos) for name, pos in zip(msg.name, msg.position)}
        try:
            joints = [values[f"joint_{i}"] for i in range(1, 7)]
            gripper = values.get("gripper", 0.0)
        except KeyError:
            self.get_logger().warning("PiPER cmd 缺少 joint_1..joint_6", throttle_duration_sec=1.0)
            return
        self._backend.send_cmd(joints, gripper)
        self._last_cmd = time.monotonic()

    def _publish_state(self):
        if self._enable_cmd and self._last_cmd and time.monotonic() - self._last_cmd > self._command_timeout_s:
            self._backend.estop()
            self._last_cmd = 0.0
            self.get_logger().warning("PiPER cmd 超时，已停止后端目标")
        joints, gripper, enabled = self._backend.get_state()
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        msg.position = [float(x) for x in joints] + [float(gripper)]
        msg.velocity = [0.0] * 7
        msg.effort = [0.0] * 7
        msg.header.frame_id = "piper_enabled" if enabled else "piper_disabled"
        self._state_pub.publish(msg)

    def destroy_node(self):
        try:
            self._backend.estop()
            self._backend.disable()
            self._backend.disconnect()
        finally:
            super().destroy_node()


def main():
    rclpy.init()
    node = PiperArmNode()
    signal.signal(signal.SIGTERM, lambda *_: rclpy.shutdown())
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
