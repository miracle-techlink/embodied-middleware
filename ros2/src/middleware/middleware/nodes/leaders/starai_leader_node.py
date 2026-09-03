#!/usr/bin/env python
"""starai_leader_node — StarAI Violin 主臂的 ROS2 驱动节点。

复用 lerobot 侧 ``StaraiViolinLeader``(含标定文件加载、4ms 单发防总线抖动策略),
``get_action()`` 的输出原样转成 JointState 发布:

    发布 /rebot/leader/joint_state  JointState
        name     = joint_1..joint_6 + gripper
        position = 度(相对标定零位)          + [0,1] 开度比

参数:
    port      串口(默认 /dev/ttyCH341USB0;主臂在 ttyUSB0 有 udev 软链时可换)
    baudrate  默认 1_000_000
    leader_id 标定文件 id(默认 leader1)
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

from middleware.core.topic_registry import TopicRegistry

PUB_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT
)


class StaraiLeaderNode(Node):
    def __init__(self):
        super().__init__("starai_leader_node")
        self.declare_parameter("port", "/dev/ttyCH341USB0")
        self.declare_parameter("baudrate", 1_000_000)
        self.declare_parameter("leader_id", "leader1")
        self.declare_parameter("registry_json", "")
        self.declare_parameter("state_fps", 0.0)

        reg = TopicRegistry(self.get_parameter("registry_json").value or None)
        spec = reg.require("/rebot/leader/joint_state")
        fps = float(self.get_parameter("state_fps").value) or spec.default_fps

        from lerobot.teleoperators.starai_violin_leader import (
            StaraiViolinLeader,
            StaraiViolinLeaderConfig,
        )

        self._leader = StaraiViolinLeader(
            StaraiViolinLeaderConfig(
                port=self.get_parameter("port").value,
                baudrate=int(self.get_parameter("baudrate").value),
                id=self.get_parameter("leader_id").value,
            )
        )
        self.get_logger().info(
            f"连接 StarAI 主臂 {self._leader.config.port}(id={self._leader.config.id})…"
        )
        self._leader.connect(calibrate=True)

        self._pub = self.create_publisher(JointState, spec.topic_name, PUB_QOS)
        self.create_timer(1.0 / fps, self._publish_state)
        self.get_logger().info(f"发布 {spec.topic_name} @ {fps}Hz。")

    def _publish_state(self):
        try:
            action = self._leader.get_action()  # joint_1..6.pos(度) + gripper.pos([0,1])
        except Exception as e:
            self.get_logger().warning(f"leader 读数失败: {e}", throttle_duration_sec=1.0)
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        for key, val in action.items():
            name, _, kind = key.rpartition(".")
            if kind != "pos":
                continue
            msg.name.append(name)
            msg.position.append(float(val))
        self._pub.publish(msg)

    def destroy_node(self):
        try:
            if self._leader.is_connected:
                self._leader.disconnect()
        except Exception as e:
            self.get_logger().warning(f"disconnect 异常: {e}")
        super().destroy_node()


def main():
    import signal

    rclpy.init()
    node = StaraiLeaderNode()
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
