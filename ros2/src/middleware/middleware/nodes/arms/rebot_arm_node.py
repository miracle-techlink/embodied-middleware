#!/usr/bin/env python
"""rebot_arm_node — reBot B601-RS 从臂的 ROS2 驱动节点(纯 ROS 层)。

硬件交互全部在 backends/rebot/rebot_arm_backend.py(薄壳复用 lerobot RebotFollower,
与 PiperBackendBase 同协议);本节点只剩话题发布/订阅,SIGTERM 安全语义不变。

话题(名称来自 JSON 注册表):
    发布 /rebot/follower/joint_state  JointState  position=度 velocity=度/秒 effort=力矩
    订阅 /rebot/follower/joint_cmd    JointState  7 关节绝对目标角(度)

参数:
    can_port   CAN 接口名(默认 can0,先跑 setup_rebot_can.sh)
    robot_id   标定文件 id(默认 follower1,与 lerobot 标定共用)
    state_fps  关节状态发布频率(默认 200Hz,0=用注册表 default_fps)
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

from middleware.backends.rebot.rebot_arm_backend import REBOT_JOINT_NAMES, RebotArmBackend
from middleware.core.topic_registry import TopicRegistry

CMD_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT
)


class RebotArmNode(Node):
    def __init__(self):
        super().__init__("rebot_arm_node")
        self.declare_parameter("can_port", "can0")
        self.declare_parameter("robot_id", "follower1")
        self.declare_parameter("registry_json", "")
        self.declare_parameter("state_fps", 0.0)

        reg = TopicRegistry(self.get_parameter("registry_json").value or None)
        state_spec = reg.require("/rebot/follower/joint_state")
        cmd_spec = reg.require("/rebot/follower/joint_cmd")
        fps = float(self.get_parameter("state_fps").value) or state_spec.default_fps

        self._backend = RebotArmBackend(
            can_port=self.get_parameter("can_port").value,
            robot_id=self.get_parameter("robot_id").value,
        )
        self.get_logger().info(
            f"连接 reBot 从臂 {self._backend._can_port}(回零/限位/夹爪直驱沿用 RebotFollower)…"
        )
        self._backend.connect()

        self._state_pub = self.create_publisher(JointState, state_spec.topic_name, CMD_QOS)
        self.create_subscription(JointState, cmd_spec.topic_name, self._on_cmd, CMD_QOS)
        self.create_timer(1.0 / fps, self._publish_state)
        self.get_logger().info(
            f"发布 {state_spec.topic_name} @ {fps}Hz,订阅 {cmd_spec.topic_name}。"
        )

    # ---------------- cmd 下行 ----------------
    def _on_cmd(self, msg: JointState):
        by_name = {name: float(pos) for name, pos in zip(msg.name, msg.position)}
        try:
            joints = [by_name[n] for n in REBOT_JOINT_NAMES]
        except KeyError:
            self.get_logger().warning(
                f"cmd 缺关节(需要 {REBOT_JOINT_NAMES})", throttle_duration_sec=1.0
            )
            return
        try:
            self._backend.send_cmd(joints)
        except Exception as e:
            self.get_logger().warning(f"send_action 失败: {e}", throttle_duration_sec=1.0)

    # ---------------- 状态上行 ----------------
    def _publish_state(self):
        try:
            pos, vel, eff = self._backend.get_state()
        except Exception as e:
            self.get_logger().warning(f"get_observation 失败: {e}", throttle_duration_sec=1.0)
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(REBOT_JOINT_NAMES)
        msg.position = pos
        msg.velocity = vel
        msg.effort = eff
        self._state_pub.publish(msg)

    def destroy_node(self):
        # disconnect 内部:先平滑回零(坐姿)再卸力矩 —— 节点被杀时机械臂不砸落。
        try:
            self._backend.disconnect()
        except Exception as e:
            self.get_logger().warning(f"disconnect 异常: {e}")
        super().destroy_node()


def main():
    import logging
    import signal

    logging.basicConfig(level=logging.INFO)  # 让 RebotFollower 的回零/告警日志可见
    rclpy.init()
    node = RebotArmNode()
    # SIGTERM(kill/timeout/systemd stop)也要走 finally → disconnect(平滑回零+卸力矩),
    # 否则进程被杀 = 机械臂直接失力矩砸落。rclpy 自己只处理了 SIGINT。
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
