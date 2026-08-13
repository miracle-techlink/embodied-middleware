#!/usr/bin/env python
"""rebot_arm_node — reBot B601-RS 从臂的 ROS2 驱动节点。

复用 lerobot 侧 ``RebotFollower``(cameras={}),所有精调过的硬件行为原样保留:
关节限位夹取、夹爪直驱(motor 7 位置 MIT)、退出前平滑回零,一行不搬。

话题(名称来自 JSON 注册表):
    发布 /rebot/follower/joint_state  JointState  position=度 velocity=度/秒 effort=力矩
    订阅 /rebot/follower/joint_cmd    JointState  7 关节绝对目标角(度)

并发:状态发布定时器与 cmd 订阅回调都会碰 CAN 总线,用一把锁串行化(lerobot 实现
本身非线程安全;锁竞争可忽略:send_action ~ms @100Hz,poll ~2ms @200Hz)。

参数:
    can_port   CAN 接口名(默认 can0,先跑 setup_rebot_can.sh)
    robot_id   标定文件 id(默认 follower1,与 lerobot 标定共用)
    state_fps  关节状态发布频率(默认 200Hz)
"""

from __future__ import annotations

import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

from rebot_msg_center.topic_registry import TopicRegistry

CMD_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT
)


class RebotArmNode(Node):
    def __init__(self):
        super().__init__("rebot_arm_node")
        self.declare_parameter("can_port", "can0")
        self.declare_parameter("robot_id", "follower1")
        self.declare_parameter("registry_json", "")
        self.declare_parameter("state_fps", 0.0)  # 0 = 用注册表 default_fps

        reg = TopicRegistry(self.get_parameter("registry_json").value or None)
        state_spec = reg.require("/rebot/follower/joint_state")
        cmd_spec = reg.require("/rebot/follower/joint_cmd")
        fps = float(self.get_parameter("state_fps").value) or state_spec.default_fps

        from lerobot.robots.rebot_follower import RebotFollower, RebotFollowerConfig

        cfg = RebotFollowerConfig(
            port=self.get_parameter("can_port").value,
            id=self.get_parameter("robot_id").value,
            cameras={},  # 相机由 orbbec/uvc 节点独立发布,这里只管教臂
        )
        self._robot = RebotFollower(cfg)
        self._bus_lock = threading.Lock()
        self.get_logger().info(
            f"连接 reBot 从臂 {cfg.port}(id={cfg.id}),回零/限位/夹爪直驱沿用 RebotFollower…"
        )
        with self._bus_lock:
            self._robot.connect(calibrate=True)  # 标定文件缺失时会走交互标定(与 lerobot 一致)

        self._state_pub = self.create_publisher(JointState, state_spec.topic_name, CMD_QOS)
        self.create_subscription(JointState, cmd_spec.topic_name, self._on_cmd, CMD_QOS)
        self.create_timer(1.0 / fps, self._publish_state)
        self.get_logger().info(
            f"发布 {state_spec.topic_name} @ {fps}Hz,订阅 {cmd_spec.topic_name}。"
        )

    # ---------------- cmd 下行 ----------------
    def _on_cmd(self, msg: JointState):
        action = {f"{name}.pos": float(pos) for name, pos in zip(msg.name, msg.position)}
        try:
            with self._bus_lock:
                self._robot.send_action(action)
        except Exception as e:
            self.get_logger().warning(f"send_action 失败: {e}", throttle_duration_sec=1.0)

    # ---------------- 状态上行 ----------------
    def _publish_state(self):
        try:
            with self._bus_lock:
                obs = self._robot.get_observation()  # cameras={} → 只有电机反馈(非阻塞路径)
        except Exception as e:
            self.get_logger().warning(f"get_observation 失败: {e}", throttle_duration_sec=1.0)
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        for key, val in obs.items():
            name, _, kind = key.rpartition(".")
            if name not in msg.name:
                msg.name.append(name)
                msg.position.append(0.0)
                msg.velocity.append(0.0)
                msg.effort.append(0.0)
            i = msg.name.index(name)
            if kind == "pos":
                msg.position[i] = float(val)
            elif kind == "vel":
                msg.velocity[i] = float(val)
            elif kind == "torque":
                msg.effort[i] = float(val)
        self._state_pub.publish(msg)

    def destroy_node(self):
        # disconnect 内部:先平滑回零(坐姿)再卸力矩 —— 节点被杀时机械臂不砸落。
        try:
            with self._bus_lock:
                if self._robot.is_connected:
                    self._robot.disconnect()
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
