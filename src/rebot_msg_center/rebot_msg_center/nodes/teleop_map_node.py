#!/usr/bin/env python
"""teleop_map_node — leader joint_state → follower joint_cmd 的映射节点。

纯数学移植自 lerobot 插件 ``starai_to_rebot_leader``(映射/启动 ramp/夹爪换算),
差别只在数据源:leader 读数从 topic 来、目标角发到 topic 去。控制回路由此节点自持
频率(默认 100Hz),与录制循环彻底解耦 —— record 卡了机械臂照样跟手。

映射(absolute 模式,与现栈一致):
    reBot[j] = rebot_home_deg[j] + sign[j] * scale * (leader[j] - 0)

话题:
    订阅 /rebot/leader/joint_state   JointState(joint_1..6 度 + gripper [0,1])
    发布 /rebot/follower/joint_cmd   JointState(7 关节绝对目标角,度)
    订阅 /rebot/teleop/enable        Bool  False=冻结(停发,从臂保持最后目标)
    订阅 /rebot/teleop/rearm         Empty 重新武装启动 ramp(闸门录制每条开录前发)
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty

from rebot_msg_center.topic_registry import TopicRegistry

QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT
)
LATCH_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE
)

REBOT_ARM_MOTORS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_yaw", "wrist_roll"]


def _parse_flip(s: str) -> list[float]:
    sign = [1.0] * 6
    for tok in s.split(","):
        tok = tok.strip()
        if tok:
            sign[int(tok) - 1] = -1.0
    return sign


class TeleopMapNode(Node):
    def __init__(self):
        super().__init__("teleop_map_node")
        self.declare_parameter("registry_json", "")
        self.declare_parameter("cmd_fps", 0.0)
        # 以下默认值与 StaraiToRebotLeaderConfig 逐一对齐(改参时两边一起改)。
        self.declare_parameter("rebot_home_deg", [0.0, 0.0, 0.0, -1.0, 1.5, 2.0])
        self.declare_parameter("flip", "3,4,5")
        self.declare_parameter("scale", 1.0)
        self.declare_parameter("startup_ramp_deg_per_step", 6.0)
        self.declare_parameter("grip_close_deg", 20.0)
        self.declare_parameter("grip_open_deg", 250.0)
        self.declare_parameter("grip_clamp_deg", 25.0)
        self.declare_parameter("grip_ratio_min", 0.05)
        self.declare_parameter("grip_ratio_max", 0.95)
        self.declare_parameter("enabled", True)

        reg = TopicRegistry(self.get_parameter("registry_json").value or None)
        leader_spec = reg.require("/rebot/leader/joint_state")
        cmd_spec = reg.require("/rebot/follower/joint_cmd")
        fps = float(self.get_parameter("cmd_fps").value) or cmd_spec.default_fps

        p = lambda n: self.get_parameter(n).value
        self._home = [float(v) for v in p("rebot_home_deg")]
        self._sign = _parse_flip(p("flip"))
        self._scale = float(p("scale"))
        self._ramp_step = float(p("startup_ramp_deg_per_step"))
        close, open_, clamp = float(p("grip_close_deg")), float(p("grip_open_deg")), float(p("grip_clamp_deg"))
        close_dir = -1.0 if close <= open_ else 1.0
        self._grip_close_eff = close + close_dir * clamp
        self._grip_open = open_
        self._grip_ratio_min = float(p("grip_ratio_min"))
        self._grip_ratio_max = float(p("grip_ratio_max"))

        self._enabled = bool(p("enabled"))
        self._leader: dict[str, float] | None = None  # 最新 leader 读数(name → 值)
        self._cmd_arm: list[float] | None = None      # 启动 ramp 用:当前臂输出目标
        self._ramped_in = False

        self.create_subscription(JointState, leader_spec.topic_name, self._on_leader, QOS)
        self.create_subscription(Bool, "/rebot/teleop/enable", self._on_enable, LATCH_QOS)
        self.create_subscription(Empty, "/rebot/teleop/rearm", self._on_rearm, LATCH_QOS)
        self._cmd_pub = self.create_publisher(JointState, cmd_spec.topic_name, QOS)
        self.create_timer(1.0 / fps, self._publish_cmd)
        self.get_logger().info(
            f"{leader_spec.topic_name} → {cmd_spec.topic_name} @ {fps}Hz,"
            f"enable={'ON' if self._enabled else 'OFF'}(/rebot/teleop/enable 可切)。"
        )

    # ---------------- 输入 ----------------
    def _on_leader(self, msg: JointState):
        self._leader = {name: float(pos) for name, pos in zip(msg.name, msg.position)}

    def _on_enable(self, msg: Bool):
        if msg.data and not self._enabled:
            self._ramped_in = False  # 冻结期间主臂可能被挪动,恢复时重新限速(等价 rearm_ramp)
        self._enabled = bool(msg.data)
        self.get_logger().info(f"teleop {'恢复(重新 ramp)' if self._enabled else '冻结'}。")

    def _on_rearm(self, _msg: Empty):
        self._ramped_in = False
        self.get_logger().info("启动 ramp 已重新武装。")

    # ---------------- 映射 + 发布 ----------------
    def _publish_cmd(self):
        if not self._enabled or self._leader is None:
            return  # 冻结/主臂未上线:停发,从臂保持最后 MIT 目标
        la = self._leader
        try:
            leader = [la[f"joint_{i + 1}"] for i in range(6)]
        except KeyError:
            return  # 主臂消息缺关节,等下一帧

        # absolute 模式:leader 标定零位(0)恒对应 rebot_home
        target = [
            self._home[i] + self._sign[i] * self._scale * leader[i]
            for i in range(6)
        ]

        # 启动 ramp:从 home/保持位限速滑向目标(纯输出插值,不夹传感器 → 不抖),收敛后直通
        if self._cmd_arm is None:
            self._cmd_arm = list(self._home)
        if not self._ramped_in and self._ramp_step > 0.0:
            residual = 0.0
            for i in range(6):
                d = max(-self._ramp_step, min(self._ramp_step, target[i] - self._cmd_arm[i]))
                self._cmd_arm[i] += d
                residual = max(residual, abs(target[i] - self._cmd_arm[i]))
            arm = list(self._cmd_arm)
            if residual < 0.5:
                self._ramped_in = True
        else:
            arm = target
            self._cmd_arm = list(target)

        # 夹爪:leader ratio → [close_eff, open]
        raw = float(la.get("gripper", 0.0))
        denom = max(self._grip_ratio_max - self._grip_ratio_min, 1e-3)
        ratio = min(1.0, max(0.0, (raw - self._grip_ratio_min) / denom))
        grip = self._grip_close_eff + ratio * (self._grip_open - self._grip_close_eff)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = REBOT_ARM_MOTORS + ["gripper"]
        msg.position = [float(v) for v in arm] + [grip]
        self._cmd_pub.publish(msg)


def main():
    import signal

    rclpy.init()
    node = TeleopMapNode()
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
