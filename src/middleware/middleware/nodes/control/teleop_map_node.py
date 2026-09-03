#!/usr/bin/env python
"""teleop_map_node — leader joint_state → follower joint_cmd 的映射节点。

映射/启动 ramp/夹爪换算的数学**不是这里的实现**,全部来自 lerobot 插件
``lerobot.teleoperators.starai_to_rebot_leader.mapping``(两条栈共用,改映射只改那边);
参数默认值也直接取 ``StaraiToRebotLeaderConfig`` 的数据类默认值(单源,不再手工对齐)。
这里只剩 topic IO:leader 读数从 topic 来、目标角发到 topic 去。控制回路由此节点自持
频率(默认 100Hz),与录制循环彻底解耦 —— record 卡了机械臂照样跟手。

映射(absolute 模式,与现栈一致):
    reBot[j] = rebot_home_deg[j] + sign[j] * scale * (leader[j] - 0)

话题:
    订阅 /rebot/leader/joint_state   JointState(joint_1..6 度 + gripper [0,1])
    发布 /rebot/follower/joint_cmd   JointState(7 关节绝对目标角,度)
    订阅 /rebot/teleop/enable        Bool  False=冻结(停发,从臂保持最后目标)
    订阅 /rebot/teleop/rearm         Empty 重新武装启动 ramp(闸门录制每条开录前发;
                                      同时退出 go_home 模式)
    订阅 /rebot/teleop/go_home       Empty 自主回零:leader 视作全零 → 目标=home,
                                      复用启动 ramp 限速滑过去(闸门录制每条录完发,
                                      等价直连版 safe_zero);enable 恢复/rearm 退出该模式
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty

from lerobot.teleoperators.starai_to_rebot_leader.config_starai_to_rebot_leader import (
    StaraiToRebotLeaderConfig,
)
from lerobot.teleoperators.starai_to_rebot_leader.mapping import REBOT_ARM_MOTORS, LeaderToRebotMap
from middleware.core.topic_registry import TopicRegistry

QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT
)
LATCH_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE
)

_CFG_DEFAULTS = StaraiToRebotLeaderConfig()  # 参数默认值单源:与 lerobot teleop 永远一致


class TeleopMapNode(Node):
    def __init__(self):
        super().__init__("teleop_map_node")
        self.declare_parameter("registry_json", "")
        self.declare_parameter("cmd_fps", 0.0)
        # 默认值来自 StaraiToRebotLeaderConfig(单源);这里声明只是为了 launch 时可覆盖。
        self.declare_parameter("rebot_home_deg", list(_CFG_DEFAULTS.rebot_home_deg))
        self.declare_parameter("flip", _CFG_DEFAULTS.flip)
        self.declare_parameter("scale", _CFG_DEFAULTS.scale)
        self.declare_parameter("absolute", _CFG_DEFAULTS.absolute)
        self.declare_parameter("startup_ramp_deg_per_step", _CFG_DEFAULTS.startup_ramp_deg_per_step)
        self.declare_parameter("grip_close_deg", _CFG_DEFAULTS.grip_close_deg)
        self.declare_parameter("grip_open_deg", _CFG_DEFAULTS.grip_open_deg)
        self.declare_parameter("grip_clamp_deg", _CFG_DEFAULTS.grip_clamp_deg)
        self.declare_parameter("grip_ratio_min", _CFG_DEFAULTS.grip_ratio_min)
        self.declare_parameter("grip_ratio_max", _CFG_DEFAULTS.grip_ratio_max)
        self.declare_parameter("enabled", True)
        # go_home 回零限速(度/步 @100Hz;默认 0.5 ≈ 50°/s,比默认 ramp 慢一个量级,回零不甩)
        self.declare_parameter("go_home_ramp_deg_per_step", 0.5)

        reg = TopicRegistry(self.get_parameter("registry_json").value or None)
        leader_spec = reg.require("/rebot/leader/joint_state")
        cmd_spec = reg.require("/rebot/follower/joint_cmd")
        fps = float(self.get_parameter("cmd_fps").value) or cmd_spec.default_fps

        p = lambda n: self.get_parameter(n).value
        self._map = LeaderToRebotMap(
            rebot_home_deg=[float(v) for v in p("rebot_home_deg")],
            flip=str(p("flip")),
            scale=float(p("scale")),
            absolute=bool(p("absolute")),
            startup_ramp_deg_per_step=float(p("startup_ramp_deg_per_step")),
            grip_close_deg=float(p("grip_close_deg")),
            grip_open_deg=float(p("grip_open_deg")),
            grip_clamp_deg=float(p("grip_clamp_deg")),
            grip_ratio_min=float(p("grip_ratio_min")),
            grip_ratio_max=float(p("grip_ratio_max")),
        )

        self._enabled = bool(p("enabled"))
        self._leader: dict[str, float] | None = None  # 最新 leader 读数(name → 值)
        self._go_home = False  # go_home 模式:目标=home(leader 视作零),rearm/enable 恢复时退出

        self.create_subscription(JointState, leader_spec.topic_name, self._on_leader, QOS)
        self.create_subscription(Bool, "/rebot/teleop/enable", self._on_enable, LATCH_QOS)
        self.create_subscription(Empty, "/rebot/teleop/rearm", self._on_rearm, LATCH_QOS)
        self.create_subscription(Empty, "/rebot/teleop/go_home", self._on_go_home, LATCH_QOS)
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
            self._map.rearm_ramp()  # 冻结期间主臂可能被挪动,恢复时重新限速
            self._go_home = False
        self._enabled = bool(msg.data)
        self.get_logger().info(f"teleop {'恢复(重新 ramp)' if self._enabled else '冻结'}。")

    def _on_rearm(self, _msg: Empty):
        self._go_home = False
        self._map.rearm_ramp()
        self.get_logger().info("启动 ramp 已重新武装(退出 go_home 模式)。")

    def _on_go_home(self, _msg: Empty):
        # 自主回零:leader 视作全零 → 目标=home;用专用慢速 ramp 从当前输出滑过去
        # (默认 ramp 6°/步@100Hz≈600°/s,回零太快会甩)。不看 _enabled —— 录制会话
        # 起手是冻结态,录完照样要能回零。收敛后 ramp 自动恢复默认并直通保持 home。
        self._go_home = True
        self._map.rearm_ramp(
            step_deg_per_step=float(self.get_parameter("go_home_ramp_deg_per_step").value)
        )
        self.get_logger().info("go_home:限速滑回 home(rearm/enable 恢复退出)。")

    # ---------------- 映射 + 发布 ----------------
    def _publish_cmd(self):
        if self._go_home:
            leader, grip = [0.0] * 6, 0.0  # leader=零 → 目标=home;夹爪张开
        else:
            if not self._enabled or self._leader is None:
                return  # 冻结/主臂未上线:停发,从臂保持最后 MIT 目标
            la = self._leader
            try:
                leader = [la[f"joint_{i + 1}"] for i in range(6)]
            except KeyError:
                return  # 主臂消息缺关节,等下一帧
            grip = float(la.get("gripper", 0.0))

        out7 = self._map.update(leader, grip)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = REBOT_ARM_MOTORS + ["gripper"]
        msg.position = [float(v) for v in out7]
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
