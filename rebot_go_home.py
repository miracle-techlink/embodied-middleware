#!/usr/bin/env python
"""发一次 /rebot/teleop/go_home(Empty):map 节点把 leader 视作零,从臂限速滑回 home。

用法:rebot_go_home.py [--wait]
  不带 --wait 发完即走;--wait 会盯着 /rebot/follower/joint_state 直到 6 臂关节
  距 home ≤2°(或 30s 超时退 1)。需要 map 节点 ≥2026-08-20(带 go_home 订阅)。"""

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty

from lerobot.teleoperators.starai_to_rebot_leader.config_starai_to_rebot_leader import (
    StaraiToRebotLeaderConfig,
)
from lerobot.teleoperators.starai_to_rebot_leader.mapping import REBOT_ARM_MOTORS

HOME6 = dict(zip(REBOT_ARM_MOTORS, StaraiToRebotLeaderConfig().rebot_home_deg))

BEST_EFFORT_QOS = QoSProfile(  # 与消息中心 topic QoS 一致
    history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT
)
RELIABLE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE
)


def main() -> None:
    wait = "--wait" in sys.argv[1:]
    rclpy.init()
    node = Node("rebot_go_home_cli")
    pub = node.create_publisher(Empty, "/rebot/teleop/go_home", RELIABLE_QOS)
    state: dict[str, tuple[float, float]] = {}

    def _on_js(msg: JointState) -> None:
        for n, p in zip(msg.name, msg.position):
            state[n] = (float(p), time.monotonic())

    if wait:
        node.create_subscription(JointState, "/rebot/follower/joint_state", _on_js, BEST_EFFORT_QOS)

    for _ in range(2):
        pub.publish(Empty())
        time.sleep(0.3)
    print("已发 /rebot/teleop/go_home(从臂限速滑回 home)。")

    if wait:
        print("等待到位(6 关节 ≤2°,最多 30s)...")
        t0 = time.monotonic()
        ok = False
        while time.monotonic() - t0 < 30.0:
            rclpy.spin_once(node, timeout_sec=0.1)
            errs = [
                abs(state[n][0] - h)
                for n, h in HOME6.items()
                if n in state and time.monotonic() - state[n][1] < 0.5
            ]
            if len(errs) == len(HOME6) and max(errs) <= 2.0:
                ok = True
                break
        print("✓ 已到 home" if ok else "⚠️  30s 未到 home(臂断流/卡住?rebot_doctor.sh 查)")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0 if ok else 1)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
