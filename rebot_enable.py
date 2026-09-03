#!/usr/bin/env python
"""发一次 /rebot/teleop/enable(true/false)。用法:rebot_enable.py true|false

场景:闸门录制退出后 teleop 处于冻结(enable=false),普通录制/手动跟手前要恢复;
或反过来想临时冻住从臂。连发两次(间隔 0.3s)确保 RELIABLE 到达。
注意:map 节点只在 False→True 转换时 re-arm 限速 ramp —— 恢复跟手天然不弹射。"""

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


def main() -> None:
    if len(sys.argv) != 2 or sys.argv[1] not in ("true", "false"):
        sys.exit("用法: rebot_enable.py true|false")
    on = sys.argv[1] == "true"

    rclpy.init()
    node = Node("rebot_enable_cli")
    pub = node.create_publisher(
        Bool, "/rebot/teleop/enable",
        QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                   reliability=ReliabilityPolicy.RELIABLE),
    )
    msg = Bool()
    msg.data = on
    for _ in range(2):
        pub.publish(msg)
        time.sleep(0.3)  # 给 DDS 发现+传输留时间
    node.destroy_node()
    rclpy.shutdown()
    print(f"已发 /rebot/teleop/enable={on}"
          + ("(map 从保持位限速 ramp,不弹射)" if on else "(从臂保持当前目标,不跟手)"))


if __name__ == "__main__":
    main()
