#!/usr/bin/env python
"""rebot_doctor 的频率探针:QoS 匹配(BEST_EFFORT)地测一个 topic 的实际 Hz。

用法: rebot_rate.py <topic> [窗口秒=2]
先等首帧(最多 3s,等 DDS 发现),再在窗口内计数,打印 Hz(保留 1 位)。
收不到首帧打印 0 并退 1 —— 与"发布者死/断流"同判。"""

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, JointState

topic = sys.argv[1]
dur = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0

rclpy.init()
node = Node("rebot_doctor_rate")
# middleware 全部 BEST_EFFORT keep-last 1,订阅必须匹配(RELIABLE 会被判不兼容静默收不到)
qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

# joint_cmd/joint_state/leader 都是 JointState;图像 topic 是 CompressedImage —— 按名字猜不中就两种都试
cnt = [0]


def _make_sub(msg_type):
    return node.create_subscription(msg_type, topic, lambda m: cnt.__setitem__(0, cnt[0] + 1), qos)


sub = _make_sub(JointState)
t0 = time.monotonic()
while cnt[0] == 0 and time.monotonic() - t0 < 3.0:
    rclpy.spin_once(node, timeout_sec=0.1)
if cnt[0] == 0:
    sub.destroy()
    sub = _make_sub(CompressedImage)  # 可能是图像 topic
    t0 = time.monotonic()
    while cnt[0] == 0 and time.monotonic() - t0 < 1.5:
        rclpy.spin_once(node, timeout_sec=0.1)

if cnt[0] == 0:
    print("0")
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(1)

cnt[0] = 0  # 首帧只当发现信号,重开窗口
t0 = time.monotonic()
while time.monotonic() - t0 < dur:
    rclpy.spin_once(node, timeout_sec=0.1)
print(f"{cnt[0] / dur:.1f}")
node.destroy_node()
rclpy.shutdown()
