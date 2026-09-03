#!/usr/bin/env python
"""msg_center_bench — 控制通路的延迟/抖动闸门测试。

不碰硬件:一个发布者按目标频率发 JointState(publish 时刻写进 header.stamp),
订阅者统计:单跳延迟(订阅回调时刻 - stamp)、回调周期抖动、丢帧数。

跑法(两个终端,或 --both 单进程内):
    ros2 run middleware msg_center_bench --ros-args -p role:=pub
    ros2 run middleware msg_center_bench --ros-args -p role:=sub
    ros2 run middleware msg_center_bench --ros-args -p role:=both -p hz:=100.0 -p seconds:=10

判定标准(对照现有直连 30Hz/13ms 循环):
    单跳延迟 p99 < 2ms、回调周期 p99 抖动 < 5ms → 控制走 topic 放行。
"""

from __future__ import annotations

import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

NAMES = [f"joint_{i}" for i in range(7)]


def _qos() -> QoSProfile:
    # 控制通路语义:宁可丢旧帧也不要排队积压(类似相机 keep-last)。
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
    )


class BenchNode(Node):
    def __init__(self):
        super().__init__("msg_center_bench")
        self.declare_parameter("role", "both")      # pub | sub | both
        self.declare_parameter("hz", 100.0)
        self.declare_parameter("seconds", 10.0)
        self.declare_parameter("topic", "/rebot/bench/joint_cmd")

        role = self.get_parameter("role").value
        hz = float(self.get_parameter("hz").value)
        self.seconds = float(self.get_parameter("seconds").value)
        topic = self.get_parameter("topic").value

        self.lat_ms: list[float] = []
        self.period_ms: list[float] = []
        self._last_cb_ns: int | None = None
        self._count = 0

        if role in ("sub", "both"):
            self.create_subscription(JointState, topic, self._on_msg, _qos())
        if role in ("pub", "both"):
            self._pub = self.create_publisher(JointState, topic, _qos())
            self.create_timer(1.0 / hz, self._on_timer)
            self.get_logger().info(f"发布 {topic} @ {hz}Hz,{self.seconds}s 后出报告…")
        else:
            self.get_logger().info(f"订阅 {topic},收 {self.seconds}s 后出报告…")

        self.create_timer(self.seconds, self._report)

    def _on_timer(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = NAMES
        msg.position = [float(self._count)] * 7
        self._pub.publish(msg)
        self._count += 1

    def _on_msg(self, msg: JointState):
        now_ns = time.monotonic_ns()
        stamp = msg.header.stamp
        pub_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        # stamp 是节点时钟(系统时间),与 monotonic 不同基准;本机同钟直接用系统钟再取一次。
        now_sys_ns = time.time_ns()
        self.lat_ms.append((now_sys_ns - pub_ns) / 1e6)
        if self._last_cb_ns is not None:
            self.period_ms.append((now_ns - self._last_cb_ns) / 1e6)
        self._last_cb_ns = now_ns

    def _report(self):
        raise SystemExit(report(self.lat_ms, self.period_ms))


def report(lat_ms: list[float], period_ms: list[float]) -> int:
    if not lat_ms:
        print("BENCH: 一帧没收到,检查对端是否启动。")
        return 2
    lat = np.asarray(lat_ms)
    per = np.asarray(period_ms) if period_ms else np.zeros(1)
    p = lambda a, q: float(np.percentile(a, q))
    print(f"BENCH: 收 {len(lat)} 帧")
    print(f"  单跳延迟 ms: mean={lat.mean():.3f} p50={p(lat,50):.3f} p99={p(lat,99):.3f} max={lat.max():.3f}")
    print(f"  回调周期 ms: mean={per.mean():.3f} p99={p(per,99):.3f} max={per.max():.3f}")
    ok = p(lat, 99) < 2.0 and (not period_ms or abs(p(per, 99) - per.mean()) < 5.0)
    print(f"  判定: {'PASS — 控制走 topic 放行' if ok else 'FAIL — 先治抖动再往下'}")
    return 0 if ok else 1


def main():
    rclpy.init()
    node = BenchNode()
    try:
        rclpy.spin(node)
    except SystemExit as e:
        code = int(e.code or 0)
    except KeyboardInterrupt:
        code = report(node.lat_ms, node.period_ms)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(code)


if __name__ == "__main__":
    main()
