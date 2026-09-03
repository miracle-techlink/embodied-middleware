#!/usr/bin/env python
"""ROS2 订阅总线 — lerobot 进程内的单例 rclpy 订阅器。

bridge 插件(ros2_rebot_follower / ros2_rebot_teleop)共用:后台线程跑 spin,
各 topic 只缓存**最新一帧** + 到达时刻(单调钟),读取永远非阻塞 —— 与
``rebot_follower`` 的 cameras_nonblocking 语义一致。

QoS 必须 BEST_EFFORT keep-last 1:rebot_msg_center 各节点都这么发,RELIABLE 订阅
会被 DDS 判不兼容直接收不到(亲测)。

依赖:rclpy 不在 lerobot 依赖里。用前先::
    source /opt/ros/jazzy/setup.bash
    export PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages:...
"""

from __future__ import annotations

import threading
import time

import numpy as np


def _import_rclpy():
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import CompressedImage, JointState
    except ImportError as e:
        raise ImportError(
            "rclpy 不可导入。先: source /opt/ros/jazzy/setup.bash && "
            "export PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages:$PYTHONPATH"
        ) from e
    QOS = QoSProfile(
        history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT
    )
    return rclpy, Node, QOS, JointState, CompressedImage


class Ros2Bus:
    """进程级单例:一个 rclpy 节点 + 后台 spin 线程,按需挂订阅。"""

    _instance: "Ros2Bus | None" = None
    _lock = threading.Lock()

    def __init__(self):
        rclpy, Node, self._qos, JointState, CompressedImage = _import_rclpy()
        self._rclpy = rclpy
        if not rclpy.ok():
            rclpy.init()
        self._node = Node("lerobot_ros2_bridge")
        self._joint_state_type = JointState
        self._compressed_type = CompressedImage
        # topic → (payload, monotonic 到达时刻); joint_state payload = {name: (pos, vel, eff)}
        self._latest_js: dict[str, tuple[dict[str, tuple[float, float, float]], float]] = {}
        self._latest_img: dict[str, tuple[np.ndarray, float]] = {}
        self._stop = threading.Event()
        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()
        # 解释器退出前必须先停 spin 线程再销毁节点,否则 C++ 层析构撞上在飞的
        # spin_once → "terminate called without an active exception" SIGABRT。
        import atexit

        atexit.register(self.shutdown)

    def _spin(self):
        while not self._stop.is_set() and self._rclpy.ok():
            self._rclpy.spin_once(self._node, timeout_sec=0.1)

    def shutdown(self):
        self._stop.set()
        if self._spin_thread.is_alive():
            self._spin_thread.join(timeout=1.0)
        try:
            self._node.destroy_node()
        except Exception:
            pass
        try:
            if self._rclpy.ok():
                self._rclpy.shutdown()
        except Exception:
            pass

    @classmethod
    def instance(cls) -> "Ros2Bus":
        with cls._lock:
            if cls._instance is None:
                cls._instance = Ros2Bus()
            return cls._instance

    # ---------------- joint_state ----------------
    def sub_joint_state(self, topic: str) -> None:
        def cb(msg):
            data = {}
            for i, name in enumerate(msg.name):
                pos = msg.position[i] if i < len(msg.position) else 0.0
                vel = msg.velocity[i] if i < len(msg.velocity) else 0.0
                eff = msg.effort[i] if i < len(msg.effort) else 0.0
                data[name] = (float(pos), float(vel), float(eff))
            self._latest_js[topic] = (data, time.monotonic())

        self._node.create_subscription(self._joint_state_type, topic, cb, self._qos)

    def latest_joint_state(
        self, topic: str
    ) -> tuple[dict[str, tuple[float, float, float]], float] | None:
        """返回 ({name: (pos, vel, eff)}, 到达时刻 monotonic);没收过返回 None。"""
        return self._latest_js.get(topic)

    # ---------------- compressed image ----------------
    def sub_image(self, topic: str, kind: str) -> None:
        """kind: 'color'(jpeg → RGB (H,W,3) uint8)或 'depth'(png → (H,W,1) uint16)。"""

        def cb(msg):
            import cv2

            buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
            if kind == "color":
                img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
                if img is None:
                    return
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            else:  # depth: 16UC1 png,保持 uint16 毫米
                img = cv2.imdecode(buf, cv2.IMREAD_UNCHANGED)
                if img is None:
                    return
                img = img[:, :, None]  # (H,W) → (H,W,1),与 rebot_follower 深度特征一致
            self._latest_img[topic] = (img, time.monotonic())

        self._node.create_subscription(self._compressed_type, topic, cb, self._qos)

    def latest_image(self, topic: str) -> tuple[np.ndarray, float] | None:
        return self._latest_img.get(topic)

    # ---------------- 发布(teleop 侧用) ----------------
    def publish(self, msg_type, topic: str, msg) -> None:
        key = f"__pub__{topic}"
        pub = getattr(self, key, None)
        if pub is None:
            from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

            pub = self._node.create_publisher(
                msg_type,
                topic,
                QoSProfile(
                    history=HistoryPolicy.KEEP_LAST,
                    depth=1,
                    reliability=ReliabilityPolicy.RELIABLE,
                ),
            )
            setattr(self, key, pub)
        pub.publish(msg)
