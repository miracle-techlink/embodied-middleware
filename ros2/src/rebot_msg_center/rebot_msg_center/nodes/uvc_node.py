#!/usr/bin/env python
"""uvc_node — 前视 UVC USB 相机的 ROS2 驱动节点。

    发布 /rebot/front/color/compressed   CompressedImage  format="jpeg"

直接用 cv2.VideoCapture 而不用 lerobot OpenCVCamera:后者对 CAP_PROP_FRAME_WIDTH 的
set() 返回值做严格校验,而这台 SN0002 的 V4L2 驱动 set 返回 False 但值实际生效
(cv2 5.0 已知怪癖),会被误判为失败。这里设完后以**回读值**为准,不符才报错。

参数:
    device   设备路径(默认 /dev/video0;注意 UVC 节点号重启会漂移,
             稳定的写法是 /dev/v4l/by-path/... 软链)
    width/height/fps  默认 640x480@30
    jpeg_quality  默认 90
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage

from rebot_msg_center.topic_registry import TopicRegistry

IMG_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT
)


class UvcNode(Node):
    def __init__(self):
        super().__init__("uvc_node")
        self.declare_parameter("device", "/dev/video0")
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 30)
        self.declare_parameter("jpeg_quality", 90)
        self.declare_parameter("registry_json", "")

        reg = TopicRegistry(self.get_parameter("registry_json").value or None)
        spec = reg.require("/rebot/front/color/compressed")
        self._jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        fps = int(self.get_parameter("fps").value)
        w, h = int(self.get_parameter("width").value), int(self.get_parameter("height").value)

        import cv2

        self._cv2 = cv2
        device = self.get_parameter("device").value
        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not cap.isOpened():
            raise RuntimeError(f"打不开 {device}")
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        cap.set(cv2.CAP_PROP_FPS, fps)
        aw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        ah = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        if (aw, ah) != (w, h):
            cap.release()
            raise RuntimeError(f"{device} 分辨率设置失败: 要 {w}x{h},实际 {aw}x{ah}")
        self.get_logger().info(f"UVC 相机 {device} 已开({aw}x{ah} @ 目标 {fps}fps,MJPG)。")
        self._cap = cap

        self._pub = self.create_publisher(CompressedImage, spec.topic_name, IMG_QOS)
        self.create_timer(1.0 / fps, self._publish_frame)
        self.get_logger().info(f"发布 {spec.topic_name} @ {fps}Hz。")

    def _publish_frame(self):
        cv2 = self._cv2
        ok, frame = self._cap.read()  # (H,W,3) BGR
        if not ok:
            self.get_logger().warning("取帧失败(cap.read False)", throttle_duration_sec=1.0)
            return
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"
        ok, buf = cv2.imencode(
            ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_quality]
        )
        if ok:
            msg.data = buf.tobytes()
            self._pub.publish(msg)

    def destroy_node(self):
        try:
            self._cap.release()
        except Exception:
            pass
        super().destroy_node()


def main():
    import signal

    rclpy.init()
    node = UvcNode()
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
