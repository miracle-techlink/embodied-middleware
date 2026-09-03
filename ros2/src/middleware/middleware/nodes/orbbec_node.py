#!/usr/bin/env python
"""orbbec_node — 腕部 Orbbec(Gemini 305)深度相机的 ROS2 驱动节点。

复用 lerobot 侧 ``OrbbecCamera``(含非阻塞 read_latest / read_latest_depth),
彩色 jpeg、深度 PNG(16UC1 无损)压缩后以 CompressedImage 发布:

    发布 /rebot/wrist/color/compressed   CompressedImage  format="jpeg"
    发布 /rebot/wrist/depth/compressed   CompressedImage  format="16UC1; png compressed"(uint16 毫米)

参数:
    serial   相机序列号(默认 CV275610002L)
    width/height/fps  默认 640x480@30(与 record_rebot_gated.sh 一致)
    use_depth  默认 true;false 时只发彩色
    jpeg_quality  默认 90
"""

from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage

from middleware.topic_registry import TopicRegistry

IMG_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT
)


class OrbbecNode(Node):
    def __init__(self):
        super().__init__("orbbec_node")
        self.declare_parameter("serial", "CV275610002L")
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 30)
        self.declare_parameter("use_depth", True)
        self.declare_parameter("jpeg_quality", 90)
        self.declare_parameter("registry_json", "")
        # 曝光锁定: warmup 后让 AE 再收敛 N 秒, 读出当前 exposure/gain 关自动锁死
        # (重启会话间画面分布恒定 —— 相机重启不再是策略观测漂移源)。0=始终自动。
        self.declare_parameter("lock_after_s", 8.0)

        reg = TopicRegistry(self.get_parameter("registry_json").value or None)
        color_spec = reg.require("/rebot/wrist/color/compressed")
        depth_spec = reg.require("/rebot/wrist/depth/compressed")
        self._use_depth = bool(self.get_parameter("use_depth").value)
        self._jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        fps = int(self.get_parameter("fps").value)

        from lerobot.cameras.orbbec.camera_orbbec import OrbbecCamera
        from lerobot.cameras.orbbec.configuration_orbbec import OrbbecCameraConfig

        self._cam = OrbbecCamera(
            OrbbecCameraConfig(
                serial_number_or_name=self.get_parameter("serial").value,
                width=int(self.get_parameter("width").value),
                height=int(self.get_parameter("height").value),
                fps=fps,
                use_depth=self._use_depth,
            )
        )
        self.get_logger().info(f"连接 Orbbec {self._cam.config.serial_number_or_name}(warmup ~15s)…")
        self._cam.connect(warmup=True)

        self._color_pub = self.create_publisher(CompressedImage, color_spec.topic_name, IMG_QOS)
        self._depth_pub = (
            self.create_publisher(CompressedImage, depth_spec.topic_name, IMG_QOS)
            if self._use_depth else None
        )
        self.create_timer(1.0 / fps, self._publish_frames)
        lock_after = float(self.get_parameter("lock_after_s").value)
        if lock_after > 0:
            self._lock_timer = self.create_timer(lock_after, self._lock_exposure)
        self.get_logger().info(
            f"发布 {color_spec.topic_name}"
            + (f" + {depth_spec.topic_name}" if self._use_depth else "")
            + f" @ {fps}Hz。"
        )

    def _lock_exposure(self) -> None:
        """AE 收敛后读出 exposure/gain → 关自动 → 写回锁死(一次性定时器)。"""
        self._lock_timer.cancel()
        try:
            import pyorbbecsdk as ob

            dev = self._cam.pipeline.get_device()
            prop = ob.OBPropertyID
            exp = dev.get_int_property(prop.OB_PROP_COLOR_EXPOSURE_INT)
            gain = dev.get_int_property(prop.OB_PROP_COLOR_GAIN_INT)
            dev.set_bool_property(prop.OB_PROP_COLOR_AUTO_EXPOSURE_BOOL, False)
            dev.set_int_property(prop.OB_PROP_COLOR_EXPOSURE_INT, exp)
            dev.set_int_property(prop.OB_PROP_COLOR_GAIN_INT, gain)
            self.get_logger().info(
                f"曝光锁定: exposure={exp} gain={gain} "
                f"(回读 exp={dev.get_int_property(prop.OB_PROP_COLOR_EXPOSURE_INT)})"
            )
        except Exception as e:
            self.get_logger().warning(f"曝光锁定失败(继续自动): {e}")

    def _publish_frames(self):
        import cv2

        stamp = self.get_clock().now().to_msg()
        try:
            color = self._cam.read_latest(max_age_ms=500)  # (H,W,3) RGB
        except Exception as e:
            self.get_logger().warning(f"彩色取帧失败: {e}", throttle_duration_sec=1.0)
            return
        msg = CompressedImage()
        msg.header.stamp = stamp
        msg.format = "jpeg"
        ok, buf = cv2.imencode(
            ".jpg", cv2.cvtColor(color, cv2.COLOR_RGB2BGR),
            [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_quality],
        )
        if ok:
            msg.data = buf.tobytes()
            self._color_pub.publish(msg)

        if self._depth_pub is not None:
            try:
                depth = self._cam.read_latest_depth(max_age_ms=500)  # (H,W,1) uint16 毫米
            except Exception as e:
                self.get_logger().warning(f"深度取帧失败: {e}", throttle_duration_sec=1.0)
                return
            d2 = depth.squeeze(-1) if depth.ndim == 3 else depth  # cv2 编码要 2D
            dmsg = CompressedImage()
            dmsg.header.stamp = stamp
            dmsg.format = "16UC1; png compressed"
            ok, buf = cv2.imencode(".png", d2.astype(np.uint16))
            if ok:
                dmsg.data = buf.tobytes()
                self._depth_pub.publish(dmsg)

    def destroy_node(self):
        try:
            if self._cam.is_connected:
                self._cam.disconnect()
        except Exception as e:
            self.get_logger().warning(f"disconnect 异常: {e}")
        super().destroy_node()


def main():
    import signal

    rclpy.init()
    node = OrbbecNode()
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
