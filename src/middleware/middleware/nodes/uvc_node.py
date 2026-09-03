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

from middleware.topic_registry import TopicRegistry

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
        # 曝光锁定: 开机让 AE/AWB 自动收敛 N 帧后读出当前值并关自动锁死 —— 会话内
        # 零漂移, 重启后重新收敛再锁(光照变了也自适应)。0=始终自动(旧行为)。
        self.declare_parameter("lock_after_frames", 90)

        reg = TopicRegistry(self.get_parameter("registry_json").value or None)
        spec = reg.require("/rebot/front/color/compressed")
        self._jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        fps = int(self.get_parameter("fps").value)
        w, h = int(self.get_parameter("width").value), int(self.get_parameter("height").value)

        import cv2

        self._cv2 = cv2
        self._device = self.get_parameter("device").value
        self._w, self._h, self._fps = w, h, fps
        self._fail_streak = 0
        self._lock_after = int(self.get_parameter("lock_after_frames").value)
        self._frames = 0
        self._open_camera()

        self._pub = self.create_publisher(CompressedImage, spec.topic_name, IMG_QOS)
        self.create_timer(1.0 / fps, self._publish_frame)
        self.get_logger().info(f"发布 {spec.topic_name} @ {fps}Hz。")

    def _open_camera(self) -> None:
        """打开并配置 V4L2 采集;取帧层冻住后的重开也走这里。"""
        cv2 = self._cv2
        cap = cv2.VideoCapture(self._device, cv2.CAP_V4L2)
        if not cap.isOpened():
            raise RuntimeError(f"打不开 {self._device}")
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._h)
        cap.set(cv2.CAP_PROP_FPS, self._fps)
        aw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        ah = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        if (aw, ah) != (self._w, self._h):
            cap.release()
            raise RuntimeError(f"{self._device} 分辨率设置失败: 要 {self._w}x{self._h},实际 {aw}x{ah}")
        self.get_logger().info(f"UVC 相机 {self._device} 已开({aw}x{ah} @ 目标 {self._fps}fps,MJPG)。")
        self._cap = cap

    def _lock_exposure(self) -> None:
        """读出 AE/AWB 收敛值 → 关自动 → 写回锁死(策略的画面分布从此会话内恒定)。"""
        cv2 = self._cv2
        exp = self._cap.get(cv2.CAP_PROP_EXPOSURE)
        wb = self._cap.get(cv2.CAP_PROP_WB_TEMPERATURE)
        self._cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)   # V4L2: 1=manual, 3=auto
        self._cap.set(cv2.CAP_PROP_AUTO_WB, 0)
        if exp > 0:
            self._cap.set(cv2.CAP_PROP_EXPOSURE, exp)
        if wb > 0:
            self._cap.set(cv2.CAP_PROP_WB_TEMPERATURE, wb)
        self.get_logger().info(
            f"曝光锁定: exposure={exp:.0f} wb={wb:.0f} "
            f"(回读 exp={self._cap.get(cv2.CAP_PROP_EXPOSURE):.0f})"
        )

    def _publish_frame(self):
        cv2 = self._cv2
        ok, frame = self._cap.read()  # (H,W,3) BGR
        if ok:
            self._frames += 1
            if self._lock_after > 0 and self._frames == self._lock_after:
                try:
                    self._lock_exposure()
                except Exception as e:
                    self.get_logger().warning(f"曝光锁定失败(继续自动): {e}")
        if not ok:
            # 偶发丢帧直接跳过;连续失败 ≈ 设备层冻死(实测 SN0002 会卡在 USB
            # 传输层,cap.read 永久 False)——每 15 帧(≈0.5s)释放重开一次自愈,
            # 不用重启节点,更不用动臂。
            self._fail_streak += 1
            if self._fail_streak % 15 == 0:
                self.get_logger().error(
                    f"连续 {self._fail_streak} 帧取帧失败,重开 {self._device} 自愈…"
                )
                try:
                    self._cap.release()
                except Exception:
                    pass
                try:
                    self._open_camera()
                    self._fail_streak = 0
                    self._frames = 0  # 重新收敛 AE 后再锁
                    self.get_logger().info("重开成功,恢复发布。")
                except Exception as e:
                    self.get_logger().error(f"重开失败: {e}(0.5s 后再试)")
            else:
                self.get_logger().warning("取帧失败(cap.read False)", throttle_duration_sec=1.0)
            return
        self._fail_streak = 0
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
