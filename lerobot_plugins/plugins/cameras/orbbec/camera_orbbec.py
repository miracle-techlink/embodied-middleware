# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Provides the OrbbecCamera class for capturing color + aligned depth frames from
Orbbec depth cameras (Gemini / Femto / Astra) through the ``pyorbbecsdk`` v2 library.

The public interface mirrors :class:`RealSenseCamera`: a background thread keeps the
latest color and depth frames, and ``read``/``async_read`` (color) and
``read_depth``/``async_read_depth`` return numpy arrays. Depth is returned as
``(H, W, 1)`` ``uint16`` in millimeters; when ``align_to_color`` is set (default) the
depth map is spatially aligned to the color frame (software D2C).
"""

import logging
import time
from threading import Event, Lock, Thread
from typing import TYPE_CHECKING, Any

import cv2  # type: ignore  # TODO: add type stubs for OpenCV
import numpy as np  # type: ignore  # TODO: add type stubs for numpy
from numpy.typing import NDArray  # type: ignore  # TODO: add type stubs for numpy.typing

from lerobot.utils.import_utils import _pyorbbecsdk_available, require_package

if TYPE_CHECKING or _pyorbbecsdk_available:
    import pyorbbecsdk as ob
else:
    ob = None

from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected
from lerobot.utils.errors import DeviceNotConnectedError

from ..camera import Camera
from ..configs import ColorMode
from ..utils import get_cv2_rotation
from .configuration_orbbec import OrbbecCameraConfig

logger = logging.getLogger(__name__)

# The Orbbec SDK is built around a SINGLE process-wide Context that owns the USB
# enumerator. Creating one Context per camera makes concurrent open/enumerate calls
# deadlock, which breaks multi-camera setups. Share one Context across all cameras.
_shared_context: Any = None


def _get_context():
    global _shared_context
    if _shared_context is None:
        _shared_context = ob.Context()
        _shared_context.set_logger_level(ob.OBLogLevel.ERROR)
    return _shared_context


class OrbbecCamera(Camera):
    """
    Manages an Orbbec depth camera (color + optional aligned depth) via ``pyorbbecsdk`` v2.

    The camera is identified by its unique serial number (recommended) or by a device
    name when only one such camera is connected. This is more robust than a UVC device
    index, especially with multiple cameras on the same host.

    Example:
        ```python
        from lerobot.cameras.orbbec import OrbbecCamera, OrbbecCameraConfig
        from lerobot.cameras import ColorMode, Cv2Rotation

        config = OrbbecCameraConfig(serial_number_or_name="CV2856D0006R", use_depth=True)
        camera = OrbbecCamera(config)
        camera.connect()

        color = camera.read()             # (H, W, 3) uint8, RGB by default
        depth = camera.read_depth()       # (H, W, 1) uint16, millimeters (aligned to color)

        camera.disconnect()
        ```
    """

    def __init__(self, config: OrbbecCameraConfig):
        require_package("pyorbbecsdk2", extra="orbbec", import_name="pyorbbecsdk")
        super().__init__(config)

        self.config = config
        self.serial_number_or_name = config.serial_number_or_name
        self.serial_number: str | None = None  # resolved at connect()

        self.fps = config.fps
        self.color_mode = config.color_mode
        self.use_rgb = config.use_rgb
        self.use_depth = config.use_depth
        self.align_to_color = config.align_to_color
        self.align_mode = getattr(config, "align_mode", "sw")
        self.color_format = config.color_format
        self.warmup_s = config.warmup_s

        self.pipeline: Any = None
        self.align_filter: Any = None

        self.thread: Thread | None = None
        self.stop_event: Event | None = None
        self.frame_lock: Lock = Lock()
        self.latest_color_frame: NDArray[Any] | None = None
        self.latest_depth_frame: NDArray[Any] | None = None
        self.latest_timestamp: float | None = None
        self.new_frame_event: Event = Event()

        self.rotation: int | None = get_cv2_rotation(config.rotation)

        self.capture_width, self.capture_height = self.width, self.height
        if self.height and self.width and self.rotation in [
            cv2.ROTATE_90_CLOCKWISE,
            cv2.ROTATE_90_COUNTERCLOCKWISE,
        ]:
            self.capture_width, self.capture_height = self.height, self.width

    def __str__(self) -> str:
        return f"{self.__class__.__name__}({self.serial_number or self.serial_number_or_name})"

    @property
    def is_connected(self) -> bool:
        return self.pipeline is not None

    # ------------------------------------------------------------------ discovery
    @staticmethod
    def _iter_devices(device_list) -> list[Any]:
        return [device_list.get_device_by_index(i) for i in range(device_list.get_count())]

    @staticmethod
    def find_cameras() -> list[dict[str, Any]]:
        """List connected Orbbec devices (serial, name, pid, connection type)."""
        device_list = _get_context().query_devices()
        found: list[dict[str, Any]] = []
        for i in range(device_list.get_count()):
            info = device_list.get_device_by_index(i).get_device_info()
            found.append(
                {
                    "type": "Orbbec",
                    "id": info.get_serial_number(),
                    "name": info.get_name(),
                    "pid": hex(info.get_pid()),
                    "connection_type": info.get_connection_type(),
                }
            )
        return found

    def _find_device(self):
        """Return the ``ob.Device`` matching the configured serial number or name."""
        device_list = _get_context().query_devices()
        if device_list.get_count() == 0:
            raise ConnectionError(
                "No Orbbec camera detected. Check the USB connection and udev rules "
                "(install with pyorbbecsdk `install_udev_rules.sh`)."
            )

        devices = self._iter_devices(device_list)
        infos = [d.get_device_info() for d in devices]

        target = self.serial_number_or_name
        by_serial = [d for d, i in zip(devices, infos) if i.get_serial_number() == target]
        if by_serial:
            self.serial_number = target
            return by_serial[0]

        by_name = [d for d, i in zip(devices, infos) if i.get_name() == target]
        if len(by_name) == 1:
            self.serial_number = by_name[0].get_device_info().get_serial_number()
            return by_name[0]
        if len(by_name) > 1:
            serials = [i.get_serial_number() for i in infos if i.get_name() == target]
            raise ValueError(
                f"Multiple Orbbec cameras named '{target}' found. Use a unique serial number instead. "
                f"Found SNs: {serials}"
            )

        available = [f"{i.get_name()} (SN={i.get_serial_number()})" for i in infos]
        raise ValueError(
            f"No Orbbec camera found with serial/name '{target}'. Available: {available}"
        )

    # ------------------------------------------------------------------ connect
    @staticmethod
    def _describe_color_profiles(color_profiles) -> str:
        from collections import defaultdict

        by_fmt: dict[str, list[str]] = defaultdict(list)
        seen: set[tuple] = set()
        for j in range(color_profiles.get_count()):
            p = color_profiles.get_stream_profile_by_index(j).as_video_stream_profile()
            key = (str(p.get_format()), p.get_width(), p.get_height(), p.get_fps())
            if key in seen:
                continue
            seen.add(key)
            by_fmt[key[0]].append(f"{p.get_width()}x{p.get_height()}@{p.get_fps()}")
        return "\n".join(f"  {fmt}: {', '.join(sizes)}" for fmt, sizes in by_fmt.items())

    def _build_config(self, pipeline) -> Any:
        cfg = ob.Config()

        if self.use_rgb:
            color_profiles = pipeline.get_stream_profile_list(ob.OBSensorType.COLOR_SENSOR)
            color_profile = None
            if self.width and self.height and self.fps:
                # Exact (w, h, fps) match. Format preference order depends on color_format;
                # "auto" prefers uncompressed RGB. We do NOT silently fall back to a different
                # resolution: a wrong (often higher-res) profile can saturate USB2 bandwidth
                # and stall the stream. On a USB2 link use color_format="mjpg".
                fmt_order = {
                    "auto": (ob.OBFormat.RGB, ob.OBFormat.MJPG, ob.OBFormat.YUYV, ob.OBFormat.BGR),
                    "rgb": (ob.OBFormat.RGB,),
                    "mjpg": (ob.OBFormat.MJPG,),
                    "yuyv": (ob.OBFormat.YUYV,),
                    "bgr": (ob.OBFormat.BGR,),
                }[self.color_format.lower()]
                for fmt in fmt_order:
                    try:
                        color_profile = color_profiles.get_video_stream_profile(
                            self.capture_width, self.capture_height, fmt, self.fps
                        )
                        break
                    except Exception:
                        continue
                if color_profile is None:
                    raise ConnectionError(
                        f"{self}: no color profile {self.capture_width}x{self.capture_height}@{self.fps} "
                        f"(RGB/MJPG/YUYV/BGR). Available:\n{self._describe_color_profiles(color_profiles)}"
                    )
            else:
                try:
                    color_profile = color_profiles.get_video_stream_profile(0, 0, ob.OBFormat.RGB, 0)
                except Exception:
                    color_profile = color_profiles.get_default_video_stream_profile()
            cfg.enable_stream(color_profile)

        if self.use_depth:
            depth_profiles = pipeline.get_stream_profile_list(ob.OBSensorType.DEPTH_SENSOR)
            cfg.enable_stream(depth_profiles.get_default_video_stream_profile())

        # Only emit an aggregate frame once every enabled stream is present, so the
        # aligned color+depth pair is always coherent.
        if self.use_rgb and self.use_depth:
            cfg.set_frame_aggregate_output_mode(ob.OBFrameAggregateOutputMode.FULL_FRAME_REQUIRE)

        # Hardware D2C: align on the depth ASIC (frees the host CPU). Requested here so the
        # SDK negotiates a HW-alignable profile; the software AlignFilter is then skipped.
        if self.use_rgb and self.use_depth and self.align_to_color and self.align_mode == "hw":
            try:
                cfg.set_align_mode(ob.OBAlignMode.HW_MODE)
            except Exception as e:
                logger.warning(f"{self} hardware D2C not available ({e}); falling back to software align.")
                self.align_mode = "sw"

        return cfg

    @check_if_already_connected
    def connect(self, warmup: bool = True) -> None:
        # Self-heal: if the device is wedged (a prior crash leaked its stream session so
        # warmup gets no frames), USB-reset it and retry, instead of just failing.
        retries = max(0, int(getattr(self.config, "reset_retries", 1))) if getattr(
            self.config, "reset_on_stall", True
        ) else 0
        last_err: Exception | None = None
        for attempt in range(retries + 1):
            try:
                self._open_once()
                return
            except ConnectionError as e:
                last_err = e
                self._cleanup_partial()
                if attempt < retries:
                    logger.warning(
                        f"{self} connect stalled ({e}); USB-resetting the camera and retrying "
                        f"({attempt + 1}/{retries})..."
                    )
                    self._usb_reset()
                    time.sleep(3.0)  # 等设备重新枚举
        assert last_err is not None
        raise last_err

    def _open_once(self) -> None:
        device = self._find_device()
        self.pipeline = ob.Pipeline(device)
        cfg = self._build_config(self.pipeline)

        # Software D2C only when not doing hardware alignment (HW_MODE aligns in-SDK).
        if self.use_depth and self.use_rgb and self.align_to_color and self.align_mode != "hw":
            self.align_filter = ob.AlignFilter(align_to_stream=ob.OBStreamType.COLOR_STREAM)

        try:
            self.pipeline.start(cfg)
        except Exception as e:
            self.pipeline = None
            self.align_filter = None
            raise ConnectionError(f"Failed to start {self}: {e}") from e

        self._start_read_thread()

        # Wait for the first coherent frame(s). A cold USB2 link can take several
        # seconds (depth engine spin-up + low aggregate fps), so poll up to a generous
        # ceiling and break as soon as the needed streams have delivered — this keeps a
        # healthy USB3 camera at ~1-2s while giving USB2 the time it needs.
        def _ready() -> bool:
            with self.frame_lock:
                rgb_ok = (not self.use_rgb) or self.latest_color_frame is not None
                depth_ok = (not self.use_depth) or self.latest_depth_frame is not None
            return rgb_ok and depth_ok

        ceiling_s = max(self.warmup_s, 8)
        warmup_read = self.async_read if self.use_rgb else self.async_read_depth
        start_time = time.time()
        while time.time() - start_time < ceiling_s and not _ready():
            try:
                warmup_read(timeout_ms=1000)
            except TimeoutError:
                pass
            time.sleep(0.05)

        if not _ready():
            raise ConnectionError(
                f"{self} failed to capture frames within {ceiling_s:.0f}s "
                f"(likely a leaked stream session from a prior crash; a USB reset recovers it)."
            )

        logger.info(f"{self} connected (color={self.use_rgb}, depth={self.use_depth}).")

    def _cleanup_partial(self) -> None:
        """Tear down a half-open attempt without the connected-state guards (for retry)."""
        try:
            self._stop_read_thread()
        except Exception:
            pass
        if self.pipeline is not None:
            try:
                self.pipeline.stop()
            except Exception:
                pass
            self.pipeline = None
        self.align_filter = None
        with self.frame_lock:
            self.latest_color_frame = None
            self.latest_depth_frame = None
            self.latest_timestamp = None
            self.new_frame_event.clear()

    def _usb_reset(self) -> None:
        """Force a USB port reset of the Orbbec device to clear a leaked stream session left
        by an abnormally-terminated prior process. No sudo if udev grants /dev/bus/usb access
        (install_orbbec.sh). Also drops the shared SDK context so the next query re-enumerates."""
        global _shared_context
        import fcntl
        import glob
        import os

        USBDEVFS_RESET = ord("U") << 8 | 20  # _IO('U', 20)
        n = 0
        for d in glob.glob("/sys/bus/usb/devices/*"):
            try:
                vid = open(os.path.join(d, "idVendor")).read().strip()
            except OSError:
                continue
            if vid.lower() != "2bc5":  # Orbbec vendor id
                continue
            try:
                busnum = int(open(os.path.join(d, "busnum")).read())
                devnum = int(open(os.path.join(d, "devnum")).read())
                node = f"/dev/bus/usb/{busnum:03d}/{devnum:03d}"
                fd = os.open(node, os.O_WRONLY)
                try:
                    fcntl.ioctl(fd, USBDEVFS_RESET, 0)
                    n += 1
                    logger.info(f"{self}: USB reset {node}")
                finally:
                    os.close(fd)
            except Exception as e:
                logger.warning(f"{self}: USB reset failed for {os.path.basename(d)}: {e}")
        if n == 0:
            logger.warning(f"{self}: no Orbbec USB device found to reset (vid 2bc5).")
        # 让下一次 _find_device 用全新 context 重新枚举(旧 handle 已随 reset 失效)
        _shared_context = None

    # ------------------------------------------------------------------ decoding
    def _decode_color(self, color_frame) -> NDArray[Any]:
        """Decode an Orbbec color frame to an ``(H, W, 3)`` array in ``self.color_mode``."""
        w = color_frame.get_width()
        h = color_frame.get_height()
        fmt = color_frame.get_format()
        data = np.asanyarray(color_frame.get_data())

        if fmt == ob.OBFormat.RGB:
            rgb = data.reshape((h, w, 3))
        elif fmt == ob.OBFormat.BGR:
            rgb = cv2.cvtColor(data.reshape((h, w, 3)), cv2.COLOR_BGR2RGB)
        elif fmt == ob.OBFormat.YUYV:
            rgb = cv2.cvtColor(data.reshape((h, w, 2)), cv2.COLOR_YUV2RGB_YUYV)
        elif fmt == ob.OBFormat.UYVY:
            rgb = cv2.cvtColor(data.reshape((h, w, 2)), cv2.COLOR_YUV2RGB_UYVY)
        elif fmt == ob.OBFormat.MJPG:
            bgr = cv2.imdecode(data, cv2.IMREAD_COLOR)
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        elif fmt in (ob.OBFormat.NV12, ob.OBFormat.NV21, ob.OBFormat.I420):
            code = {
                ob.OBFormat.NV12: cv2.COLOR_YUV2RGB_NV12,
                ob.OBFormat.NV21: cv2.COLOR_YUV2RGB_NV21,
                ob.OBFormat.I420: cv2.COLOR_YUV2RGB_I420,
            }[fmt]
            rgb = cv2.cvtColor(data.reshape((h * 3 // 2, w)), code)
        else:
            raise RuntimeError(f"{self}: unsupported color format {fmt}.")

        return self._postprocess(rgb)

    def _decode_depth(self, depth_frame) -> NDArray[np.uint16]:
        """Decode an Orbbec depth frame to ``(H, W, 1)`` ``uint16`` millimeters."""
        w = depth_frame.get_width()
        h = depth_frame.get_height()
        scale = depth_frame.get_depth_scale()
        raw = np.frombuffer(depth_frame.get_data(), dtype=np.uint16).reshape((h, w))
        # depth_scale converts raw units to millimeters (typically 0.1..1.0).
        depth_mm = (raw.astype(np.float32) * scale).round().astype(np.uint16)
        depth_mm = self._postprocess(depth_mm, depth=True)
        if depth_mm.ndim == 2:
            depth_mm = depth_mm[..., np.newaxis]
        return depth_mm

    def _postprocess(self, image: NDArray[Any], depth: bool = False) -> NDArray[Any]:
        if not depth and self.color_mode == ColorMode.BGR:
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if self.rotation in [cv2.ROTATE_90_CLOCKWISE, cv2.ROTATE_90_COUNTERCLOCKWISE, cv2.ROTATE_180]:
            image = cv2.rotate(image, self.rotation)
        return image

    # ------------------------------------------------------------------ read thread
    def _read_loop(self) -> None:
        stop_event = self.stop_event
        if stop_event is None:
            raise RuntimeError(f"{self}: stop_event is not initialized before starting read loop.")

        failure_count = 0
        while not stop_event.is_set():
            try:
                frames = self.pipeline.wait_for_frames(1000)
                if not frames:
                    continue
                if self.align_filter is not None:
                    frames = self.align_filter.process(frames)
                    if not frames:
                        continue

                color_np = depth_np = None
                if self.use_rgb:
                    color_frame = frames.get_color_frame()
                    if not color_frame:
                        continue
                    color_np = self._decode_color(color_frame)
                if self.use_depth:
                    depth_frame = frames.get_depth_frame()
                    if not depth_frame:
                        continue
                    depth_np = self._decode_depth(depth_frame)

                capture_time = time.perf_counter()
                with self.frame_lock:
                    if self.use_rgb:
                        self.latest_color_frame = color_np
                    if self.use_depth:
                        self.latest_depth_frame = depth_np
                    self.latest_timestamp = capture_time
                self.new_frame_event.set()
                failure_count = 0

            except DeviceNotConnectedError:
                break
            except Exception as e:
                if failure_count <= 10:
                    failure_count += 1
                    logger.warning(f"Error reading frame in background thread for {self}: {e}")
                else:
                    raise RuntimeError(f"{self} exceeded maximum consecutive read failures.") from e

    def _start_read_thread(self) -> None:
        self._stop_read_thread()
        self.stop_event = Event()
        self.thread = Thread(target=self._read_loop, args=(), name=f"{self}_read_loop")
        self.thread.daemon = True
        self.thread.start()

    def _stop_read_thread(self) -> None:
        if self.stop_event is not None:
            self.stop_event.set()
        if self.thread is not None and self.thread.is_alive():
            self.thread.join(timeout=2.0)
            if self.thread.is_alive():  # pragma: no cover
                logger.warning(f"{self} read thread did not terminate within timeout.")
        self.thread = None
        self.stop_event = None
        with self.frame_lock:
            self.latest_color_frame = None
            self.latest_depth_frame = None
            self.latest_timestamp = None
            self.new_frame_event.clear()

    # ------------------------------------------------------------------ read API
    def _async_read(self, timeout_ms: float, read_depth: bool = False) -> NDArray[Any]:
        if self.thread is None or not self.thread.is_alive():
            raise RuntimeError(f"{self} read thread is not running.")
        if not self.new_frame_event.wait(timeout=timeout_ms / 1000.0):
            raise TimeoutError(f"Timed out waiting for a frame from {self} after {timeout_ms} ms.")
        with self.frame_lock:
            frame = self.latest_depth_frame if read_depth else self.latest_color_frame
            self.new_frame_event.clear()
        if frame is None:
            raise RuntimeError(f"Internal error: event set but no frame available for {self}.")
        return frame

    def _read(self, read_depth: bool = False) -> NDArray[Any]:
        if self.thread is None or not self.thread.is_alive():
            raise RuntimeError(f"{self} read thread is not running.")
        self.new_frame_event.clear()
        return self._async_read(timeout_ms=10000, read_depth=read_depth)

    @check_if_not_connected
    def read(self, color_mode: ColorMode | None = None, timeout_ms: int = 0) -> NDArray[Any]:
        """Blocking read of the latest color frame, shape ``(H, W, 3)``."""
        if not self.use_rgb:
            raise RuntimeError(f"{self}: cannot read color — camera was configured with use_rgb=False.")
        return self._read()

    @check_if_not_connected
    def read_depth(self, timeout_ms: int = 0) -> NDArray[np.uint16]:
        """Blocking read of the latest depth frame, shape ``(H, W, 1)`` uint16 millimeters."""
        if not self.use_depth:
            raise RuntimeError(f"{self}: cannot read depth — camera was configured with use_depth=False.")
        return self._read(read_depth=True)

    @check_if_not_connected
    def async_read(self, timeout_ms: float = 200) -> NDArray[Any]:
        """Return the most recent color frame captured by the background thread."""
        if not self.use_rgb:
            raise RuntimeError(f"{self}: cannot read color — camera was configured with use_rgb=False.")
        return self._async_read(timeout_ms=timeout_ms)

    @check_if_not_connected
    def async_read_depth(self, timeout_ms: float = 200) -> NDArray[np.uint16]:
        """Return the most recent depth frame (``(H, W, 1)`` uint16 millimeters)."""
        if not self.use_depth:
            raise RuntimeError(f"{self}: cannot read depth — camera was configured with use_depth=False.")
        return self._async_read(timeout_ms=timeout_ms, read_depth=True)

    def _read_latest(self, max_age_ms: int, read_depth: bool = False) -> NDArray[Any]:
        if self.thread is None or not self.thread.is_alive():
            raise RuntimeError(f"{self} read thread is not running.")
        with self.frame_lock:
            frame = self.latest_depth_frame if read_depth else self.latest_color_frame
            timestamp = self.latest_timestamp
        if frame is None or timestamp is None:
            raise RuntimeError(f"{self} has not captured any frames yet.")
        age_ms = (time.perf_counter() - timestamp) * 1e3
        if age_ms > max_age_ms:
            raise TimeoutError(f"{self} latest frame is too old: {age_ms:.1f} ms (max {max_age_ms} ms).")
        return frame

    @check_if_not_connected
    def read_latest(self, max_age_ms: int = 500) -> NDArray[Any]:
        """Non-blocking peek at the most recent color frame."""
        if not self.use_rgb:
            raise RuntimeError(f"{self}: cannot read color — camera was configured with use_rgb=False.")
        return self._read_latest(max_age_ms=max_age_ms)

    @check_if_not_connected
    def read_latest_depth(self, max_age_ms: int = 500) -> NDArray[np.uint16]:
        """Non-blocking peek at the most recent depth frame."""
        if not self.use_depth:
            raise RuntimeError(f"{self}: cannot read depth — camera was configured with use_depth=False.")
        return self._read_latest(max_age_ms=max_age_ms, read_depth=True)

    # ------------------------------------------------------------------ teardown
    def disconnect(self) -> None:
        if not self.is_connected and self.thread is None:
            raise DeviceNotConnectedError(
                f"Attempted to disconnect {self}, but it appears already disconnected."
            )
        if self.thread is not None:
            self._stop_read_thread()
        if self.pipeline is not None:
            # ``Pipeline.stop()`` can block indefinitely on a wedged link (observed with a
            # Gemini 305 on a USB2 port). Run it in a watchdog thread so a stuck stop never
            # hangs teleop teardown; if it times out we drop the handle and warn.
            pipeline = self.pipeline

            def _stop():
                try:
                    pipeline.stop()
                except Exception as e:  # pragma: no cover
                    logger.warning(f"{self}: error stopping pipeline: {e}")

            stopper = Thread(target=_stop, name=f"{self}_stop", daemon=True)
            stopper.start()
            stopper.join(timeout=3.0)
            if stopper.is_alive():
                logger.warning(
                    f"{self}: pipeline.stop() did not return within 3s (USB link may be wedged; "
                    f"consider a USB3 port or replugging the camera)."
                )
            self.pipeline = None
        self.align_filter = None
        with self.frame_lock:
            self.latest_color_frame = None
            self.latest_depth_frame = None
            self.latest_timestamp = None
            self.new_frame_event.clear()
        logger.info(f"{self} disconnected.")
