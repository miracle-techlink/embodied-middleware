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

from dataclasses import dataclass

from ..configs import CameraConfig, ColorMode, Cv2Rotation


@CameraConfig.register_subclass("orbbec")
@dataclass
class OrbbecCameraConfig(CameraConfig):
    """Configuration for Orbbec depth cameras (Gemini / Femto / Astra) via ``pyorbbecsdk`` v2.

    Mirrors :class:`RealSenseCameraConfig`: the camera is identified by its unique
    serial number (recommended, stable across replug/USB port) or by a device name,
    and it can stream a color frame, an aligned depth map, or both.

    When ``use_depth`` is True the depth frame is spatially aligned to the color
    camera (software Depth-to-Color / D2C), so ``<cam>`` (color, ``H,W,3`` uint8) and
    ``<cam>_depth`` (depth, ``H,W,1`` uint16 millimeters) share the same pixel grid.

    Example (Gemini 305, select by serial):
    ```python
    OrbbecCameraConfig("CV2856D0006R")                          # color only, default profile
    OrbbecCameraConfig("CV2856D0006R", use_depth=True)          # color + aligned depth
    OrbbecCameraConfig("CV2856D0006R", 30, 1280, 800, use_depth=True)
    ```

    Attributes:
        serial_number_or_name: Unique serial number (e.g. "CV2856D0006R") or device name
            (e.g. "Orbbec Gemini 305", only if a single such camera is connected).
        color_mode: Output color layout, RGB or BGR. Defaults to RGB (dataset standard).
        use_rgb: Enable the color stream. Defaults to True.
        use_depth: Enable the (aligned) depth stream. Defaults to False.
        align_to_color: When ``use_depth``, align depth into the color frame (D2C).
            Defaults to True. Set False to keep depth in its native geometry.
        rotation: Image rotation (0/90/180/270). Defaults to no rotation.
        warmup_s: Seconds to read frames before returning from ``connect()``.

    Note:
        - Either a serial number or a unique name must be provided.
        - At least one of ``use_rgb`` / ``use_depth`` must be enabled.
        - ``fps``/``width``/``height`` must be set all together or not at all; when unset
          the SDK's default stream profile for the device is used.
    """

    serial_number_or_name: str
    color_mode: ColorMode = ColorMode.RGB
    use_rgb: bool = True
    use_depth: bool = False
    align_to_color: bool = True
    # How to do Depth-to-Color alignment when ``align_to_color`` is set:
    #   "sw" - software AlignFilter on the host CPU (default; works everywhere).
    #   "hw" - hardware D2C on the camera's depth ASIC (offloads the host CPU, lower
    #          latency). Needs device + firmware support for the chosen profile; if the
    #          stream fails to start, fall back to "sw". Gemini 305 exposes HW_MODE.
    align_mode: str = "sw"
    # On-wire color format requested from the device:
    #   "auto" - prefer uncompressed RGB, then MJPG/YUYV/BGR (good on USB3).
    #   "mjpg" - force JPEG-compressed color. REQUIRED for higher-res/fps color+depth
    #            on a USB2 link, where uncompressed RGB would exceed the ~35 MB/s budget.
    #   "rgb" / "yuyv" / "bgr" - force that exact uncompressed format.
    # Output frames are always decoded to (H, W, 3) in ``color_mode`` regardless.
    color_format: str = "auto"
    rotation: Cv2Rotation = Cv2Rotation.NO_ROTATION
    warmup_s: int = 1
    # Self-heal a wedged device on connect: if warmup can't get frames within the ceiling,
    # USB-reset the camera and retry. This recovers the common failure where a prior process
    # died abnormally (core dump / kill -9) without stopping the stream, leaving a leaked
    # session so the next connect() stalls. Needs udev access to /dev/bus/usb (no sudo; see
    # install_orbbec.sh). Set reset_on_stall=False to disable.
    reset_on_stall: bool = True
    reset_retries: int = 1

    def __post_init__(self) -> None:
        self.color_mode = ColorMode(self.color_mode)
        self.rotation = Cv2Rotation(self.rotation)

        if not self.use_rgb and not self.use_depth:
            raise ValueError("At least one of `use_rgb` or `use_depth` must be enabled.")

        values = (self.fps, self.width, self.height)
        if any(v is not None for v in values) and any(v is None for v in values):
            raise ValueError(
                "For `fps`, `width` and `height`, either all of them need to be set, or none of them."
            )
