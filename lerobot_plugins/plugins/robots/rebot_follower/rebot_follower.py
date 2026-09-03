#!/usr/bin/env python
# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License").
"""`rebot_follower` — 单臂 Seeed reBot B601-RS 的原生 lerobot Robot,带深度相机。

薄薄地继承官方 ``SeeedB601RSFollower``,**只加深度**:官方 ``get_observation`` 只用
``cam.async_read()`` 取彩色,深度从没进过 observation。这里为每个 ``use_depth=True`` 的相机
额外声明 ``<cam>_depth (H,W,1)`` 特征,并在观测里补上对齐后的深度帧(uint16 毫米)。

其余(电机连接/标定/send_action 的关节+夹爪力控/断开回零)全部继承官方实现,所以:

    lerobot-teleoperate --robot.type=rebot_follower ... --teleop.type=starai_to_rebot_leader ...
    lerobot-record      --robot.type=rebot_follower ... --teleop.type=starai_to_rebot_leader ... --dataset...

action 空间 = reBot 自己的关节空间(``shoulder_pan.pos`` ... ``gripper.pos``,7 维),与
``observation.state`` 同帧。先用 ``seeed_b601_rs_follower`` 类型标定一次(共用标定文件)。
"""

import logging
import math
import time

import numpy as np

from lerobot.utils.errors import DeviceAlreadyConnectedError
from lerobot_robot_seeed_b601.seeed_b601_follower import MotorBridgeController
from lerobot_robot_seeed_b601.seeed_b601_rs_follower import SeeedB601RSFollower

from .config_rebot_follower import RebotFollowerConfig

logger = logging.getLogger(__name__)

REBOT_ARM_MOTORS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_yaw", "wrist_roll"]


class RebotFollower(SeeedB601RSFollower):
    config_class = RebotFollowerConfig
    name = "rebot_follower"

    def __init__(self, config: RebotFollowerConfig):
        super().__init__(config)
        self.config = config
        # 每个深度相机的最近一帧深度,取帧偶发超时时回退,保证 dataset 帧不缺键。
        self._last_depth: dict[str, np.ndarray] = {}
        self._last_color: dict[str, np.ndarray] = {}  # nonblocking 回退用
        self._grip_set: float | None = None  # 夹爪设定点(度),直驱 motor 7

    # ---------------- 容错接入:臂必需,相机可缺 ----------------
    @property
    def is_connected(self) -> bool:
        """臂总线在 = connected。相机是外设:它掉线不该把 disconnect/safe_zero 挡在门外
        (父版要求所有相机在线才算 connected,相机中途死掉 → 退出时臂卸不了力矩,危险)。
        纯相机模式(allow_missing_arm,bus 为 None)下有任一相机在线即算 connected。"""
        if self.bus is not None:
            return True
        return any(cam.is_connected for cam in self.cameras.values())

    @property
    def _armless(self) -> bool:
        """纯相机模式:allow_missing_arm=True 且臂总线没接上。"""
        return self.bus is None

    def connect(self, calibrate: bool = True) -> None:
        """接入:臂 CAN 总线必需(连不上直接报错,除非 allow_missing_arm=True);
        相机逐个尝试,掉线的按配置容错。

        与父类(lerobot_robot_seeed_b601 v1.0.2)connect() 的差异:父类任一相机 connect
        失败就整体中止。这里失败的相机从 self.cameras 摘除并继续 —— 特征表(_cameras_ft)
        遍历的是 self.cameras,数据集结构随之少这一路,保持自洽,不会录进全黑帧。除非该
        相机在 config.required_cameras 里,或 config.allow_missing_cameras=False。
        **父类包升级时需对照其 connect() 重新同步本方法。**
        """
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        logger.info(f"Connecting arm on {self.config.port} (adapter={self.config.can_adapter})...")
        try:
            if self.config.can_adapter == "damiao":
                self.bus = MotorBridgeController.from_dm_serial(
                    serial_port=self.config.port,
                    baud=self.config.dm_serial_baud,
                )
            elif self.config.can_adapter == "robstride":
                raise NotImplementedError(
                    "RobStride dedicated USB-to-CAN adapter is not yet supported in motorbridge Python SDK."
                )
            else:
                # Default: socketcan (PCAN, slcan, etc.)
                self.bus = MotorBridgeController(channel=self.config.port)
            self._add_motors_to_bus()
        except NotImplementedError:
            raise  # 配置错误,不是设备掉线,不容错
        except Exception as e:
            if not self.config.allow_missing_arm:
                raise
            logger.error(
                f"{self}: 臂 CAN 总线接入失败({e})——allow_missing_arm=True,"
                f"进入纯相机模式:电机观测补零,send_action 不执行。"
            )
            self.bus = None

        if not self._armless and not self.is_calibrated and calibrate:
            logger.info(
                "Mismatch between calibration values in the motor and the calibration file or no calibration file found"
            )
            self.calibrate()
            # calibrate() set _skip_safe_zero_on_disconnect for the calibrate
            # script's benefit; clear it here so a later disconnect (after
            # normal use of this connected arm) still runs safe_zero().
            self._skip_safe_zero_on_disconnect = False

        for cam_name in list(self.cameras):
            cam = self.cameras[cam_name]
            try:
                cam.connect()
            except Exception as e:
                if not self.config.allow_missing_cameras or cam_name in self.config.required_cameras:
                    raise
                logger.error(f"{self}: 相机 '{cam_name}' 接入失败({e}),已摘除,其余设备继续。")
                try:
                    cam.disconnect()  # 半开状态收尾,释放 fd
                except Exception:
                    pass
                del self.cameras[cam_name]

        if not self._armless:
            self.configure()

        logger.info(f"{self} connected. arm={'no(纯相机模式)' if self._armless else 'yes'} cameras={list(self.cameras)}")

    def _has_depth(self, cam_name: str) -> bool:
        return bool(getattr(self.config.cameras[cam_name], "use_depth", False))

    def _cam_dims(self, cam_name: str) -> tuple[int, int]:
        """有效 (H, W):配置给了用配置;配置留 None(= 不向相机发任何 SET 命令,迁就
        拒绝 SET 的坏固件相机,如那台 1080P USB Camera)时,用连接后相机实际协商出的值
        (OpenCVCamera.connect 会把默认分辨率写回 cam.width/height)。"""
        cfg = self.config.cameras[cam_name]
        h, w = cfg.height, cfg.width
        if h is None or w is None:
            cam = self.cameras.get(cam_name)
            ch, cw = getattr(cam, "height", None), getattr(cam, "width", None)
            if ch and cw:
                h, w = ch, cw
        if h is None or w is None:
            raise ValueError(
                f"相机 '{cam_name}' 分辨率未知:配置留 None 时需先 connect 才能确定特征形状。"
            )
        return int(h), int(w)

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        """彩色 (H,W,3);深度相机额外一路 <cam>_depth (H,W,1) → lerobot 认作深度图。"""
        ft: dict[str, tuple] = {}
        for cam_name in self.cameras:
            h, w = self._cam_dims(cam_name)
            ft[cam_name] = (h, w, 3)
            if self._has_depth(cam_name):
                ft[f"{cam_name}_depth"] = (h, w, 1)
        return ft

    def get_observation(self) -> dict:
        if self._armless or getattr(self.config, "cameras_nonblocking", False):
            return self._get_observation_nonblocking()

        # 默认路径:父类取电机状态 + 各相机彩色(阻塞 async_read),这里补对齐深度。
        obs = super().get_observation()
        for cam_name, cam in self.cameras.items():
            if not self._has_depth(cam_name):
                continue
            try:
                # 彩色刚被 async_read 更新过,深度取同一聚合帧的最新缓存(非阻塞、低延迟)。
                depth = cam.read_latest_depth(max_age_ms=1000)
            except Exception:
                depth = self._last_depth.get(cam_name)
                if depth is None:
                    h, w = self._cam_dims(cam_name)
                    depth = np.zeros((h, w, 1), dtype=np.uint16)
            self._last_depth[cam_name] = depth
            obs[f"{cam_name}_depth"] = depth
        return obs

    def _get_observation_nonblocking(self) -> dict:
        """非阻塞观测:电机 CAN 反馈(同官方)+ 相机 read_latest(不阻塞等帧)。见 config
        ``cameras_nonblocking``。相机帧超过 ``stale_frame_ms`` 会告警并回退上一帧(暴露卡死)。"""
        obs: dict = {}
        # 电机反馈:批量请求 + 一次 poll + 读缓存状态(与官方 get_observation 完全一致)
        for motor in self.motors.values():
            motor.request_feedback()
        if self.bus is not None:
            try:
                self.bus.poll_feedback_once()
            except Exception:
                logger.warning("can bus poll feedback failed.")
        if self.motors:
            motor_names = list(self.motors)
        else:
            # 纯相机模式(allow_missing_arm):没有电机对象,按配置补零,数据集结构不变
            motor_names = list(self.config.motor_can_ids)
        for motor_name in motor_names:
            motor = self.motors.get(motor_name)
            state = motor.get_state() if motor is not None else None
            if state is not None:
                obs[f"{motor_name}.pos"] = math.degrees(state.pos)
                obs[f"{motor_name}.vel"] = math.degrees(state.vel)
                obs[f"{motor_name}.torque"] = state.torq
            else:
                obs[f"{motor_name}.pos"] = 0.0
                obs[f"{motor_name}.vel"] = 0.0
                obs[f"{motor_name}.torque"] = 0.0

        # 相机:非阻塞取最新缓存帧(彩色 + 可选对齐深度)
        max_age = int(getattr(self.config, "stale_frame_ms", 200))
        for cam_name, cam in self.cameras.items():
            try:
                obs[cam_name] = cam.read_latest(max_age_ms=max_age)
                self._last_color[cam_name] = obs[cam_name]
            except Exception as e:
                logger.warning(f"{cam_name} color read_latest stale/failed ({e}); 回退上一帧。")
                fallback = self._last_color.get(cam_name)
                if fallback is not None:
                    obs[cam_name] = fallback
                else:
                    h, w = self._cam_dims(cam_name)
                    obs[cam_name] = np.zeros((h, w, 3), dtype=np.uint8)
            if self._has_depth(cam_name):
                try:
                    depth = cam.read_latest_depth(max_age_ms=max_age)
                except Exception as e:
                    logger.warning(f"{cam_name} depth read_latest stale/failed ({e}); 回退上一帧。")
                    depth = self._last_depth.get(cam_name)
                    if depth is None:
                        h, w = self._cam_dims(cam_name)
                        depth = np.zeros((h, w, 1), dtype=np.uint16)
                self._last_depth[cam_name] = depth
                obs[f"{cam_name}_depth"] = depth
        return obs

    # ---------------- send_action:臂走官方(不含夹爪),夹爪直驱 motor 7 ----------------
    def send_action(self, action: dict) -> dict:
        action = dict(action)
        if self._armless:
            # 纯相机模式:无臂可驱,透传目标值(录下的 action = 操作员意图,供日后筛用)
            if not getattr(self, "_armless_action_warned", False):
                logger.warning(f"{self}: 纯相机模式,send_action 只透传不执行。")
                self._armless_action_warned = True
            return action
        grip = action.pop("gripper.pos", None)  # 摘掉夹爪 → 官方只处理 6 臂关节(无阻塞 poll,更顺)
        sent = super().send_action(action)
        if grip is not None and getattr(self.config, "grip_follow", True):
            self._drive_gripper(float(grip))
            sent["gripper.pos"] = float(grip)
        return sent

    def _drive_gripper(self, target_deg: float) -> None:
        glo, ghi = self.config.joint_limits.get("gripper", (0.0, 270.0))
        target_deg = max(min(glo, ghi), min(max(glo, ghi), target_deg))  # 夹到夹爪限位
        if self._grip_set is None:
            self._grip_set = target_deg
        step = max(-self.config.grip_max_step_deg, min(self.config.grip_max_step_deg, target_deg - self._grip_set))
        self._grip_set += step
        gm = self.motors.get("gripper")
        if gm is not None:
            gm.send_mit(math.radians(self._grip_set), 0.0, self.config.grip_kp, self.config.grip_kd, 0.0)

    # ---------------- 退出安全:先平滑回零位,再交给父类卸力矩 ----------------
    def _return_home(self) -> None:
        freq = 30.0
        dt = 1.0 / freq
        step = max(0.3, self.config.exit_speed_deg_s * dt)  # 每周期度数
        # 只读电机(避开相机),取当前臂关节角
        for motor in self.motors.values():
            motor.request_feedback()
        try:
            self.bus.poll_feedback_once()
        except Exception:
            pass
        cur = {}
        for m in REBOT_ARM_MOTORS:
            st = self.motors[m].get_state() if m in self.motors else None
            cur[m] = math.degrees(st.pos) if (st is not None and getattr(st, "pos", None) is not None) else 0.0
        home = {m: float(self.config.exit_home_deg[i]) for i, m in enumerate(REBOT_ARM_MOTORS)}
        logger.info(f"{self}: 退出前平滑回零位(坐姿)中,手别挡…")
        for _ in range(4000):
            done = True
            act = {}
            for m in REBOT_ARM_MOTORS:
                err = home[m] - cur[m]
                if abs(err) > 0.5:
                    done = False
                    cur[m] += math.copysign(min(step, abs(err)), err)
                act[f"{m}.pos"] = cur[m]
            self.send_action(act)  # 只发臂关节(夹爪保持不动)
            if done:
                break
            time.sleep(dt)

    def disconnect(self) -> None:
        if getattr(self.config, "return_home_on_exit", False) and not self._armless and self.is_connected:
            try:
                self._return_home()
            except Exception as e:
                logger.warning(f"{self}: 回零失败(仍将卸力矩): {e}")
        if self._armless:
            # 纯相机模式:父类 disconnect 的电机/总线步骤无对象,只收相机
            for cam_name, cam in list(self.cameras.items()):
                try:
                    cam.disconnect()
                except Exception as e:
                    logger.warning(f"{self}: 相机 '{cam_name}' disconnect 报错: {e}")
            return
        try:
            super().disconnect()
        except Exception as e:
            # 父类收尾顺序是 safe_zero → 电机卸力矩/关闭 → bus.close → 相机 disconnect,
            # 最后一步掉线相机会抛错;臂的安全步骤在此之前已完成,这里只告警不阻断。
            logger.warning(f"{self}: disconnect 收尾有设备报错(多为中途掉线的相机): {e}")
