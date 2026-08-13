#!/usr/bin/env python
# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

import json
import logging
import struct
import threading
import time
from pathlib import Path

import numpy as np
import serial

from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from ..teleoperator import Teleoperator
from .config_starai_violin_leader import StaraiViolinLeaderConfig

logger = logging.getLogger(__name__)


class StaraiViolinLeader(Teleoperator):
    """StarAI Violin leader arm read over the Fashionstar UART servo bus.

    Calibration records BOTH a zero (homing) pose AND each joint's range of motion.
    ``get_action`` outputs a NORMALIZED position per joint, centered on the zero pose
    and scaled by that joint's half-range → ~[-1, 1] (keys ``joint_1.pos`` ...
    ``joint_6.pos`` + ``gripper.pos``). The follower maps this into its own
    zero+half-range. Raw angle readings are unwrapped during the range sweep so a
    servo crossing the ±180° boundary does not corrupt the recorded range.
    """

    config_class = StaraiViolinLeaderConfig
    name = "starai_violin_leader"

    def __init__(self, config: StaraiViolinLeaderConfig):
        # all set before super().__init__ (may auto-load calibration)
        self._home = None       # zero-pose raw angles (deg)
        self._range_min = None  # per-joint min (deg, unwrapped, home frame)
        self._range_max = None
        super().__init__(config)
        self.config = config
        self._uart: serial.Serial | None = None
        self._uservo = None
        self._servo_ids = [*config.arm_servo_ids, config.gripper_servo_id]
        self._keys = [f"joint_{i + 1}.pos" for i in range(len(config.arm_servo_ids))] + ["gripper.pos"]

    @property
    def action_features(self) -> dict[str, type]:
        return {k: float for k in self._keys}

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self._uart is not None and self._uart.is_open

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        # PyPI 包是 fashionstar-uart-sdk(import 路径 fashionstar_uart_sdk.uservo);
        # 老的手动拷贝方式是裸 uservo.py。两种都兼容。
        try:
            from uservo import Packet, UartServoManager, UartServoInfo
        except ImportError:
            from fashionstar_uart_sdk.uservo import Packet, UartServoManager, UartServoInfo
        self._Packet = Packet

        self._uart = serial.Serial(
            port=self.config.port, baudrate=self.config.baudrate,
            parity=serial.PARITY_NONE, stopbits=1, bytesize=8, timeout=0,
        )
        self._uservo = UartServoManager(self._uart, srv_num=self.config.gripper_servo_id + 1)
        # 新版 PyPI 包(fashionstar_uart_sdk)构造时预填全部 254 个 ID 的幻影舵机,
        # query_all_srv_angle 会逐个查询 -> 每个幻影都干等超时,扫一圈要几分钟,
        # 标定扫限位/遥操作读角度全被拖死。裁剪到配置的 ID,并用 ping 做真实在线检查。
        # (老版裸 uservo.py 只登记扫到的舵机,可能缺项,先补齐再裁剪。)
        for sid in self._servo_ids:
            if sid not in self._uservo.servos:
                self._uservo.servos[sid] = UartServoInfo(sid)
        self._uservo.servos = {sid: self._uservo.servos[sid] for sid in self._servo_ids}
        missing = [sid for sid in self._servo_ids if not self._uservo.ping(sid)]
        if missing:
            raise ConnectionError(
                f"{self} connected but servos {missing} not responding "
                f"(expect {self._servo_ids}). Check power/baudrate ({self.config.baudrate})."
            )
        found = set(self._servo_ids)
        # 后台读取线程:全速轮询舵机角度,控制循环只取最新缓存 ——
        # 臂盒 MCU 管家轮询造成的总线碰撞/丢包不再进入控制循环(同相机 async_read 的设计)。
        # 附带收益:我们持续占线时 MCU 会载波监听让路(实测孤立泛洪查询零丢包),
        # 空闲等待反而是丢包的温床。
        self._latest_deg = None
        self._latest_lock = threading.Lock()
        self._reader_stop = threading.Event()
        self._reader = threading.Thread(target=self._reader_loop, daemon=True,
                                        name=f"{self.config.id}-reader")
        self._reader.start()
        t0 = time.time()
        while self._latest_deg is None and time.time() - t0 < 2.0:
            time.sleep(0.01)
        if self._latest_deg is None:
            raise ConnectionError(f"{self} no servo angles received within 2s.")
        if not self.is_calibrated and calibrate:
            logger.info(f"{self} not calibrated; running calibration.")
            self.calibrate()
        logger.info(f"{self} connected, servos: {sorted(found)}")

    def _reader_loop(self) -> None:
        """后台全速轮询:逐舵机查询(短等),读到即更新缓存;丢了立刻下一个。
        缓存存**连续展开角**(unwrap ±180):舵机原始角在 ±180 回绕,挥臂过线时
        直接给目标会造成 360° 瞬跳(遥操作随机顿挫的实锤根因,17/18 异常帧全是)。
        展开条件:两次应答间物理转动 <180°(丢包窗口 ~300ms 内 560°/s 挥臂≈170°,满足)。
        同时维护每关节速度估计,供 _read_raw_deg 做外插(消除采样保持台阶)。"""
        us = self._uservo
        n = len(self._servo_ids)
        latest = np.zeros(n)               # 连续展开角
        last_t = np.full(n, -1e9)          # 每关节最新应答时刻
        vel = np.zeros(n)                  # 每关节速度估计 (deg/s)
        first = [True] * n
        filled = set()
        try:
            while not self._reader_stop.is_set():
                for i, sid in enumerate(self._servo_ids):
                    if self._reader_stop.is_set():
                        return
                    us.send_request(us.CODE_QUERY_SERVO_ANGLE, struct.pack("<B", sid))
                    t0 = time.time()
                    got = False
                    while not got and time.time() - t0 < 0.006:
                        w = us.uart.in_waiting
                        if w:
                            for b in us.uart.read(w):
                                us.pkt_buffer.update(struct.pack("<B", b))
                            while us.pkt_buffer.has_valid_packet():
                                code, params = self._Packet.unpack(us.pkt_buffer.get_packet())
                                handler = us.response_handle_funcs.get(code)
                                if handler:
                                    handler(params)
                                if code == us.CODE_QUERY_SERVO_ANGLE and params and params[0] == sid:
                                    got = True
                        else:
                            time.sleep(0.0002)
                    if got:
                        now = time.time()
                        new_ang = float(us.servos[sid].angle)   # 原始角,±180 回绕
                        if first[i]:
                            latest[i] = new_ang
                            first[i] = False
                        else:
                            # 相对"已展开的最新值"求回绕安全增量,再累计 —— 展开不断链
                            d = (new_ang - latest[i] + 180.0) % 360.0 - 180.0
                            dt = now - last_t[i]
                            if 0.003 < dt < 0.5:
                                v = max(-500.0, min(500.0, d / dt))
                                vel[i] = 0.6 * vel[i] + 0.4 * v
                            latest[i] += d
                        last_t[i] = now
                        filled.add(i)
                        if len(filled) == n:
                            with self._latest_lock:
                                self._latest_deg = latest.copy()
                                self._latest_ts = last_t.copy()
                                self._latest_vel = vel.copy()
        except Exception as e:  # uart 在 disconnect 时被关,线程静默退出
            if not self._reader_stop.is_set():
                logger.warning(f"{self} reader loop died: {e}")

    @property
    def is_calibrated(self) -> bool:
        return self._home is not None and self._range_min is not None and self._range_max is not None

    def _read_raw_deg(self) -> np.ndarray:
        # 取后台线程缓存 + 按速度外插到"现在"。采样保持(20~100ms 随机台阶)是
        # 快速运动时随机卡顿的来源;外插把台阶补成连续轨迹。外插窗口钳 60ms,
        # 超时(长陈旧)退回保持,避免方向反转时过冲甩出去。
        while True:
            with self._latest_lock:
                if self._latest_deg is not None:
                    age = time.time() - self._latest_ts
                    horizon = np.minimum(age, 0.06)
                    return self._latest_deg + self._latest_vel * horizon
            time.sleep(0.005)  # 仅在 connect 后首轮缓存就绪前会走到

    def calibrate(self) -> None:
        input(f"\n[{self}] ① 零位:把主臂移到【零位姿态】(与从臂零位对应),扶稳后按 Enter...")
        self._home = self._read_raw_deg()
        print(f"零位记录: {np.round(self._home, 1).tolist()}")

        input("② 限位:按 Enter 开始,然后把【每个关节都缓慢转到两端极限】来回扫一遍...")
        print("记录中... 所有关节都转满后按 Enter 结束。")
        cont = self._home.copy()          # continuous (unwrapped) angle, starts at home
        prev = self._read_raw_deg()
        mins = self._home.copy()
        maxs = self._home.copy()
        done = threading.Event()
        threading.Thread(target=lambda: (input(), done.set()), daemon=True).start()
        while not done.is_set():
            raw = self._read_raw_deg()
            cont = cont + (((raw - prev + 180.0) % 360.0) - 180.0)  # unwrap step
            prev = raw
            mins = np.minimum(mins, cont)
            maxs = np.maximum(maxs, cont)
            print("  min: " + " ".join(f"{m:+6.1f}" for m in mins)
                  + " | max: " + " ".join(f"{m:+6.1f}" for m in maxs), end="\r")
            time.sleep(0.02)
        self._range_min, self._range_max = mins, maxs
        self._save_calibration()
        print(f"\n标定完成,已保存到 {self.calibration_fpath}")
        logger.info(f"{self} home={np.round(self._home,1).tolist()} "
                    f"min={np.round(mins,1).tolist()} max={np.round(maxs,1).tolist()}")

    def configure(self) -> None:
        pass

    def _load_calibration(self, fpath: Path | None = None) -> None:
        fpath = self.calibration_fpath if fpath is None else fpath
        with open(fpath) as f:
            d = json.load(f)
        self._home = np.asarray(d["homing_offset_deg"], dtype=float)
        self._range_min = np.asarray(d["range_min_deg"], dtype=float)
        self._range_max = np.asarray(d["range_max_deg"], dtype=float)

    def _save_calibration(self, fpath: Path | None = None) -> None:
        fpath = self.calibration_fpath if fpath is None else fpath
        with open(fpath, "w") as f:
            json.dump({"homing_offset_deg": self._home.tolist(),
                       "range_min_deg": self._range_min.tolist(),
                       "range_max_deg": self._range_max.tolist()}, f, indent=4)

    @check_if_not_connected
    def get_action(self) -> dict[str, float]:
        raw = self._read_raw_deg()  # 连续展开角(±180 已 unwrap)
        # 直接做差即"相对零位的连续角度"。不能再取模回 [-180,180] —— 那会把
        # 物理上过回绕线的连续运动折成 360° 瞬跳(遥操作随机顿挫的根因)。
        rel = raw - self._home
        # Arm joints: emit degrees-from-home (1:1 direct-angle mapping downstream).
        out = {k: float(rel[i]) for i, k in enumerate(self._keys[:-1])}
        # Gripper (last key): emit travel FRACTION [0,1] over its full range (its home
        # may sit at an end, so a centered [-1,1] would only use half the travel).
        gi = len(self._keys) - 1
        span_g = max(self._range_max[gi] - self._range_min[gi], 1e-3)
        frac = (raw[gi] - self._range_min[gi]) / span_g
        out[self._keys[gi]] = float(np.clip(frac, 0.0, 1.0))
        return out

    def send_feedback(self, feedback: dict[str, float]) -> None:
        pass

    def disconnect(self) -> None:
        if getattr(self, "_reader_stop", None) is not None:
            self._reader_stop.set()
            reader = getattr(self, "_reader", None)
            if reader is not None and reader.is_alive():
                reader.join(timeout=1.0)
        if self._uart is not None and self._uart.is_open:
            self._uart.close()
        self._uart = None
        self._uservo = None
        logger.info(f"{self} disconnected.")
