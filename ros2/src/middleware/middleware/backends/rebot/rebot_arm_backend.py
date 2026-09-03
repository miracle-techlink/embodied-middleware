#!/usr/bin/env python
"""rebot_arm_backend — reBot B601-RS 从臂的硬件后端薄壳。

为什么单独一层:与 PiperBackendBase 对齐同一协议(connect/disconnect/get_state/
send_cmd/estop),让 rebot_arm_node 与 piper_arm_node 在 backends/ 层完全对称——
节点只剩 ROS 话题逻辑,硬件差异全部收在 backends/<厂商>/ 里。

薄壳不复制任何行为,全部透传给 lerobot 的 RebotFollower(cameras={}):
关节限位夹取、夹爪直驱(motor 7 MIT 位置控制)、退出前平滑回零、相机缺失容错……
这些精调过的实现一行不搬。本层只做两件事:
    1) 把 RebotFollower 的 dict 接口(observation/action)适配成 piper 同款的
       (joints_deg[6]+gripper, ...) 元组协议;
    2) 持有 CAN 总线锁——状态发布定时器与 cmd 订阅回调都会碰总线,
       lerobot 实现本身非线程安全,用一把锁串行化(原在节点里,移入后端)。

单位约定(与 /rebot/* 消息层一致,piper 侧同口径):
    关节 position=度、velocity=度/秒、effort=力矩;cmd=绝对目标角(度)。
    gripper 用度(与 RebotFollower 的 gripper.pos 口径一致,0~270°,不同于
    piper 的毫米行程——两支臂的夹爪单位各自贴各自硬件,消息层关节名做区分)。
"""

from __future__ import annotations

import threading

REBOT_JOINT_NAMES = [
    "shoulder_pan", "shoulder_lift", "elbow_flex",
    "wrist_flex", "wrist_yaw", "wrist_roll", "gripper",
]


class RebotArmBackend:
    """RebotFollower 的薄壳:协议与 PiperBackendBase 对齐,行为零重写。

    connect(calibrate=True) 沿用 lerobot 交互标定;disconnect 内部平滑回零+卸力矩
    (RebotFollower.disconnect 的既有行为,SIGTERM 安全由节点层保证)。
    """

    def __init__(self, can_port: str = "can0", robot_id: str = "follower1"):
        self._can_port = can_port
        self._robot_id = robot_id
        self._lock = threading.Lock()  # 串行化所有 CAN 访问;节点不再持锁
        self._robot = None

    # ---- 协议实现(锁在本层,与 PiperSdkBackend 一致) ----
    def connect(self) -> None:
        from lerobot.robots.rebot_follower import RebotFollower, RebotFollowerConfig

        cfg = RebotFollowerConfig(port=self._can_port, id=self._robot_id, cameras={})
        robot = RebotFollower(cfg)
        with self._lock:
            # 标定文件缺失时会走交互标定(与 lerobot 一致)
            robot.connect(calibrate=True)
            self._robot = robot

    def disconnect(self) -> None:
        # disconnect 内部:先平滑回零(坐姿)再卸力矩 —— 进程被杀时机械臂不砸落。
        with self._lock:
            if self._robot is not None and self._robot.is_connected:
                self._robot.disconnect()
            self._robot = None

    def get_state(self) -> tuple[list[float], list[float], list[float]]:
        """(position_deg, velocity_dps, effort_torque) 按 REBOT_JOINT_NAMES 顺序。"""
        with self._lock:
            obs = self._robot.get_observation()  # cameras={} → 只有电机反馈(非阻塞路径)
        pos, vel, eff = [], [], []
        for name in REBOT_JOINT_NAMES:
            pos.append(float(obs.get(f"{name}.pos", 0.0)))
            vel.append(float(obs.get(f"{name}.vel", 0.0)))
            eff.append(float(obs.get(f"{name}.torque", 0.0)))
        return pos, vel, eff

    def send_cmd(self, joints_deg: list[float]) -> None:
        """7 关节绝对目标角(度),顺序同 REBOT_JOINT_NAMES;限位/夹爪直驱在 RebotFollower 内。"""
        action = {f"{name}.pos": float(p) for name, p in zip(REBOT_JOINT_NAMES, joints_deg)}
        with self._lock:
            self._robot.send_action(action)

    @property
    def is_connected(self) -> bool:
        return self._robot is not None and self._robot.is_connected
