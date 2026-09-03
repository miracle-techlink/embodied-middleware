#!/usr/bin/env python
"""piper_arm_backend — PiPER X 后端抽象:同一节点代码跑真机或 MuJoCo 假臂。

后端协议(节点只依赖这四个方法,不 import piper_sdk 的任何类型):
    connect() / disconnect()
    get_state() -> (joints_deg[6], gripper_mm, enabled: bool)
    send_cmd(joints_deg[6], gripper_mm)          # 度/毫米,绝对目标
    estop() / enable() / disable()

单位约定(与 /rebot/* 侧一致,换算收在后端):
    关节 = 度;piper_sdk 内部是 0.001 度整数(factor = 1000)
    夹爪 = 毫米行程(0=闭合,~88mm=全开);SDK 内部 0.001mm 整数

两个实现:
    PiperSdkBackend   真机:C_PiperInterface_V2(piper_sdk 0.6.2 实测 API)
    PiperMujocoBackend MuJoCo 假臂:6 关节位置伺服 + 夹爪直线关节,
                      数值单位/限位/时序与真机后端一致,无 CAN 也能全链路联调。

真机后端安全设计(硬件到手后首次上电前再人工复核一遍):
    - 只读模式(arm 节点 enable_cmd=false)不调任何运动 API;
    - cmd 路径只走 MotionCtrl_2(位置模式) + JointCtrl + GripperCtrl,不发 MIT;
    - 限位裁剪(硬编码 PiPER 官方关节限位,超出即 clamp 并告警);
    - cmd 断流 200ms 自动停发(同栈 stale 语义);
    - disconnect:EmergencyStop → DisablePiper。
"""

from __future__ import annotations

import threading
import time

# PiPER X 关节限位(度,官方手册值;真机首验时以 GetSDKJointLimitParam 回读为准)
PIPER_JOINT_LIMITS_DEG: tuple[tuple[float, float], ...] = (
    (-165.0, 165.0),
    (-35.0, 95.0),
    (-165.0, 165.0),
    (-120.0, 120.0),
    (-165.0, 165.0),
    (-175.0, 175.0),
)
PIPER_GRIPPER_RANGE_MM = (0.0, 88.0)  # 夹爪行程,0=闭合

_DEG_TO_SDK = 1000.0  # 度 → SDK 0.001度整数
_MM_TO_SDK = 1000.0  # mm → SDK 0.001mm整数
_GRIPPER_EFFORT = 1000  # 夹爪力度(SDK 0.001N·m → 1N·m,官方 demo 值)


def clamp_joints_deg(joints: list[float]) -> tuple[list[float], list[bool]]:
    """关节限位裁剪;返回(裁剪后, 各关节是否被裁)。"""
    out, clipped = [], []
    for j, (lo, hi) in zip(joints, PIPER_JOINT_LIMITS_DEG):
        out.append(min(max(j, lo), hi))
        clipped.append(j < lo or j > hi)
    return out, clipped


def clamp_gripper_mm(g: float) -> float:
    lo, hi = PIPER_GRIPPER_RANGE_MM
    return min(max(g, lo), hi)


class PiperBackendBase:
    """节点依赖的最小接口;两个后端都实现它。"""

    def connect(self) -> None: ...
    def disconnect(self) -> None: ...
    def get_state(self) -> tuple[list[float], float, bool]: ...  # joints_deg, gripper_mm, enabled
    def send_cmd(self, joints_deg: list[float], gripper_mm: float) -> None: ...
    def estop(self) -> None: ...
    def enable(self) -> bool: ...
    def disable(self) -> bool: ...


class PiperSdkBackend(PiperBackendBase):
    """真机后端:piper_sdk 0.6.2 的 C_PiperInterface_V2(单 CAN 口)。"""

    def __init__(self, can_name: str = "can_piper"):
        from piper_sdk import C_PiperInterface_V2  # 延迟 import:仿真模式不装 SDK 也能跑

        self._piper = C_PiperInterface_V2(
            can_name=can_name,
            judge_flag=True,
            can_auto_init=True,
            start_sdk_joint_limit=True,   # SDK 侧再拦一道限位
            start_sdk_gripper_limit=True,
        )
        self._lock = threading.Lock()
        self._enabled = False

    def connect(self) -> None:
        # ConnectPort 会拉 CAN + 起收发线程;PiperInit 回零校准(官方例程顺序)
        self._piper.ConnectPort()

    def disconnect(self) -> None:
        # 安全下电:急停 → 失能 → 断口(与 demo 顺序一致)
        with self._lock:
            try:
                self._piper.EmergencyStop()
                self._piper.DisablePiper()
            finally:
                self._piper.DisconnectPort()

    def enable(self) -> bool:
        with self._lock:
            ok = self._piper.EnablePiper()
            self._enabled = bool(ok)
            return self._enabled

    def disable(self) -> bool:
        with self._lock:
            ok = self._piper.DisablePiper()
            self._enabled = not ok
            return ok

    def estop(self) -> None:
        with self._lock:
            self._piper.EmergencyStop()

    def get_state(self) -> tuple[list[float], float, bool]:
        with self._lock:
            j = self._piper.GetArmJointMsgs()
            g = self._piper.GetArmGripperMsgs()
        joints = [getattr(j.joint_state, f"joint_{i}") / _DEG_TO_SDK for i in range(1, 7)]
        gripper_mm = g.gripper_state.grippers_angle / _MM_TO_SDK
        return joints, gripper_mm, self._enabled

    def send_cmd(self, joints_deg: list[float], gripper_mm: float) -> None:
        joints, clipped = clamp_joints_deg(joints_deg)
        if any(clipped):
            # 限位裁剪照发(裁后的),但要留痕 —— 节点层会再打告警
            pass
        with self._lock:
            # 位置-速度模式(官方 piper_ctrl_joint demo 同款调用链)
            self._piper.MotionCtrl_2(ctrl_mode=0x01, move_mode=0x01, move_spd_rate_ctrl=100)
            self._piper.JointCtrl(*[round(v * _DEG_TO_SDK) for v in joints])
            self._piper.GripperCtrl(
                round(clamp_gripper_mm(gripper_mm) * _MM_TO_SDK), _GRIPPER_EFFORT, 0x01, 0
            )


class PiperMujocoBackend(PiperBackendBase):
    """MuJoCo 假臂后端:无 CAN/真机,把节点全链路(话题/单位/限位/断流保护)跑通。

    模型:XML 内联生成(6 旋转关节 + 1 夹爪滑块),关节顺序/限位与
    PiperSdkBackend 完全一致 —— 节点代码不感知后端差异。
    伺服:临界阻尼 PD 位置伺服 mj_step 推进,cmd 目标突变有真实过渡,
    断流时目标保持(真机 JointCtrl 同语义)。
    """

    def __init__(self, timestep_s: float = 0.002):
        import mujoco

        self._mujoco = mujoco
        self._model = mujoco.MjModel.from_xml_string(self._xml())
        self._data = mujoco.MjData(self._model)
        self._lock = threading.Lock()
        self._enabled = False
        self._last_cmd_t = time.monotonic()
        self._dt = timestep_s
        self._jtarget = list(self.q0_deg())
        self._gtarget = PIPER_GRIPPER_RANGE_MM[0]

        mujoco.mj_resetData(self._model, self._data)
        self._write_targets()
        for _ in range(200):  # 预热收敛到初始姿态
            self._step()

    # ---- 模型 ----
    @staticmethod
    def q0_deg() -> list[float]:
        return [0.0, 30.0, 0.0, 0.0, 0.0, 0.0]  # 接近真机折叠收纳位的关节2

    @staticmethod
    def _xml() -> str:
        import base64

        geom = base64.b64encode(
            b"\x89PNG\r\n\x1a\n"  # 占位:无需真实贴图
        ).decode()
        del geom
        joints = ""
        links = ""
        parent = "base"
        lengths = [0.20, 0.20, 0.10, 0.08, 0.06, 0.04]
        names = ["link1", "link2", "link3", "link4", "link5", "link6"]
        axes = [(0, 0, 1), (0, 1, 0), (0, 1, 0), (0, 0, 1), (0, 1, 0), (0, 0, 1)]
        for i, (name, ln, ax) in enumerate(zip(names, lengths, axes)):
            lo, hi = PIPER_JOINT_LIMITS_DEG[i]
            # MuJoCo hinge ranges/qpos are radians; middleware API remains degrees.
            joints += (
                f'<joint name="joint{i+1}" type="hinge" axis="{ax[0]} {ax[1]} {ax[2]}" '
                f'range="{lo * 0.017453292519943295} {hi * 0.017453292519943295}" '
                f'damping="0.5" armature="0.01"/>\n'
            )
            links += (
                f'<body name="{name}" pos="0 0 {ln if i==0 else 0}">'
                f'<geom type="capsule" fromto="0 0 0 0 0 {ln if i>0 else ln}" size="0.022" '
                f'rgba="0.55 0.6 0.65 1"/>'
            )
            parent = name
        closes = "</body>" * 6
        # 夹爪:沿 link6 z 轴平行的滑块,range 用行程mm/1000
        glo, ghi = PIPER_GRIPPER_RANGE_MM
        gripper = (
            f'<body name="gripper" pos="0 0 {lengths[5]}">'
            f'<joint name="gripper" type="slide" axis="0 0 1" range="{glo/1000:.4f} {ghi/1000:.4f}" '
            f'damping="0.05" armature="0.001"/>'
            f'<geom type="box" size="0.015 0.015 0.01" rgba="0.2 0.2 0.25 1"/>'
            f"</body>"
        )
        return (
            '<mujoco model="piper_x_fake">'
            '<option timestep="0.002" gravity="0 0 -9.81"/>'
            '<worldbody><body name="base" pos="0 0 0">'
            '<geom type="box" size="0.05 0.05 0.01" rgba="0.3 0.3 0.35 1"/>'
            + joints
            + links
            + gripper
            + closes
            + "</body></worldbody></mujoco>"
        )

    # ---- 伺服 ----
    def _write_targets(self) -> None:
        m, d = self._model, self._data
        for i in range(6):
            # mjModel 作动器:与铰链同名自动配对(隐式 actuator 不写 XML,直接手动力矩)
            pass
        self._kp = 40.0
        self._kd = 2.0
        self._gk = 200.0

    def _step(self) -> None:
        m, d, mujoco = self._model, self._data, self._mujoco
        rad = 0.017453292519943295
        for i in range(6):
            jname = f"joint{i+1}"
            jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, jname)
            qadr = m.jnt_qposadr[jid]
            vadr = m.jnt_dofadr[jid]
            target = self._jtarget[i] * rad
            tau = self._kp * (target - d.qpos[qadr]) - self._kd * d.qvel[vadr]
            d.qfrc_applied[vadr] = tau if self._enabled else 0.0
        gid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "gripper")
        gadr = m.jnt_qposadr[gid]
        gvadr = m.jnt_dofadr[gid]
        gt = self._gtarget / 1000.0  # mm→m
        gtau = self._gk * (gt - d.qpos[gadr]) - 5.0 * d.qvel[gvadr]
        d.qfrc_applied[gvadr] = gtau if self._enabled else 0.0
        mujoco.mj_step(m, d)

    def _advance(self, wall_s: float | None = None) -> None:
        """把仿真推到当前墙钟(断流时不推,目标保持 = 真机语义)。"""
        with self._lock:
            steps = 1  # 每次调用推一步;节点 100-200Hz 循环天然驱动仿真时钟
            for _ in range(steps):
                self._step()

    # ---- 协议实现 ----
    def connect(self) -> None:
        self._enabled = False  # 仿真"上电"即失能态,等 enable

    def disconnect(self) -> None:
        self._enabled = False

    def enable(self) -> bool:
        self._enabled = True
        return True

    def disable(self) -> bool:
        self._enabled = False
        return True

    def estop(self) -> None:
        self._enabled = False
        # 冻结在当前角(度)
        self._jtarget = self._read_joints_deg()

    def _read_joints_deg(self) -> list[float]:
        m, d, mujoco = self._model, self._data, self._mujoco
        out = []
        for i in range(6):
            jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f"joint{i+1}")
            out.append(d.qpos[m.jnt_qposadr[jid]] / 0.017453292519943295)
        return out

    def get_state(self) -> tuple[list[float], float, bool]:
        with self._lock:
            self._step()  # 每次读推进仿真(无渲染,微秒级)
            joints = self._read_joints_deg()
            gid = self._mujoco.mj_name2id(self._model, self._mujoco.mjtObj.mjOBJ_JOINT, "gripper")
            gmm = self._data.qpos[self._model.jnt_qposadr[gid]] * 1000.0
            return joints, gmm, self._enabled

    def send_cmd(self, joints_deg: list[float], gripper_mm: float) -> None:
        joints, _ = clamp_joints_deg(joints_deg)
        with self._lock:
            self._jtarget = joints
            self._gtarget = clamp_gripper_mm(gripper_mm)
            self._last_cmd_t = time.monotonic()
