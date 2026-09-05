#!/usr/bin/env python3
"""follow_check — 主从跟随链路自检核心(采样/判定),供两处复用:

    自动模式:   ros2/runtime/follow_check.py   (start_teleop.sh 收尾调用)
    交互模式:   tools/diagnostics/joint_sweep.py

判定的不是「管道通不通」(fleet_validate/rig_doctor 管那个),而是
「主臂动 → 映射出 → 从臂到」这条端到端数据正确性:

    leader 动了吗     —— 主臂舵机离线/关节恒死 → leader 方差≈0
    cmd 响应了吗      —— 映射参数错(flip/home/顺序) → cmd 恒压在 home 不动
    从臂到了吗        —— 限位夹死/电机掉线 → state 追不上 cmd

全部只订阅 topic,不直接碰硬件;单位按注册表约定(度)。

夹爪刻意不查:闭合端有 clamp 过冲设计(cmd 故意压过闭合位产生夹持力),
|cmd-state| 大是设计行为,按跟踪判会误报。
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

if TYPE_CHECKING:  # 仅为类型注解;运行期不导入 ROS
    from sensor_msgs.msg import JointState

# rclpy/sensor_msgs 只在 FollowSampler 里用到,全部延迟到其 __init__ 内导入 ——
# 本模块 import 不依赖 ROS,evaluate 判定逻辑不起 ROS、不碰硬件即可单测
# (见 tools/diagnostics 的假数据用例;类型注解经 __future__.annotations 均为字符串)。

# 默认 topic(可被 profile/参数覆盖);单位均为度(见 rebot_single_arm.json)
LEADER_TOPIC = "/rebot/leader/joint_state"
CMD_TOPIC = "/rebot/follower/joint_cmd"
STATE_TOPIC = "/rebot/follower/joint_state"

ARM_JOINTS = 6  # 臂关节数(夹爪第 7 维单独看)

REBOT_JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_yaw",
    "wrist_roll",
]


@dataclass
class JointTrace:
    """单个关节的一条采样轨迹。"""

    name: str
    samples: list[float] = field(default_factory=list)

    @property
    def span(self) -> float:
        """峰峰值(度)。"""
        if len(self.samples) < 2:
            return 0.0
        return max(self.samples) - min(self.samples)

    @property
    def last(self) -> float:
        return self.samples[-1] if self.samples else float("nan")


class FollowSampler:
    """订阅三条 topic,按名字对齐采样各关节轨迹。内部持有一个最小 ROS Node。

    不直接继承 rclpy Node —— 那样类定义时就要求 rclpy 可导入,违背本模块
    「判定逻辑纯 Python、不起 ROS 可单测」的设计。ROS 依赖全部收进 __init__。
    """

    def __init__(
        self,
        leader_topic: str = LEADER_TOPIC,
        cmd_topic: str = CMD_TOPIC,
        state_topic: str = STATE_TOPIC,
    ):
        import rclpy  # 延迟导入:见文件头说明
        from rclpy.node import Node
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import JointState

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT
        )
        self._node = Node("follow_check_sampler")
        self._traces: dict[str, dict[str, JointTrace]] = {
            "leader": {},
            "cmd": {},
            "state": {},
        }
        self._node.create_subscription(
            JointState, leader_topic, lambda m: self._on("leader", m), qos
        )
        self._node.create_subscription(
            JointState, cmd_topic, lambda m: self._on("cmd", m), qos
        )
        self._node.create_subscription(
            JointState, state_topic, lambda m: self._on("state", m), qos
        )
        self._counts = {"leader": 0, "cmd": 0, "state": 0}

    def destroy_node(self):
        self._node.destroy_node()

    def _on(self, src: str, msg: JointState):
        self._counts[src] += 1
        for name, pos in zip(msg.name, msg.position):
            tr = self._traces[src].setdefault(name, JointTrace(name))
            tr.samples.append(float(pos))

    def sample_for(self, seconds: float):
        """采集 seconds 秒。"""
        import rclpy

        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.02)

    # ---------------- 取数 ----------------
    def count(self, src: str) -> int:
        return self._counts[src]

    def trace(self, src: str, name: str) -> JointTrace | None:
        return self._traces[src].get(name)

    def names(self, src: str) -> list[str]:
        return list(self._traces[src].keys())


@dataclass
class JointVerdict:
    joint: str
    ok: bool
    verified: bool  # True=主臂动了、映射也验过;False=主臂静止,只验了跟踪
    reason: str  # 失败原因(给人看)


@dataclass
class CheckResult:
    ok: bool
    verdicts: list[JointVerdict]
    fatal: str  # 链路段级失败(topic 没数据等),空=无

    @property
    def n_verified(self) -> int:
        return sum(1 for v in self.verdicts if v.verified)


def _leader_joint_name(i: int) -> str:
    return f"joint_{i + 1}"


def evaluate(
    sampler: FollowSampler,
    *,
    leader_move_deg: float = 2.0,
    cmd_response_deg: float = 1.0,
    track_err_deg: float = 15.0,
) -> CheckResult:
    """对已采样的轨迹做逐关节判定。

    阈值(度):
      leader_move_deg   主臂该关节峰峰值 ≥ 此 → 视为「动了」,可做全三段判定
      cmd_response_deg  主臂动了但 cmd 该关节峰峰值低于此 → 映射没输出
      track_err_deg     |cmd - state| 最新值差超过此 → 从臂没跟上(限位/掉线)

    主臂静止的关节不算失败(操作员没动 ≠ 舵机离线):映射段无从验证,
    只验「恒定目标下 state 追上 cmd」,并在 verdict.verified 里标记。
    """
    for src, topic in (("leader", LEADER_TOPIC), ("cmd", CMD_TOPIC), ("state", STATE_TOPIC)):
        if sampler.count(src) == 0:
            return CheckResult(
                ok=False,
                verdicts=[],
                fatal=f"{topic} 无数据 —— 对应节点没起或没发布",
            )

    verdicts: list[JointVerdict] = []
    for i, rj in enumerate(REBOT_JOINTS):
        lj = _leader_joint_name(i)
        lt = sampler.trace("leader", lj)
        ct = sampler.trace("cmd", rj)
        st = sampler.trace("state", rj)

        if lt is None or not lt.samples:
            verdicts.append(JointVerdict(rj, False, False, f"leader 缺 {lj}(主臂消息缺关节)"))
            continue
        if ct is None or not ct.samples:
            verdicts.append(JointVerdict(rj, False, False, f"cmd 缺 {rj}(映射没发这个关节)"))
            continue
        if st is None or not st.samples:
            verdicts.append(JointVerdict(rj, False, False, f"state 缺 {rj}(从臂没回报)"))
            continue

        leader_moved = lt.span >= leader_move_deg

        if leader_moved:
            # 全三段:主臂动 → 映射出 → 从臂到
            if ct.span < cmd_response_deg:
                verdicts.append(
                    JointVerdict(
                        rj, False, False,
                        f"主臂 {lj} 动了 {lt.span:.1f}° 但 cmd {rj} 只动 {ct.span:.2f}°:映射没输出(flip/home/顺序错)",
                    )
                )
                continue
            err = abs(ct.last - st.last)
            if err > track_err_deg:
                verdicts.append(
                    JointVerdict(
                        rj, False, False,
                        f"cmd {ct.last:.1f}° 与 state {st.last:.1f}° 差 {err:.1f}°:从臂没跟上(限位夹死/电机掉线)",
                    )
                )
                continue
            verdicts.append(JointVerdict(rj, True, True, ""))
        else:
            # 主臂静止:只验恒定目标下的跟踪;但 cmd 自己在大动而 leader 没动 = 异常
            if ct.span > track_err_deg:
                verdicts.append(
                    JointVerdict(
                        rj, False, False,
                        f"主臂 {lj} 静止(峰峰值 {lt.span:.2f}°)但 cmd {rj} 在动({ct.span:.1f}°):映射基准/冻结状态异常",
                    )
                )
                continue
            err = abs(ct.last - st.last)
            if err > track_err_deg:
                verdicts.append(
                    JointVerdict(
                        rj, False, False,
                        f"恒定目标下从臂没到位:cmd {ct.last:.1f}° vs state {st.last:.1f}° 差 {err:.1f}°(电机掉线/限位夹死)",
                    )
                )
                continue
            verdicts.append(JointVerdict(rj, True, False, "主臂静止,映射未验证"))

    return CheckResult(ok=all(v.ok for v in verdicts), verdicts=verdicts, fatal="")
