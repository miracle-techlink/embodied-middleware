#!/usr/bin/env python3
"""piper_zero_check — PiPER 主/从臂参考零位档案与一致性自检(纯只读,不动臂)。

背景(为什么需要它):
    PiPER 链路没有 lerobot 风格的软件映射标定文件 —— 关节零位是每台臂
    固件/驱动器 EEPROM 里的绝对参考(官方 piper_set_joint_zero.py 用
    JointConfig(motor, 0xAE) 写入,掉电不丢)。因此「同一个数值 = 同一个
    物理姿态」能否跨 session / 跨天成立,取决于主、从两臂的 EEPROM 零位
    是否以同一物理参考设定、且之后没有被重设 / 换电机 / 机械变动。
    SDK 的 PiperInit() 并不回零(只发查询),别指望每次 connect 自动标定。

本工具只订阅两个 topic(从臂反馈 /piper/joint_state 100Hz、主臂位姿
/piper/leader/joint_state 事件驱动+20Hz保持),不直接碰硬件:

    sample   采样当前两臂读数(中位数),打印 主/从/差 三列 —— 主从差
             逐关节可直接看出两臂 EEPROM 零位是否对齐(固件直通下,
             静止时 leader 应 ≈ state,偏差=零位错位+跟踪残差)
    archive  把当前姿势存成参考档案(建档:两臂摆到约定的参考零位姿态,
             扶稳后采样写入 ~/.cache/huggingface/lerobot/calibration/piper_home.json)
    verify   再次采样并与档案逐关节比对(主、从各自),报漂移结论;
             同时打印当次主从对齐度(零位一致性)

判定阈值: 臂关节默认 2.0°(含固件直通跟踪残差),夹爪默认 5.0mm。
退出码: 0=通过 / 1=超差 / 2=环境失败(节点没起、无档案、数据不足)。
用法见同目录 piper_zero_check.sh(它补 ROS 环境与 PYTHONPATH)。
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

# topic / 关节名 / 单位约定(与 piper_msg_center.json、piper_leader_node 一致)
STATE_TOPIC = "/piper/joint_state"        # 从臂反馈(度 + 夹爪 mm)
LEADER_TOPIC = "/piper/leader/joint_state"  # 主臂位姿(度 + 夹爪 mm)
ARM_JOINTS = [f"joint_{i}" for i in range(1, 7)]
GRIPPER_NAME = "gripper"
DEFAULT_OUT = Path.home() / ".cache" / "huggingface" / "lerobot" / "calibration" / "piper_home.json"

TOL_DEG = 2.0    # 臂关节默认容差(度)
TOL_MM = 5.0     # 夹爪默认容差(mm)

VER = "1.0"


# ---------------------------------------------------------------------------
# 采样器:只订阅 topic,不碰硬件
# ---------------------------------------------------------------------------

class _Sampler:
    """订阅两条 topic,按关节名收集采样。ROS 依赖延迟到 __init__。"""

    def __init__(self, seconds: float):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import JointState

        rclpy.init()
        self._node = Node("piper_zero_check")
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self._data: dict[str, dict[str, list[float]]] = {"state": {}, "leader": {}}
        self._counts = {"state": 0, "leader": 0}
        self._node.create_subscription(
            JointState, STATE_TOPIC, lambda m: self._on("state", m), qos)
        self._node.create_subscription(
            JointState, LEADER_TOPIC, lambda m: self._on("leader", m), qos)
        self._seconds = seconds

    def _on(self, src: str, msg):
        self._counts[src] += 1
        for name, pos in zip(msg.name, msg.position):
            self._data[src].setdefault(name, []).append(float(pos))

    def run(self) -> dict:
        import rclpy
        deadline = time.monotonic() + self._seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.02)
        self._node.destroy_node()
        rclpy.shutdown()
        return self._finish()

    def _finish(self) -> dict:
        med, span = {}, {}
        for src in ("state", "leader"):
            med[src], span[src] = {}, {}
            for name, vals in self._data[src].items():
                s = sorted(vals)
                med[src][name] = s[len(s) // 2]          # 中位数(抗抖动)
                span[src][name] = s[-1] - s[0]           # 峰峰值(稳不稳)
        return {
            "counts": dict(self._counts),
            "median": med,
            "span": span,
        }


# ---------------------------------------------------------------------------
# 读数:主从差 / 档案比对
# ---------------------------------------------------------------------------

def _rows(med: dict) -> list[tuple[str, float | None, float | None, float | None]]:
    """按 ARM_JOINTS+gripper 固定顺序输出 (关节, state, leader, leader-state)。"""
    out = []
    for name in ARM_JOINTS + [GRIPPER_NAME]:
        s = med["state"].get(name)
        l = med["leader"].get(name)
        d = None if (s is None or l is None) else l - s
        out.append((name, s, l, d))
    return out


def _fmt(v: float | None, unit: str = "") -> str:
    return "--" if v is None else f"{v:.2f}{unit}"


def print_sample_table(med: dict, span: dict, counts: dict) -> None:
    print(f"== 采样完成:state {counts['state']} 帧 / leader {counts['leader']} 帧")
    print(f"   {'关节':<8}{'从臂state':>12}{'主臂leader':>12}{'差(主-从)':>12}   从臂峰峰")
    for name, s, l, d in _rows(med):
        peak = span["state"].get(name)
        peak_s = "--" if peak is None else f"{peak:.2f}"
        print(f"   {name:<8}{_fmt(s):>12}{_fmt(l):>12}{_fmt(d):>12}   {peak_s:>6}")
    print("   单位:joint_1..6 = 度,gripper = mm(0=闭合)。差≈0 说明主从零位对齐(误差≈固件跟踪残差)。")


def load_archive(path: Path) -> dict:
    if not path.exists():
        print(f"!! 无参考档案: {path}", file=sys.stderr)
        print("   先摆好参考零位姿态并建档: bash piper_zero_check.sh archive --label '参考姿态'",
              file=sys.stderr)
        sys.exit(2)
    return json.loads(path.read_text())


def verify_against(archive: dict, sample: dict, tol_deg: float, tol_mm: float,
                   path: Path) -> int:
    med = sample["median"]
    print(f"== 与档案比对: {path}")
    print(f"   档案: {archive.get('created','?')}  rig={archive.get('rig','?')}  "
          f"label={archive.get('pose_label','?')}")
    print(f"   {'关节':<8}{'档案state':>11}{'当前state':>11}{'漂移':>8} | "
          f"{'档案leader':>11}{'当前leader':>11}{'漂移':>8}  结论")
    bad: list[str] = []
    for name, s, l, d in _rows(med):
        tol = tol_mm if name == GRIPPER_NAME else tol_deg
        u = "mm" if name == GRIPPER_NAME else "°"
        a_s, a_l = archive.get("follower", {}).get(name), archive.get("leader", {}).get(name)
        ds = None if (a_s is None or s is None) else abs(a_s - s)
        dl = None if (a_l is None or l is None) else abs(a_l - l)
        worst = max([v for v in (ds, dl) if v is not None] or [0.0])
        ok = worst <= tol
        tag = "ok" if ok else f"超差({worst:.2f}{u}>{tol:g}{u})"
        if not ok:
            bad.append(f"{name} {tag}")
        print(f"   {name:<8}{_fmt(a_s,u):>11}{_fmt(s,u):>11}{_fmt(ds,u):>8} | "
              f"{_fmt(a_l,u):>11}{_fmt(l,u):>11}{_fmt(dl,u):>8}  {tag}")
    print("   主从对齐(本次 leader-state,应与零位一致):")
    for name, s, l, d in _rows(med):
        if d is not None:
            mark = "" if abs(d) <= (tol_mm if name == GRIPPER_NAME else tol_deg) else "  <-- 对齐超差"
            print(f"     {name:<8}{_fmt(d):>10}{mark}")
    if bad:
        print(f"\n结论: 超差 {len(bad)} 项 -> {bad}")
        print("   可能原因:两臂未摆到同一参考姿态 / EEPROM 零位漂移 / 换过电机或重设过零位。")
        print("   处理:确认参考姿态后重测;真漂移需重新标定零位(piper_set_joint_zero.py,人在场)。")
        return 1
    print("\n结论: 通过 —— 与档案一致,主从零位对齐,可以采集。")
    return 0


# ---------------------------------------------------------------------------
# 入口
# ---------------------------------------------------------------------------

def main() -> int:
    ap = argparse.ArgumentParser(description="PiPER 参考零位档案与一致性自检(只读)")
    ap.add_argument("cmd", choices=["sample", "archive", "verify"])
    ap.add_argument("--seconds", type=float, default=4.0, help="采样秒数(默认 4)")
    ap.add_argument("--out", type=Path, default=DEFAULT_OUT,
                    help=f"档案路径(默认 {DEFAULT_OUT})")
    ap.add_argument("--rig", default="piper-01")
    ap.add_argument("--label", default="", help="archive 时标注姿态名,如 '上电直立参考'")
    ap.add_argument("--tol-deg", type=float, default=TOL_DEG)
    ap.add_argument("--tol-mm", type=float, default=TOL_MM)
    args = ap.parse_args()

    if args.seconds < 1.0 or args.seconds > 120.0:
        print("!! --seconds 需在 1..120 之间", file=sys.stderr)
        return 2

    try:
        sampler = _Sampler(args.seconds)
        sample = sampler.run()
    except Exception as e:  # rclpy 起不来 / 无 ROS 环境
        print(f"!! 采样失败(节点没起或 ROS 环境缺失?): {e}", file=sys.stderr)
        return 2

    counts = sample["counts"]
    if counts["state"] == 0:
        print(f"!! {STATE_TOPIC} 无数据 —— piper_arm_node 在跑吗?", file=sys.stderr)
        return 2
    if counts["leader"] == 0:
        print(f"!! {LEADER_TOPIC} 无数据 —— piper_leader_node 在跑吗?", file=sys.stderr)
        return 2

    print_sample_table(sample["median"], sample["span"], counts)

    if args.cmd == "sample":
        print("\n(提示)摆到目标姿态扶稳后,可用 archive 建档或 verify 比对。")
        return 0

    if args.cmd == "archive":
        args.out.parent.mkdir(parents=True, exist_ok=True)
        doc = {
            "tool_version": VER,
            "kind": "piper_home_reference",
            "rig": args.rig,
            "pose_label": args.label or "(未命名参考姿态)",
            "created": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "units": {"joint": "deg", "gripper": "mm(0=闭合)"},
            "note": "PiPER 关节零位在固件/驱动器 EEPROM;本档案记录参考姿态下的读数,"
                    "用于跨 session 零位一致性自检。PiperInit 不回零,零位由 JointConfig 0xAE 设定。",
            "follower": dict(sample["median"]["state"]),
            "leader": dict(sample["median"]["leader"]),
            "alignment_leader_minus_state": {
                n: (sample["median"]["leader"].get(n, 0.0)
                    - sample["median"]["state"].get(n, 0.0))
                for n in ARM_JOINTS + [GRIPPER_NAME]
                if n in sample["median"]["leader"] and n in sample["median"]["state"]
            },
        }
        args.out.write_text(json.dumps(doc, indent=2, ensure_ascii=False))
        print(f"\n== 已建档: {args.out}")
        return 0

    # verify
    archive = load_archive(args.out)
    return verify_against(archive, sample, args.tol_deg, args.tol_mm, args.out)


if __name__ == "__main__":
    sys.exit(main())
