#!/usr/bin/env python3
"""follow_check.py — 主从跟随链路启动自检(自动模式,不动臂)。

由 start_teleop.sh 起完节点后调用:采样约 SECONDS 秒,逐关节断言
「主臂动 → 映射出 → 从臂到」。判定逻辑在 middleware/core/follow_check.py
(与交互模式 joint_sweep 共用)。

主臂在采样窗内静止的关节,映射段无从验证 —— 不算失败,只验「恒定目标下
从臂到位」并单独计数;有未验证关节时退出码 0 但打印提示,建议跑交互模式
joint_sweep.py 逐关节摆一遍(那是唯一能验方向对错的方式)。

用法:
    python3 follow_check.py                 # 采样 4s
    SECONDS=6 python3 follow_check.py       # 采样窗加长

退出码: 0=无失败 / 1=有关节明确失败 / 2=topic 段级失败
"""

from __future__ import annotations

import os
import sys

import rclpy

from middleware.core.follow_check import FollowSampler, evaluate

SECONDS = float(os.environ.get("SECONDS", "4"))


def main() -> int:
    rclpy.init()
    sampler = FollowSampler()
    print(f"[follow_check] 采样 {SECONDS:.0f}s(leader/cmd/state)…", flush=True)
    sampler.sample_for(SECONDS)

    result = evaluate(sampler)

    if result.fatal:
        print(f"[follow_check] ✗ {result.fatal}", flush=True)
        sampler.destroy_node()
        rclpy.shutdown()
        return 2

    for v in result.verdicts:
        if v.ok and v.verified:
            print(f"[follow_check] ✓ {v.joint} 跟随正常", flush=True)
        elif v.ok:
            print(f"[follow_check] ~ {v.joint} {v.reason}", flush=True)
        else:
            print(f"[follow_check] ✗ {v.joint} — {v.reason}", flush=True)

    bad = [v.joint for v in result.verdicts if not v.ok]
    n_unverified = sum(1 for v in result.verdicts if v.ok and not v.verified)

    if bad:
        print(
            f"[follow_check] ✗ {len(bad)} 个关节失败: {', '.join(bad)}。"
            f"逐关节定位: bash tools/diagnostics/joint_sweep.sh",
            flush=True,
        )
    elif n_unverified:
        print(
            f"[follow_check] ✓ 跟踪正常;但 {n_unverified} 个关节主臂静止、映射未验证"
            f"(摆一遍主臂或跑 joint_sweep.sh 补验)",
            flush=True,
        )
    else:
        print("[follow_check] ✓ 6 关节全部跟随正常", flush=True)

    sampler.destroy_node()
    rclpy.shutdown()
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
