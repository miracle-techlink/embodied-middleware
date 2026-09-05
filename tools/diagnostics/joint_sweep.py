#!/usr/bin/env python3
"""joint_sweep.py — 主从跟随逐关节交互自检(遥操作前跑,约 30 秒)。

自动模式(follow_check.py)只能判「动不动」,判不了「方向对不对」——flip 取反
从数据上看从臂照样在动,只是反了。所以方向对错必须让人指认。本脚本逐关节:

    1. 提示「请缓慢摆动主臂 joint_i,观察从臂 <rebot 关节名>」
    2. 采样数秒,断言:leader 动了 → cmd 跟着动了 → 从臂 state 追上了
    3. 问「从臂该关节动了吗?方向对吗?」(y=对 / n=动了但反了或错了 / s=没动)

任一步失败立即打印定位(主臂离线 / 映射没输出 / 限位夹死 / 方向反),最后给汇总。

用法(消息中心已起,teleop 模式):
    PY=~/miniconda3/envs/data_collect/bin/python bash joint_sweep.sh
    # 或直接:
    ~/miniconda3/envs/data_collect/bin/python tools/diagnostics/joint_sweep.py
"""

from __future__ import annotations

import sys

import rclpy

from middleware.core.follow_check import (
    REBOT_JOINTS,
    FollowSampler,
    _leader_joint_name,
)

# 每关节采样窗(秒):人摆一个来回够了
SWEEP_SECONDS = 5.0
# 判定阈值(度),比自动模式略宽(人摆得慢)
LEADER_MOVE_DEG = 3.0
CMD_RESPONSE_DEG = 1.5
TRACK_ERR_DEG = 18.0


def ask(prompt: str) -> str:
    try:
        return input(prompt).strip().lower()
    except EOFError:
        return "q"


def check_joint(sampler: FollowSampler, i: int) -> tuple[bool, str]:
    """对第 i 个关节做人机联合判定。返回 (通过?, 失败定位)。"""
    lj = _leader_joint_name(i)
    rj = REBOT_JOINTS[i]

    input(f"\n[{i + 1}/6] 回车后,缓慢摆动主臂 {lj},眼睛看从臂「{rj}」…")
    sampler.sample_for(SWEEP_SECONDS)

    lt = sampler.trace("leader", lj)
    ct = sampler.trace("cmd", rj)
    st = sampler.trace("state", rj)

    if lt is None or lt.span < LEADER_MOVE_DEG:
        return False, f"主臂 {lj} 没读到运动(峰峰值 {lt.span if lt else 0:.2f}°):舵机离线/关节恒死"
    if ct is None or ct.span < CMD_RESPONSE_DEG:
        return False, f"主臂动了 {lt.span:.1f}° 但 cmd 只动 {ct.span if ct else 0:.2f}°:映射没输出(flip/home/顺序错)"
    err = abs(ct.last - st.last) if st and st.samples else float("nan")
    if st is None or not st.samples:
        return False, "从臂没有 state 回报"
    if err > TRACK_ERR_DEG:
        return False, f"cmd {ct.last:.1f}° 与从臂 {st.last:.1f}° 差 {err:.1f}°:从臂没跟上(限位夹死/电机掉线)"

    # 数据通了,方向交给人
    while True:
        a = ask(f"    从臂「{rj}」动了吗、方向对吗? [y=对 / n=反了或错了 / s=没动] ")
        if a in ("y", ""):
            return True, ""
        if a == "n":
            return False, f"数据通但方向/对应错:flip 里第 {i + 1} 个关节符号要取反"
        if a == "s":
            return False, "数据通但从臂没动:看从臂该关节电机/限位"
        if a == "q":
            print("中止。")
            sys.exit(2)
        print("    输入 y / n / s / q")


def main() -> int:
    rclpy.init()
    sampler = FollowSampler()

    # 先确认三条 topic 有数据,否则逐关节没意义
    print("[joint_sweep] 等 topic 就绪…", flush=True)
    sampler.sample_for(2.0)
    for src, label in (("leader", "主臂"), ("cmd", "映射"), ("state", "从臂")):
        if sampler.count(src) == 0:
            print(f"[joint_sweep] ✗ {label} topic 无数据,先起消息中心(start_teleop.sh)", flush=True)
            sampler.destroy_node()
            rclpy.shutdown()
            return 2
    print("[joint_sweep] topic 就绪。接下来逐关节摆主臂、看从臂。", flush=True)

    results: list[tuple[str, bool, str]] = []
    for i in range(6):
        ok, why = check_joint(sampler, i)
        results.append((REBOT_JOINTS[i], ok, why))
        print(f"    {'✓' if ok else '✗'} {REBOT_JOINTS[i]}{('' if ok else ' — ' + why)}", flush=True)

    print("\n===== 汇总 =====")
    bad = [(n, w) for n, ok, w in results if not ok]
    if not bad:
        print("✓ 6 个关节全部:主臂动 → 映射出 → 从臂到 → 方向对")
    else:
        for n, w in bad:
            print(f"✗ {n}: {w}")

    sampler.destroy_node()
    rclpy.shutdown()
    return 0 if not bad else 1


if __name__ == "__main__":
    sys.exit(main())
