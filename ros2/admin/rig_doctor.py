#!/usr/bin/env python3
"""按 rig profile 做只读体检,不操作硬件或进程。"""

from __future__ import annotations

import argparse
from pathlib import Path

from middleware.core.profiles import find_repo_root, load_profile
from middleware.core.rig_runtime import health_streams, measure_hz, read_state


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("mode", nargs="?", default="teleop")
    parser.add_argument("--rig", default="rebot_starai_orbbec")
    parser.add_argument("--override", default=None)
    parser.add_argument("--repo-root", default=None)
    args = parser.parse_args()

    root = find_repo_root(args.repo_root or Path(__file__).resolve().parents[2])
    profile = load_profile(args.rig, root, args.override)
    if args.mode not in profile["modes"]:
        raise KeyError(f"未知 mode={args.mode!r},profile 可用: {sorted(profile['modes'])}")

    raw, running = read_state(profile, root)
    failures = 0
    print(f"===== rig 体检 [{args.rig}:{args.mode}] =====")
    if raw and raw.get("mode") != args.mode:
        print(f"! 状态文件模式={raw.get('mode')},本次检查模式={args.mode}")
    for name in profile["modes"][args.mode]:
        pid = running.get(name)
        if pid:
            print(f"✓ 节点 {name} pid={pid}")
        else:
            print(f"✗ 节点 {name} 未由本 rig supervisor 管理或已经退出")
            failures += 1

    for name, topic, minimum in health_streams(profile, args.mode):
        hz = measure_hz(profile, root, topic)
        if hz >= minimum:
            print(f"✓ {name} {topic} {hz:.1f}Hz (最低 {minimum:g})")
        else:
            print(f"✗ {name} {topic} {hz:.1f}Hz (最低 {minimum:g})")
            failures += 1

    print(f"===== 结论: {'通过' if failures == 0 else f'{failures} 项失败'} =====")
    return min(failures, 125)


if __name__ == "__main__":
    raise SystemExit(main())
