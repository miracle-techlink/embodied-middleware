#!/usr/bin/env python3
"""profile 驱动的数据流看门狗。

check 只读检查;heal 发现异常后按 rig 精确停止整组节点并重新启动。
arm shutdown 始终走 SIGTERM;超时后不发送 SIGKILL。
"""

from __future__ import annotations

import argparse
from pathlib import Path
import signal
import subprocess
import time

from middleware.core.profiles import find_repo_root, load_profile
from middleware.core.rig_runtime import health_streams, measure_hz, read_state


def failures(profile: dict, root: Path, mode: str, quiet: bool = False) -> list[str]:
    failed = []
    _, running = read_state(profile, root)
    for name in profile["modes"][mode]:
        if name not in running:
            failed.append(f"node:{name}")
            if not quiet:
                print(f"死 node:{name}")
    for name, topic, minimum in health_streams(profile, mode):
        hz = measure_hz(profile, root, topic)
        if hz < minimum:
            failed.append(f"stream:{name}:{topic}")
        if not quiet:
            state = "活" if hz >= minimum else "死"
            print(f"{state} {name} {topic} {hz:.1f}Hz / 最低 {minimum:g}")
    return failed


def launcher(root: Path) -> Path:
    ros2_root = root / "ros2" if (root / "ros2").is_dir() else root
    return ros2_root / "start_rig.sh"


def heal(profile: dict, root: Path, rig: str, mode: str, override: str | None) -> bool:
    cmd = [str(launcher(root)), "stop", "--rig", rig, "--repo-root", str(root)]
    if override:
        cmd.extend(["--override", override])
    subprocess.run(cmd, check=False)

    start = [str(launcher(root)), mode, "--rig", rig, "--repo-root", str(root)]
    if override:
        start.extend(["--override", override])
    log_dir = root / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)
    log = open(log_dir / f"watchdog_{rig}.log", "ab")
    proc = subprocess.Popen(start, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    log.close()
    time.sleep(max((float(profile["devices"][n].get("warmup_s", 0)) for n in profile["modes"][mode]), default=0) + 8)
    return proc.poll() is None


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("action", nargs="?", choices=("check", "heal", "watch"), default="check")
    parser.add_argument("--rig", default="rebot_starai_orbbec")
    parser.add_argument("--mode", default="infer")
    parser.add_argument("--override", default=None)
    parser.add_argument("--repo-root", default=None)
    parser.add_argument("--interval", type=float, default=15)
    args = parser.parse_args()

    root = find_repo_root(args.repo_root or Path(__file__).resolve().parents[2])
    profile = load_profile(args.rig, root, args.override)
    if args.mode not in profile["modes"]:
        raise KeyError(f"未知 mode={args.mode!r},profile 可用: {sorted(profile['modes'])}")

    if args.action == "check":
        return min(len(failures(profile, root, args.mode)), 125)
    if args.action == "heal":
        failed = failures(profile, root, args.mode)
        if not failed:
            print("全部健康,无需修复")
            return 0
        print(f"发现 {len(failed)} 项异常;整 rig 优雅重启: {failed}")
        if not heal(profile, root, args.rig, args.mode, args.override):
            return 1
        return min(len(failures(profile, root, args.mode)), 125)

    running = True

    def stop_watch(*_):
        nonlocal running
        running = False

    signal.signal(signal.SIGTERM, stop_watch)
    signal.signal(signal.SIGINT, stop_watch)
    while running:
        failed = failures(profile, root, args.mode, quiet=True)
        if failed:
            print(f"发现异常,开始修复: {failed}", flush=True)
            heal(profile, root, args.rig, args.mode, args.override)
        time.sleep(args.interval)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
