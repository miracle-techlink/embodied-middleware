#!/usr/bin/env python3
"""start_rig.py — profile 驱动的消息中心启动器(canonical 新入口)。

旧 start_msg_center.sh 保持兼容;新机器/新设备优先走这里。
只负责启动与 SIGTERM 优雅停止,不复制设备逻辑。
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from middleware.core.profiles import find_repo_root, load_profile


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("mode", help="profile.modes 中的模式名,或 stop")
    parser.add_argument("--rig", default="rebot_starai_orbbec")
    parser.add_argument("--override", default=None)
    parser.add_argument("--repo-root", default=None)
    return parser.parse_args()


def source_root(repo_root: Path) -> Path:
    for candidate in (repo_root / "src/middleware", repo_root / "ros2/src/middleware"):
        if (candidate / "middleware").is_dir():
            return candidate
    raise FileNotFoundError(f"找不到 middleware Python 包: {repo_root}")


def ros_env(profile: dict, repo_root: Path) -> dict:
    env = dict(os.environ)
    distro = profile["rig"]["ros"]["distro"]
    paths = [f"/opt/ros/{distro}/lib/python3.12/site-packages", str(source_root(repo_root))]
    if env.get("PYTHONPATH"):
        paths.append(env["PYTHONPATH"])
    env["PYTHONPATH"] = ":".join(paths)
    env["ROS_DOMAIN_ID"] = str(profile["rig"]["ros"].get("domain_id", 0))
    return env


def main() -> int:
    args = parse_args()
    repo_hint = args.repo_root or Path(__file__).resolve().parents[2]
    profile = load_profile(args.rig, find_repo_root(repo_hint), args.override)
    repo_root = Path(profile["repo_root"])
    py = profile["rig"]["env"]["python"]
    if args.mode == "stop":
        subprocess.run(["pkill", "-TERM", "-f", "middleware.nodes"], check=False)
        return 0
    if args.mode not in profile["modes"]:
        raise KeyError(f"未知 mode={args.mode!r},profile 可用: {sorted(profile['modes'])}")
    names = profile["modes"][args.mode]
    procs = []
    logs = repo_root / "logs" / f"sess_{time.strftime('%Y%m%d_%H%M%S')}_{args.mode}"
    logs.mkdir(parents=True, exist_ok=True)
    try:
        for name in names:
            dev = profile["devices"][name]
            pre = dev.get("pre_start", {})
            if pre.get("usb_reset"):
                reset_script = source_root(repo_root) / "scripts/usb_reset.py"
                subprocess.run(["python3", str(reset_script), pre["usb_reset"]], check=False)
            cmd = [py, "-m", dev["node"]]
            params = dev.get("params", {})
            if params:
                cmd.extend(["--ros-args"])
                for key, value in params.items():
                    cmd.extend(["-p", f"{key}:={value}"])
            log = open(logs / f"{name}.log", "ab")
            procs.append((name, subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, env=ros_env(profile, repo_root))))
            time.sleep(float(dev.get("warmup_s", 0) if dev.get("kind") == "camera" else 0))
        print(f"已启动 {args.rig}:{args.mode} -> {[name for name, _ in procs]}")
        def stop_all(*_):
            for _, proc in procs:
                if proc.poll() is None:
                    proc.terminate()
            for _, proc in procs:
                try:
                    proc.wait(timeout=45)
                except subprocess.TimeoutExpired:
                    print(f"警告: {proc.args} 在 45 秒内未退出;为保护机械臂,不发送 SIGKILL", file=sys.stderr)
            raise SystemExit(0)
        signal.signal(signal.SIGTERM, stop_all)
        signal.signal(signal.SIGINT, stop_all)
        for _, proc in procs:
            proc.wait()
        return 0
    finally:
        for _, proc in procs:
            if proc.poll() is None:
                proc.terminate()


if __name__ == "__main__":
    raise SystemExit(main())
