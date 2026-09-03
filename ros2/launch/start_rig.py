#!/usr/bin/env python3
"""start_rig.py — profile 驱动的消息中心启动器(canonical 新入口)。

旧 start_msg_center.sh 保持兼容;新机器/新设备优先走这里。
只负责按 profile 启停节点;所有停止都是按 rig 状态精确 SIGTERM,绝不 SIGKILL。
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
from middleware.core.rig_runtime import (
    clear_state,
    process_alive,
    read_state,
    state_path,
    write_state,
)

STOP_TIMEOUT_S = 45


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


def stop_pid(pid: int, name: str, timeout_s: int = STOP_TIMEOUT_S) -> bool:
    if not process_alive(pid):
        return True
    os.kill(pid, signal.SIGTERM)
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if not process_alive(pid):
            return True
        time.sleep(0.2)
    print(f"警告: {name}(pid={pid}) 在 {timeout_s}s 内未退出;为保护机械臂,不发送 SIGKILL", file=sys.stderr)
    return False


def stop(profile: dict, repo_root: Path) -> int:
    raw, procs = read_state(profile, repo_root)
    if not procs:
        print(f"{profile['rig_name']}: 没有运行中的 rig 状态")
        return 0
    # 逆启动顺序停:teleop_map 先停,arm 最后停(让臂平滑回零)。
    mode = raw.get("mode")
    mode_order = list(reversed(profile["modes"][mode])) if mode in profile["modes"] else []
    ordered = [name for name in mode_order if name in procs]
    ordered.extend(name for name in procs if name not in ordered)
    failed = False
    for name in ordered:
        failed = not stop_pid(procs[name], name) or failed
    if not failed:
        clear_state(profile)
    return 1 if failed else 0


def launch(profile: dict, repo_root: Path, mode: str) -> int:
    names = profile["modes"][mode]
    py = profile["rig"]["env"]["python"]
    logs = repo_root / "logs" / f"sess_{time.strftime('%Y%m%d_%H%M%S')}_{mode}"
    logs.mkdir(parents=True, exist_ok=True)
    latest = repo_root / "logs" / "latest"
    if latest.is_symlink():
        latest.unlink()
    elif latest.exists():
        raise RuntimeError(f"日志 latest 已存在且不是软链: {latest}")
    latest.symlink_to(logs)

    _, existing = read_state(profile, repo_root)
    if existing:
        raise RuntimeError(f"{profile['rig_name']} 已在运行: {existing};先执行 stop")

    procs: dict[str, subprocess.Popen] = {}
    handles = []
    shutdown = False

    def stop_all(*_):
        nonlocal shutdown
        if shutdown:
            return
        shutdown = True
        for name in reversed(names):
            proc = procs.get(name)
            if proc and proc.poll() is None:
                proc.terminate()
        deadline = time.monotonic() + STOP_TIMEOUT_S
        for name in reversed(names):
            proc = procs.get(name)
            if proc:
                remain = max(0, deadline - time.monotonic())
                try:
                    proc.wait(timeout=remain)
                except subprocess.TimeoutExpired:
                    print(f"警告: {name}(pid={proc.pid}) 在超时内未退出;为保护机械臂,不发送 SIGKILL", file=sys.stderr)
        raise SystemExit(0)

    signal.signal(signal.SIGTERM, stop_all)
    signal.signal(signal.SIGINT, stop_all)
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
                cmd.append("--ros-args")
                for key, value in params.items():
                    cmd.extend(["-p", f"{key}:={value}"])
            log = open(logs / f"{name}.log", "ab")
            handles.append(log)
            env = ros_env(profile, repo_root)
            # cwd 固定到仓库根,便于 /proc 校验与相对 registry_json 配置。
            procs[name] = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, env=env, cwd=repo_root)
            if dev.get("kind") == "camera":
                time.sleep(float(dev.get("warmup_s", 0)))
        state = {name: proc.pid for name, proc in procs.items()}
        write_state(profile, repo_root, mode, state)
        print(f"已启动 {profile['rig_name']}:{mode} -> {names}")
        while not shutdown:
            for name in names:
                proc = procs[name]
                if proc.poll() is not None:
                    print(f"节点退出: {name}(pid={proc.pid}, rc={proc.returncode});按安全规则整组 SIGTERM", file=sys.stderr)
                    stop_all()
            time.sleep(1)
    finally:
        for handle in handles:
            handle.close()
        clear_state(profile)
    return 0


def main() -> int:
    args = parse_args()
    repo_hint = args.repo_root or Path(__file__).resolve().parents[2]
    repo_root = find_repo_root(repo_hint)
    profile = load_profile(args.rig, repo_root, args.override)
    if args.mode == "stop":
        return stop(profile, repo_root)
    if args.mode not in profile["modes"]:
        raise KeyError(f"未知 mode={args.mode!r},profile 可用: {sorted(profile['modes'])}")
    return launch(profile, repo_root, args.mode)


if __name__ == "__main__":
    raise SystemExit(main())
