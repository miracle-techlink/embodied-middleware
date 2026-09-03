"""运行期状态与健康检查公共函数。

状态文件只记录本 rig 由 start_rig 启动的 PID;操作前同时核对 node module 和 cwd,
避免误停同机其他 rig 或无关 Python 进程。
"""

from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess

from .profiles import topic_name

STATE_DIR = Path("/tmp/middleware_rigs")


def state_path(profile: dict) -> Path:
    return STATE_DIR / f"{profile['rig_name']}.json"


def process_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def process_matches(pid: int, profile: dict, name: str, repo_root: Path) -> bool:
    try:
        text = Path(f"/proc/{pid}/cmdline").read_bytes().decode(errors="ignore")
        cwd = Path(f"/proc/{pid}/cwd").resolve()
    except OSError:
        return False
    node = profile["devices"][name].get("node")
    return bool(node) and node in text and cwd == repo_root


def write_state(profile: dict, repo_root: Path, mode: str, procs: dict[str, int]) -> None:
    STATE_DIR.mkdir(parents=True, exist_ok=True)
    payload = {
        "rig": profile["rig_name"],
        "namespace": profile["rig"]["namespace"],
        "mode": mode,
        "repo_root": str(repo_root),
        "procs": procs,
    }
    path = state_path(profile)
    tmp = path.with_suffix(".tmp")
    tmp.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n")
    tmp.replace(path)


def clear_state(profile: dict) -> None:
    state_path(profile).unlink(missing_ok=True)


def read_state(profile: dict, repo_root: Path) -> tuple[dict, dict[str, int]]:
    path = state_path(profile)
    try:
        raw = json.loads(path.read_text())
    except (OSError, json.JSONDecodeError):
        return {}, {}
    if raw.get("rig") != profile["rig_name"] or raw.get("repo_root") != str(repo_root):
        return raw, {}
    procs: dict[str, int] = {}
    for name, pid in raw.get("procs", {}).items():
        if name in profile["devices"] and isinstance(pid, int) and pid > 0:
            if process_alive(pid) and process_matches(pid, profile, name, repo_root):
                procs[name] = pid
    return raw, procs


def health_streams(profile: dict, mode: str) -> list[tuple[str, str, float]]:
    streams = []
    for name in profile["modes"][mode]:
        dev = profile["devices"][name]
        for role, spec in dev.get("health", {}).get("streams", {}).items():
            suffix = dev.get("topics", {}).get(role)
            if suffix:
                streams.append((name, topic_name(profile, suffix), float(spec.get("min_hz", 0))))
    return streams


def measure_hz(profile: dict, repo_root: Path, topic: str, duration_s: float = 2.0) -> float:
    ros2_root = repo_root / "ros2" if (repo_root / "ros2").is_dir() else repo_root
    probe = ros2_root / "admin/rebot_rate.py"
    env = dict(os.environ)
    distro = profile["rig"]["ros"]["distro"]
    env["PYTHONPATH"] = f"/opt/ros/{distro}/lib/python3.12/site-packages"
    env["ROS_DOMAIN_ID"] = str(profile["rig"]["ros"].get("domain_id", 0))
    result = subprocess.run(
        [profile["rig"]["env"]["python"], str(probe), topic, str(duration_s)],
        capture_output=True,
        text=True,
        timeout=duration_s + 6,
        env=env,
        check=False,
    )
    try:
        return float(result.stdout.strip())
    except ValueError:
        return 0.0
