#!/usr/bin/env python3
"""fleet_validate.py — 多机 profile 与代码结构的离线验证(不接触硬件)。

每项检查打印 PASS/FAIL,任何 FAIL 退出码非零。供 ssh 到任意采集机后一键跑通:
  python3 ros2/launch/fleet_validate.py [--rig <rig>]
不带 --rig 时验证 profiles/rigs/ 下全部 rig。
"""

from __future__ import annotations

import argparse
from pathlib import Path
import os
import sys

# 让本脚本在仓库任意位置都能跑
_HERE = Path(__file__).resolve()
for _cand in [_HERE.parents[2], _HERE.parents[1], *_HERE.parents]:
    if (_cand / "src/middleware/middleware").is_dir():
        sys.path.insert(0, str(_cand / "src/middleware"))
        break

from middleware.core.profiles import (  # noqa: E402
    device_names,
    find_repo_root,
    load_profile,
    topic_name,
)
from middleware.core.rig_runtime import health_streams  # noqa: E402

REQUIRED_TOP = ("rig", "modes", "devices")
REQUIRED_RIG = ("name", "namespace")
NODE_KINDS = {"arm", "leader", "camera", "control", "service"}

PASS = "\033[1;32mPASS\033[0m"
FAIL = "\033[1;31mFAIL\033[0m"

failures = 0


def check(ok: bool, label: str, detail: str = "") -> bool:
    global failures
    tag = PASS if ok else FAIL
    print(f"[{tag}] {label}" + (f" — {detail}" if detail else ""))
    if not ok:
        failures += 1
    return ok


def validate_profile(root: Path, rig: str, ros_distro: str | None) -> None:
    print(f"\n=== rig: {rig} ===")
    try:
        p = load_profile(rig, root)
    except Exception as e:  # noqa: BLE001
        check(False, "load_profile", str(e))
        return
    check(True, "load_profile", f"name={p['rig_name']}")

    for key in REQUIRED_TOP:
        check(key in p, f"顶层字段 {key}")
    for key in REQUIRED_RIG:
        check(key in p.get("rig", {}), f"rig.{key}")

    # namespace 非空且 topic_name 可用
    ns = p["rig"].get("namespace", "")
    check(bool(str(ns).strip("/")), "rig.namespace 非空", ns)
    check(topic_name(p, "x") == f"/{str(ns).strip('/')}/x", "topic_name 命名空间前缀")

    # mode 引用的设备都存在
    for mode in p.get("modes", {}):
        try:
            names = device_names(p, mode)
            check(True, f"mode[{mode}] 设备齐全", f"{len(names)} 台")
        except KeyError as e:
            check(False, f"mode[{mode}] 设备齐全", str(e))

    # 每台设备字段
    for name, dev in p.get("devices", {}).items():
        check(dev.get("kind") in NODE_KINDS, f"devices.{name}.kind", str(dev.get("kind")))
        check(bool(dev.get("node")), f"devices.{name}.node 非空", str(dev.get("node")))
        # node 模块可导入:经 ros2 环境,并注入 conda site-packages(lerobot 等在 conda env)
        node = dev.get("node")
        if node:
            import subprocess as _sp
            ros = ros_distro or "jazzy"
            py = p["rig"]["env"].get("python", sys.executable)
            conda_sp = ""
            if py and "/envs/" in py:
                import sysconfig
                v = sysconfig.get_python_version()
                conda_sp = f"{Path(py).parents[1]}/lib/python{v}/site-packages"
            extra = f"{root}/ros2/src/middleware" + (f":{conda_sp}" if conda_sp else "")
            cmd = (
                f"source /opt/ros/{ros}/setup.bash 2>/dev/null; "
                f"export PYTHONPATH=/opt/ros/{ros}/lib/python3.12/site-packages:{extra}; "
                f"{py} -c 'import importlib; importlib.import_module(\"{node}\")'"
            )
            r = _sp.run(["bash", "-c", cmd], capture_output=True, text=True, check=False)
            if r.returncode == 0:
                check(True, f"devices.{name}.node 可导入", node)
            else:
                err = r.stderr.strip().splitlines()[-1] if r.stderr.strip() else "unknown"
                check(False, f"devices.{name}.node 可导入", f"{node}: {err}")
        # health.streams 引用的 topic 必须存在
        topics = dev.get("topics", {})
        for role in dev.get("health", {}).get("streams", {}):
            check(role in topics, f"devices.{name}.health.{role} 对应 topics.{role}")

    # 健康流可解析
    for mode in p.get("modes", {}):
        try:
            streams = health_streams(p, mode)
            check(True, f"health_streams[{mode}]", f"{len(streams)} 条流")
        except Exception as e:  # noqa: BLE001
            check(False, f"health_streams[{mode}]", str(e))

    # local override 若存在可加载
    ov = root / "profiles/local/overrides.yaml"
    if ov.exists():
        try:
            load_profile(rig, root, ov)
            check(True, "local overrides.yaml 可合并")
        except Exception as e:  # noqa: BLE001
            check(False, "local overrides.yaml 可合并", str(e))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--rig", default=None, help="只验证这一个 rig;缺省验证全部")
    ap.add_argument("--repo-root", default=None)
    args = ap.parse_args()

    try:
        root = find_repo_root(args.repo_root or _HERE.parents[2])
    except FileNotFoundError as e:
        print(f"[{FAIL}] 定位仓库根: {e}")
        return 1
    print(f"仓库根: {root}")

    rigs = [args.rig] if args.rig else sorted(p.stem for p in (root / "profiles/rigs").glob("*.yaml"))
    if not rigs:
        print(f"[{FAIL}] profiles/rigs/ 下没有任何 rig")
        return 1
    for rig in rigs:
        try:
            distro = load_profile(rig, root)["rig"]["ros"].get("distro")
        except Exception:  # noqa: BLE001
            distro = None
        validate_profile(root, rig, distro)

    print(f"\n===== 结论: {'全部通过' if failures == 0 else f'{failures} 项失败'} =====")
    return min(failures, 125)


if __name__ == "__main__":
    raise SystemExit(main())
