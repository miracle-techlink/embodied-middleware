#!/usr/bin/env python
"""rigctl profile 子命令:show/validate/list。"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from .profiles import device_names, find_repo_root, load_profile, topic_name

KINDS = {"arm", "leader", "camera", "control", "service"}


def validate(profile: dict) -> list[str]:
    errors: list[str] = []
    for section in ("rig", "modes", "devices"):
        if section not in profile:
            errors.append(f"缺少顶层字段: {section}")
    if errors:
        return errors
    if not profile["rig"].get("namespace"):
        errors.append("rig.namespace 不能为空")
    for name, dev in profile["devices"].items():
        if dev.get("kind") not in KINDS:
            errors.append(f"devices.{name}.kind 非法: {dev.get('kind')!r}")
        if not dev.get("node"):
            errors.append(f"devices.{name}.node 不能为空")
        for role, suffix in dev.get("topics", {}).items():
            if not isinstance(suffix, str) or not suffix.strip("/"):
                errors.append(f"devices.{name}.topics.{role} 非法")
    for mode in profile["modes"]:
        try:
            device_names(profile, mode)
        except KeyError as e:
            errors.append(str(e))
    return errors


def main() -> int:
    parser = argparse.ArgumentParser(prog="rigctl-profile")
    parser.add_argument("action", choices=("list", "show", "validate"))
    parser.add_argument("--rig", default="rebot_starai_orbbec")
    parser.add_argument("--override", default=None)
    args = parser.parse_args()
    root = find_repo_root()
    if args.action == "list":
        for p in sorted((root / "profiles" / "rigs").glob("*.yaml")):
            print(p.stem)
        return 0
    profile = load_profile(args.rig, root, args.override)
    errors = validate(profile)
    if args.action == "validate":
        if errors:
            for error in errors:
                print(f"ERROR: {error}")
            return 1
        print(f"OK: {args.rig} ({len(profile['devices'])} devices, modes={list(profile['modes'])})")
        for name, dev in profile["devices"].items():
            topics = {role: topic_name(profile, suffix) for role, suffix in dev.get("topics", {}).items()}
            print(f"  {name}: {dev['kind']} {dev['node']} {topics}")
        return 0
    print(json.dumps(profile, indent=2, ensure_ascii=False))
    return 1 if errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
