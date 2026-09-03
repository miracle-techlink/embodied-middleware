#!/usr/bin/env python
"""profile loader — 多机采集平台的机器配置解析。

优先级:内置 profile → profiles/local/overrides.yaml → 调用方显式覆盖。
只读配置,不接触硬件。
"""

from __future__ import annotations

from copy import deepcopy
from pathlib import Path
import os

import yaml

REPO_ROOT_MARKERS = ("profiles", "src/middleware", "launch")


def find_repo_root(start: str | Path | None = None) -> Path:
    roots = [Path(start).resolve()] if start is not None else [Path.cwd().resolve(), Path(__file__).resolve()]
    candidates = []
    for root in roots:
        for candidate in (root, *root.parents):
            if candidate not in candidates:
                candidates.append(candidate)
    for cand in candidates:
        if all((cand / marker).exists() for marker in REPO_ROOT_MARKERS):
            return cand
    # 兼容归档树布局:仓库根/ros2/src/middleware/middleware/core/profiles.py
    for cand in candidates:
        if (cand / "profiles").exists() and (cand / "ros2/src/middleware").exists():
            return cand
    raise FileNotFoundError(f"找不到 middleware 仓库根(从 {roots[0]} 或模块路径向上搜索)")


def _merge_dict(base: dict, override: dict) -> dict:
    out = deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = _merge_dict(out[key], value)
        else:
            out[key] = value
    return out


def _expand(value):
    if isinstance(value, dict):
        return {k: _expand(v) for k, v in value.items()}
    if isinstance(value, list):
        return [_expand(v) for v in value]
    if isinstance(value, str):
        return os.path.expandvars(os.path.expanduser(value))
    return value


def load_profile(rig: str, repo_root: str | Path | None = None, local_override: str | Path | None = None) -> dict:
    root = find_repo_root(repo_root)
    profile_path = root / "profiles" / "rigs" / f"{rig}.yaml"
    if not profile_path.exists():
        raise FileNotFoundError(f"rig profile 不存在: {profile_path}")
    profile = yaml.safe_load(profile_path.read_text()) or {}
    override_path = Path(local_override).expanduser() if local_override else root / "profiles" / "local" / "overrides.yaml"
    if override_path.exists():
        profile = _merge_dict(profile, yaml.safe_load(override_path.read_text()) or {})
    profile = _expand(profile)
    profile["repo_root"] = str(root)
    profile["rig_name"] = rig
    return profile


def topic_name(profile: dict, suffix: str) -> str:
    namespace = str(profile["rig"]["namespace"]).strip("/")
    suffix = suffix.lstrip("/")
    return f"/{namespace}/{suffix}"


def device_names(profile: dict, mode: str) -> list[str]:
    try:
        names = list(profile["modes"][mode])
    except KeyError as e:
        raise KeyError(f"profile 缺少 mode={mode}(现有: {sorted(profile['modes'])})") from e
    devices = profile.get("devices", {})
    missing = [name for name in names if name not in devices]
    if missing:
        raise KeyError(f"mode={mode} 引用了未定义设备: {missing}")
    return names
