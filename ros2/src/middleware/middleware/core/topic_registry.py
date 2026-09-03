#!/usr/bin/env python
"""JSON 话题注册表 — 照 AgilexCobotMagic msg_center 的 schema 声明本机所有 ROS2 topic。

各节点启动时从注册表读自己的 topic 名/类型/fps,而不是硬编码字符串:
新增设备 = 改 JSON + 写一个节点,消费侧代码零改动。

用法:
    reg = TopicRegistry()                       # 默认读包内 config/rebot_single_arm.json
    reg = TopicRegistry("/path/to/other.json")
    spec = reg.require("/rebot/leader/joint_state")   # 不存在/未 enable 会抛错
    spec.topic_name, spec.type_name, spec.default_fps
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

# core/ 在包内深一层:parents[2] = src/middleware(包根),config/ 与其同级
_DEFAULT_JSON = Path(__file__).resolve().parents[2] / "config" / "rebot_single_arm.json"


@dataclass(frozen=True)
class TopicSpec:
    topic_name: str
    type_name: str
    default_fps: float
    producer: str = ""
    description: str = ""


class TopicRegistry:
    def __init__(self, json_path: str | Path | None = None):
        path = Path(json_path) if json_path else _DEFAULT_JSON
        with open(path, encoding="utf-8") as f:
            raw = json.load(f)
        self._topics: dict[str, TopicSpec] = {}
        for entry in raw.get("ros_topics", []):
            if not entry.get("enable", False):
                continue
            self._topics[entry["topic_name"]] = TopicSpec(
                topic_name=entry["topic_name"],
                type_name=entry["type_name"],
                default_fps=float(entry.get("default_fps", 0)),
                producer=entry.get("producer", ""),
                description=entry.get("description", ""),
            )

    def require(self, topic_name: str) -> TopicSpec:
        """取一个已 enable 的 topic 声明;没有就报错(防止节点间 topic 名拼写漂移)。"""
        if topic_name not in self._topics:
            raise KeyError(f"topic 未在注册表 enable: {topic_name}(现有: {sorted(self._topics)})")
        return self._topics[topic_name]

    def by_producer(self, producer: str) -> list[TopicSpec]:
        return [t for t in self._topics.values() if t.producer == producer]

    def all(self) -> list[TopicSpec]:
        return list(self._topics.values())
