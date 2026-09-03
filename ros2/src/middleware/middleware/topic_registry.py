"""旧路径兼容:canonical 实现已移至 middleware.core.topic_registry。

本模块只做 re-export,保证 `from middleware.topic_registry import TopicRegistry`
与 `python -m middleware.topic_registry` 等历史用法不破。新代码请引用 canonical 路径。
"""

from middleware.core.topic_registry import TopicRegistry, TopicSpec, _DEFAULT_JSON  # noqa: F401
