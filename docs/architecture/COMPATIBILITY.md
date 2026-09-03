# 兼容性与不变契约

本次目录整理不改变设备链路，只改变代码归属。以下契约是后续分类/接入新硬件的红线。

## 运行入口

- `~/middleware/start_teleop.sh`、`start_infer.sh`、`start_msg_center.sh`
- `record_ros2.sh`、`record_rebot_gated_ros2.sh`、`watchdog.sh`
- `rebot_doctor.sh`、`rebot_watch.sh`、`rebot_rate.py`、`rebot_enable.py`、`rebot_go_home.py`

根入口是兼容 wrapper, canonical 实现在 `launch/`、`runtime/`、`admin/`。
脚本参数、环境变量、日志目录和 SIGTERM 行为必须保持不变;不得用 `kill -9` 关闭机械臂。

## Python 入口

旧模块路径继续可导入:

- `middleware.topic_registry`
- `middleware.nodes.rebot_arm_node`
- `middleware.nodes.piper_arm_node`
- `middleware.nodes.piper_arm_backend`
- `middleware.nodes.starai_leader_node`
- `middleware.nodes.orbbec_node`
- `middleware.nodes.uvc_node`
- `middleware.nodes.teleop_map_node`
- `middleware.nodes.msg_center_bench`

新代码应依职责使用 `middleware.core`、`middleware.backends.piper`、
`middleware.nodes.arms/leaders/cameras/control/benchmarks`。ROS2 console script 名仍为
`rebot_arm_node`、`piper_arm_node`、`orbbec_node` 等。

## 数据契约

- `/rebot/...` topic 前缀不变;现有 topic 名、消息 schema、外部数据集字段不变。
- 数据流保持 `KEEP_LAST depth=1 + BEST_EFFORT`;开关类 topic 的可靠性不因移动目录改变。
- PiPER registry 与 rebot registry 分离,后端的度/mm 换算不外泄到消息层。
- 旧入口 wrapper 不复制业务逻辑,canonical 实现只保留一份。

## 验证规则

每次新增分类层都要通过:

```bash
source /opt/ros/jazzy/setup.bash
export PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages:$HOME/middleware/src/middleware
python -m compileall -q src/middleware
python -c 'from middleware.core.topic_registry import TopicRegistry; print(len(TopicRegistry().all()))'
python -c 'from middleware.topic_registry import TopicRegistry; print(len(TopicRegistry().all()))'
```

实机回归需另行按设备验收顺序执行;目录整理阶段不主动启动机械臂、相机或 CAN。
