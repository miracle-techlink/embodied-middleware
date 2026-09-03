#!/usr/bin/env python
"""Config for `ros2_rebot_teleop` — 镜像 teleop_map_node 的 cmd 流作为 action。

消息中心模式下,真正产生动作的是 teleop_map_node(100Hz 自持)。本 teleop 只是
订阅 /rebot/follower/joint_cmd 把"实际下发给臂的目标"喂给 record 循环,保证
数据集里的 action 与从臂真实执行的指令同帧。rearm_ramp() 转发到 /rebot/teleop/rearm。
"""

from dataclasses import dataclass

from lerobot.teleoperators.config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("ros2_rebot_teleop")
@dataclass
class Ros2RebotTeleopConfig(TeleoperatorConfig):
    cmd_topic: str = "/rebot/follower/joint_cmd"
    rearm_topic: str = "/rebot/teleop/rearm"
    # cmd 帧龄上限(ms):超过视为控制回路断流,get_action 抛错而不是录陈旧目标
    stale_ms: int = 500
