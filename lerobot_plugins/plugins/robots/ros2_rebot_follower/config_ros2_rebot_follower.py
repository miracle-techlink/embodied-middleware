#!/usr/bin/env python
"""Config for `ros2_rebot_follower` — 数据源走 rebot_msg_center(ROS2)的 reBot 从臂。

与 ``rebot_follower`` 的唯一区别是数据来源:不碰 CAN/相机硬件,观测全部订阅自
rebot_msg_center 的 topic(arm 节点发的 joint_state、相机节点发的压缩图)。
数据集特征与 ``rebot_follower`` 完全一致(7 关节 + 腕彩/腕深/前视)。

``own_cmd_topic``: False(默认,消息中心模式)= teleop_map_node 独立持有控制回路,
send_action 不再转发(只回显);True = 无 map 节点时(如 policy rollout 走 record),
send_action 直接把 action 发到 /rebot/follower/joint_cmd。
"""

from dataclasses import dataclass, field

from lerobot.robots.robot import RobotConfig

REBOT_MOTORS = [
    "shoulder_pan", "shoulder_lift", "elbow_flex",
    "wrist_flex", "wrist_yaw", "wrist_roll", "gripper",
]


@dataclass
class Ros2CamSpec:
    topic: str
    kind: str  # "color" | "depth"
    width: int = 640
    height: int = 480
    fps: int = 30  # RobotConfig.__post_init__ 会查这个字段


@RobotConfig.register_subclass("ros2_rebot_follower")
@dataclass
class Ros2RebotFollowerConfig(RobotConfig):
    registry_json: str = ""  # 空 = 用 rebot_msg_center 包内默认 JSON
    state_topic: str = "/rebot/follower/joint_state"
    cmd_topic: str = "/rebot/follower/joint_cmd"
    own_cmd_topic: bool = False
    # 帧龄上限(ms):超过回退上一帧并告警(同 rebot_follower stale_frame_ms 语义)
    stale_frame_ms: int = 200
    cameras: dict[str, Ros2CamSpec] = field(
        default_factory=lambda: {
            "wrist": Ros2CamSpec("/rebot/wrist/color/compressed", "color"),
            "wrist_depth": Ros2CamSpec("/rebot/wrist/depth/compressed", "depth"),
            "front": Ros2CamSpec("/rebot/front/color/compressed", "color"),
        }
    )
