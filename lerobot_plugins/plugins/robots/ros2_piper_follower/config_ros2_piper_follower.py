#!/usr/bin/env python
"""Config for `ros2_piper_follower` — 观测来自 middleware piper_arm_node 的 PiPER X 从臂。

与 ``ros2_rebot_follower`` 同构,区别只在 topic 前缀与关节命名:
    state  ← /piper/joint_state(joint_1..6 度 + gripper 毫米行程)
    action ← own_cmd_topic=True 时发到 /piper/joint_cmd(同语义)

硬件由 middleware 的 piper_arm_node 持有(backend=sdk 真机 / mujoco 仿真,
对本类透明),录进数据集的特征键为 joint_1.pos … joint_6.pos + gripper.pos。
"""

from dataclasses import dataclass, field

from lerobot.robots.robot import RobotConfig

PIPER_MOTORS = [f"joint_{i}" for i in range(1, 7)] + ["gripper"]


@RobotConfig.register_subclass("ros2_piper_follower")
@dataclass
class Ros2PiperFollowerConfig(RobotConfig):
    state_topic: str = "/piper/joint_state"
    cmd_topic: str = "/piper/joint_cmd"
    own_cmd_topic: bool = False
    # 帧龄上限(ms):超过回退上一帧并告警(同 ros2_rebot_follower 语义)
    stale_frame_ms: int = 200
