#!/usr/bin/env python
"""Config for `ros2_piper_teleop` — 订阅主臂位姿 topic 作为 action 源。

Piper 主臂走固件直通(MasterSlaveConfig 0xFA),不产生 ROS 控制指令流;
middleware 的 ``piper_leader_node`` 被动监听主臂联动帧(0x155/156/157/159)
还原主臂位姿,发到 /piper/leader/joint_state。本 teleop 订阅该 topic,把
**主臂此刻的位姿**喂给 record 循环当 action —— 这正是"操作员想让从臂到哪儿"。

与 ros2_rebot_teleop 的区别:rebot 的 action 是 teleop_map 下发的从臂目标
(经映射+ramp),piper 的 action 是主臂原始位姿(固件直通,无中间映射)。
两者语义一致 —— action = 操作员意图;且因主从同款臂、固件直驱,
主臂位姿 ≈ 从臂实际位姿(忽略从臂跟踪延迟)。

注意:主臂不掰不发帧(事件驱动),静置时本 teleop 读到的是最后一次值 —
— 静止语义正确,stale_ms 判断的是"leader 节点是否还活着",不是"主臂是否在动"。
"""

from dataclasses import dataclass

from lerobot.teleoperators.config import TeleoperatorConfig

PIPER_MOTORS = [f"joint_{i}" for i in range(1, 7)] + ["gripper"]


@TeleoperatorConfig.register_subclass("ros2_piper_teleop")
@dataclass
class Ros2PiperTeleopConfig(TeleoperatorConfig):
    state_topic: str = "/piper/leader/joint_state"
    # 帧龄上限(ms):主臂事件驱动,静置时不发新帧,所以这里判断的是 leader 节点活性,
    # 给得比 rebot 宽 —— 只要 piper_leader_node 在跑,哪怕主臂不动也算健康。
    # (节点活性由 follower 的 observation 流兜底,真正"录到静止"是合法的)
    stale_ms: int = 5000
