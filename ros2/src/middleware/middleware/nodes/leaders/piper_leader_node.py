#!/usr/bin/env python
"""PiPER 主臂(teaching/leader)ROS2 producer —— 监听主臂固件联动帧还原主臂位置。

Piper 主从直通的物理事实(2026-09-03 实测 can0):
    主臂配成固件主臂模式(MasterSlaveConfig 0xFA)后,**不发周期性位置反馈**,
    只在被掰动、位置变化超阈值时,把新目标以**联动控制帧**直接发给从臂:
        0x155 = J1,J2   0x156 = J3,J4   0x157 = J5,J6   (各两个 int32 LE, 0.001°)
        0x159 = 夹爪 (int32 LE 0.001mm + effort)
        0x151 = 模式心跳 (ctrl/move/spd/mit)
    这些帧"发"在总线上,目标是让从臂跟随;本节点只是**被动监听**同一总线,
    把主臂发出去的目标位置还原出来,就是主臂此刻的位姿。

    ⚠️ 与从臂的区别:从臂(piper_arm_node)走 piper_sdk 读 2Ax 周期反馈,100Hz 连续;
    主臂**不掰不发**,所以本节点是事件驱动 —— 收到联动帧就更新并发布一帧,
    没收到就保持上一次值。静置时主臂位姿不漂,数据集里 action 静止是正确语义。

话题:
    publish /piper/leader/joint_state  sensor_msgs/JointState (事件驱动,非定频)
        name = joint_1..joint_6 + gripper
        position = 度(joint) + 毫米(gripper),与 /piper/joint_state 同单位
    frame_id = "piper_leader" (便于 teleop 插件识别)

只在总线上读、不写任何控制帧,对主从联动零干扰。
"""

from __future__ import annotations

import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

# BEST_EFFORT 与 piper_arm_node 一致,teleop 插件用同款 QoS 订阅
QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
)
JOINT_NAMES = [f"joint_{i}" for i in range(1, 7)] + ["gripper"]

# 主臂联动帧 CAN ID → (帧内第几个 int32 槽位对应哪个关节索引)
# 0x155: [J1(0:4), J2(4:8)]  0x156: [J3, J4]  0x157: [J5, J6]  0x159: [gripper_angle, effort]
_JOINT_FRAME_IDS = (0x155, 0x156, 0x157)
_GRIPPER_FRAME_ID = 0x159


def _i32_be(b: bytes, off: int) -> int:
    """Piper CAN 帧是大端(2026-09-03 实测对照从臂 2Ax 真值确认)。"""
    return int.from_bytes(b[off : off + 4], "big", signed=True)


class PiperLeaderNode(Node):
    """被动监听主臂联动帧,还原并发布主臂位姿。事件驱动,不主动轮询。"""

    def __init__(self):
        super().__init__("piper_leader_node")
        self.declare_parameter("can_name", "can0")
        self.declare_parameter("topic_name", "/piper/leader/joint_state")
        # 是否要求至少收满一帧完整 7 关节才开始发布(避免开局发半空向量)
        self.declare_parameter("require_full_first", True)
        # 启动种子:主臂不掰不发帧,lerobot teleop connect 需已有首帧。
        # seed_from_slave=True 时,启动 1.5s 内没收到主臂帧就从从臂 2Ax 反馈读当前位姿播种子
        # (主从同位是合理假设:主臂联动正驱动从臂),之后真实主臂帧一到即覆盖。
        self.declare_parameter("seed_from_slave", True)

        can_name = str(self.get_parameter("can_name").value)
        topic = str(self.get_parameter("topic_name").value)
        self._require_full = bool(self.get_parameter("require_full_first").value)
        self._seed_from_slave = bool(self.get_parameter("seed_from_slave").value)

        self._lock = threading.Lock()
        self._joints = [0.0] * 6          # 度
        self._gripper = 0.0               # 毫米
        self._seen = [False] * 7          # 6 关节 + 夹爪 是否已各收到至少一帧
        self._frame_count = 0
        self._seeded = False

        self._pub = self.create_publisher(JointState, topic, QOS)
        self._can_name = can_name
        self._bus = None
        self._rx_thread = None
        self._running = False

        self._open_bus()
        if self._seed_from_slave:
            self.create_timer(1.5, self._seed_once)  # 一次性:等总线反馈稳定后播种子
        self.create_timer(1.0 / 20.0, self._republish_hold)  # 20Hz 周期重发保持值
        self.get_logger().info(
            f"PiPER leader 监听 {can_name} 联动帧(0x155/156/157/159)→ {topic} "
            f"(事件驱动;require_full_first={self._require_full}, seed_from_slave={self._seed_from_slave})"
        )

    def _seed_once(self):
        """启动 1.5s 后若还没收到任何主臂帧,用从臂 2Ax 反馈播种子位姿。只跑一次。"""
        if self._seeded:
            return
        self._seeded = True
        with self._lock:
            already = any(self._seen)
        if already:
            self.get_logger().info("已收到主臂帧,跳过种子")
            return
        # 从从臂 2Ax 反馈读当前位姿(主从同位假设)
        try:
            import can
            bus = can.interface.Bus(channel=self._can_name, interface="socketcan")
            latest = {}
            t0 = self.get_clock().now()
            import time as _t
            end = _t.monotonic() + 1.0
            while _t.monotonic() < end and len(latest) < 4:
                m = bus.recv(timeout=0.2)
                if m and m.arbitration_id in (0x2A5, 0x2A6, 0x2A7, 0x2A8):
                    latest[m.arbitration_id] = bytes(m.data)
            bus.shutdown()
            if 0x2A5 in latest and 0x2A6 in latest and 0x2A7 in latest:
                j = []
                for a in (0x2A5, 0x2A6, 0x2A7):
                    d = latest[a]
                    j.append(_i32_be(d, 0) / 1000.0)
                    j.append(_i32_be(d, 4) / 1000.0)
                grip = _i32_be(latest[0x2A8], 0) / 1000.0 if 0x2A8 in latest else 0.0
                with self._lock:
                    self._joints = j
                    self._gripper = grip
                    self._seen = [True] * 7
                self._publish()
                self.get_logger().info(
                    f"种子位姿已发布(来自从臂 2Ax): {[round(x,1) for x in j]} + 夹爪{grip:.1f}mm"
                )
            else:
                self.get_logger().warning("种子失败:没抓到从臂 2Ax 反馈帧,仍等主臂掰动")
        except Exception as e:
            self.get_logger().warning(f"种子失败: {e},仍等主臂掰动")

    # ---------------- CAN 总线 ----------------
    def _open_bus(self):
        import can

        self._bus = can.interface.Bus(channel=self._can_name, interface="socketcan")
        self._running = True
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

    def _rx_loop(self):
        while self._running and self._bus is not None:
            try:
                msg = self._bus.recv(timeout=0.5)
            except Exception as e:  # 总线瞬断,别让线程死
                self.get_logger().warning(f"CAN recv 异常: {e}", throttle_duration_sec=2.0)
                continue
            if msg is None:
                continue
            self._handle_frame(msg.arbitration_id, bytes(msg.data))

    def _handle_frame(self, arb_id: int, data: bytes):
        if len(data) < 8:
            return
        updated = False
        with self._lock:
            if arb_id in _JOINT_FRAME_IDS:
                pair_base = (arb_id - 0x155) * 2  # 0x155→J1,J2 ; 0x156→J3,J4 ; 0x157→J5,J6
                j_a = _i32_be(data, 0) / 1000.0
                j_b = _i32_be(data, 4) / 1000.0
                self._joints[pair_base] = j_a
                self._joints[pair_base + 1] = j_b
                self._seen[pair_base] = True
                self._seen[pair_base + 1] = True
                updated = True
            elif arb_id == _GRIPPER_FRAME_ID:
                self._gripper = _i32_be(data, 0) / 1000.0
                self._seen[6] = True
                updated = True
        # 事件驱动立刻发一帧(低延迟跟随)
        if updated:
            self._frame_count += 1
            self._publish()

    # ---------------- 发布 ----------------
    def _publish(self):
        with self._lock:
            if self._require_full and not all(self._seen):
                return  # 还没收满 7 个自由度的首帧,先不发
            joints = list(self._joints)
            grip = self._gripper
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "piper_leader"
        msg.name = list(JOINT_NAMES)
        msg.position = joints + [grip]
        msg.velocity = [0.0] * 7
        msg.effort = [0.0] * 7
        self._pub.publish(msg)

    def _republish_hold(self):
        """周期重发保持值(20Hz):主臂不掰不动,但订阅端(lerobot 采集)需要连续流。

        主臂是事件驱动(只在掰动时发联动帧),静置时没有新帧。lerobot teleop 的
        get_action 要持续拿到 action,断流会判错。所以这里周期重发最后一次位姿 —
        — 静止时重复发同一值,这正是"操作员保持不动"的正确语义。"""
        self._publish()

    def destroy_node(self):
        self._running = False
        if self._bus is not None:
            try:
                self._bus.shutdown()
            except Exception:
                pass
        super().destroy_node()


def main():
    rclpy.init()
    node = PiperLeaderNode()
    import signal

    signal.signal(signal.SIGTERM, lambda *_: rclpy.shutdown())
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
