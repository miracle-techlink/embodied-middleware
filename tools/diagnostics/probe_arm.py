#!/usr/bin/env python
"""从臂电机 ACK 探测(record/teleop 的 preflight 用)。

CAN 接口 UP 只说明 PCAN 网卡活着,**不说明臂上电**:电机断电 / 急停拍下时总线静默,
socketcan 写入无人 ACK 也不报错,会一直蒙到 connect 才在 disable_all 炸
``control ack timeout``(踩过)。这里用 robstride_ping 逐台 ping(与 motorbridge-cli
scan 同一机制):有应答 exit 0,全静默 exit 1(调用方据此自动重拉 CAN 或报"查电源/急停")。

注意:别用 request_feedback + get_state 探 —— 空闲臂不回反馈包,会误报全静默(踩过,
2026-08-15:probe 报静默而 motorbridge-cli scan 能 ping 通 7/7)。

用法: python probe_arm.py [can0]
依赖: 只需 motorbridge(data_collect env 里有)。
"""

import sys

from motorbridge.core import Controller

# 与 seeed_b601_rs_follower 默认 motor_can_ids / motor_model_mapping 一致
MOTORS = [
    ("shoulder_pan", 0x01, 0xFD, "rs-06"),
    ("shoulder_lift", 0x02, 0xFD, "rs-06"),
    ("elbow_flex", 0x03, 0xFD, "rs-06"),
    ("wrist_flex", 0x04, 0xFD, "rs-00"),
    ("wrist_yaw", 0x05, 0xFD, "rs-00"),
    ("wrist_roll", 0x06, 0xFD, "rs-00"),
    ("gripper", 0x07, 0xFD, "rs-00"),
]


def main() -> int:
    channel = sys.argv[1] if len(sys.argv) > 1 else "can0"

    ctl = Controller(channel)
    try:
        motors = [(name, ctl.add_robstride_motor(sid, rid, model)) for name, sid, rid, model in MOTORS]
        alive, dead = [], []
        for name, m in motors:
            try:
                device_id, _responder = m.robstride_ping()
                alive.append(f"{name}(id={device_id})")
            except Exception:
                dead.append(name)
        if alive and not dead:
            print(f"[probe] {channel} 电机在线 7/7: {', '.join(alive)}")
            return 0
        if alive:
            print(f"[probe] {channel} 部分电机在线 {len(alive)}/7,无应答: {', '.join(dead)}", file=sys.stderr)
            return 1
        print(f"[probe] {channel} 7 台电机全部静默 —— 无 ACK(断电/急停/CAN 接线/臂盒没上电)", file=sys.stderr)
        return 1
    finally:
        try:
            ctl.close()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
