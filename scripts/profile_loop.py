#!/usr/bin/env python
"""测量遥操作/采集主循环的耗时分布,定位真正的瓶颈(不猜)。

连接 robot + teleop(与录制同一套 config),空跑 N 次,分别计时:
  - get_observation  (读 7 电机 CAN 反馈 + 相机 async_read)
  - get_action       (读 StarAI 主臂串口 + 映射)
  - send_action      (下发 CAN + 夹爪)
输出每段 min/mean/p95 ms 与可达 Hz。据此决定 Tier-2/3 改哪(例如串口读慢 → 优化 teleop;
相机 async_read 慢 → 后台线程没跟上,去 mjpg/上硬件 D2C)。

CLI 与 lerobot-record 一致(--robot.* --teleop.* [--robot.cameras=...]);
迭代次数用环境变量 PROFILE_ITERS(默认 300 ≈ 10s@30Hz)。不写数据集、不显示。
"""

import os
import time
from statistics import mean

from lerobot.configs import parser
from lerobot.robots import make_robot_from_config
from lerobot.teleoperators import make_teleoperator_from_config
from lerobot.utils.utils import init_logging
from lerobot.scripts.lerobot_record import RecordConfig


def _stats(xs: list[float]) -> str:
    xs = sorted(xs)
    p95 = xs[min(len(xs) - 1, int(len(xs) * 0.95))]
    return f"min {xs[0]*1e3:6.2f}  mean {mean(xs)*1e3:6.2f}  p95 {p95*1e3:6.2f}  max {xs[-1]*1e3:6.2f} ms"


@parser.wrap()
def main(cfg: RecordConfig) -> None:
    init_logging()
    iters = int(os.environ.get("PROFILE_ITERS", "300"))
    robot = make_robot_from_config(cfg.robot)
    teleop = make_teleoperator_from_config(cfg.teleop)
    robot.connect()
    teleop.connect()
    print(f"\n[profile] warmup 30 帧 ...")
    for _ in range(30):
        robot.get_observation(); teleop.get_action()

    t_obs, t_act, t_send, t_total = [], [], [], []
    # PACE_FPS>0(默认 30):按真实采集节奏限速循环 —— get_action 缓存化后裸奔会
    # 以数百 Hz 灌 SocketCAN TX 队列(ENOBUFS),测出来的不是真实工况。
    pace_fps = float(os.environ.get("PACE_FPS", "30"))
    try:
        for i in range(iters):
            loop_start = time.perf_counter()
            s0 = loop_start
            robot.get_observation()
            s1 = time.perf_counter()
            act = teleop.get_action()
            s2 = time.perf_counter()
            robot.send_action(act)
            s3 = time.perf_counter()
            t_obs.append(s1 - s0); t_act.append(s2 - s1); t_send.append(s3 - s2); t_total.append(s3 - s0)
            if pace_fps > 0:
                dt = time.perf_counter() - loop_start
                if dt < 1.0 / pace_fps:
                    time.sleep(1.0 / pace_fps - dt)
            if (i + 1) % 60 == 0:
                print(f"  {i+1}/{iters}  total {mean(t_total[-60:])*1e3:.1f}ms")
    finally:
        robot.disconnect(); teleop.disconnect()

    print("\n================ 主循环耗时分布 ================")
    print(f"get_observation : {_stats(t_obs)}")
    print(f"get_action      : {_stats(t_act)}")
    print(f"send_action     : {_stats(t_send)}")
    print(f"LOOP total      : {_stats(t_total)}")
    ach = 1.0 / mean(t_total)
    print(f"\n可达帧率(纯 IO,不含 dataset/rerun): {ach:.1f} Hz  (目标 30Hz 预算 33.3ms)")
    worst = max([("get_observation", mean(t_obs)), ("get_action", mean(t_act)), ("send_action", mean(t_send))], key=lambda x: x[1])
    print(f"头号耗时: {worst[0]}  ({worst[1]*1e3:.1f}ms/帧)")


if __name__ == "__main__":
    main()
