# embodied-middleware

单臂机器人遥操作数据采集栈:把机械臂/相机包成 LeRobot 插件或 ROS2 topic,录成标准 [LeRobot](https://github.com/huggingface/lerobot) 数据集,用于 VLA / 模仿学习训练。

| 链路 | 从臂 | 主臂 | 相机 |
|---|---|---|---|
| **reBot** | Seeed reBot B601-RS(RobStride,7 轴,SocketCAN @1Mbps) | StarAI Violin(UART) | 腕部 Orbbec(彩+深度)+ 前视 UVC |
| **PiPER** | PiPER(固件主从直通,CAN) | PiPER 主臂(同总线监听) | 两路 Orbbec Gemini 335(可选) |

## 架构

```
                ┌────────────────────── LeRobot 插件(直连,单进程)──────────────────────┐
                │  lerobot-record/teleoperate                                         │
                │    robot: rebot_follower ──→ CAN(RobStride 电机)                    │
                │    teleop: starai_to_rebot_leader ──→ UART(绝对映射 + 启动 ramp)    │
                │    cameras: orbbec(彩+对齐深度)/ uvc                                │
                └──────────────────────────────────────────────────────────────────────┘
                                             (二选一,数据集格式完全一致)
                ┌────────────────────── ROS2 消息中心(多进程,推荐)────────────────────┐
                │                                                                      │
 reBot:         │  starai_leader_node ─/rebot/leader/joint_state(100Hz)→              │
                │                        teleop_map_node(映射+ramp+闸门,100Hz)        │
                │                          └─/rebot/follower/joint_cmd→ rebot_arm_node ─→ CAN
                │  rebot_arm_node   ─/rebot/follower/joint_state(200Hz)─┐             │
                │  orbbec_node      ─/rebot/wrist/{color,depth}(30Hz)──┤             │
                │  uvc_node         ─/rebot/front/color(30Hz)──────────┤             │
                │                                                      ↓             │
 PiPER:         │  piper_leader_node ─/piper/leader/joint_state(监听固件主从帧)      │
                │  piper_arm_node    ─/piper/joint_state(100Hz,纯观测) ─┤            │
                │  (主从跟随在固件内完成,无 teleop_map)                  ↓            │
                │         ros2_*_follower + ros2_*_teleop(桥)→ lerobot-record        │
                └──────────────────────────────────────────────────────────────────────┘
```

两种架构的产物是**同一个格式**的 LeRobot 数据集(`observation.state` / `action` / `observation.images.*`)。
ROS2 模式把控制回路从录制循环里拆出去:控制 100Hz 由节点自持,录制只是被动订阅;相机掉线重启单个节点即可,不用停采集。

## 快速开始

### 1. 安装(新机器一次)

```bash
git clone https://github.com/miracle-techlink/embodied-middleware ~/middleware
cd ~/middleware
bash tools/environment/setup_env.sh              # conda env + lerobot v0.6.1 克隆 + 依赖 + 插件
WITH_SUDO=1 bash tools/environment/setup_env.sh  # 顺带配 udev 规则 / dialout 组
```

### 2. 起链路(终端 1)

```bash
# reBot:五节点全起(leader + arm + 双相机 + teleop_map),Ctrl-C 全停、arm 平滑回零
~/middleware/ros2/start_teleop.sh

# PiPER:从臂观测 + 主臂监听(CAMERA=1 顺序起两路 Orbbec)
~/middleware/ros2/launch/start_piper_teleop.sh

# 多机/profile 驱动(推荐新机器):mode 与设备全部定义在 profiles/rigs/<rig>.yaml
~/middleware/ros2/start_rig.sh teleop --rig rebot_starai_orbbec
```

### 3. 采集(终端 2)

```bash
# reBot 闸门式:回车开录 → 每条 15s → 当场保留/丢弃 → 回车下一条
REPO_ID=用户名/数据集名 TASK="任务描述" \
  bash ~/middleware/ros2/record_rebot_gated_ros2.sh

# PiPER 闸门式(等价的官方连录是 record_piper.sh,自带复位窗口)
REPO_ID=用户名/数据集名 TASK="任务描述" \
  bash ~/middleware/ros2/runtime/record_piper_gated.sh
```

交互:录制中 `→`/`Esc` 提前结束;录完回车/`k` 保留、`d` 丢弃重录、`q` 保存退出。
断点续录:`RESUME=1 REPO_ID=<带时间戳的完整数据集名>`。
数据集落盘 `~/.cache/huggingface/lerobot/<REPO_ID>_<时间戳>/`。

### 4. 体检与部署自检

```bash
python3 ~/middleware/ros2/launch/fleet_validate.py                # 离线校验 rig profile(新机器先跑)
~/middleware/ros2/rig_doctor.sh teleop --rig rebot_starai_orbbec  # 节点/频率/日志错误一键体检
~/middleware/ros2/rig_watchdog.sh watch                           # 健康流低于阈值自动 heal
~/middleware/ros2/rebot_watch.sh                                  # 实时日志窗(错误红/警告黄)
```

## 目录结构

```
profiles/rigs/       每台采集机的真值配置(namespace/mode/设备/topic/健康阈值)
ros2/launch/         启动(start_rig.py 新入口 / start_teleop / start_piper_teleop)
ros2/runtime/        录制与看门狗(record_*_gated / record_ros2 / rig_watchdog)
ros2/admin/          体检运维(rig_doctor / rebot_doctor / rebot_watch)
ros2/src/middleware/ ROS2 Python 包(core / backends / nodes / maintenance)
lerobot_plugins/     LeRobot 插件源与安装器(robots / teleoperators / cameras)
tools/               直连采集与环境硬件工具(acquisition / environment / hardware / diagnostics)
docs/                环境搭建、LeRobot 笔记、架构约定
```

`ros2/` 根与 `scripts/` 下的同名脚本是对上表 canonical 路径的**兼容 wrapper**,旧用法不破;新代码不进这两处。
多机部署:每台机器一个 `profiles/rigs/<rig>.yaml`,本机微调放 git 忽略的 `profiles/local/overrides.yaml`。

## 关键设计

- **绝对映射遥操作**:leader 标定零位恒定对应从臂 home,进入遥操作由限速 ramp 平滑对接,不暴力弹射。
- **对齐深度**:腕部 Orbbec 软件 D2C,`wrist_depth` uint16 毫米,LeRobot 深度编码器(gray12le/hevc 无损)保存。
- **安全停止**:arm 节点注册 SIGTERM 优雅退出(回零→卸力矩),`start_rig.py` 按逆启动顺序精确 SIGTERM,绝不 SIGKILL。
- **QoS**:全部 topic 为 BEST_EFFORT keep-last 1,订阅方必须匹配(RELIABLE 会被 DDS 静默判不兼容)。

## 常见问题

- **Orbbec 卡死 / statusCode 8**:固件一次性会话,任何会话结束后需 USB 复位才能再开流;采集脚本启动前已自动复位,复位与录制之间不要用其他程序碰相机。
- **`can*` / `/dev/video*` 号漂移**:USB 重枚举导致。CAN 由启动脚本自动发现;相机用 `/dev/v4l/by-id/` 稳定路径。
- **CAN 反复掉线**:共享 USB2 hub + autosuspend。`sudo bash tools/hardware/can/install_can_udev.sh` 装持久规则;根治是 PCAN 插独立 USB 口。
- **rerun 无画面**:检查 shell 残留的 `FRONT_CAM`/`WRIST_CAM` 环境变量覆盖了脚本默认值。
- **采集循环掉帧**:先 `sudo bash tools/hardware/power/maxn_lock.sh` 锁频;相机与 CAN/串口分开 USB 控制器;非阻塞取帧已默认开启。
- **kill -9 砸臂后**:`bash tools/hardware/power/estop_release.sh` 给电机卸力矩。

更多:[docs/INDEX.md](docs/INDEX.md) · [ros2/README.md](ros2/README.md) · [docs/ENV_SETUP.md](docs/ENV_SETUP.md)

## 许可

Apache-2.0(沿用 LeRobot / HuggingFace 头)。
