# rebot_datacollect

单臂 **Seeed reBot B601-RS** 的 [LeRobot](https://github.com/huggingface/lerobot) 遥操作数据采集套件:StarAI Violin 主臂 → reBot 从臂,腕部 Orbbec 深度 + 第二视角 USB 相机,录成标准 LeRobot 数据集(带对齐深度),用于 VLA / 模仿学习训练。

从 [`Galaxea_rebot_starai_tele`](https://github.com/miracle-techlink/Galaxea_rebot_starai_tele) 中把「单臂 reBot 采集」这条链路抽出来做成的独立、聚焦仓库。

## 特性

- **原生 LeRobot 接口**:`--robot.type=rebot_follower` + `--teleop.type=starai_to_rebot_leader`,直接配合官方 `lerobot-teleoperate` / `lerobot-record`,`--display_data` 免费给 rerun 可视化。
- **绝对映射遥操作**:leader 标定零位恒定对应从臂 home,进入遥操作即对上主臂**绝对位姿**(不是把「进入那一刻」当零位);启动这一跳由 teleop 端限速 ramp 平滑滑过去,不暴力弹射。见 `starai_to_rebot_leader`。
- **带深度**:腕部 Orbbec Gemini 305 输出彩色 + 软件 D2C 对齐深度(`observation.images.wrist` / `wrist_depth`,uint16 毫米),LeRobot 自动用深度编码器保存。
- **闸门式采集**:`record_rebot_gated.py` —— 每条固定时长 → 当场选保留/丢弃 → 回车开始下一条(比官方自动连录更适合精细任务)。

## 硬件

| 部件 | 说明 |
|---|---|
| 从臂 | Seeed reBot B601-RS(**RobStride** 电机,7 轴含夹爪),SocketCAN @ 1 Mbps,PCAN-USB(peak_usb) |
| 主臂 | StarAI Violin(飞特 UART 舵机,6 臂 + 1 夹爪),`/dev/ttyCH341USB0` |
| 腕部相机 | Orbbec Gemini 305(彩色 + 深度),**建议 USB3 口** |
| 第二视角 | 任意 UVC USB 相机(V4L2) |
| 主机 | 已在 Jetson Thor(Seeed reComputer J601)验证 |

## 依赖

```bash
# 在你的 lerobot conda/venv 里
pip install lerobot-robot-seeed-b601        # 官方 reBot RobStride follower(被 rebot_follower 继承)+ motorbridge-cli
pip install pyorbbecsdk2                      # Orbbec 相机(录深度需要)
pip install fashionstar-uart-sdk              # StarAI Violin 主臂舵机(import 名 uservo;不是 fashionstar-uart-servo!)
pip install pyserial rerun-sdk                # 主臂串口 + --display_data 可视化
# reBot CAN 需要 peak_usb 内核模块(PCAN-USB);Jetson 无此模块需自行 out-of-tree 编译
```

## 安装

**全新机器一键安装**(独立 conda env + 独立 lerobot v0.6.1 克隆,不碰已有环境,多机采集直接照抄):

```bash
bash scripts/setup_env.sh                # 建 env(默认 data_collect)+ 克隆 lerobot + 装依赖 + 打插件
WITH_SUDO=1 bash scripts/setup_env.sh    # 连 Orbbec udev 规则 / dialout 组一起配
```

**手动分步**(已有 lerobot 环境时):

```bash
# 1) 把插件装进 lerobot 源码树并注册
LEROBOT_SRC=/path/to/lerobot bash lerobot_plugins/install.sh
# 2) 装 Orbbec 相机插件(录深度用)
LEROBOT_SRC=/path/to/lerobot bash lerobot_plugins/install_orbbec.sh
# 3) 深度编码修复(若 pyav 不支持 gray12le 的 numpy 转换,带深度录制会崩在 save_episode)
LEROBOT_SRC=/path/to/lerobot bash lerobot_plugins/install_depthfix.sh
```
> 注:插件是「拷进 lerobot 源码树」的方式,lerobot 升级会覆盖,升级后重跑 install。
> 实测过的完整环境清单(含各 sudo 步骤、串口节点名差异、Orbbec 掉 USB2 的坑)见 [docs/ENV_SETUP.md](docs/ENV_SETUP.md)。

## 快速开始

```bash
# 0) (Jetson)锁频提性能 —— 消除 DVFS 抖动,采集循环更容易稳到 30Hz
sudo bash scripts/maxn_lock.sh

# 1) 拉起 reBot CAN(自动找 PCAN 接口,USB 重枚举后 can 号会漂)
sudo bash scripts/setup_rebot_can.sh

# 2) 标定从臂(首次;类型必须是 rebot_follower)
#    lerobot-calibrate --robot.type=rebot_follower --robot.id=follower1 --robot.port=<canX> --robot.can_adapter=socketcan

# 3) 空跑遥操作,rerun 里确认视角 / 手感 / 夹爪方向
PY=/path/to/env/python CAN=<canX> FRONT_CAM=/dev/videoN bash scripts/teleop_rebot.sh

# 4) 闸门式采集(每条 15s → 保留/丢弃 → 回车下一条)
PY=/path/to/env/python CAN=<canX> FRONT_CAM=/dev/videoN \
  REPO_ID="你的用户名/数据集名" TASK="pick up the black object and place it in the box" \
  EPISODES=50 EP_TIME=15 PUSH=false \
  bash scripts/record_rebot_gated.sh
```

### 采集交互(在启动终端里按键)
- `▶ 回车开始录制` —— 摆好主臂/物体,回车开录
- 录制中 `→` 或 `Esc` = 提前结束本条(不想等满时长)
- `■ 回车/k=保留   d=丢弃重录   q=保存已录并退出`

数据集存 `~/.cache/huggingface/lerobot/<REPO_ID>_<时间戳>/`(LeRobot 每次采集自动加时间戳保证唯一)。

**默认开**:`NONBLOCK=1`(非阻塞相机,76.9Hz)、单条错误隔离(CAN 掉线/相机抖只丢这一条)、收尾容错。
**`STREAM_ENCODE` 默认关**:每条 save 阻塞把视频编完(~10-16s)才返回——慢但安全;开(=1)对无损深度会把
backlog 攒到退出集中 flush,中途 SIGINT 可能死锁 → parquet 损坏丢整批(踩过一次丢 27 条),别轻易开。

**断点续录(会话中途死了不丢已录的)**:
```bash
# 用刚才那个数据集的完整名(含时间戳)+ RESUME=1;EPISODES 视为总目标条数
REPO_ID="Liuyue9698/rebot_pick_place_20260708_074051" RESUME=1 EPISODES=50 \
  PY=... CAN=can0 FRONT_CAM=/dev/video10 TASK="..." bash scripts/record_rebot_gated.sh
```

## 脚本一览

| 脚本 | 作用 |
|---|---|
| `scripts/setup_env.sh` | **一键环境**:conda env + lerobot 克隆 + 依赖 + 插件三件套(新机器/多机铺开用) |
| `scripts/teleop_rebot.sh` | 空跑遥操作 + rerun 双视角(不写数据集) |
| `scripts/record_rebot_gated.sh` / `.py` | **闸门式**采集(15s / 保留丢弃 / 回车下一条) |
| `scripts/record_rebot.sh` | 官方自动连录(方向键控制,连续 N 条) |
| `scripts/setup_rebot_can.sh` | 拉起 PCAN CAN 总线(自动找接口) |
| `scripts/maxn_lock.sh` | Jetson MAXN + 锁频(Tier-0 性能) |
| `scripts/usbreset_orbbec.py` | Orbbec 卡死时 USB 复位(免拔插) |
| `scripts/estop_release.sh` | 软停:进程被 -9 硬杀后一键给 reBot 电机卸力矩(手臂变软;需总线在线) |

## 数据集内容

- `observation.state` — reBot 7 关节角(`shoulder_pan.pos` … `gripper.pos`)
- `observation.images.wrist` — 腕部 Orbbec 彩色 `(H,W,3)`
- `observation.images.wrist_depth` — 腕部对齐深度 `(H,W,1)` uint16 毫米
- `observation.images.front` — 第二视角彩色
- `action` — reBot 关节空间目标(与 `observation.state` 同帧;映射在 teleop 里完成 → 训练数据 action 与 obs 同坐标系)

## 常见问题

- **Orbbec 抓帧超时 / 卡死 / `statusCode 8`(setXu failed, propertyId 42)**:固件是**一次性会话** —— 任何会话结束后(包括正常退出:pyorbbecsdk 销毁时会 `terminate called without an active exception` 崩溃,以及 `kill -9`/core dump)固件都会卡死,下次 connect 必报 statusCode 8。两层自愈:① `connect()` warmup 拿不到帧自动 USB 复位重试(`reset_on_stall=True`,默认开);② `record_rebot_gated.sh` **启动前无条件 USB 复位**(`ORBBEC_RESET=0` 可关),保证每次从干净状态开流。推论:**复位和录制之间不要用任何别的程序碰相机**(find_cameras/测试脚本都会消耗掉唯一的干净会话);软复位救不回就物理拔插(断电重启固件)。
- **rerun 没有画面**:九成是相机 connect 失败被容错摘除(`相机 'xxx' 接入失败...已摘除`),录是还在录但没有图像流。排查:① 终端有没有「已摘除」日志;② `ls -l /proc/<record_pid>/fd | grep -oE "video[0-9]+|bus/usb"` 确认进程真的持有相机;③ **检查 shell 里残留的 `FRONT_CAM`/`WRIST_CAM` 环境变量** —— export 过的旧值会覆盖脚本修好的默认值(踩过:export 了 `/dev/video0`,重枚举后那是 Orbbec 的节点);④ Orbbec 见上一条。
- **`can*` / `/dev/video*` 号变了**:USB 重新枚举会漂。`setup_rebot_can.sh` 自动找 PCAN 接口;相机**别用 `/dev/videoN`,用 by-id 稳定路径**(`/dev/v4l/by-id/usb-<型号>_<序列号>-video-index0`,重枚举不变)—— `record_rebot_gated.sh` 的 `FRONT_CAM` 默认已改成这种写法,换机器时照 `v4l2-ctl --list-devices` 抄一条即可。
- **CAN 反复掉线**(`-71 Rx urb aborted` / `can0 removed` / `Network is down` / `No such device`):PCAN 在共享 USB2 hub 上被 **USB autosuspend** 连累掉线(实测每 ~77s 掉一次)。跑 **`sudo bash scripts/install_can_udev.sh`** 装持久 udev 规则(关 autosuspend + 掉线后自动拉起 can 口),掉线频率降 3×、且 re-attach 自愈。**根治**:把 PCAN 插独立/直连主板 USB 口,别用共享 hub。残留偶发掉线由采集脚本的单条错误隔离兜底(只丢一条)。
- **`Unsupported video codec: libsvtav1`**:某些 pyav 构建没有 svtav1。脚本已默认 `--dataset.rgb_encoder.vcodec=h264`(深度用 hevc)。
- **带深度录制崩在 `save_episode`**(`gray12le not supported` / `canonical_name` / `add_stream_from_template`):都是这台 pyav 版本偏旧、缺若干 API。跑 `bash lerobot_plugins/install_depthfix.sh` 一次性打 5 处兼容补丁(编码构造器 / 解码读 plane / codec.name 回退 / add_stream(template=) 拼接),保持 gray12le/hevc/mp4 无损。lerobot 升级会覆盖 → 重跑该脚本。
- **record loop < 30Hz**:先 `maxn_lock.sh`;把 Orbbec 挪 USB3、CAN/主臂串口与相机分开 USB 控制器;批量录制可 `--display_data=false`。

## 性能优化(采集循环稳到 30Hz)

采集是固定时间预算(30Hz=33.3ms/帧)的实时循环,掉到 27-28Hz 会丢帧。按性价比分层:

**先测,别猜** —— 用打点脚本定位真瓶颈(get_observation / get_action / send_action 各占多少 ms):
```bash
PY=/path/env/python CAN=canX FRONT_CAM=/dev/videoN bash scripts/profile_loop.sh
```

| Tier | 做法 | 命令 / 开关 | 风险 |
|---|---|---|---|
| **0 系统** | Jetson MAXN + 锁频,消 DVFS 抖动 | `sudo bash scripts/maxn_lock.sh` | 无。**最先做,常常一步到位** |
| **1 USB 拓扑** | 相机与 CAN/主臂串口**分开 USB 控制器**(相机挪独立 USB3 口),别共用一个 USB2 hub | `python scripts/check_usb.py` 体检 + 物理换口 | 无 |
| **2 软件** | 批量录制关 rerun(省主循环) | `NO_DISPLAY=1` | 无 |
| **2b** | 流式编码(不建议开:见下) | `STREAM_ENCODE=1` | **高:中途 SIGINT 死锁丢整批** |
| **3 相机** | USB3 上彩色去 mjpg 免解码;深度用硬件 D2C 卸 CPU | `CAM_FORMAT=rgb`、`ALIGN_MODE=hw` | 低(需设备支持;失败自动回退 sw) |

**实测发现 + A/B(Jetson Thor,单臂+腕深+front)**:主循环 33.5ms=29.9Hz,头号耗时是
`get_observation` 20ms —— 根因是官方用 `cam.async_read()` **阻塞等下一帧**(30fps→33ms),把控制循环
死锁在相机帧率上,任何一次迭代超时都要多等整整一个相机周期 → 掉到 27-28Hz。开关 `cameras_nonblocking`
(env `NONBLOCK=1`)改用 `read_latest` 非阻塞取最新帧:实测 `get_observation` **20.46→0.06 ms**,循环
**33.5→13.0 ms**,可达帧率 **29.9→76.9 Hz**,30Hz 录制留 ~20ms 余量给 dataset/rerun,断崖消除。
**现已默认开**(`cameras_nonblocking=True`,硬件验证过);`NONBLOCK=0` 可回退官方阻塞行为。A/B 对比:
```bash
bash scripts/profile_loop.sh              # 默认(阻塞)基线
NONBLOCK=1 bash scripts/profile_loop.sh   # 非阻塞,对比 get_observation 与可达 Hz
```
> `read_latest` 取舍:相机 fps < 循环 fps 时会读到重复帧(30/30 基本 1:1);帧超 `stale_frame_ms`(默认
> 200ms)会告警+回退上一帧,不会静默录冻结帧。相机保持 30fps 时无实际影响。

示例(USB3 全优化 + 关显示批量录;**不开 STREAM_ENCODE**):
```bash
PY=/path/env/python CAN=canX FRONT_CAM=/dev/videoN \
  REPO_ID=... TASK="..." NO_DISPLAY=1 CAM_FORMAT=rgb ALIGN_MODE=hw \
  bash scripts/record_rebot_gated.sh
```

> 注:`ALIGN_MODE=hw`(硬件 D2C)与 `CAM_FORMAT=rgb` 属**可选加速项**,需设备/固件支持,建议先 `teleop_rebot.sh` 带这俩开关空跑验证深度对齐正常、再用于录制。硬件 D2C 不支持时插件会告警并自动回退软件对齐。

## 保存/编码提速(每条 save 从 ~12s → ~2.5s)

`save_episode` 慢的**唯一卡点是无损深度视频编码**(实测占单条编码 71%:深度 hevc lossless 8.3s/300帧
vs 彩色 h264 1.65s)。默认路径本就**三路并行编码**(每路一进程),所以单条 wall-clock ≈ 深度那一路。

- **深度无法硬件加速**:Thor 的 NVENC 做不了无损 12-bit(实测丢低位)、pyav 也够不到 nvenc、
  v4l2m2m 在 Thor 无设备。深度只能留软件 libx265 lossless(gray12le,位精确)。
- **解法 = 深度 `preset=ultrafast`**(无损下 preset 只改速度/体积,不改质量,仍位精确无损、误差=纯 12-bit 量化 1mm)。
  **真实 Orbbec 深度 420 帧实测**:

  | preset | 编码 | 提速 | 体积 |
  |---|---|---|---|
  | medium(原默认) | 13.7s | 1× | 27MB |
  | **ultrafast(现默认)** | **3.3s** | **4.1×** | 42MB(+56%) |
  | superfast(平衡) | 6.0s | 2.3× | 33MB(+22%) |

- 配合三路并行 → **单条 save ~13-16s → ~4-5s**,走安全的非流式路径,不碰 streaming 死锁。
  磁盘紧张就 `DEPTH_PRESET=superfast`(省 ~22% 体积、仍 2.3×)或 `=medium`(原速最省)。
  (注:此前误用合成数据得出 "+1.8% 体积" 是错的,真实深度是 +56%——已按真实实测修正。)

> 为什么不用 streaming 后台队列?它能让 save 近乎 0 延迟,但对无损深度会攒 backlog,中途 SIGINT 时
> lerobot 的 image_writer 无超时 `queue.join()` 会死锁 → parquet 损坏丢整批(我们踩过,丢了 27 条)。
> `ultrafast` 已把 save 降到 ~2.5s,不值得为了那点延迟去冒死锁风险。

## 许可

插件基于 Apache-2.0(沿用 LeRobot / HuggingFace 头)。
