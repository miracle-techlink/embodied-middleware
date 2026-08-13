# 采集环境实测清单(ENV_SETUP)

2026-08-13 在 x86 主机(Ubuntu,内核 6.17)从零搭通的全记录。新机器铺开采集时对照此清单,
能避开所有我们踩过的坑。一键脚本 `scripts/setup_env.sh` 已把下面 1–4 自动化。

## 1. 隔离环境(推荐做法)

采集环境与训练栈完全隔离,互不污染:

- conda env:`data_collect`(python 3.12)。`conda` 命令不存在 ≠ 没装 miniconda ——
  先查 `~/miniconda3`,`~/miniconda3/bin/conda init bash` 后重开终端即可。
- lerobot 源码:**独立克隆上游 tag `v0.6.1`**(如 `~/lerobot-datacollect`),editable 安装。
  不要用训练用的 fork 树,也不要往训练 venv 里装采集依赖。

## 2. pip 依赖(包名勘误!)

| 包 | 用途 | 坑 |
|---|---|---|
| `lerobot-robot-seeed-b601` | reBot 官方 follower + `motorbridge-cli` | |
| `pyorbbecsdk2` | Orbbec 相机 | import 名是 `pyorbbecsdk`(无 2) |
| **`fashionstar-uart-sdk`** | StarAI 主臂舵机 | **不是 `fashionstar-uart-servo`**(PyPI 上不存在);import 路径是 `fashionstar_uart_sdk.uservo`,插件已做 `uservo` / `fashionstar_uart_sdk.uservo` 双兼容 |
| `pyserial` | 主臂串口 | |
| `rerun-sdk` | `--display_data` 可视化 | 漏装会在连臂前一刻才报 ImportError |

## 3. sudo / 系统级步骤

```bash
# Orbbec udev 规则 —— 不装则 find_cameras 抛 "usbEnumerator openUsbDevice failed!"
sudo bash <env>/lib/python3.12/site-packages/pyorbbecsdk/shared/install_udev_rules.sh
sudo udevadm control --reload && sudo udevadm trigger

# 串口权限 —— /dev/ttyUSB* 属 root:dialout,用户不在组里就是 Permission denied
sudo usermod -aG dialout $USER     # 重新登录生效

# 每次采集前拉起 CAN(脚本自动找 peak_usb 接口)
sudo bash scripts/setup_rebot_can.sh
```

## 4. 硬件端口核对清单

| 设备 | 期望状态 | 检查命令 |
|---|---|---|
| reBot 从臂(PCAN-USB) | `can0` UP @ 1Mbps | `ip -br link show can0` |
| reBot 电机 ×7 | ID 1–7 全部 hit | `motorbridge-cli scan --vendor robstride --channel can0 --start-id 1 --end-id 7` |
| StarAI 主臂 | 串口可写 | 节点名**因机器而异**:Jetson 定制驱动是 `/dev/ttyCH341USB0`,主线 ch341 驱动是 `/dev/ttyUSB0` —— 用 `LEADER_PORT=` 覆盖脚本默认 |
| Orbbec Gemini 305 | SDK 能枚举 | `python -c "from lerobot.cameras.orbbec import OrbbecCamera; print(OrbbecCamera.find_cameras())"` |
| USB 相机 | `/dev/videoN` | `udevadm info -q property /dev/video0 \| grep PRODUCT` |

## 5. 已知坑

- **Orbbec 掉 USB2**:`udevadm trigger` 或接触不良会让 Gemini 305 重枚举到 480M hub,
  SDK 报 `connection_type: USB2.1`。RGB+对齐深度带宽不够。发现后拔插到**背板直连 USB3 口**,
  或先试 `scripts/usbreset_orbbec.py` 免拔插复位。枚举后确认 `lsusb -t` 里它在 5000M  hub 下。
- **别让 Orbbec 和别的设备挤一个 USB2 hub**:实测它曾和 1080P 相机 + PCAN + CH341 同挂一个
  480M hub,采集必炸带宽。
- **lerobot 升级会覆盖插件**(拷进源码树的安装方式)→ 升级后重跑 `setup_env.sh` 或三件套。
- `udev trigger` 后 CAN 的 can 号可能漂 → 所以 `setup_rebot_can.sh` 默认自动探测而不是写死。
