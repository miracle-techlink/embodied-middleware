#!/usr/bin/env bash
# 官方 lerobot-teleoperate 驱动单臂 rebot_follower(StarAI 主臂 → reBot B601-RS),
# 双视角进 rerun:腕部 Orbbec(彩色+深度)+ 一路 USB 摄像头(第二视角,彩色)。
# Ctrl-C 停(reBot 按官方 follower 断开逻辑处理)。
#
# 用法:  PY=/path/to/lerobot-env/python  bash scripts/teleop_rebot.sh  [额外 --key=val ...]
#   环境变量(可选覆盖): PY / LEADER_PORT / CAN / WRIST_CAM / FRONT_CAM / NO_CAM=1 / NO_DEPTH=1
#
# 前提: bash lerobot_plugins/install.sh + install_orbbec.sh;先用 seeed_b601_rs_follower 标定过;
#       reBot CAN 起来(sudo ip link set can5 up type can bitrate 1000000 restart-ms 100)。
set -e
PY="${PY:-python}"                                    # 指向装了 lerobot 的 conda env python(如 ~/miniconda3/envs/lerobot/bin/python)
LEADER_PORT="${LEADER_PORT:-/dev/ttyCH341USB0}"
CAN="${CAN:-can5}"
WRIST_CAM="${WRIST_CAM:-CV275610002L}"                # reBot 腕部 Orbbec 深度相机序列号
FRONT_CAM="${FRONT_CAM:-/dev/video4}"                 # 第二视角 USB 摄像头(SN0002 1080P = /dev/video4)

# rerun viewer 需在 PATH 里(与 env python 同目录)
BIN_DIR="$(dirname "$("$PY" -c 'import sys; print(sys.executable)')")"
export PATH="$BIN_DIR:$PATH"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# CAN 不在 UP 自动拉起;电机 ACK 探测(断电/急停时总线静默,connect 才炸 control ack timeout,
# 提前拦)。探不通自动重拉 CAN 再探一次。PREFLIGHT=0 / ARM_PROBE=0 可分别跳过。
if [ "${PREFLIGHT:-1}" = "1" ]; then
  if [ "$(cat /sys/class/net/${CAN}/operstate 2>/dev/null)" != "up" ]; then
    echo "[preflight] $CAN 不在 UP,自动拉起(sudo 可能问密码)..."
    sudo "$SCRIPT_DIR/setup_rebot_can.sh" "$CAN" \
      || { echo "[preflight] $CAN 拉起失败 → 手动跑: sudo bash $SCRIPT_DIR/setup_rebot_can.sh"; exit 1; }
  fi
  if [ "${ARM_PROBE:-1}" = "1" ]; then
    if ! "$PY" "$SCRIPT_DIR/probe_arm.py" "$CAN"; then
      echo "[preflight] 电机无 ACK,自动重拉 CAN 再探一次..."
      sudo "$SCRIPT_DIR/setup_rebot_can.sh" "$CAN" || true
      sleep 1
      "$PY" "$SCRIPT_DIR/probe_arm.py" "$CAN" \
        || { echo "[preflight] 重拉 CAN 后电机仍静默 → 检查:臂电源开关 / 急停按钮 / CAN 线。"; exit 1; }
    fi
  fi
fi

USE_DEPTH="true"; [ "${NO_DEPTH:-0}" = "1" ] && USE_DEPTH="false"
CAM_ARG=()
if [ "${NO_CAM:-0}" != "1" ]; then
  # USB2 链路务必 color_format: mjpg;深度会对齐进彩色帧(640x480)。想 30Hz 深度请把 Orbbec 插到 USB3 口。
  # front 用 v4l2 后端(默认 ANY 后端会让 set(width) 返回 False 而报错)+ MJPG(省 USB2 带宽)
  # CAM_FORMAT(默认 mjpg;USB3 可用 rgb 免解码)/ ALIGN_MODE(默认 sw;hw=硬件 D2C 卸 CPU)
  CAMS="{ wrist: {type: orbbec, serial_number_or_name: ${WRIST_CAM}, fps: 30, width: 640, height: 480, color_format: ${CAM_FORMAT:-mjpg}, use_depth: ${USE_DEPTH}, align_mode: ${ALIGN_MODE:-sw}, warmup_s: ${WARMUP:-15}}, front: {type: opencv, index_or_path: ${FRONT_CAM}, fps: 30, width: 640, height: 480, backend: V4L2, fourcc: MJPG} }"
  CAM_ARG=(--robot.cameras="${CAMS}")
fi

exec lerobot-teleoperate \
  --robot.type=rebot_follower --robot.id=follower1 \
  --robot.port="${CAN}" --robot.can_adapter=socketcan \
  "${CAM_ARG[@]}" \
  --teleop.type=starai_to_rebot_leader --teleop.port="${LEADER_PORT}" \
  --teleop.id=rebot_leader --teleop.leader_id=leader1 \
  --fps=30 --display_data=true "$@"
