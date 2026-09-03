#!/usr/bin/env bash
# 用官方 lerobot-record 采集单臂 reBot 双视角带深度数据集。
# 记录: observation.state = reBot 7 关节(shoulder_pan.pos ... gripper.pos)
#        observation.images.wrist        = 腕部 Orbbec 彩色 (H,W,3)
#        observation.images.wrist_depth  = 腕部 Orbbec 对齐深度 (H,W,1, uint16 毫米)  ← 带深度
#        observation.images.front        = USB 摄像头第二视角 (H,W,3)
#        action = reBot 关节空间目标(与 observation.state 同帧)
#
# 用法:  PY=/path/to/env/python REPO_ID=me/rebot_pick TASK="pick the cube" \
#           bash scripts/record_rebot.sh [--dataset.num_episodes=20 ...]
#   环境变量: PY / LEADER_PORT / CAN / WRIST_CAM / FRONT_CAM / REPO_ID / TASK / EPISODES / PUSH / NO_DEPTH
set -e
PY="${PY:-python}"
LEADER_PORT="${LEADER_PORT:-/dev/ttyCH341USB0}"
CAN="${CAN:-can0}"
WRIST_CAM="${WRIST_CAM:-CV275610002L}"
FRONT_CAM="${FRONT_CAM:-/dev/video10}"
REPO_ID="${REPO_ID:?请设 REPO_ID=你的用户名/数据集名}"
TASK="${TASK:?请设 TASK=\"任务自然语言描述\"}"
EPISODES="${EPISODES:-10}"
PUSH="${PUSH:-false}"

BIN_DIR="$(dirname "$("$PY" -c 'import sys; print(sys.executable)')")"
export PATH="$BIN_DIR:$PATH"

USE_DEPTH="true"; [ "${NO_DEPTH:-0}" = "1" ] && USE_DEPTH="false"
# front 用 v4l2 后端(默认 ANY 后端会让 set(width) 返回 False 而报错)+ MJPG(省 USB2 带宽)
CAMS="{ wrist: {type: orbbec, serial_number_or_name: ${WRIST_CAM}, fps: 30, width: 640, height: 480, color_format: mjpg, use_depth: ${USE_DEPTH}, warmup_s: ${WARMUP:-15}}, front: {type: opencv, index_or_path: ${FRONT_CAM}, fps: 30, width: 640, height: 480, backend: V4L2, fourcc: MJPG} }"

exec lerobot-record \
  --robot.type=rebot_follower --robot.id=follower1 \
  --robot.port="${CAN}" --robot.can_adapter=socketcan \
  --robot.cameras="${CAMS}" \
  --teleop.type=starai_to_rebot_leader --teleop.port="${LEADER_PORT}" \
  --teleop.id=rebot_leader --teleop.leader_id=leader1 \
  --dataset.fps=30 --display_data=true \
  --dataset.rgb_encoder.vcodec=h264 \
  --dataset.depth_encoder.preset="${DEPTH_PRESET:-ultrafast}" \
  --dataset.repo_id="${REPO_ID}" \
  --dataset.single_task="${TASK}" \
  --dataset.num_episodes="${EPISODES}" \
  --dataset.push_to_hub="${PUSH}" "$@"
