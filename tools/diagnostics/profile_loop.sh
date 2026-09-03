#!/usr/bin/env bash
# 跑主循环耗时打点(定位 27.5Hz 掉速的真瓶颈)。不写数据集、不开 rerun。
# 用法: PY=/path/env/python CAN=canX FRONT_CAM=/dev/videoN [WRIST_CAM=SN] [PROFILE_ITERS=300] \
#          bash scripts/profile_loop.sh
#   NO_CAM=1 只测臂/主臂(排除相机影响);默认带相机以复现真实录制负载。
set -e
PY="${PY:-python}"
LEADER_PORT="${LEADER_PORT:-/dev/ttyCH341USB0}"
CAN="${CAN:-can0}"
WRIST_CAM="${WRIST_CAM:-CV275610002L}"
FRONT_CAM="${FRONT_CAM:-/dev/video10}"
export PROFILE_ITERS="${PROFILE_ITERS:-300}"

BIN_DIR="$(dirname "$("$PY" -c 'import sys; print(sys.executable)')")"; export PATH="$BIN_DIR:$PATH"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

CAM_ARG=()
if [ "${NO_CAM:-0}" != "1" ]; then
  CAMS="{ wrist: {type: orbbec, serial_number_or_name: ${WRIST_CAM}, fps: 30, width: 640, height: 480, color_format: ${CAM_FORMAT:-mjpg}, use_depth: true, align_mode: ${ALIGN_MODE:-sw}, warmup_s: 15}, front: {type: opencv, index_or_path: ${FRONT_CAM}, fps: 30, width: 640, height: 480, backend: V4L2, fourcc: MJPG} }"
  CAM_ARG=(--robot.cameras="${CAMS}")
fi

# NONBLOCK=1 → 相机 read_latest 非阻塞(A/B 对比:先默认跑一次,再 NONBLOCK=1 跑一次)
NB_ARG=(); [ "${NONBLOCK:-0}" = "1" ] && NB_ARG=(--robot.cameras_nonblocking=true)

exec "$PY" "$SCRIPT_DIR/profile_loop.py" \
  --robot.type=rebot_follower --robot.id=follower1 \
  --robot.port="${CAN}" --robot.can_adapter=socketcan \
  "${CAM_ARG[@]}" "${NB_ARG[@]}" \
  --teleop.type=starai_to_rebot_leader --teleop.port="${LEADER_PORT}" \
  --teleop.id=rebot_leader --teleop.leader_id=leader1 \
  --dataset.repo_id=profile/tmp --dataset.single_task=profile \
  --dataset.rgb_encoder.vcodec=h264   # RecordConfig 解析需要,profiler 不建数据集但要能解析
