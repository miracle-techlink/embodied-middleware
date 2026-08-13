#!/usr/bin/env bash
# 闸门式采集单臂 reBot 双视角带深度数据集(每条 15s → 保留/丢弃 → 回车下一条)。
# 记录内容与 record_rebot.sh 相同(observation.state / images.wrist(+_depth) / images.front / action)。
#
# 用法:  PY=/path/to/env/python REPO_ID=用户名/数据集 TASK="任务描述" \
#           bash scripts/record_rebot_gated.sh [额外 --key=val ...]
#   环境变量: PY / LEADER_PORT / CAN / WRIST_CAM / FRONT_CAM / REPO_ID / TASK / EPISODES /
#             EP_TIME(每条秒数,默认15) / PUSH / NO_DEPTH / WARMUP / FPS
#             PREFLIGHT=0        跳过启动前设备自检(默认开:CAN/主臂/两相机缺一即 fail-fast)
#             ORBBEC_RESET=0     跳过启动前 Orbbec USB 复位(默认开:固件一次性会话坑)
#             DEPTH_PRESET(深度无损 x265 preset,默认 ultrafast。真实 Orbbec 深度实测:
#                          medium 13.7s/27MB → ultrafast 3.3s(4×)/42MB(+56%) → superfast 6.0s(2.3×)/33MB(+22%)。
#                          仍位精确无损;要省体积用 superfast,要最省磁盘用 medium)
#   性能 / 韧性开关(见 README「优化」):
#     NONBLOCK=1|0              非阻塞相机 read_latest,29.9→76.9Hz。**默认 1(开)**;=0 回退官方阻塞
#     STREAM_ENCODE=1|0        流式视频编码(编码后台化)。**默认 0(关,数据安全)**;=1 有中途打断
#                              死锁丢数据的风险(见脚本内注释),仅在确定不中途 SIGINT 时才开
#     RESUME=1                 续录:配合 REPO_ID=完整已存在数据集名(含时间戳),接着往同一数据集录、不新建
#                              (会话中途死了不丢已录的;EPISODES 视为「总目标条数」)
#     CAM_FORMAT=mjpg|rgb|yuyv  腕部彩色在线格式(USB3 用 rgb 免 CPU 解码;USB2 必须 mjpg)。默认 mjpg
#     ALIGN_MODE=sw|hw          深度 D2C 对齐:hw=硬件(卸 CPU,需设备支持)。默认 sw
#     NO_DISPLAY=1              关 rerun(批量录制省主循环开销)
set -e
PY="${PY:-python}"
LEADER_PORT="${LEADER_PORT:-/dev/ttyCH341USB0}"
CAN="${CAN:-can0}"                                   # PCAN reBot 总线(USB 重枚举后现在是 can0)
_ENV_WRIST="${WRIST_CAM-}"; _ENV_FRONT="${FRONT_CAM-}"   # 记住是否来自 shell 环境(踩过:export 了旧 /dev/videoN 覆盖脚本修好的默认值)
WRIST_CAM="${WRIST_CAM:-CV2856D0006R}"
FRONT_CAM="${FRONT_CAM:-/dev/v4l/by-id/usb-SN0002_1080P_USB_Camera_44434000_P030C01_SN0002-video-index0}"  # by-id 稳定路径,videoN 编号重枚举后会变,别用 /dev/videoN
[ -n "$_ENV_WRIST" ] && echo "[cams] 注意: WRIST_CAM 被环境变量覆盖为 $_ENV_WRIST"
[ -n "$_ENV_FRONT" ] && echo "[cams] 注意: FRONT_CAM 被环境变量覆盖为 $_ENV_FRONT"
REPO_ID="${REPO_ID:?请设 REPO_ID=你的用户名/数据集名}"
TASK="${TASK:?请设 TASK=\"任务自然语言描述\"}"
EPISODES="${EPISODES:-50}"
EP_TIME="${EP_TIME:-15}"                             # 每条固定时长(秒)
FPS="${FPS:-30}"
PUSH="${PUSH:-false}"

BIN_DIR="$(dirname "$("$PY" -c 'import sys; print(sys.executable)')")"
export PATH="$BIN_DIR:$PATH"

# 设备 preflight:缺设备直接 fail-fast 报可执行的修复提示,别带病启动 ——
# 相机被容错摘除会录出无图像数据集;CAN 没起会崩在 disable_all;主臂不在会连不上。
# PREFLIGHT=0 可跳过。
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ "${PREFLIGHT:-1}" = "1" ]; then
  FAIL=0
  [ "$(cat /sys/class/net/${CAN}/operstate 2>/dev/null)" = "up" ] \
    || { echo "[preflight] $CAN 不在 UP → sudo bash $SCRIPT_DIR/setup_rebot_can.sh"; FAIL=1; }
  [ -e "$LEADER_PORT" ] \
    || { echo "[preflight] 主臂串口 $LEADER_PORT 不存在 → 检查 USB 连接 / udev 规则(99-starai-leader)"; FAIL=1; }
  [ -e "$FRONT_CAM" ] \
    || { echo "[preflight] 前视相机 $FRONT_CAM 不存在 → v4l2-ctl --list-devices 重认(用 by-id 路径,别用 /dev/videoN)"; FAIL=1; }
  lsusb 2>/dev/null | grep -q "2bc5:" \
    || { echo "[preflight] 枚举不到 Orbbec(2bc5) → 检查 USB(直插主板口,别上共享 hub)"; FAIL=1; }
  [ "$FAIL" = "1" ] && exit 1
fi

# Orbbec 固件「一次性会话」坑:任何会话结束后(包括正常 disconnect —— SDK 退出时会
# 崩溃 terminate called,以及异常被杀)固件都会卡死,下次 connect 必报 statusCode 8
# (setXu failed)。所以每次启动前自动 USB 复位,保证从干净状态开流。ORBBEC_RESET=0 可关。
if [ "${ORBBEC_RESET:-1}" = "1" ]; then
  echo "[orbbec] 启动前 USB 复位(固件一次性会话坑,ORBBEC_RESET=0 可关)..."
  "$PY" "$SCRIPT_DIR/usbreset_orbbec.py" || echo "[orbbec] 复位失败,继续尝试直连"
  sleep 4
fi

USE_DEPTH="true"; [ "${NO_DEPTH:-0}" = "1" ] && USE_DEPTH="false"
CAMS="{ wrist: {type: orbbec, serial_number_or_name: ${WRIST_CAM}, fps: ${FPS}, width: 640, height: 480, color_format: ${CAM_FORMAT:-mjpg}, use_depth: ${USE_DEPTH}, align_mode: ${ALIGN_MODE:-sw}, warmup_s: ${WARMUP:-15}}, front: {type: opencv, index_or_path: ${FRONT_CAM}, fps: ${FPS}, width: 640, height: 480, backend: V4L2, fourcc: MJPG} }"

# 性能 / 韧性开关 → CLI
OPT_ARG=()
[ "${NO_DISPLAY:-0}" = "1" ] && DISPLAY_FLAG="false" || DISPLAY_FLAG="true"
# 流式视频编码:**默认关(数据安全)**。开(STREAM_ENCODE=1)会把编码后台化,但对无损深度这种重
# 编码,backlog 会攒到退出时集中 flush;若中途 Ctrl-C/SIGINT,VideoEncodingManager.__exit__ 的编码
# 线程收尾可能死锁 → finalize 没跑完 → parquet 损坏 → 整批数据丢(踩过一次,丢了 27 条)。
# 默认关 = 每条 save 阻塞把视频编完(~10-16s)才返回,慢但安全:退出时无 backlog、秒退、不丢数据。
if [ "${STREAM_ENCODE:-0}" = "1" ]; then
  OPT_ARG+=(--dataset.streaming_encoding=true --dataset.encoder_threads="${ENC_THREADS:-4}")
fi
# 非阻塞相机:默认开(29.9→76.9Hz,已验证)。NONBLOCK=0 回退官方阻塞 async_read
[ "${NONBLOCK:-1}" = "0" ] && OPT_ARG+=(--robot.cameras_nonblocking=false)
# 续录:RESUME=1 + REPO_ID=完整已存在数据集名(含时间戳)→ 接着往同一数据集录,不新建
# resume() 必须显式 --dataset.root(否则会崩 "resume() requires an explicit 'root'")——
# 本地数据集默认落在 HF_LEROBOT_HOME/<repo_id>,这里自动推导并先验存在性。
if [ "${RESUME:-0}" = "1" ]; then
  LROOT="${HF_LEROBOT_HOME:-$HOME/.cache/huggingface/lerobot}/${REPO_ID}"
  [ -d "$LROOT" ] || { echo "[resume] 目标数据集不存在: $LROOT(REPO_ID 要给带时间戳的完整名)"; exit 1; }
  OPT_ARG+=(--resume=true --dataset.root="$LROOT")
fi

exec "$PY" "$SCRIPT_DIR/record_rebot_gated.py" \
  --robot.type=rebot_follower --robot.id=follower1 \
  --robot.port="${CAN}" --robot.can_adapter=socketcan \
  --robot.cameras="${CAMS}" \
  --teleop.type=starai_to_rebot_leader --teleop.port="${LEADER_PORT}" \
  --teleop.id=rebot_leader --teleop.leader_id=leader1 \
  --dataset.fps="${FPS}" --display_data="${DISPLAY_FLAG}" \
  --dataset.rgb_encoder.vcodec=h264 \
  --dataset.depth_encoder.preset="${DEPTH_PRESET:-ultrafast}" \
  --dataset.repo_id="${REPO_ID}" \
  --dataset.single_task="${TASK}" \
  --dataset.num_episodes="${EPISODES}" \
  --dataset.episode_time_s="${EP_TIME}" \
  --dataset.push_to_hub="${PUSH}" "${OPT_ARG[@]}" "$@"
