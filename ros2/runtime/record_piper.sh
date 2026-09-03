#!/usr/bin/env bash
# PiPER 链路一键录制(观测走 topic,数据集格式与 rebot 链路一致)。
#
# 前提:已跑 start_piper_teleop.sh 且 /piper/leader/joint_state 有数据。
# 用法:  REPO_ID=用户名/数据集名 TASK="任务描述" bash record_piper.sh
#   可选: EPISODES(默认50) EP_TIME(每条秒数,默认15) FPS(默认30)
#         NO_DISPLAY=1 关 rerun / PUSH=true 推 hub / RESUME=1 续录
#         CAMERA=1 留位:Piper 侧相机接入后打开 video(默认无相机,纯关节数据)
#   注意: flag 全用下划线(--dataset.single_task),连字符版 draccus 不认。
#
# 与 rebot record_ros2.sh 的差异:
#   - action 源是主臂位姿 topic(主臂固件直通,无 map 节点),不需要等 joint_cmd 发布者
#   - 无相机时不录视频:--dataset.video=false,但仍需显式 h264(vcodec 校验不过会炸,踩过)
set -eo pipefail   # 不开 -u:ros2 setup.bash 裸引用未定义变量会被炸

_SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [ -d "$_SELF/src/middleware" ]; then WS="$_SELF"; else WS="${MIDDLEWARE_HOME:-$HOME/middleware}"; fi

REPO_ID="${REPO_ID:?请设 REPO_ID=你的用户名/数据集名}"
TASK="${TASK:?请设 TASK=\"任务自然语言描述\"}"
EPISODES="${EPISODES:-50}"
EP_TIME="${EP_TIME:-15}"
FPS="${FPS:-30}"

# 全程留档
export PYTHONUNBUFFERED=1
LOG_ROOT="$WS/logs"; mkdir -p "$LOG_ROOT"
LOGF="$LOG_ROOT/record_piper_$(date +%Y%m%d_%H%M%S).log"
exec > >(tee -a "$LOGF") 2>&1
echo "== $(date '+%F %T') 启动 record_piper,全程留档: $LOGF"

source /opt/ros/jazzy/setup.bash
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$WS/src/middleware"
BIN_DIR="$HOME/miniconda3/envs/data_collect/bin"
export PATH="$BIN_DIR:$PATH"

# —— 链路就绪检查:leader topic 必须有发布者(piper_leader_node 在跑)——
leader_has_pub() {
    local pc
    pc=$(timeout 10 ros2 topic info /piper/leader/joint_state 2>/dev/null | awk '/Publisher count/{print $3}')
    [ "${pc:-0}" -ge 1 ] 2>/dev/null
}
_pub=0
for _i in 1 2 3; do leader_has_pub && { _pub=1; break; }; sleep 2; done
if [ "$_pub" != 1 ]; then
    echo "!! /piper/leader/joint_state 无发布者 —— piper_leader_node 不在。先跑:"
    echo "   bash $WS/launch/start_piper_teleop.sh"
    exit 1
fi
echo "== 链路就绪(leader 节点在跑)。录制开始后掰动主臂即可,从臂固件跟随。"

OPT_ARG=()
if [ "${NO_DISPLAY:-0}" = "1" ]; then OPT_ARG+=(--display_data=false); else OPT_ARG+=(--display_data=true); fi
if [ "${RESUME:-0}" = "1" ]; then
    LROOT="${HF_LEROBOT_HOME:-$HOME/.cache/huggingface/lerobot}/${REPO_ID}"
    [ -d "$LROOT" ] || { echo "[resume] 目标数据集不存在: $LROOT(REPO_ID 要给带时间戳的完整名)"; exit 1; }
    OPT_ARG+=(--resume=true --dataset.root="$LROOT")
fi

# 相机:默认无(Piper 侧相机未接入)。CAMERA=1 时录视频。
VIDEO_ARG=(--dataset.video=false --dataset.rgb_encoder.vcodec=h264)
if [ "${CAMERA:-0}" = "1" ]; then VIDEO_ARG=(--dataset.video=true --dataset.rgb_encoder.vcodec=h264); fi

exec "$BIN_DIR/lerobot-record" \
    --robot.type=ros2_piper_follower --robot.id=piper_follower1 \
    --robot.state_topic=/piper/joint_state --robot.own_cmd_topic=false \
    --teleop.type=ros2_piper_teleop --teleop.id=piper_leader1 \
    --teleop.state_topic=/piper/leader/joint_state \
    --dataset.repo_id="${REPO_ID}" \
    --dataset.single_task="${TASK}" \
    --dataset.fps="${FPS}" \
    --dataset.num_episodes="${EPISODES}" \
    --dataset.episode_time_s="${EP_TIME}" \
    --dataset.push_to_hub="${PUSH:-false}" \
    "${VIDEO_ARG[@]}" \
    "${OPT_ARG[@]}" "$@"
