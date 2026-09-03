#!/usr/bin/env bash
# PiPER 链路闸门式录制(rebot record_rebot_gated 的 Piper 等价物)。
#
# 与 record_piper.sh 的区别:那个走 lerobot-record 内置流程(录完自动进复位窗口、
# 自动开下一条);这个走 record_piper_gated_ros2.py —— 每条录完**停住等人工确认**:
#   闸门 1:回车开录(提示主臂摆回起始位)→ 3 2 1 倒计时 → 录制(EP_TIME 秒,→/Esc 提前结束)
#   闸门 2:录完停住 → 回车/k=保留  d=丢弃重录  q=保存已录退出 → 回车下一条
#
# 前提:已跑 start_piper_teleop.sh(CAMERA=1 起相机则这里也 CAMERA=1)。
# 用法:  REPO_ID=用户名/数据集名 TASK="任务描述" bash record_piper_gated.sh
#   可选: EPISODES(默认50) EP_TIME(每条秒数,默认15) FPS(默认30)
#         START_COUNTDOWN(开录倒计时,默认3,0=关) END_COUNTDOWN(结束前倒计时,默认5,0=关)
#         NO_DISPLAY=1 关 rerun / PUSH=true 推 hub / RESUME=1 续录 / CAMERA=1 录视频
#   注意: flag 全用下划线(--dataset.single_task),连字符版 draccus 不认。
set -eo pipefail   # 不开 -u:ros2 setup.bash 裸引用未定义变量会被炸

_SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [ -d "$_SELF/src/middleware" ]; then WS="$_SELF"; else WS="${MIDDLEWARE_HOME:-$HOME/middleware}"; fi

REPO_ID="${REPO_ID:?请设 REPO_ID=你的用户名/数据集名}"
TASK="${TASK:?请设 TASK=\"任务自然语言描述\"}"
EPISODES="${EPISODES:-50}"
EP_TIME="${EP_TIME:-15}"
FPS="${FPS:-30}"

# 全程留档(交互式 input() 需要终端 stdin,只能 tee 输出,不能重定向整个 stdout)
export PYTHONUNBUFFERED=1
LOG_ROOT="$WS/logs"; mkdir -p "$LOG_ROOT"
LOGF="$LOG_ROOT/record_piper_gated_$(date +%Y%m%d_%H%M%S).log"
exec > >(tee -a "$LOGF") 2>&1
echo "== $(date '+%F %T') 启动 record_piper_gated,全程留档: $LOGF"

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
echo "== 链路就绪(leader 节点在跑)。闸门式:回车开录 → 倒计时 → 录制 → 回车保留/丢弃 → 回车下一条。"

OPT_ARG=()
if [ "${NO_DISPLAY:-0}" = "1" ]; then OPT_ARG+=(--display_data=false); else OPT_ARG+=(--display_data=true); fi
if [ "${RESUME:-0}" = "1" ]; then
    LROOT="${HF_LEROBOT_HOME:-$HOME/.cache/huggingface/lerobot}/${REPO_ID}"
    [ -d "$LROOT" ] || { echo "[resume] 目标数据集不存在: $LROOT(REPO_ID 要给带时间戳的完整名)"; exit 1; }
    OPT_ARG+=(--resume=true --dataset.root="$LROOT")
fi

# 相机:CAMERA=1 录视频(需先 CAMERA=1 起 orbbec 节点),否则 video=false 只录关节。
if [ "${CAMERA:-0}" = "1" ]; then
    VIDEO_ARG=(--dataset.video=true --dataset.rgb_encoder.vcodec=h264 --dataset.rgb_encoder.crf="${CRF:-23}")
else
    VIDEO_ARG=(--dataset.video=false --dataset.rgb_encoder.vcodec=h264)
fi

exec "$BIN_DIR/python" "$WS/runtime/record_piper_gated_ros2.py" \
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
