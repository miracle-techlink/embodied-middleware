#!/usr/bin/env bash
# ROS2 链路一键录制:补齐 source/PYTHONPATH/conda env,免手打长命令。
# 观测走 topic(相机掉线重启节点即可,不动录制进程),数据集格式与直连链路一致。
#
# 前提:终端 1 已跑 start_msg_center.sh 且 ros2 topic list | grep rebot 有输出。
# 用法:  REPO_ID=用户名/数据集名 TASK="任务描述" bash ~/middleware/record_ros2.sh
#   可选环境变量: EPISODES(默认50) EP_TIME(每条秒数,默认15) FPS(默认30)
#                 NO_DISPLAY=1 关 rerun / PUSH=true 推 hub
#                 RESUME=1 + REPO_ID=带时间戳的完整已存在数据集名 → 续录
#   注意: 本脚本 flag 全用下划线(--dataset.single_task),连字符版 draccus 不认(踩过)。
set -eo pipefail   # 不开 -u:ros2 setup.bash 裸引用未定义变量会被炸

REPO_ID="${REPO_ID:?请设 REPO_ID=你的用户名/数据集名}"
TASK="${TASK:?请设 TASK=\"任务自然语言描述\"}"
EPISODES="${EPISODES:-50}"
EP_TIME="${EP_TIME:-15}"
FPS="${FPS:-30}"

# 全程留档:从这行起脚本所有输出(切换/轮询/录制)都进 logs/record_<时间>.log
export PYTHONUNBUFFERED=1
LOG_ROOT="$HOME/middleware/logs"; mkdir -p "$LOG_ROOT"
LOGF="$LOG_ROOT/record_$(date +%Y%m%d_%H%M%S).log"
exec > >(tee -a "$LOGF") 2>&1
echo "== $(date '+%F %T') 启动 record_ros2,全程留档: $LOGF"

source /opt/ros/jazzy/setup.bash
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$HOME/middleware/src/middleware"
BIN_DIR="$HOME/miniconda3/envs/data_collect/bin"
export PATH="$BIN_DIR:$PATH"

# —— 消息中心就绪检查 + 自动切 teleop 模式 ——
# joint_cmd 必须有**发布者**(map 节点在)才能录;只 grep topic list 会被骗 —— arm 节点是
# joint_cmd 的订阅者,topic 照样出现在列表里(infer 模式/挂了,踩过 2026-08-20)。
# 无发布者 = 没起消息中心、或跑在 infer 模式(故意不起 leader/map)。默认自动处理:
# 优雅停当前布局(arm 平滑回零)→ 后台拉起 teleop 模式 → 等 joint_cmd 有发布者。
# AUTO_SWITCH=0 关掉(只报错退出,不动硬件)。
cmd_has_pub() {
    # 不用 `| grep -q`:它命中即退,上游没写完会吃 SIGPIPE,pipefail 把整条管道判失败;
    # awk 全量读完再判,顺带把 count 取出来。
    local pc
    pc=$(timeout 10 ros2 topic info /rebot/follower/joint_cmd 2>/dev/null | awk '/Publisher count/{print $3}')
    [ "${pc:-0}" -ge 1 ] 2>/dev/null
}
# 初判带重试:ros2 daemon 换环境会重启,重启窗口内 topic info 会失败 —— 一次误判就会把
# 好端端的 teleop 停掉重切(踩过 2026-08-20 10:59)。连测 3 次失败才算真没有。
_pub=0
for _i in 1 2 3; do cmd_has_pub && { _pub=1; break; }; sleep 2; done
if [ "$_pub" != 1 ]; then
    if [ "${AUTO_SWITCH:-1}" != "1" ]; then
        echo "!! /rebot/follower/joint_cmd 无发布者 —— teleop_map_node 不在(没起消息中心或在 infer 模式)。终端 1 跑 ~/middleware/start_teleop.sh,或去掉 AUTO_SWITCH=0 让本脚本自动切换。"
        exit 1
    fi
    echo "== joint_cmd 无发布者 → 自动切 teleop 模式(先优雅停当前布局,arm 会回零)..."
    bash "$HOME/middleware/start_msg_center.sh" stop
    sleep 2
    setsid nohup bash "$HOME/middleware/start_teleop.sh" \
        >"$LOG_ROOT/switch_$(date +%Y%m%d_%H%M%S).log" 2>&1 < /dev/null &
    echo "== teleop 消息中心已后台拉起(Orbbec warmup ~18s;停它:~/middleware/start_teleop.sh stop)"
    _ok=""
    for _i in $(seq 1 45); do
        cmd_has_pub && { _ok=1; break; }
        [ $((_i % 8)) -eq 0 ] && echo "  … 还没就绪(已等 $((_i * 2))s),继续等;卡住会有心跳,别急着杀"
        sleep 2
    done
    [ -n "$_ok" ] || { echo "!! 90s 内 joint_cmd 仍无发布者 —— 查 logs/latest/map_node.log 与 logs/switch_*.log"; exit 1; }
    echo "== teleop 模式就绪。"
fi

OPT_ARG=()
# rerun 默认开(与直连版一致;这版 lerobot 的 display_data 默认是 False,不显式传 true 永远不弹,踩过)
if [ "${NO_DISPLAY:-0}" = "1" ]; then OPT_ARG+=(--display_data=false); else OPT_ARG+=(--display_data=true); fi
# 续录:必须显式 --dataset.root,否则崩 "resume() requires an explicit 'root'"
if [ "${RESUME:-0}" = "1" ]; then
    LROOT="${HF_LEROBOT_HOME:-$HOME/.cache/huggingface/lerobot}/${REPO_ID}"
    [ -d "$LROOT" ] || { echo "[resume] 目标数据集不存在: $LROOT(REPO_ID 要给带时间戳的完整名)"; exit 1; }
    OPT_ARG+=(--resume=true --dataset.root="$LROOT")
fi

# 解冻 teleop:上一次闸门录制退出后 map 可能停在 enable=false(从臂不跟手),
# 普通录制需要 cmd 流,不解除会在 teleop.connect 超时。已开则无害(连 true 不触发 rearm)。
"$BIN_DIR/python" "$HOME/middleware/rebot_enable.py" true || true

# (全程留档已在脚本开头用 exec> >(tee) 挂好,stdout/stderr 一直进 LOGF;这里直接起 py)
exec "$BIN_DIR/lerobot-record" \
    --robot.type=ros2_rebot_follower --robot.id=follower1 \
    --teleop.type=ros2_rebot_teleop --teleop.id=rebot_leader \
    --dataset.repo_id="${REPO_ID}" \
    --dataset.single_task="${TASK}" \
    --dataset.fps="${FPS}" \
    --dataset.num_episodes="${EPISODES}" \
    --dataset.episode_time_s="${EP_TIME}" \
    --dataset.push_to_hub="${PUSH:-false}" \
    --dataset.rgb_encoder.vcodec=h264 \
    "${OPT_ARG[@]}" "$@"
