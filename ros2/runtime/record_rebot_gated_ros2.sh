#!/usr/bin/env bash
# ROS2 消息中心链路的闸门式采集(每条固定时长 → 当场选留/丢 → 回车下一条)。
# 直连版 record_rebot_gated.sh 的消息中心等价物,节奏钩子的 ROS2 映射见 .py 头注释。
#
# 前提:终端 1 已跑 start_msg_center.sh。
# 用法:  REPO_ID=用户名/数据集名 TASK="任务描述" \
#           bash ~/middleware/record_rebot_gated_ros2.sh
#   可选: EPISODES(默认50) EP_TIME(每条秒数,默认15) FPS(默认30)
#         NO_DISPLAY=1 关 rerun / PUSH=true 推 hub
#         RESUME=1 + REPO_ID=带时间戳的完整已存在数据集名 → 续录
#   节奏开关(py 直读环境变量): ZERO_AFTER_EPISODE(默认1,录完自动回零 go_home)/
#         [倒计时期间从臂冻结不跟手,结束那一刻才恢复跟随并开录;无对齐;0位确认仅整轮开头一次]/
#         HOME_TOL_DEG(2.0)/ HOME_TIMEOUT_S(30)/ ALIGN_LEADER / ALIGN_TIMEOUT_S /
#         SETTLE_DEG / START_COUNTDOWN / END_COUNTDOWN / JOINT_STALE_MS
#         (主臂 0 位为人工确认:回车前自己摆好,回车后打印偏差仅供参考,不自动判定)
#   map 节点版本:脚本会查 /rebot/teleop/go_home 有无订阅,旧版自动重启消息中心换新
#   注意: flag 全用下划线(--dataset.single_task),连字符版 draccus 不认(踩过)。
set -eo pipefail   # 不开 -u:ros2 setup.bash 裸引用未定义变量会被炸

# WS: 脚本自身所在仓库根优先,回退 MIDDLEWARE_HOME/~/middleware
_SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [ -d "$_SELF/src/middleware" ]; then WS="$_SELF"
elif [ -d "$_SELF/ros2/src/middleware" ]; then WS="$_SELF/ros2"
else WS="${MIDDLEWARE_HOME:-$HOME/middleware}"; fi

REPO_ID="${REPO_ID:?请设 REPO_ID=你的用户名/数据集名}"
TASK="${TASK:?请设 TASK=\"任务自然语言描述\"}"
EPISODES="${EPISODES:-50}"
EP_TIME="${EP_TIME:-15}"
FPS="${FPS:-30}"

# 全程留档:从这行起脚本所有输出(切换/轮询/录制)都进 logs/record_gated_<时间>.log
# —— 2026-08-20 踩过:切换+轮询阶段没留档,静默死无据可查。
export PYTHONUNBUFFERED=1
LOG_ROOT="$WS/logs"; mkdir -p "$LOG_ROOT"
LOGF="$LOG_ROOT/record_gated_$(date +%Y%m%d_%H%M%S).log"
exec > >(tee -a "$LOGF") 2>&1
echo "== $(date '+%F %T') 启动 record_rebot_gated_ros2,全程留档: $LOGF"

source /opt/ros/jazzy/setup.bash
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$WS/src/middleware"
PY="$HOME/miniconda3/envs/data_collect/bin/python"
BIN_DIR="$(dirname "$PY")"
export PATH="$BIN_DIR:$PATH"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

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
# 好端端的 teleop 停掉重切(踩过 2026-08-20 10:59:五节点全在却被误切)。连测 3 次失败才算真没有。
_pub=0
for _i in 1 2 3; do cmd_has_pub && { _pub=1; break; }; sleep 2; done
if [ "$_pub" != 1 ]; then
    if [ "${AUTO_SWITCH:-1}" != "1" ]; then
        echo "!! /rebot/follower/joint_cmd 无发布者 —— teleop_map_node 不在(没起消息中心或在 infer 模式)。终端 1 跑 ~/middleware/start_teleop.sh,或去掉 AUTO_SWITCH=0 让本脚本自动切换。"
        exit 1
    fi
    echo "== joint_cmd 无发布者 → 自动切 teleop 模式(先优雅停当前布局,arm 会回零)..."
    bash "$WS/start_msg_center.sh" stop
    sleep 2
    setsid nohup bash "$WS/start_teleop.sh" \
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

# —— map 节点版本检查:/rebot/teleop/go_home 是新话题(2026-08-20,录完自动回零靠它),
#    旧 map 节点没订阅它。没有就自动重启消息中心换新(arm 会先回零,约 40s)——
go_home_sub() {
    timeout 8 ros2 topic info /rebot/teleop/go_home 2>/dev/null | awk '/Subscription count/{print $3}'
}
# 注意 || true:topic 不存在时 ros2 topic info 退出 1,pipefail 会把它带进赋值状态,
# set -e 下裸赋值直接杀脚本零输出(踩过 2026-08-20 11:53:卡在留档行后"不能重启")
_ghs=$(go_home_sub) || true
if [ "${_ghs:-0}" -lt 1 ] 2>/dev/null; then
    echo "== map 节点是旧版(无 go_home 订阅,录完不会自动回零)→ 重启消息中心换新..."
    bash "$WS/start_msg_center.sh" stop
    sleep 2
    setsid nohup bash "$WS/start_teleop.sh" \
        >"$LOG_ROOT/switch_$(date +%Y%m%d_%H%M%S)_for_gohome.log" 2>&1 < /dev/null &
    _ok=""
    for _i in $(seq 1 45); do
        [ "$(go_home_sub)" -ge 1 ] 2>/dev/null && { _ok=1; break; }
        [ $((_i % 8)) -eq 0 ] && echo "  … 还没就绪(已等 $((_i * 2))s)"
        sleep 2
    done
    [ -n "$_ok" ] || { echo "!! 90s 内 go_home 仍无订阅 —— 查 logs/latest/map_node.log"; exit 1; }
    echo "== 新版 map 节点就绪(支持录完自动回零)。"
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

# (全程留档已在脚本开头用 exec> >(tee) 挂好,stdout/stderr 一直进 LOGF;这里直接起 py)
exec "$PY" "$SCRIPT_DIR/record_rebot_gated_ros2.py" \
    --robot.type=ros2_rebot_follower --robot.id=follower1 \
    --teleop.type=ros2_rebot_teleop --teleop.id=rebot_leader \
    --dataset.repo_id="${REPO_ID}" \
    --dataset.single_task="${TASK}" \
    --dataset.fps="${FPS}" \
    --dataset.num_episodes="${EPISODES}" \
    --dataset.episode_time_s="${EP_TIME}" \
    --dataset.push_to_hub="${PUSH:-false}" \
    --dataset.rgb_encoder.vcodec=h264 \
    --dataset.depth_encoder.preset="${DEPTH_PRESET:-ultrafast}" \
    "${OPT_ARG[@]}" "$@"
