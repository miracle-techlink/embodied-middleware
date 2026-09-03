#!/usr/bin/env bash
# middleware 一键启动/停止 —— 两种模式 + CAN 接口检查。
#
#   ./start_msg_center.sh teleop   # 采集/遥操作模式:五节点全起(默认)
#                                   leader + arm + uvc + orbbec + teleop_map
#   ./start_msg_center.sh infer    # 推理模式:只起 arm + uvc + orbbec
#                                   不起 leader/teleop_map —— follower 指令源只有策略,不被遥操作抢占
#   ./start_msg_center.sh stop     # 只停(等价于 Ctrl-C 后台实例)
#
# 顺序:leader/arm/相机先起,teleop_map 最后起(cmd 一旦开始流,从臂即 ramp 跟随主臂)。
set -eo pipefail   # -u 不开:ros2 的 setup.bash 裸引用未定义变量(AMENT_TRACE_SETUP_FILES)会被 -u 炸掉

# WS: 脚本自身所在仓库根优先(支持多机多 checkout),回退 MIDDLEWARE_HOME/~/middleware
SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [ -d "$SELF/src/middleware" ]; then WS="$SELF"
elif [ -d "$SELF/ros2/src/middleware" ]; then WS="$SELF/ros2"
else WS="${MIDDLEWARE_HOME:-$HOME/middleware}"; fi
PY="$HOME/miniconda3/envs/data_collect/bin/python"
LEADER_PORT="${LEADER_PORT:-/dev/ttyCH341USB0}"
CAN_IFACE="${CAN_IFACE:-can0}"
FRONT_CAM="${FRONT_CAM:-/dev/video0}"      # UVC 节点号重启会漂,必要时换 /dev/v4l/by-path/...
ORBBEC_SN="${ORBBEC_SN:-CV2856D0006R}"
MODE="${1:-${MODE:-teleop}}"   # teleop=采集(五节点) | infer=推理(无 leader/teleop_map)

source /opt/ros/jazzy/setup.bash
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$WS/src/middleware"

stop_all() {
    echo "== 停消息中心(SIGTERM,arm 节点会平滑回零)…"
    pkill -TERM -f "middleware.nodes" || true
}
trap stop_all EXIT INT TERM

if [[ "$MODE" == "stop" ]]; then
    stop_all; trap - EXIT; exit 0
fi
if [[ "$MODE" != "teleop" && "$MODE" != "infer" ]]; then
    echo "用法: $0 [teleop|infer|stop]"; exit 1
fi

# 日志:每次启动一个会话目录 logs/sess_<时间>_<模式>/,latest 软链指向它(rebot_watch/doctor 用);
# /tmp/<节点>.log 保留为兼容软链(旧习惯/旧引用不破)。
LOG_ROOT="$WS/logs"
SESS="$LOG_ROOT/sess_$(date +%Y%m%d_%H%M%S)_$MODE"
mkdir -p "$SESS"
ln -sfn "$SESS" "$LOG_ROOT/latest"
node_log() {  # 用法: node_log <名>  → 打印会话内路径并建 /tmp 兼容软链
    local f="$SESS/$1.log"
    ln -sf "$f" "/tmp/$1.log"
    echo "$f"
}
L_LEADER="$(node_log leader_node)"; L_ARM="$(node_log arm_node)"
L_UVC="$(node_log uvc_node)"; L_ORBBEC="$(node_log orbbec_node)"; L_MAP="$(node_log map_node)"

# CAN 接口检查(不在就尝试拉起)
if ! ip link show "$CAN_IFACE" &>/dev/null; then
    echo "== $CAN_IFACE 不在,尝试拉起(1Mbps, restart-ms 100)…"
    sudo ip link set "$CAN_IFACE" up type can bitrate 1000000 restart-ms 100
fi

echo "== 启动 middleware [mode=$MODE](日志: $SESS;兼容软链 /tmp/{leader,arm,uvc,orbbec,map}_node.log)"
if [[ "$MODE" == "teleop" ]]; then
    "$PY" -m middleware.nodes.starai_leader_node --ros-args -p port:="$LEADER_PORT" \
        > "$L_LEADER" 2>&1 &
fi
"$PY" -m middleware.nodes.rebot_arm_node --ros-args -p can_port:="$CAN_IFACE" \
    > "$L_ARM" 2>&1 &
"$PY" -m middleware.nodes.uvc_node --ros-args -p device:="$FRONT_CAM" \
    > "$L_UVC" 2>&1 &
# Orbbec 固件一次性会话坑:上次会话(含正常退出)结束后固件卡死,connect 必 statusCode 8 —— 起节点前先 USB 软复位(免 sudo,udev 0666)
if [[ "${ORBBEC_RESET:-1}" == "1" ]]; then
    python3 "$WS/src/middleware/scripts/usb_reset.py" orbbec || true
fi
"$PY" -m middleware.nodes.orbbec_node --ros-args -p serial:="$ORBBEC_SN" \
    > "$L_ORBBEC" 2>&1 &

sleep 18  # 等 Orbbec warmup(~15s)
if [[ "$MODE" == "teleop" ]]; then
    "$PY" -m middleware.nodes.teleop_map_node > "$L_MAP" 2>&1 &
    sleep 2
fi

ros2 topic list | grep rebot || { echo "!! 没有 topic 起来,查日志"; exit 1; }
if [[ "$MODE" == "infer" ]]; then
    echo "== 消息中心运行中 [推理模式:无 leader/teleop_map,follower 只听策略]。Ctrl-C 全停。"
else
    echo "== 消息中心运行中 [采集模式]。录制另开终端跑 record(见 README)。Ctrl-C 全停。"
fi
wait
