#!/usr/bin/env bash
# PiPER 主从采集/遥操作链路一键启动/停止。
#
#   ./start_piper_teleop.sh        # 起两节点(从臂观测 + 主臂监听),前台,Ctrl-C 全停
#   ./start_piper_teleop.sh stop   # 只停
#
# 拓扑(PiPER 与 reBot 的关键区别):
#   主臂已配固件主从模式(MasterSlaveConfig 0xFA),摆主臂→从臂**固件直通**跟随,
#   不经任何软件控制回路。所以这里不需要 teleop_map 节点。
#     piper_arm_node    → /piper/joint_state(从臂反馈 100Hz,观测真值)
#     piper_leader_node → /piper/leader/joint_state(主臂位姿,事件驱动+20Hz保持,作 action)
#   录制用 ros2_piper_follower + ros2_piper_teleop(见 record_piper.sh)。
#
# 顺序:先 arm 后 leader(leader 启动种子要读从臂 2Ax 反馈)。
set -eo pipefail

SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"   # ros2/ 根
WS="$SELF"
PY="$HOME/miniconda3/envs/data_collect/bin/python"
CAN_IFACE="${CAN_IFACE:-can0}"
MODE="${1:-teleop}"

source /opt/ros/jazzy/setup.bash
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$WS/src/middleware"

stop_all() {
    echo "== 停 PiPER 链路(SIGTERM,arm 节点 estop+disable 从臂)…"
    pkill -TERM -f "middleware.nodes.arms.piper_arm_node" || true
    pkill -TERM -f "middleware.nodes.leaders.piper_leader_node" || true
}
trap stop_all EXIT INT TERM

if [[ "$MODE" == "stop" ]]; then
    stop_all; trap - EXIT; exit 0
fi

# 日志
LOG_ROOT="$WS/logs"
SESS="$LOG_ROOT/sess_$(date +%Y%m%d_%H%M%S)_piper"
mkdir -p "$SESS"; ln -sfn "$SESS" "$LOG_ROOT/latest_piper"
L_ARM="$SESS/piper_arm_node.log"; L_LEADER="$SESS/piper_leader_node.log"

# CAN 拉起(幂等;gs_usb 不支持 restart-ms,别加)
if ! ip link show "$CAN_IFACE" 2>/dev/null | grep -q "state UP"; then
    echo "== $CAN_IFACE 未 UP,拉起(1Mbps)…"
    sudo ip link set "$CAN_IFACE" up type can bitrate 1000000 || {
        echo "!! 拉起失败,先手动: sudo ip link set $CAN_IFACE up type can bitrate 1000000"; exit 1; }
fi

echo "== 启动 PiPER 链路(日志: $SESS)"
# 从臂观测节点:enable_cmd=false 纯观测(控制权在主臂固件,软件不写从臂,避免双写)
"$PY" -m middleware.nodes.arms.piper_arm_node --ros-args \
    -p backend:=sdk -p can_name:="$CAN_IFACE" -p enable_cmd:=false -p state_fps:=100.0 \
    -p registry_json:="$WS/src/middleware/config/piper_msg_center.json" \
    > "$L_ARM" 2>&1 &
sleep 2
# 主臂监听节点:监听 15x 联动帧,种子+20Hz 保持
"$PY" -m middleware.nodes.leaders.piper_leader_node --ros-args \
    -p can_name:="$CAN_IFACE" -p topic_name:=/piper/leader/joint_state \
    > "$L_LEADER" 2>&1 &
sleep 4

ros2 topic list | grep -q "/piper/leader/joint_state" || {
    echo "!! /piper/leader/joint_state 没起来,查 $L_LEADER"; exit 1; }
echo "== PiPER 链路运行中: /piper/joint_state(从臂) + /piper/leader/joint_state(主臂)。Ctrl-C 全停。"
wait
