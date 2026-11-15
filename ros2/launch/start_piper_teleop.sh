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
# 相机:CAMERA=1 时起两路 Orbbec(腕部彩+深 / 前视彩)。默认不开(纯关节采集)。
CAMERA="${CAMERA:-0}"
WRIST_SN="${WRIST_SN:-CP0CB530016X}"   # 腕部 Orbbec Gemini 335
FRONT_SN="${FRONT_SN:-CP0CB53000DX}"   # 前视 Orbbec Gemini 335
# 相机分辨率/画质(2026-09-03:640x480+jpeg90 太糊,升 720p+jpeg95;rebot 链路不受影响)
CAM_W="${CAM_W:-1280}"
CAM_H="${CAM_H:-720}"
JPEG_Q="${JPEG_Q:-95}"

source /opt/ros/jazzy/setup.bash
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$WS/src/middleware"

stop_all() {
    echo "== 停 PiPER 链路(SIGTERM,arm 节点 estop+disable 从臂)…"
    pkill -TERM -f "middleware.nodes.arms.piper_arm_node" || true
    pkill -TERM -f "middleware.nodes.leaders.piper_leader_node" || true
    pkill -TERM -f "middleware.nodes.cameras.orbbec_node" || true
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

# 相机(可选):两路 Orbbec Gemini 335。**顺序启动**(Orbbec 同时 connect 会竞争 USB 资源,
# 后起的那个 uvc_open -6,踩过 2026-09-04):wrist warmup 完再起 front。
if [[ "$CAMERA" == "1" ]]; then
    echo "== 起相机:腕部 $WRIST_SN(彩+深)→ 前视 $FRONT_SN(彩),顺序 warmup"
    # 复位仅在设备卡死时需要(ORBBEC_RESET=1):复位会改 USB 路径+重新枚举,反而易起不来。
    # 平时顺序启动(wrist warmup 完再起 front)即可避开 uvc_open 竞争。默认不复位。
    if [[ "${ORBBEC_RESET:-0}" == "1" ]]; then
      echo "== 软复位 Orbbec(2bc5:0800,sysfs authorized 翻转)"
      for d in /sys/bus/usb/devices/*; do
        if [ -f "$d/idVendor" ] && [ "$(cat "$d/idVendor")" = "2bc5" ] && [ "$(cat "$d/idProduct")" = "0800" ]; then
          echo 0 | sudo -n tee "$d/authorized" >/dev/null 2>&1 || true
        fi
      done
      sleep 1
      for d in /sys/bus/usb/devices/*; do
        if [ -f "$d/idVendor" ] && [ "$(cat "$d/idVendor")" = "2bc5" ] && [ "$(cat "$d/idProduct")" = "0800" ]; then
          echo 1 | sudo -n tee "$d/authorized" >/dev/null 2>&1 || true
        fi
      done
      sleep 6  # 复位后重新枚举慢,多等
    fi
    "$PY" -m middleware.nodes.cameras.orbbec_node --ros-args \
        -p serial:="$WRIST_SN" -p use_depth:=true \
        -p width:="$CAM_W" -p height:="$CAM_H" -p jpeg_quality:="$JPEG_Q" \
        -p color_topic:=/piper/wrist/color/compressed -p depth_topic:=/piper/wrist/depth/compressed \
        -p registry_json:="$WS/src/middleware/config/piper_msg_center.json" \
        > "$SESS/orbbec_wrist.log" 2>&1 &
    sleep 18  # wrist warmup 完(固件会话建立)再起 front
    "$PY" -m middleware.nodes.cameras.orbbec_node --ros-args \
        -p serial:="$FRONT_SN" -p use_depth:=false \
        -p width:="$CAM_W" -p height:="$CAM_H" -p jpeg_quality:="$JPEG_Q" \
        -p color_topic:=/piper/front/color/compressed -p depth_topic:=/piper/front/depth/compressed \
        -p registry_json:="$WS/src/middleware/config/piper_msg_center.json" \
        > "$SESS/orbbec_front.log" 2>&1 &
    sleep 18  # front warmup
fi

ros2 topic list | grep -q "/piper/leader/joint_state" || {
    echo "!! /piper/leader/joint_state 没起来,查 $L_LEADER"; exit 1; }
echo "== PiPER 链路运行中: /piper/joint_state(从臂) + /piper/leader/joint_state(主臂)。Ctrl-C 全停。"
if [[ "$CAMERA" == "1" ]]; then
    ros2 topic list | grep -q "/piper/wrist/color/compressed" && echo "   相机: /piper/wrist/{color,depth} + /piper/front/color 已发" \
        || echo "   !! 相机 topic 未见,查 $SESS/orbbec_*.log(Orbbec warmup 慢,可再等等)"
fi

# —— 参考零位自检(可选,纯只读,不动臂)——
# 已建档(piper_home.json)且 HOME_CHECK!=0 且终端可交互时:提示把主/从臂摆到
# 建档时的参考姿态,回车后自动 verify 比对。无档案则只提示建档命令。
# 背景:PiPER 零位在固件/驱动器 EEPROM(JointConfig 0xAE),PiperInit 不回零;
#       跨 session 数据/推理一致性靠「参考姿态读数」与档案比对来保证。
if [[ "${HOME_CHECK:-1}" != "0" && -t 0 ]]; then
    PIPER_HOME_FILE="${PIPER_HOME_FILE:-$HOME/.cache/huggingface/lerobot/calibration/piper_home.json}"
    ZC="$WS/../tools/diagnostics/piper_zero_check.sh"
    if [ -f "$PIPER_HOME_FILE" ]; then
        echo "== 参考零位自检:请把主/从臂摆到建档参考姿态并扶稳,回车开始回读比对(只读,不动臂)…"
        read -r _
        if bash "$ZC" verify --out "$PIPER_HOME_FILE"; then
            echo "== 零位自检通过(与档案一致,可采集)。"
        else
            echo "!! 零位自检未过(见上方逐关节结果)——先查机械/零位,别直接开录!"
        fi
    else
        echo "== (提示)尚无参考零位档案: $PIPER_HOME_FILE"
        echo "   摆好参考姿态后建档: bash $ZC archive --label '参考姿态名'"
        echo "   (之后每次起链会自动比对;HOME_CHECK=0 可关本自检)"
    fi
fi
wait
