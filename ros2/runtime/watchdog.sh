#!/bin/bash
# 消息中心硬件看门狗 — 真实数据流检查 + 只重启死掉的节点(臂不动)
#
#   ./watchdog.sh check   只检查三个流(关节/前相机/腕相机), 打印 活/死, 退出码=死掉的流数
#   ./watchdog.sh heal    检查 + 重启死掉的节点并等到流恢复(Orbbec warmup ~18s)
#   ./watchdog.sh watch   常驻: 每 15s heal 一轮(tmux 里跑, 推理期间守护)
#
# 设计:
#   - 检查的是"数据流"(2s 内有帧), 不是 topic 存在 —— 今晚两次冻死都是进程活着流没了
#   - UVC 冻死: release+reopen 即可救回(无 sudo); Orbbec 冻死: 固件一次性会话,
#     必须先 usb_reset 再起节点(udev 0666 免 sudo)
#   - 臂节点死(joint_state 停)最重: 优雅停整个中心再 infer 模式拉起(臂会回零)
set -u
SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [ -d "$SELF/src/middleware" ]; then WS="$SELF"
elif [ -d "$SELF/ros2/src/middleware" ]; then WS="$SELF/ros2"
else WS="${MIDDLEWARE_HOME:-$HOME/middleware}"; fi
PY="$HOME/miniconda3/envs/data_collect/bin/python"
FRONT_CAM="${FRONT_CAM:-/dev/video0}"
ORBBEC_SN="${ORBBEC_SN:-CV2856D0006R}"

set +u; source /opt/ros/jazzy/setup.bash; set -u
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$WS/src/middleware"

STATE=/rebot/follower/joint_state
FRONT=/rebot/front/color/compressed
WRIST=/rebot/wrist/color/compressed

flow_ok() {  # 2s 内收到帧率输出即活(ros2 topic hz 首行 ~1-2s)
  timeout 5 ros2 topic hz "$1" --window 10 2>/dev/null | head -2 | grep -q "average rate"
}

check_all() {  # 设置 state_ok/front_ok/wrist_ok
  flow_ok "$STATE" && state_ok=1 || state_ok=0
  flow_ok "$FRONT" && front_ok=1 || front_ok=0
  flow_ok "$WRIST" && wrist_ok=1 || wrist_ok=0
  printf "[watchdog] state=%s front=%s wrist=%s\n" \
    "$([[ $state_ok = 1 ]] && echo 活 || echo 死)" \
    "$([[ $front_ok = 1 ]] && echo 活 || echo 死)" \
    "$([[ $wrist_ok = 1 ]] && echo 活 || echo 死)"
}

restart_front() {
  echo "[watchdog] 重启 UVC 前相机节点…"
  pkill -TERM -f "nodes.uvc_node" || true
  sleep 1
  nohup "$PY" -m middleware.nodes.uvc_node --ros-args -p device:="$FRONT_CAM" \
    >> /tmp/uvc_node.log 2>&1 &
  sleep 4
}

restart_wrist() {
  echo "[watchdog] 重启 Orbbec 腕相机节点(先 USB 复位)…"
  pkill -TERM -f "nodes.orbbec_node" || true
  sleep 2
  "$PY" "$WS/src/middleware/scripts/usb_reset.py" orbbec || true
  nohup "$PY" -m middleware.nodes.orbbec_node --ros-args -p serial:="$ORBBEC_SN" \
    -p width:=640 -p height:=480 -p jpeg_quality:=90 \
    >> /tmp/orbbec_node.log 2>&1 &
  echo "[watchdog] Orbbec warmup ~18s…"
  sleep 18
}

restart_center() {
  echo "[watchdog] joint_state 死 —— 整中心优雅重启(臂会回零,注意安全)…"
  "$WS/launch/start_msg_center.sh" stop
  sleep 3
  tmux kill-session -t msgcenter 2>/dev/null
  tmux new-session -d -s msgcenter "$WS/launch/start_msg_center.sh infer"
  sleep 25  # arm + 双相机 + orbbec warmup
}

heal_once() {
  check_all
  [[ $state_ok = 0 ]] && restart_center && check_all
  [[ $front_ok = 0 ]] && restart_front && flow_ok "$FRONT" && front_ok=1 || true
  [[ $wrist_ok = 0 ]] && restart_wrist && flow_ok "$WRIST" && wrist_ok=1 || true
}

case "${1:-check}" in
  check)
    check_all
    exit $(( (state_ok^1) + (front_ok^1) + (wrist_ok^1) ))
    ;;
  heal)
    for attempt in 1 2 3; do
      heal_once
      dead=$(( (state_ok^1) + (front_ok^1) + (wrist_ok^1) ))
      [[ $dead = 0 ]] && echo "[watchdog] 三流全部恢复。" && exit 0
      echo "[watchdog] 仍有 $dead 个流死(第 $attempt 轮),重试…"
    done
    echo "[watchdog] 修复失败,看 /tmp/{uvc,orbbec,arm}_node.log"
    exit 1
    ;;
  watch)
    echo "[watchdog] 常驻守护(每 15s 一轮),Ctrl-C 停。"
    while true; do
      check_all >/dev/null 2>&1 || heal_once >/dev/null 2>&1
      sleep 15
    done
    ;;
  *)
    echo "用法: $0 [check|heal|watch]"; exit 2 ;;
esac
