#!/usr/bin/env bash
# 软停 / 卸力矩:给 reBot 全部 RobStride 电机发 disable,释放力矩让手臂变软。
# 用途:采集/遥操作进程被硬杀(kill -9)后没走优雅退出,电机还 latch 着力矩发硬 → 跑这个卸掉。
#
# ⚠️ 卸力矩后手臂失去支撑,若处于抬起姿态会因重力下坠 —— 先用手扶住再跑!
#
# 用法: bash estop_release.sh
#   env: PY(装了 motorbridge 的 python)/ CAN(默认自动找 PCAN 接口)/ IDS(默认 "1 2 3 4 5 6 7")
#        MODEL(默认 rs-00)/ CLEAR=1(先 clear-error 再 disable,针对 fault 卡住的关节)
set -e
PY="${PY:-python}"
IDS="${IDS:-1 2 3 4 5 6 7}"
MODEL="${MODEL:-rs-00}"

BIN_DIR="$(dirname "$("$PY" -c 'import sys; print(sys.executable)')")"
export PATH="$BIN_DIR:$PATH"

# CAN 接口:env 覆盖,否则自动找 peak_usb(PCAN-USB)
CAN="${CAN:-}"
if [ -z "$CAN" ]; then
  for net in /sys/class/net/can*; do
    [ -e "$net" ] || continue
    drv="$(readlink -f "$net/device/driver" 2>/dev/null | xargs -r basename)"
    [ "$drv" = "peak_usb" ] && { CAN="$(basename "$net")"; break; }
  done
fi
[ -z "$CAN" ] && { echo "!! 找不到 PCAN 接口,手动设 CAN=canX"; exit 1; }
ip link show "$CAN" 2>/dev/null | grep -q "state UP\|UP," || echo "[warn] $CAN 可能没 UP,先: sudo ip link set $CAN up type can bitrate 1000000 restart-ms 100"

echo "[estop] 在 $CAN 上卸力矩(电机: $IDS)。⚠️ 扶好手臂..."
for id in $IDS; do
  if [ "${CLEAR:-0}" = "1" ]; then
    timeout 8 motorbridge-cli run --vendor robstride --channel "$CAN" --model "$MODEL" \
      --motor-id "$id" --feedback-id 0xFD --mode clear-error --loop 1 >/dev/null 2>&1 || true
  fi
  timeout 8 motorbridge-cli run --vendor robstride --channel "$CAN" --model "$MODEL" \
    --motor-id "$id" --feedback-id 0xFD --mode disable --loop 1 >/dev/null 2>&1 \
    && echo "  motor $id disabled" || echo "  motor $id 无响应(试 CLEAR=1)"
done
echo "[estop] 完成。手臂应已变软。"
