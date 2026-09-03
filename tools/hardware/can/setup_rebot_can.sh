#!/usr/bin/env bash
# 拉起 reBot B601-RS 的 CAN 总线(PCAN-USB / peak_usb,1 Mbps)。
# reBot 用 RobStride 电机,走 SocketCAN(不是 Damiao 的 /dev/ttyACM 串口桥)。
#
# 用法: sudo bash setup_rebot_can.sh [can接口名]
#   不给接口名时自动找 PCAN-USB(peak_usb 驱动)对应的 canX。
#   注意:USB 重新枚举后 can 号会漂,所以默认自动探测而不是写死 can5/can0。
set -e

IFACE="$1"
if [ -z "$IFACE" ]; then
  # 自动定位 peak_usb(PCAN-USB)绑定的网络接口
  for net in /sys/class/net/can*; do
    [ -e "$net" ] || continue
    drv="$(readlink -f "$net/device/driver" 2>/dev/null | xargs -r basename)"
    if [ "$drv" = "peak_usb" ]; then IFACE="$(basename "$net")"; break; fi
  done
fi
[ -z "$IFACE" ] && { echo "!! 没找到 PCAN-USB(peak_usb)接口。检查:lsusb | grep 0c72 ; dmesg | grep peak_usb"; exit 1; }

echo "[can] 拉起 $IFACE @ 1 Mbps ..."
ip link set "$IFACE" down 2>/dev/null || true
ip link set "$IFACE" up type can bitrate 1000000 restart-ms 100
ip -br link show "$IFACE"
echo "[can] OK。验证电机在线(应看到 id 1-7):"
echo "  motorbridge-cli scan --vendor robstride --channel $IFACE --start-id 1 --end-id 7"
