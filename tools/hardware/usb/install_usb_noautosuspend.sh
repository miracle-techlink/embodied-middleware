#!/usr/bin/env bash
# 装"采集四设备禁 USB autosuspend"的持久 udev 规则(需 sudo)。
#
# 背景:USB 自动挂起会导致 Orbbec 取帧超时、CAN/串口偶发掉线(2026-08-13 四设备
# 同时消失事件,Rx urb aborted -71)。规则覆盖:Orbbec 2bc5:0840 / 前视 1080P 相机
# 2bdf:0289 / PCAN 0c72:000c / CH341 主臂串口 1a86:7523,重插重启都生效。
#
# 用法: sudo bash scripts/install_usb_noautosuspend.sh
set -e
[ "$(id -u)" = 0 ] || { echo "!! 需要 root: sudo bash $0"; exit 1; }

cat > /etc/udev/rules.d/99-rebot-usb-noautosuspend.rules <<'EOF'
# rebot 采集设备禁 USB autosuspend(Orbbec 挂起=取帧超时;母仓 SETUP_LOG 前车之鉴)
ACTION=="add", SUBSYSTEM=="usb", ATTR{idVendor}=="2bc5", ATTR{idProduct}=="0840", ATTR{power/control}="on"
ACTION=="add", SUBSYSTEM=="usb", ATTR{idVendor}=="2bdf", ATTR{idProduct}=="0289", ATTR{power/control}="on"
ACTION=="add", SUBSYSTEM=="usb", ATTR{idVendor}=="0c72", ATTR{idProduct}=="000c", ATTR{power/control}="on"
ACTION=="add", SUBSYSTEM=="usb", ATTR{idVendor}=="1a86", ATTR{idProduct}=="7523", ATTR{power/control}="on"
EOF

udevadm control --reload-rules
udevadm trigger --subsystem-match=usb --action=add || true
echo "[usb-noautosuspend] 已装 /etc/udev/rules.d/99-rebot-usb-noautosuspend.rules 并触发"
