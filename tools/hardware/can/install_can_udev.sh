#!/usr/bin/env bash
# 修复 PCAN-USB(peak_usb)在共享 USB2 hub 上反复 `-71 Rx urb aborted` → `can0 removed` 掉线。
# 深度诊断结论(2026-07-08):
#   - 主因:上游 USB hub 的**自动挂起(autosuspend)**会连累下游 CAN 适配器掉线。
#     关掉后稳定期从 ~77s 拉长到 ~232s(掉线频率降 3×)。
#   - 残留:仍有偶发 removed(疑似 out-of-tree peak_usb 驱动 / hub/线缆/供电),软件难根除。
#   - 根治:把 PCAN 插到**独立/直连主板的 USB 口**(别用共享 USB2 hub)。
#
# 本脚本装两条持久 udev 规则(重启 / USB 重枚举都生效):
#   1) 关 USB hub + PCAN 的 autosuspend
#   2) peak_usb 的 CAN 口一出现就自动 `ip link set up 1Mbps`(掉线 re-attach 后自愈,否则回来是 DOWN)
# 配合采集脚本的「单条错误隔离」,一次掉线最多丢一条 episode、会话不中断。
#
# 用法: sudo bash install_can_udev.sh
set -e
[ "$(id -u)" = "0" ] || { echo "需要 sudo: sudo bash $0"; exit 1; }
IP="$(command -v ip)"; [ -n "$IP" ] || IP=/usr/sbin/ip

cat > /etc/udev/rules.d/50-usb-no-autosuspend.rules <<EOF
# 关闭 USB 自动挂起 —— 修复 PCAN-USB(peak_usb)在共享 hub 上反复 -71/removed 掉线。
# hub 自动挂起会连累下游 CAN 适配器。对插电的机器人台架,常开无副作用。
ACTION=="add", SUBSYSTEM=="usb", DRIVER=="hub", ATTR{power/control}="on"
ACTION=="add", SUBSYSTEM=="usb", ATTRS{idVendor}=="0c72", ATTR{power/control}="on"

# reBot PCAN CAN 口自愈:掉线 re-attach 后自动拉起 1Mbps(否则回来是 DOWN,后续操作全失败)。
ACTION=="add", SUBSYSTEM=="net", ENV{ID_NET_DRIVER}=="peak_usb", RUN+="$IP link set %k up type can bitrate 1000000 restart-ms 100"
EOF

# 立即对现有 hub 生效(udev 规则只在 add 时触发)
for h in /sys/bus/usb/devices/*/; do
  drv="$(cat "$h/bDeviceClass" 2>/dev/null || true)"
  [ -e "$h/power/control" ] && echo on > "$h/power/control" 2>/dev/null || true
done
echo -1 > /sys/module/usbcore/parameters/autosuspend 2>/dev/null || true

udevadm control --reload-rules
echo "[can-udev] 已装:关 autosuspend + CAN 口自愈拉起。"
echo "[can-udev] 根治仍建议:把 PCAN 插独立 USB 口(别用共享 USB2 hub)。"
