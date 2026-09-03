#!/usr/bin/env bash
# Tier-0 性能优化:把 Jetson 切到 MAXN 全功率 + 锁频,消除 DVFS 抖动。
# 对采集实时循环(30Hz)很关键:默认 schedutil governor 会让 CPU 在低频徘徊、
# 按需缓慢升频,给固定时间预算的控制循环注入延迟抖动 → 掉帧到 27-28Hz。
#
# 用法: sudo bash maxn_lock.sh
#   MAXN 电源模式会持久(写进 nvpmodel);jetson_clocks 锁频重启后失效,
#   要开机自动锁频请把本脚本做成 systemd 服务或加到 rc.local。
set -e
echo "[perf] 切 MAXN 电源模式 (mode 0) ..."
nvpmodel -m 0 || echo "  (nvpmodel 不可用 / 已是 MAXN)"
echo "[perf] jetson_clocks 锁全核到最高频 ..."
jetson_clocks || echo "  (jetson_clocks 不可用)"
echo "[perf] 当前状态:"
nvpmodel -q 2>/dev/null | grep -i "power mode" || true
echo -n "  cpu0 freq: "; cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq 2>/dev/null
echo "[perf] 完成。空跑一次采集看 record loop 是否稳定 30Hz。"
