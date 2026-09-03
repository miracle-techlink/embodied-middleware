#!/usr/bin/env bash
# rebot 平台一键体检(全只读,不动硬件/进程):进程、topic 发布者、关键频率、
# 近期日志错误、CAN、磁盘。跑不动/录不出问题时先跑这个,输出可直接贴给排查。
#   rebot_doctor.sh            # 按 teleop 模式体检(五节点)
#   rebot_doctor.sh infer      # 按 infer 模式体检(三节点:arm+uvc+orbbec)
set -o pipefail   # 不开 -u:ros2 setup.bash 裸引用未定义变量会被炸(同 start_msg_center 坑)
WS="$HOME/middleware"
MODE="${1:-teleop}"

source /opt/ros/jazzy/setup.bash 2>/dev/null || true
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages"

RED=$'\033[1;31m'; GRN=$'\033[1;32m'; YLW=$'\033[1;33m'; RST=$'\033[0m'
ok()   { echo "${GRN}✓${RST} $*"; }
bad()  { echo "${RED}✗${RST} $*"; FAIL=1; }
warn() { echo "${YLW}!${RST} $*"; }
FAIL=0

echo "===== rebot 体检 $(date '+%F %T') [模式假设: $MODE] ====="

# 1. 节点进程(模块名:starai_leader_node / rebot_arm_node / uvc_node / orbbec_node / teleop_map_node)
declare -A WANT=( [starai_leader_node]=0 [rebot_arm_node]=1 [uvc_node]=1 [orbbec_node]=1 [teleop_map_node]=0 )
if [ "$MODE" = teleop ]; then WANT[starai_leader_node]=1; WANT[teleop_map_node]=1; fi
for n in starai_leader_node rebot_arm_node uvc_node orbbec_node teleop_map_node; do
    if pgrep -f "middleware.nodes.$n" >/dev/null; then
        if [ "${WANT[$n]}" = 1 ]; then ok "节点 $n 在跑"; else warn "节点 $n 在跑(但 $MODE 模式不需要)"; fi
    else
        if [ "${WANT[$n]}" = 1 ]; then bad "节点 $n 不在!"; else ok "节点 $n 不在($MODE 模式正确)"; fi
    fi
done

# 2. topic 发布者(有订阅者的 topic 也会出现在列表里,必须看 Publisher count,踩过)
declare -A NEED_PUB=( [/rebot/follower/joint_state]=1 [/rebot/follower/joint_cmd]=0
                      [/rebot/front/color/compressed]=1 [/rebot/wrist/color/compressed]=1
                      [/rebot/wrist/depth/compressed]=1 [/rebot/leader/joint_state]=0 )
if [ "$MODE" = teleop ]; then NEED_PUB[/rebot/follower/joint_cmd]=1; NEED_PUB[/rebot/leader/joint_state]=1; fi
for t in "${!NEED_PUB[@]}"; do
    pc=$(timeout 8 ros2 topic info "$t" 2>/dev/null | awk '/Publisher count/{print $3}')
    if [ "${NEED_PUB[$t]}" = 1 ]; then
        if [ "${pc:-0}" -ge 1 ] 2>/dev/null; then ok "$t 发布者=${pc}"; else bad "$t 发布者=${pc:-0}(应为 ≥1)"; fi
    else
        if [ "${pc:-0}" -ge 1 ] 2>/dev/null; then warn "$t 有发布者=${pc}($MODE 模式不应有)"; else ok "$t 无发布者($MODE 模式正确)"; fi
    fi
done

# 2b. map 节点版本(teleop 模式):go_home 是 2026-08-20 新话题,没订阅=旧版,录完不回零
if [ "$MODE" = teleop ]; then
    ghs=$(timeout 8 ros2 topic info /rebot/teleop/go_home 2>/dev/null | awk '/Subscription count/{print $3}')
    if [ "${ghs:-0}" -ge 1 ] 2>/dev/null; then
        ok "map 节点支持 go_home(录完自动回零可用)"
    else
        warn "map 节点无 go_home 订阅(旧版,录完不会自动回零)—— 重启消息中心换新"
    fi
fi

# 3. 关键 topic 实测频率(ros2 topic hz 这版没有 QoS 参数,RELIABLE 默认订阅收不到
#    BEST_EFFORT 流 —— 用自带探针 rebot_rate.py,QoS 匹配,先等首帧再开窗计数)
PYENV="$HOME/miniconda3/envs/data_collect/bin/python"
for t in /rebot/follower/joint_state /rebot/leader/joint_state /rebot/follower/joint_cmd; do
    if [ "$MODE" = infer ] && [ "$t" != /rebot/follower/joint_state ]; then continue; fi
    hz=$(timeout 12 "$PYENV" "$WS/rebot_rate.py" "$t" 2 2>/dev/null)
    if [ -n "$hz" ] && [ "$hz" != "0" ] 2>/dev/null; then ok "$t ≈${hz}Hz"; else bad "$t 测不到频率(发布者死/断流)"; fi
done

# 4. 近期日志错误扫描(latest 会话 + 最近 3 份录制留档)
LOGD="$WS/logs"
ERRPAT='ERROR|Traceback|断流|无消息|statusCode 8|ConnectionError'
if [ -d "$LOGD/latest" ]; then
    files="$(ls "$LOGD"/latest/*.log 2>/dev/null) $(ls -t "$LOGD"/record_*.log 2>/dev/null | head -3)"
    for f in $files; do
        [ -f "$f" ] || continue
        n=$(grep -cE "$ERRPAT" "$f" 2>/dev/null); n=${n:-0}
        if [ "$n" -gt 0 ] 2>/dev/null; then
            warn "$(basename "$f"): $n 处错误,最后一条:"
            grep -E "$ERRPAT" "$f" | tail -1 | cut -c1-160 | sed 's/^/      /'
        fi
    done
    ok "日志目录: logs/latest → $(basename "$(readlink "$LOGD/latest")")"
else
    warn "没有 logs/latest(消息中心还没用新版 start 脚本跑过,先用旧 /tmp 日志)"
fi

# 5. CAN 与磁盘
for i in can0 can5; do
    st=$(cat /sys/class/net/$i/operstate 2>/dev/null || echo gone)
    if [ "$st" = up ]; then ok "$i UP"; else warn "$i $st"; fi
done
df -h "$HOME" | awk 'NR==2{printf "  磁盘: %s 剩余/%s (用 %s)\n",$4,$2,$5}'
du -sh "${HF_LEROBOT_HOME:-$HOME/.cache/huggingface/lerobot}" 2>/dev/null | awk '{printf "  数据集缓存: %s\n",$1}'

echo "===== 结论: $([ $FAIL = 0 ] && echo "${GRN}通过${RST}" || echo "${RED}有 FAIL 项,按上面 ✗ 逐个排查${RST}") ====="
exit $FAIL
