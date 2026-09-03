#!/usr/bin/env bash
# rebot 平台实时日志窗:聚合 tail 最新会话的全部节点/录制日志,错误红色、警告黄色。
#   rebot_watch.sh            # 跟踪 logs/latest/*
#   rebot_watch.sh -a         # 跟踪 logs/ 下全部日志(含历史录制留档)
# Ctrl-C 退出(只读,不动任何进程)。
set -o pipefail   # 不开 -u:ros2 setup.bash 裸引用未定义变量会被炸(同 start_msg_center 坑)
WS="$HOME/middleware"

source /opt/ros/jazzy/setup.bash 2>/dev/null || true

if [ "${1:-}" = "-a" ]; then
    FILES=$(ls "$WS"/logs/*.log 2>/dev/null)
    [ -z "$FILES" ] && FILES=$(ls "$WS"/logs/latest/*.log 2>/dev/null)
else
    FILES=$(ls "$WS"/logs/latest/*.log 2>/dev/null)
fi
# 兜底:消息中心还是旧脚本起的(没有 logs/latest)→ 跟 /tmp 旧位置
[ -z "$FILES" ] && FILES=$(ls /tmp/leader_node.log /tmp/arm_node.log /tmp/uvc_node.log /tmp/orbbec_node.log /tmp/map_node.log 2>/dev/null)
[ -z "$FILES" ] && { echo "!! 没有可跟踪的日志($WS/logs/latest/ 不存在或为空)——先跑一次 start_teleop.sh 或录制"; exit 1; }

echo "== 跟踪:$FILES"
echo "== 规则:红=ERROR/Traceback/断流/无消息/statusCode,黄=WARN/警告。Ctrl-C 退出。"
tail -F $FILES 2>/dev/null | while IFS= read -r line; do
    case "$line" in
        *ERROR*|*Error*|*Traceback*|*断流*|*无消息*|*statusCode*|*失败*)
            printf '\033[1;31m%s\033[0m\n' "$line" ;;
        *WARN*|*警告*|*⚠*|*超时*)
            printf '\033[1;33m%s\033[0m\n' "$line" ;;
        *)
            printf '%s\n' "$line" ;;
    esac
done
