#!/usr/bin/env bash
# 推理模式专用启动器(只起 arm+uvc+orbbec,不起 leader/teleop_map —— follower 指令源
# 只有策略,不被遥操作映射抢占)。等价 start_msg_center.sh infer(引擎仍是它)。
#   ./start_infer.sh           # 起(前台,Ctrl-C 全停,arm 平滑回零)
#   ./start_infer.sh stop      # 只停
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
case "${1:-}" in
    stop) exec bash "$here/start_msg_center.sh" stop ;;
    *)    exec bash "$here/start_msg_center.sh" infer ;;
esac
