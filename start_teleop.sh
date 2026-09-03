#!/usr/bin/env bash
# 采集/遥操作模式专用启动器(五节点全起:leader+arm+uvc+orbbec+teleop_map)。
# 等价 start_msg_center.sh teleop(引擎仍是它,别处逻辑不复制)。
#   ./start_teleop.sh          # 起(前台,Ctrl-C 全停,arm 平滑回零)
#   ./start_teleop.sh stop     # 只停
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
case "${1:-}" in
    stop) exec bash "$here/start_msg_center.sh" stop ;;
    *)    exec bash "$here/start_msg_center.sh" teleop ;;
esac
