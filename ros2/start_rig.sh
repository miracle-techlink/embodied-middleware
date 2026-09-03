#!/usr/bin/env bash
# profile 驱动入口;旧 start_msg_center.sh 保持不变,避免现场路径中断。
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PYTHONPATH="$HERE/src/middleware:${PYTHONPATH:-}"
exec python3 "$HERE/launch/start_rig.py" "$@"
