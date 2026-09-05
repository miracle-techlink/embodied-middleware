#!/usr/bin/env bash
# 主从跟随逐关节交互自检 —— canonical 实现 joint_sweep.py,这里补环境。
# 前提:消息中心已起(~/middleware/ros2/start_teleop.sh),主臂已连。
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"          # tools/diagnostics/
REPO_ROOT="$(cd "$HERE/../.." && pwd)"                          # 仓库根
WS="$REPO_ROOT/ros2"
PY="${PY:-$HOME/miniconda3/envs/data_collect/bin/python}"

source /opt/ros/jazzy/setup.bash
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$WS/src/middleware:${PYTHONPATH:-}"

exec "$PY" "$HERE/joint_sweep.py" "$@"
