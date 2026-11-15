#!/usr/bin/env bash
# 主/从参考零位档案与一致性自检 wrapper —— canonical 实现 piper_zero_check.py,这里补环境。
#
# 用途:PiPER 无 lerobot 软件标定文件,零位在固件/驱动器 EEPROM(JointConfig 0xAE 写入)。
#       本工具建档/比对的是「参考姿态下的主从读数」,用来保证跨 session:
#       同一个数值 = 同一个物理姿态(数据采集/推理一致性)。
# 前提:start_piper_teleop.sh 已起(piper_arm_node + piper_leader_node 在跑)。
#       纯订阅 topic,只读,不动臂。
#
# 用法:
#   bash piper_zero_check.sh sample               # 采样当前姿势(先看主从差/对齐)
#   bash piper_zero_check.sh archive --label "参考姿态名" [--out FILE]   # 建档
#   bash piper_zero_check.sh verify [--out FILE]  # 与档案比对;退出码 0=通过 1=超差 2=环境失败
# 环境变量:PIPER_HOME_FILE 可覆盖默认档案路径。
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"      # tools/diagnostics/
REPO_ROOT="$(cd "$HERE/../.." && pwd)"                    # 仓库根
WS="$REPO_ROOT/ros2"
PY="${PY:-$HOME/miniconda3/envs/data_collect/bin/python}"

source /opt/ros/jazzy/setup.bash
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$WS/src/middleware:${PYTHONPATH:-}"

exec "$PY" "$HERE/piper_zero_check.py" "$@"
