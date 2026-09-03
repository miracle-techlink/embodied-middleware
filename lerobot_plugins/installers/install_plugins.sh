#!/usr/bin/env bash
# 把单臂 reBot 采集所需的 lerobot 插件装进 lerobot 源码树,并在 __init__.py 里注册
# (触发 register_subclass,使 --robot.type / --teleop.type 可用)。
#
# 用法: LEROBOT_SRC=/path/to/lerobot bash install.sh
#   LEROBOT_SRC 指向 lerobot 仓库根(其下有 src/lerobot/),默认 ~/lingbot/lerobot
#
# 依赖(装在同一个 lerobot 环境里):
#   - 官方 pip 插件 lerobot-robot-seeed-b601 (提供被继承的 seeed_b601_rs_follower)
#   - 官方 pip 插件 lerobot-teleoperator-rebot-arm-102(可选,若用 102-L 主臂)
#   - StarAI uservo (若用 StarAI Violin 主臂)
# 相机(录深度需要)另跑: bash install_orbbec.sh
set -e
HERE="$(cd "$(dirname "$0")/.." && pwd)"   # installers/ 的上一级 = lerobot_plugins 根
LEROBOT_SRC="${LEROBOT_SRC:-$HOME/lingbot/lerobot}"
LB="$LEROBOT_SRC/src/lerobot"
[ -d "$LB" ] || { echo "!! 找不到 lerobot 源码: $LB (设 LEROBOT_SRC=lerobot仓库根)"; exit 1; }

echo "[plugins] 复制 reBot 采集插件 -> $LB"
cp -r "$HERE/plugins/robots/rebot_follower"                      "$LB/robots/"
cp -r "$HERE/plugins/teleoperators/starai_violin_leader"         "$LB/teleoperators/"
cp -r "$HERE/plugins/teleoperators/starai_to_rebot_leader"       "$LB/teleoperators/"

add_import() {  # 幂等追加注册导入
  local f="$1" line="$2"
  grep -qF "$line" "$f" || echo "$line" >> "$f"
}
# rebot_follower: 继承官方 seeed_b601_rs_follower(pip)+ 深度进观测 + 退出平滑回零
add_import "$LB/robots/__init__.py"        "from . import rebot_follower  # noqa: F401"
# starai_violin_leader: StarAI Violin 主臂(飞特 UART 舵机)
add_import "$LB/teleoperators/__init__.py" "from . import starai_violin_leader  # noqa: F401"
# starai_to_rebot_leader: 把 StarAI 主臂映射到 reBot 关节空间(绝对映射 + 启动限速 ramp + 夹爪)
add_import "$LB/teleoperators/__init__.py" "from . import starai_to_rebot_leader  # noqa: F401"

echo "[plugins] 完成。已注册:"
echo "         robot         : rebot_follower"
echo "         teleoperators : starai_violin_leader / starai_to_rebot_leader"
echo "[plugins] 录深度数据集还需相机插件:  bash install_orbbec.sh"
