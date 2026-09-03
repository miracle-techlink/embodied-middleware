#!/usr/bin/env bash
# 一键搭建 reBot 采集环境 —— 全新机器从零到可采集。
# 与训练等其他栈隔离:独立 conda env + 独立 lerobot 源码克隆(不碰已有 lerobot 树)。
#
# 做的事:
#   1) conda 建隔离 env(默认 data_collect,python 3.12;已存在则复用)
#   2) 克隆上游 lerobot(默认 tag v0.6.1)到 LEROBOT_SRC,editable 安装
#   3) pip 依赖:lerobot-robot-seeed-b601(reBot 官方 follower + motorbridge-cli)
#                pyorbbecsdk2(Orbbec 深度相机)pyserial(StarAI 主臂串口)
#                fashionstar-uart-sdk(StarAI 舵机 SDK,import 名 uservo)
#                rerun-sdk(--display_data 可视化)
#   4) 打进采集插件三件套:lerobot_plugins/install.sh + install_orbbec.sh + install_depthfix.sh
#   5) WITH_SUDO=1 时顺带:Orbbec udev 规则 + 当前用户加 dialout 组(否则打印手动步骤)
#
# 用法:
#   bash scripts/setup_env.sh                          # 全默认
#   ENV_NAME=collect2 bash scripts/setup_env.sh        # 换 env 名(多机采集时区分)
#   LEROBOT_TAG=v0.6.1 LEROBOT_SRC=~/lerobot-dc bash scripts/setup_env.sh
#   WITH_SUDO=1 bash scripts/setup_env.sh              # 连 sudo 步骤一起做
#
# 做完后(新开终端或 source ~/.bashrc 之后):
#   conda activate ${ENV_NAME:-data_collect}
#   sudo bash scripts/setup_rebot_can.sh               # 拉起 CAN
#   lerobot-calibrate --robot.type=rebot_follower ...  # 首次标定
set -e
HERE="$(cd "$(dirname "$0")/../.." && pwd)"             # 仓库根(tools/environment/ 的上两级)

ENV_NAME="${ENV_NAME:-data_collect}"
PY_VER="${PY_VER:-3.12}"
LEROBOT_TAG="${LEROBOT_TAG:-v0.6.1}"
LEROBOT_SRC="${LEROBOT_SRC:-$HOME/lerobot-datacollect}"
WITH_SUDO="${WITH_SUDO:-0}"

# ---- 1) conda env -----------------------------------------------------------
if ! command -v conda >/dev/null 2>&1; then
  for base in "$HOME/miniconda3" "$HOME/anaconda3"; do
    [ -f "$base/etc/profile.d/conda.sh" ] && source "$base/etc/profile.d/conda.sh" && break
  done
fi
command -v conda >/dev/null 2>&1 || { echo "!! 找不到 conda,请先装 miniconda"; exit 1; }
source "$(conda info --base)/etc/profile.d/conda.sh"

if conda env list | grep -qE "^${ENV_NAME} "; then
  echo "[env] conda env '$ENV_NAME' 已存在,复用"
else
  echo "[env] 创建 conda env '$ENV_NAME' (python $PY_VER) ..."
  conda create -n "$ENV_NAME" "python=$PY_VER" -y
fi
conda activate "$ENV_NAME"

# ---- 2) lerobot 源码 + editable 安装 ----------------------------------------
if [ ! -d "$LEROBOT_SRC/src/lerobot" ]; then
  echo "[lerobot] 克隆 huggingface/lerobot @$LEROBOT_TAG -> $LEROBOT_SRC"
  git clone --depth 1 --branch "$LEROBOT_TAG" \
    https://github.com/huggingface/lerobot "$LEROBOT_SRC"
else
  echo "[lerobot] $LEROBOT_SRC 已存在,跳过克隆"
fi

echo "[pip] 安装 lerobot(editable)+ 采集依赖 ..."
pip install -e "$LEROBOT_SRC" \
  lerobot-robot-seeed-b601 pyorbbecsdk2 pyserial fashionstar-uart-sdk rerun-sdk

# ---- 3) 采集插件三件套 -------------------------------------------------------
export LEROBOT_SRC
bash "$HERE/lerobot_plugins/installers/install_plugins.sh"
bash "$HERE/lerobot_plugins/installers/install_orbbec.sh"
bash "$HERE/lerobot_plugins/installers/install_depthfix.sh"

# ---- 4) sudo 步骤(udev / dialout)-------------------------------------------
UDEV_SH="$(python - <<'PY'
import importlib, os
try:
    m = importlib.import_module("pyorbbecsdk")
    print(os.path.join(os.path.dirname(m.__file__), "shared", "install_udev_rules.sh"))
except Exception:
    pass
PY
)"
if [ "$WITH_SUDO" = "1" ]; then
  echo "[sudo] 装 Orbbec udev 规则 + 当前用户加 dialout 组 ..."
  [ -f "$UDEV_SH" ] && sudo bash "$UDEV_SH" && sudo udevadm control --reload && sudo udevadm trigger
  sudo usermod -aG dialout "$USER"
  echo "[sudo] 完成。dialout 组要重新登录才生效。"
else
  cat <<EOF

[手动] 还需 3 个 sudo 步骤(或重跑 WITH_SUDO=1 bash scripts/setup_env.sh):
  1) Orbbec udev 规则(否则 find_cameras 报 openUsbDevice failed):
     sudo bash $UDEV_SH && sudo udevadm control --reload && sudo udevadm trigger
  2) 串口权限(StarAI 主臂 /dev/ttyUSB*):sudo usermod -aG dialout $USER  # 重新登录生效
  3) 每次采集前拉起 CAN:sudo bash $HERE/tools/hardware/can/setup_rebot_can.sh
EOF
fi

cat <<EOF

[done] 环境就绪。验证:
  conda activate $ENV_NAME
  motorbridge-cli scan --vendor robstride --channel can0 --start-id 1 --end-id 7   # 应看到 7 个电机
  python -c "from lerobot.cameras.orbbec import OrbbecCamera; print(OrbbecCamera.find_cameras())"
注意:
  - 主臂串口节点名因机器而异:/dev/ttyCH341USB0(Jetson 定制驱动)或 /dev/ttyUSB0(主线 ch341),用 LEADER_PORT= 覆盖
  - Orbbec 务必插 USB3 口(SDK 报 USB2.1 = 掉到 USB2,深度带宽不够)
  - lerobot 升级会覆盖插件 → 重跑本脚本第 3 步(或整条)
EOF
