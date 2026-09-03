#!/usr/bin/env bash
# 可选:安装 Orbbec 深度相机(Gemini 系列)lerobot 插件(RGB + 对齐深度)。
# 遥操作本身不需要相机;录数据集/rerun 可视化时才用。需先 `pip install pyorbbecsdk2`。
# 用法: LEROBOT_SRC=/path/to/lerobot bash install_orbbec.sh
set -e
HERE="$(cd "$(dirname "$0")/.." && pwd)"   # installers/ 的上一级 = lerobot_plugins 根
LEROBOT_SRC="${LEROBOT_SRC:-/home/tommyzihao/lingbot/lerobot}"
LB="$LEROBOT_SRC/src/lerobot"
[ -d "$LB" ] || { echo "!! 找不到 lerobot 源码: $LB (设 LEROBOT_SRC)"; exit 1; }

echo "[orbbec] 复制相机插件 -> $LB/cameras/orbbec"
cp -r "$HERE/plugins/cameras/orbbec" "$LB/cameras/"

# 幂等地打进 lerobot 核心的 4 处小改(注册 type: orbbec + CLI 可解析)
python - "$LB" <<'PY'
import sys, io, os
LB = sys.argv[1]

def patch(path, anchor, addition, once_key=None):
    p = os.path.join(LB, path)
    s = io.open(p, encoding="utf-8").read()
    if (once_key or addition) in s:
        print("  skip ", path); return
    if anchor not in s:
        print("  !! anchor not found in", path, "-> 手动加:", addition.strip()); return
    s = s.replace(anchor, anchor + addition, 1)
    io.open(p, "w", encoding="utf-8").write(s); print("  patch", path)

# 1) cameras/utils.py: 工厂分发加 orbbec 分支
patch("cameras/utils.py",
      '        elif cfg.type == "zmq":\n            from .zmq.camera_zmq import ZMQCamera\n\n            cameras[key] = ZMQCamera(cfg)\n',
      '\n        elif cfg.type == "orbbec":\n            from .orbbec.camera_orbbec import OrbbecCamera\n\n            cameras[key] = OrbbecCamera(cfg)\n',
      once_key='cfg.type == "orbbec"')

# 2) utils/import_utils.py: 可用性标志
patch("utils/import_utils.py",
      '_pyrealsense2_available = is_package_available("pyrealsense2") or is_package_available(\n    "pyrealsense2-macosx", import_name="pyrealsense2"\n)',
      '\n_pyorbbecsdk_available = is_package_available("pyorbbecsdk2", import_name="pyorbbecsdk") or is_package_available(\n    "pyorbbecsdk"\n)',
      once_key="_pyorbbecsdk_available")

# 3) 各 CLI 脚本导入 config 以触发注册(draccus 解析 type: orbbec 前需注册)
for script in ["scripts/lerobot_teleoperate.py","scripts/lerobot_record.py",
               "scripts/lerobot_calibrate.py","scripts/lerobot_rollout.py"]:
    patch(script,
          "from lerobot.cameras.realsense import RealSenseCameraConfig  # noqa: F401\n",
          "from lerobot.cameras.orbbec import OrbbecCameraConfig  # noqa: F401\n",
          once_key="OrbbecCameraConfig")
PY

echo "[orbbec] 完成。udev 权限: python -c \"import pyorbbecsdk,os;print(os.path.dirname(pyorbbecsdk.__file__))\" 下的 shared/install_udev_rules.sh (sudo)"
echo "[orbbec] 列相机: python -c \"from lerobot.cameras.orbbec import OrbbecCamera;import json;print(json.dumps(OrbbecCamera.find_cameras(),default=str,indent=2))\""
