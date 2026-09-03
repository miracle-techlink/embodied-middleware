#!/usr/bin/env bash
# 装"1080P USB Camera(2bdf:0289)禁用 pipewire 探测"的用户级规则 —— 修鼠标卡顿。
#
# 背景:该相机 UVC 控制端点固件不响应,GNOME 桌面 pipewire(spa.v4l2)把它当门户摄像头
# 反复探测,每次在内核里阻塞等 5 秒超时,重试风暴卡整个 USB 栈 → 鼠标一阵一阵跳帧。
# 本规则让 wireplumber 按设备描述直接忽略它(仅 v4l2;麦克风 alsa 设备保留;
# 采集代码走 OpenCV 直连 /dev/video* 不受影响,换 USB 口也照样生效)。
#
# 用户级配置,无需 sudo。用法: bash scripts/install_no_camera_probe.sh
set -e
DIR="$HOME/.config/wireplumber/main.lua.d"
mkdir -p "$DIR"
cat > "$DIR/51-ignore-1080p-cam.lua" <<'EOF'
-- 1080P USB Camera (2bdf:0289) 的 UVC 控制端点不响应,pipewire 每次探测
-- 都要在内核里阻塞等 5 秒超时,反复重试导致整个 USB 栈抖动、鼠标卡顿。
-- 让 wireplumber 直接忽略它(采集栈走 OpenCV/pyav 直连 /dev/video*,不受影响)。
-- 按设备描述匹配,换 USB 口也生效;Orbbec 的 pipewire 可见性不受影响。
rule = {
  matches = {
    {
      { "device.description", "equals", "1080P USB Camera" },
    },
  },
  apply_properties = {
    ["device.disabled"] = true,
  },
}

table.insert(v4l2_monitor.rules, rule)
EOF

systemctl --user restart wireplumber 2>/dev/null || echo "!! wireplumber 重启失败(无桌面会话?),下次登录生效"
echo "[no-camera-probe] 已装 $DIR/51-ignore-1080p-cam.lua 并重启 wireplumber"
echo "[no-camera-probe] 验证: pw-dump | grep v4l2_device 应只剩 Orbbec,没有 1080P USB Camera"
