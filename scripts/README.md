# scripts/ — 旧入口兼容 wrapper(不再放新代码)

本目录的 `.sh` / `.py` 全部是对 `tools/` canonical 实现的透传,为旧文档、部署
脚本和肌肉记忆保留;**新代码一律进 `tools/` 对应分类目录**:

```bash
scripts/teleop_rebot.sh          → tools/acquisition/
scripts/record_rebot.sh          → tools/acquisition/
scripts/record_rebot_gated.sh    → tools/acquisition/
scripts/record_rebot_gated.py    → tools/acquisition/
scripts/setup_env.sh             → tools/environment/
scripts/setup_rebot_can.sh       → tools/hardware/can/
scripts/install_can_udev.sh      → tools/hardware/can/
scripts/check_usb.py             → tools/hardware/usb/
scripts/usbreset_orbbec.py       → tools/hardware/usb/
scripts/install_usb_noautosuspend.sh  → tools/hardware/usb/
scripts/install_no_camera_probe.sh    → tools/hardware/usb/
scripts/maxn_lock.sh             → tools/hardware/power/
scripts/estop_release.sh         → tools/hardware/power/
scripts/probe_arm.py             → tools/diagnostics/
scripts/profile_loop.sh          → tools/diagnostics/
scripts/profile_loop.py          → tools/diagnostics/
```

分类判定规则见 `docs/architecture/DIRECTORY_LAYOUT.md`。
