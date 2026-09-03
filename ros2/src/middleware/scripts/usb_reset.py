#!/usr/bin/env python3
"""usb_reset.py — 采集设备 USB 软复位,免拔插恢复。

扩展自 Galaxea_rebot_starai_tele/scripts/usbreset_orbbec.py(只复位 Orbbec),
覆盖整条采集链。USBDEVFS_RESET ioctl = 端口级软复位,设备重新枚举,进程无需重启
(但占用设备的节点要重开句柄,消息中心架构下重启对应节点即可)。

用法:
    python3 usb_reset.py            # 复位全部采集设备
    python3 usb_reset.py orbbec     # 只复位某一类:orbbec | pcan | ch341 | frontcam
"""

import fcntl
import os
import re
import subprocess
import sys

USBDEVFS_RESET = ord("U") << 8 | 20

DEVICES = {
    "orbbec": ("2bc5", "0840"),    # Gemini 305 腕部深度相机
    "pcan": ("0c72", "000c"),      # PCAN-USB(从臂 CAN)
    "ch341": ("1a86", "7523"),     # CH341(StarAI 主臂串口)
    "frontcam": ("2bdf", "0289"),  # SN0002 前视相机
}


def find_bus_dev(vid: str, pid: str) -> list[str]:
    out = subprocess.run(["lsusb"], capture_output=True, text=True).stdout
    paths = []
    for line in out.splitlines():
        m = re.match(rf"Bus (\d+) Device (\d+): ID {vid}:{pid}", line)
        if m:
            paths.append(f"/dev/bus/usb/{m.group(1)}/{m.group(2)}")
    return paths


def reset(path: str) -> bool:
    try:
        fd = os.open(path, os.O_WRONLY)
        fcntl.ioctl(fd, USBDEVFS_RESET, 0)
        os.close(fd)
        return True
    except Exception as e:
        print(f"  [FAIL] {path}: {e}")
        return False


def main():
    targets = sys.argv[1:] or list(DEVICES)
    for name in targets:
        vid, pid = DEVICES[name]
        paths = find_bus_dev(vid, pid)
        if not paths:
            print(f"{name} ({vid}:{pid}): 不在总线上,只能物理重插")
            continue
        for p in paths:
            ok = reset(p)
            print(f"{name} ({vid}:{pid}) {p}: {'[reset OK]' if ok else '[reset FAIL]'}")


if __name__ == "__main__":
    main()
