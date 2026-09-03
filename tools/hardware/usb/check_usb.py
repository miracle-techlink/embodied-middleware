#!/usr/bin/env python
"""Tier-1 体检:列出各 USB 设备的总线/协商速率,并**标记争用**——
当延迟敏感设备(CAN 适配器 PCAN、主臂串口 CH341/USB-serial)与相机共享同一个 USB2 hub 时,
相机的连续流会挤占 CAN/串口往返 → 拖慢控制循环。建议把相机(尤其 Orbbec)挪到独立 USB3 口。

纯读 /sys,不依赖额外包。用法: python scripts/check_usb.py
"""
import glob
import os

def _read(p):
    try:
        return open(p).read().strip()
    except OSError:
        return ""

def main():
    devs = []
    for d in glob.glob("/sys/bus/usb/devices/*"):
        prod = _read(os.path.join(d, "product"))
        if not prod:
            continue
        speed = _read(os.path.join(d, "speed"))  # Mbps
        busnum = _read(os.path.join(d, "busnum"))
        devpath = os.path.basename(d)  # e.g. 1-4.2.3
        devs.append((busnum, devpath, int(speed or 0), prod))

    devs.sort(key=lambda x: (x[0], x[1]))
    print(f"{'BUS':>3}  {'PATH':<12} {'SPEED':>7}  DEVICE")
    print("-" * 60)
    cam_hubs, crit_hubs = {}, {}
    for bus, path, spd, prod in devs:
        tag = ""
        low = prod.lower()
        is_cam = any(k in low for k in ("camera", "orbbec", "gemini", "webcam", "uvc"))
        is_crit = any(k in low for k in ("pcan", "peak", "can", "serial", "ch341", "cp210", "ftdi"))
        hub = ".".join(path.split(".")[:-1]) if "." in path else path  # 父 hub 路径
        if is_cam:
            tag = " [CAM]"; cam_hubs.setdefault(hub, []).append(prod)
        if is_crit:
            tag = " [CTRL]"; crit_hubs.setdefault(hub, []).append(prod)
        spd_s = f"{spd}M" + ("=USB3" if spd >= 5000 else "=USB2" if spd >= 480 else "")
        print(f"{bus:>3}  {path:<12} {spd_s:>7}  {prod}{tag}")

    print("\n=== 争用检查 ===")
    conflicts = [h for h in crit_hubs if h in cam_hubs]
    if conflicts:
        for h in conflicts:
            print(f"⚠ hub {h}: 延迟敏感设备 {crit_hubs[h]} 与相机 {cam_hubs[h]} 同 hub → 建议把相机挪到独立 USB3 口")
    else:
        print("✅ 未发现相机与 CAN/串口共享 hub 的争用。")

    orbbec = [(bus, path, spd) for bus, path, spd, prod in devs if "orbbec" in prod.lower() or "gemini" in prod.lower()]
    for bus, path, spd in orbbec:
        print(("✅ " if spd >= 5000 else "⚠ ") + f"Orbbec 在 {spd}M ({'USB3' if spd>=5000 else 'USB2 — 深度会被限速/不稳,建议挪 USB3'})")

if __name__ == "__main__":
    main()
