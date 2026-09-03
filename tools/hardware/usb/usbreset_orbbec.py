import fcntl, os, re, subprocess
USBDEVFS_RESET = ord('U') << 8 | 20
# find all Orbbec 2bc5:0840 bus/dev
out = subprocess.check_output(["lsusb"]).decode()
for line in out.splitlines():
    m = re.match(r"Bus (\d+) Device (\d+): ID 2bc5:0840", line)
    if not m:
        continue
    bus, dev = m.group(1), m.group(2)
    path = f"/dev/bus/usb/{bus}/{dev}"
    try:
        fd = os.open(path, os.O_WRONLY)
        fcntl.ioctl(fd, USBDEVFS_RESET, 0)
        os.close(fd)
        print(f"[reset OK] {path}  ({line.strip()})")
    except Exception as e:
        print(f"[reset FAIL] {path}: {type(e).__name__}: {e}")
