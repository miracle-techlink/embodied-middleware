#!/usr/bin/env python3
"""旧入口兼容 wrapper; canonical 实现位于 tools/hardware/usb/check_usb.py."""
from pathlib import Path
import runpy
runpy.run_path(str(Path(__file__).parents[1] / "tools/hardware/usb/check_usb.py"), run_name="__main__")
