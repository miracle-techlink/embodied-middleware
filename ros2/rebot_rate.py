#!/usr/bin/env python3
"""旧入口兼容 wrapper; canonical 实现位于 admin/rebot_rate.py。"""
from pathlib import Path
import runpy
runpy.run_path(str(Path(__file__).parent / "admin" / "rebot_rate.py"), run_name="__main__")
