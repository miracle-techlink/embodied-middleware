#!/usr/bin/env python3
"""旧入口兼容 wrapper; canonical 实现位于 admin/rebot_go_home.py。"""
from pathlib import Path
import runpy
runpy.run_path(str(Path(__file__).parent / "admin" / "rebot_go_home.py"), run_name="__main__")
