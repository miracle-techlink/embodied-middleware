#!/usr/bin/env python3
"""旧入口兼容 wrapper; canonical 实现位于 tools/diagnostics/probe_arm.py."""
from pathlib import Path
import runpy
runpy.run_path(str(Path(__file__).parents[1] / "tools/diagnostics/probe_arm.py"), run_name="__main__")
