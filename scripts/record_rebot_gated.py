#!/usr/bin/env python3
"""旧入口兼容 wrapper; canonical 实现位于 tools/acquisition/record_rebot_gated.py."""
from pathlib import Path
import runpy
runpy.run_path(str(Path(__file__).parents[1] / "tools/acquisition/record_rebot_gated.py"), run_name="__main__")
