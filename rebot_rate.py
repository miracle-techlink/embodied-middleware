#!/usr/bin/env python3
from pathlib import Path
runpy = __import__("runpy")
runpy.run_path(str(Path(__file__).parent / "admin" / "rebot_rate.py"), run_name="__main__")
