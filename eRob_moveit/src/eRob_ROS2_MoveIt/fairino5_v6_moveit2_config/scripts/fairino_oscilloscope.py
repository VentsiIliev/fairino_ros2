#!/usr/bin/env python3
from __future__ import annotations

import os
import runpy
import sys
from pathlib import Path

from ament_index_python.packages import get_package_prefix


def main() -> None:
    os.environ["EROB_CONFIG_PACKAGE"] = "fairino5_v6_moveit2_config"
    prefix = Path(get_package_prefix("erob_moveit_runtime"))
    oscilloscope_main = prefix / "lib" / "erob_moveit_runtime" / "simple_monitor_gui.py"
    sys.path.insert(0, str(oscilloscope_main.parent))
    runpy.run_path(str(oscilloscope_main), run_name="__main__")


if __name__ == "__main__":
    main()
