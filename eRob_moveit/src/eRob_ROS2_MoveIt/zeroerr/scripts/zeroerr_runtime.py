#!/usr/bin/env python3
from __future__ import annotations

import os
import runpy
from pathlib import Path

from ament_index_python.packages import get_package_prefix


def main() -> None:
    os.environ["EROB_CONFIG_PACKAGE"] = "zeroerr"
    prefix = Path(get_package_prefix("erob_moveit_runtime"))
    runtime_main = prefix / "lib" / "erob_moveit_runtime" / "main.py"
    runpy.run_path(str(runtime_main), run_name="__main__")


if __name__ == "__main__":
    main()
