#!/usr/bin/env python3
from setproctitle import setproctitle

setproctitle("rest_server")
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from rest.server import start_rest_server
from runtime_initializer import initialize_robot_runtime


def main():
    start_rest_server(
        start_ros=False,
        runtime_initializer=initialize_robot_runtime,
        allow_starting_without_robot=True,
    )


if __name__ == "__main__":
    main()
