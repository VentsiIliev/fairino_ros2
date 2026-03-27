#!/usr/bin/env python3
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from rest_server import start_rest_server


if __name__ == "__main__":
    start_rest_server()
