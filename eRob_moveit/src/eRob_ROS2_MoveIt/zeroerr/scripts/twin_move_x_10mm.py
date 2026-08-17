#!/usr/bin/env python3
"""Move each twin robot's current TCP pose +10 mm along its local X axis."""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

from ament_index_python.packages import get_package_prefix


os.environ.setdefault("EROB_CONFIG_PACKAGE", "zeroerr")
os.environ.setdefault("ZEROERR_ACTIVE_PROFILE", "twin_robots")

runtime_dir = Path(get_package_prefix("erob_moveit_runtime")) / "lib" / "erob_moveit_runtime"
sys.path.insert(0, str(runtime_dir))

from runtime_gateway.twin_local import TwinLocalRuntime  # noqa: E402


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Move the selected twin robot(s) 10 mm along local X."
    )
    parser.add_argument(
        "--robot",
        choices=("both", "robot1", "robot2"),
        default="both",
        help="robot to move (default: both)",
    )
    parser.add_argument(
        "--step-mm",
        type=float,
        default=10.0,
        help="positive local-X distance in millimetres (default: 10)",
    )
    parser.add_argument("--vel", type=float, default=10.0, help="velocity percent")
    parser.add_argument("--acc", type=float, default=10.0, help="acceleration percent")
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument(
        "--yes",
        action="store_true",
        help="skip the motion confirmation prompt",
    )
    return parser


def main() -> int:
    args = _parser().parse_args()
    if args.step_mm <= 0:
        print("step-mm must be positive", file=sys.stderr)
        return 2

    selected = ("robot1", "robot2") if args.robot == "both" else (args.robot,)

    with TwinLocalRuntime() as robots:
        if not robots.wait_until_ready(args.timeout):
            print("Twin runtime is not ready:", robots.readiness(), file=sys.stderr)
            return 1

        targets = {}
        for name in selected:
            pose = robots.robot(name).get_current_position()
            if pose is None or len(pose) != 6:
                print(f"{name}: current Cartesian pose is unavailable", file=sys.stderr)
                return 1
            target = [float(value) for value in pose]
            target[0] += args.step_mm
            targets[name] = target
            print(f"{name}: current={pose}")
            print(f"{name}: target ={target}")

        if not args.yes:
            answer = input("Execute these motion commands? Type 'yes' to continue: ")
            if answer.strip().lower() != "yes":
                print("Cancelled.")
                return 0

        for name in selected:
            print(f"Moving {name}...", flush=True)
            result = robots.robot(name).move_linear(
                targets[name],
                vel=args.vel,
                acc=args.acc,
                blocking=True,
            )
            print(f"{name}: result={result}")
            if int(result) != 0:
                print(f"{name}: motion failed; stopping further commands", file=sys.stderr)
                return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
