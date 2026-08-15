#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

from ament_index_python.packages import get_package_prefix


def _add_runtime_to_path() -> None:
    runtime_dir = (
        Path(get_package_prefix("erob_moveit_runtime"))
        / "lib"
        / "erob_moveit_runtime"
    )
    sys.path.insert(0, str(runtime_dir))


_add_runtime_to_path()

from enums import Direction, RobotAxis  # noqa: E402
from runtime_gateway.twin_local import TwinLocalRuntime  # noqa: E402


def _scoped_joint_positions(gateway) -> tuple[list[str], list[float]]:
    state = gateway.node.current_joint_state
    if state is None:
        return [], []
    return (
        list(getattr(state, "name", []) or []),
        [float(value) for value in (getattr(state, "position", []) or [])],
    )


def _max_delta(before: list[float], after: list[float]) -> float:
    if len(before) != len(after) or not before:
        return float("inf")
    return max(abs(a - b) for a, b in zip(after, before))


def _wait_for_idle(gateway, timeout_s: float) -> bool:
    deadline = time.monotonic() + max(0.0, float(timeout_s))
    saw_motion = False
    while time.monotonic() < deadline:
        executing = bool(getattr(gateway.node, "is_executing", False))
        saw_motion = saw_motion or executing
        if saw_motion and not executing:
            return True
        time.sleep(0.05)
    return not bool(getattr(gateway.node, "is_executing", False))


def _print_runtime_summary(robots) -> None:
    print("\nREADINESS")
    print(robots.readiness())

    for name in ("robot1", "robot2"):
        gateway = robots.robot(name)
        names, positions = _scoped_joint_positions(gateway)
        snapshot = gateway.state_snapshot()
        print(f"\n{name.upper()}")
        print("joint_names:", names)
        print("joint_positions:", positions)
        print("cartesian_position:", snapshot.get("position"))
        print("runtime_ready:", snapshot.get("runtime_ready"))
        print("motion_stack_fault:", snapshot.get("motion_stack_fault"))


def _parse_direction(value: str) -> Direction:
    normalized = str(value).strip().lower()
    if normalized in {"+", "plus", "+1", "1"}:
        return Direction.PLUS
    if normalized in {"-", "minus", "-1"}:
        return Direction.MINUS
    raise argparse.ArgumentTypeError("direction must be +, -, plus, or minus")


def _run_jog_isolation_test(robots, args) -> int:
    selected = robots.robot(args.robot)
    other_name = "robot2" if args.robot == "robot1" else "robot1"
    other = robots.robot(other_name)

    selected_names_before, selected_before = _scoped_joint_positions(selected)
    other_names_before, other_before = _scoped_joint_positions(other)

    print("\nBEFORE JOG")
    print(args.robot, selected_names_before, selected_before)
    print(other_name, other_names_before, other_before)

    axis = RobotAxis.get_by_string(args.axis)
    direction = _parse_direction(args.direction)

    print(
        f"\nCommanding {args.robot}.jog(" 
        f"axis={axis.name}, direction={direction.name}, step={args.step}, "
        f"vel={args.vel}, acc={args.acc})"
    )

    result = selected.jog(
        axis,
        direction,
        float(args.step),
        float(args.vel),
        float(args.acc),
    )
    print("jog result:", result)

    if int(result) != 0:
        print("FAIL: jog command returned non-zero result")
        return 2

    if not _wait_for_idle(selected, args.timeout):
        print("FAIL: selected robot did not become idle before timeout")
        return 3

    time.sleep(0.25)

    selected_names_after, selected_after = _scoped_joint_positions(selected)
    other_names_after, other_after = _scoped_joint_positions(other)

    selected_delta = _max_delta(selected_before, selected_after)
    other_delta = _max_delta(other_before, other_after)

    print("\nAFTER JOG")
    print(args.robot, selected_names_after, selected_after)
    print(other_name, other_names_after, other_after)
    print(f"selected max joint delta: {selected_delta:.9f} rad")
    print(f"other max joint delta:    {other_delta:.9f} rad")

    if selected_names_before != selected_names_after:
        print("FAIL: selected robot joint-name set changed")
        return 4
    if other_names_before != other_names_after:
        print("FAIL: other robot joint-name set changed")
        return 5
    if selected_delta <= args.changed_epsilon:
        print("FAIL: selected robot joint state did not change")
        return 6
    if other_delta > args.unchanged_tolerance:
        print("FAIL: non-selected robot joint state changed")
        return 7

    print(
        f"PASS: {args.robot} moved while {other_name} remained unchanged "
        f"(tolerance={args.unchanged_tolerance:g} rad)"
    )
    return 0


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="TwinLocalRuntime readiness and robot-isolation test"
    )
    parser.add_argument(
        "--jog",
        action="store_true",
        help="perform one explicit planned jog after readiness checks",
    )
    parser.add_argument("--robot", choices=("robot1", "robot2"), default="robot1")
    parser.add_argument("--axis", choices=("x", "y", "z", "rx", "ry", "rz"), default="x")
    parser.add_argument("--direction", default="+")
    parser.add_argument(
        "--step",
        type=float,
        default=5.0,
        help="planned jog step (mm for XYZ, deg for RX/RY/RZ)",
    )
    parser.add_argument("--vel", type=float, default=10.0)
    parser.add_argument("--acc", type=float, default=10.0)
    parser.add_argument("--timeout", type=float, default=15.0)
    parser.add_argument("--changed-epsilon", type=float, default=1e-6)
    parser.add_argument("--unchanged-tolerance", type=float, default=1e-7)
    return parser


def main() -> int:
    args = _build_parser().parse_args()

    print(
        "Twin test environment: "
        f"ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '<unset>')} "
        f"fake={os.environ.get('ZEROERR_USE_FAKE_HARDWARE', '<unset>')}"
    )

    with TwinLocalRuntime() as robots:
        if not robots.wait_until_ready(args.timeout):
            print("FAIL: twin runtime did not become ready")
            print(robots.readiness())
            return 1

        _print_runtime_summary(robots)

        if not args.jog:
            print("\nPASS: twin runtime ready (read-only test)")
            return 0

        return _run_jog_isolation_test(robots, args)


if __name__ == "__main__":
    raise SystemExit(main())
