#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import sys
import time
from concurrent.futures import ThreadPoolExecutor
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


def _run_concurrent_jog_test(robots, args) -> int:
    robot1 = robots.robot("robot1")
    robot2 = robots.robot("robot2")

    names1_before, pos1_before = _scoped_joint_positions(robot1)
    names2_before, pos2_before = _scoped_joint_positions(robot2)

    axis1 = RobotAxis.get_by_string(args.axis)
    axis2 = RobotAxis.get_by_string(args.axis2)
    dir1 = _parse_direction(args.direction)
    dir2 = _parse_direction(args.direction2)

    print("\nBEFORE CONCURRENT JOG")
    print("robot1", names1_before, pos1_before)
    print("robot2", names2_before, pos2_before)
    print(
        "\nStarting concurrent jogs:\n"
        f"  robot1 axis={axis1.name} direction={dir1.name} step={args.step} "
        f"vel={args.vel} acc={args.acc}\n"
        f"  robot2 axis={axis2.name} direction={dir2.name} step={args.step2} "
        f"vel={args.vel2} acc={args.acc2}"
    )

    start_gate = __import__("threading").Barrier(3)

    def run_one(gateway, axis, direction, step, vel, acc):
        start_gate.wait()
        started_at = time.monotonic()
        result = gateway.jog(axis, direction, step, vel, acc)
        returned_at = time.monotonic()
        return result, started_at, returned_at

    with ThreadPoolExecutor(max_workers=2) as pool:
        future1 = pool.submit(
            run_one,
            robot1,
            axis1,
            dir1,
            float(args.step),
            float(args.vel),
            float(args.acc),
        )
        future2 = pool.submit(
            run_one,
            robot2,
            axis2,
            dir2,
            float(args.step2),
            float(args.vel2),
            float(args.acc2),
        )
        start_gate.wait()
        result1, start1, return1 = future1.result(timeout=args.timeout)
        result2, start2, return2 = future2.result(timeout=args.timeout)

    print("robot1 jog result:", result1)
    print("robot2 jog result:", result2)
    print(f"dispatch start separation: {abs(start1 - start2):.6f}s")
    print(f"gateway return separation: {abs(return1 - return2):.6f}s")

    if int(result1) != 0 or int(result2) != 0:
        print("FAIL: one or both concurrent jog commands returned non-zero")
        return 10

    idle_deadline = time.monotonic() + max(0.0, float(args.timeout))
    while time.monotonic() < idle_deadline:
        exec1 = bool(getattr(robot1.node, "is_executing", False))
        exec2 = bool(getattr(robot2.node, "is_executing", False))
        if not exec1 and not exec2:
            break
        time.sleep(0.05)
    else:
        print("FAIL: one or both robots remained executing past timeout")
        return 11

    time.sleep(0.25)

    names1_after, pos1_after = _scoped_joint_positions(robot1)
    names2_after, pos2_after = _scoped_joint_positions(robot2)
    delta1 = _max_delta(pos1_before, pos1_after)
    delta2 = _max_delta(pos2_before, pos2_after)

    print("\nAFTER CONCURRENT JOG")
    print("robot1", names1_after, pos1_after)
    print("robot2", names2_after, pos2_after)
    print(f"robot1 max joint delta: {delta1:.9f} rad")
    print(f"robot2 max joint delta: {delta2:.9f} rad")

    if names1_before != names1_after:
        print("FAIL: robot1 joint-name set changed")
        return 12
    if names2_before != names2_after:
        print("FAIL: robot2 joint-name set changed")
        return 13
    if delta1 <= args.changed_epsilon:
        print("FAIL: robot1 joint state did not change")
        return 14
    if delta2 <= args.changed_epsilon:
        print("FAIL: robot2 joint state did not change")
        return 15

    expected1 = list(robot1.node.robot_context.joint_names)
    expected2 = list(robot2.node.robot_context.joint_names)
    if names1_after != expected1:
        print("FAIL: robot1 state contains joints outside robot1 runtime context")
        return 16
    if names2_after != expected2:
        print("FAIL: robot2 state contains joints outside robot2 runtime context")
        return 17
    if set(names1_after) & set(names2_after):
        print("FAIL: robot runtime joint-name sets overlap")
        return 18

    print("PASS: both robots moved concurrently with independent scoped joint state")
    return 0


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="TwinLocalRuntime readiness and robot-isolation test"
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument(
        "--jog",
        action="store_true",
        help="perform one explicit planned jog after readiness checks",
    )
    mode.add_argument(
        "--concurrent-jog",
        action="store_true",
        help="jog robot1 and robot2 concurrently and verify both scoped states",
    )
    parser.add_argument("--robot", choices=("robot1", "robot2"), default="robot1")
    parser.add_argument("--axis", choices=("x", "y", "z", "rx", "ry", "rz"), default="x")
    parser.add_argument("--direction", default="+")
    parser.add_argument(
        "--step",
        type=float,
        default=5.0,
        help="robot1/single planned jog step (mm for XYZ, deg for RX/RY/RZ)",
    )
    parser.add_argument("--vel", type=float, default=10.0)
    parser.add_argument("--acc", type=float, default=10.0)
    parser.add_argument("--axis2", choices=("x", "y", "z", "rx", "ry", "rz"), default="x")
    parser.add_argument("--direction2", default="+")
    parser.add_argument("--step2", type=float, default=5.0)
    parser.add_argument("--vel2", type=float, default=10.0)
    parser.add_argument("--acc2", type=float, default=10.0)
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

        if args.concurrent_jog:
            return _run_concurrent_jog_test(robots, args)
        if args.jog:
            return _run_jog_isolation_test(robots, args)

        print("\nPASS: twin runtime ready (read-only test)")
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
