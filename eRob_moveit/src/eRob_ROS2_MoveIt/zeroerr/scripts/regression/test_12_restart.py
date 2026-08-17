#!/usr/bin/env python3
"""Regression test 12: sequential runtime close/reopen.

Creates a runtime, probes it, fully tears it down (executor shutdown,
node destruction, ``rclpy.shutdown``), then constructs a fresh runtime
and verifies it reaches readiness and can move again.

Runs two runtimes **sequentially** (never concurrently) to avoid
rclpy re-initialisation issues on ROS 2 Jazzy.
"""

from __future__ import annotations

import time

import common
from common import (
    EXIT_FAIL,
    EXIT_PASS,
    SingleRobotRuntime,
    build_parser,
    describe_motion_result,
    velocity_acceleration,
    wait_for_idle,
)

_SMALL_MOVE_MM = 2.0


def _probe_runtime(runtime, args) -> int:
    if not runtime.wait_until_ready(args.ready_timeout):
        print("FAIL: runtime did not become ready")
        print(runtime.readiness())
        return EXIT_FAIL

    gateway = runtime.gateway
    before = common.current_pose(gateway)
    if before is None:
        print("FAIL: no current Cartesian position")
        return EXIT_FAIL

    vel, acc = velocity_acceleration(args)
    target = list(before)
    target[1] += _SMALL_MOVE_MM
    result = gateway.move_ptp(target, vel=vel, acc=acc, blocking=True)
    print("probe move_ptp result:", describe_motion_result(result))
    if result != 0:
        print("FAIL: probe move_ptp did not succeed")
        return EXIT_FAIL
    if not wait_for_idle(gateway, args.ready_timeout):
        print("FAIL: gateway not idle after probe move")
        return EXIT_FAIL

    after = common.current_pose(gateway)
    if after is None:
        print("FAIL: no current Cartesian position after probe")
        return EXIT_FAIL
    moved = sum((x - y) ** 2 for x, y in zip(before[:3], after[:3])) ** 0.5
    print(f"probe displacement: {moved:.2f}mm")
    if moved < 0.5:
        print("FAIL: probe did not move the robot")
        return EXIT_FAIL
    return EXIT_PASS


def _main() -> int:
    args = build_parser("test 12: sequential runtime close/reopen").parse_args()
    common.print_env()

    guard = common.require_fake_hardware(args)
    if guard is not None:
        return guard

    print("=== lifecycle 1: first runtime")
    first = SingleRobotRuntime(args.robot)
    try:
        code = _probe_runtime(first, args)
        if code != EXIT_PASS:
            return code
    finally:
        print("=== closing first runtime")
        first.close()

    time.sleep(1.0)

    print("\n=== lifecycle 2: fresh runtime (sequential reopen)")
    second = SingleRobotRuntime(args.robot)
    try:
        code = _probe_runtime(second, args)
    finally:
        second.close()

    if code != EXIT_PASS:
        return code

    print("\nPASS: runtime closed and reopened sequentially, fully functional")
    return EXIT_PASS


if __name__ == "__main__":
    raise SystemExit(_main())
