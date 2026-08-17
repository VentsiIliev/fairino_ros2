#!/usr/bin/env python3
"""Regression test 09: prepared no-op path handling.

Verifies the no-op ``PreparedTrajectory`` contract: a zero-displacement
path at the current position must yield ``noop=True`` with zero points.
Executing it returns 0 immediately, never dispatches a controller goal,
and the robot does not move.

If the planner cannot produce ``noop=True`` for this input, the test
FAILs rather than silently skipping — the no-op contract is mandatory.
"""

from __future__ import annotations

import time

import common
from common import (
    EXIT_FAIL,
    EXIT_PASS,
    build_parser,
    context_joint_positions,
    describe_motion_result,
    max_delta,
    run_with_runtime,
    settle,
    velocity_acceleration,
    wait_for_idle,
)

_PREPARE_TIMEOUT_S = 30.0
_NO_DISPATCH_WINDOW_S = 1.5
_JOINT_MATCH_EPS_RAD = 0.001


def _body(runtime, args) -> int:
    gateway = runtime.gateway

    if not runtime.wait_until_ready(args.ready_timeout):
        print("FAIL: runtime did not become ready")
        print(runtime.readiness())
        return EXIT_FAIL

    vel, acc = velocity_acceleration(args)
    before = common.current_pose(gateway)
    if before is None:
        print("FAIL: no current Cartesian position")
        return EXIT_FAIL
    print("before:", before)

    joints_before = context_joint_positions(gateway)

    zero_path = [list(before[:3])]
    print("preparing zero-displacement path (single waypoint at current pose)")
    prepared = gateway.prepare_path(
        zero_path,
        vel=vel,
        acc=acc,
        orientation_mode="constant",
        timeout_s=_PREPARE_TIMEOUT_S,
    )
    if isinstance(prepared, int):
        print("FAIL: prepare_path returned error:", describe_motion_result(prepared))
        return EXIT_FAIL

    noop = getattr(prepared, "noop", None)
    point_count = getattr(prepared, "point_count", None)
    print(f"prepared: noop={noop} point_count={point_count}")

    if noop is not True:
        print(
            f"FAIL: expected noop=True for zero-displacement path, "
            f"got noop={noop!r}"
        )
        return EXIT_FAIL
    if point_count != 0:
        print(f"FAIL: expected point_count=0 for no-op, got {point_count!r}")
        return EXIT_FAIL

    result = gateway.execute_prepared(
        prepared,
        blocking=True,
        start_policy="live_anchor",
    )
    print("execute_prepared(noop) result:", describe_motion_result(result))
    if result != 0:
        print("FAIL: no-op execute_prepared did not return 0")
        return EXIT_FAIL

    deadline = time.monotonic() + _NO_DISPATCH_WINDOW_S
    while time.monotonic() < deadline:
        if bool(getattr(gateway.node, "is_executing", False)):
            print("FAIL: no-op execute_prepared dispatched a controller goal")
            return EXIT_FAIL
        time.sleep(0.02)

    after = common.current_pose(gateway)
    if after is not None:
        displacement = sum((x - y) ** 2 for x, y in zip(before[:3], after[:3])) ** 0.5
        print(f"displacement during no-op execute: {displacement:.6f}mm")
        if displacement > 0.1:
            print("FAIL: robot moved during no-op execute_prepared")
            return EXIT_FAIL

    joints_after = context_joint_positions(gateway)
    if joints_before and joints_after and len(joints_before) == len(joints_after):
        joint_delta = max_delta(joints_before, joints_after)
        print(f"joint delta during no-op execute: {joint_delta:.6f} rad")
        if joint_delta > _JOINT_MATCH_EPS_RAD:
            print("FAIL: joint state changed during no-op execute_prepared")
            return EXIT_FAIL

    if not gateway.is_motion_stack_ready():
        print("FAIL: motion stack not ready after no-op execute")
        return EXIT_FAIL

    print("PASS: no-op prepared trajectory returned 0 without dispatching")
    return EXIT_PASS


def _main() -> int:
    args = build_parser("test 09: prepared no-op path handling").parse_args()
    common.print_env()

    guard = common.require_fake_hardware(args)
    if guard is not None:
        return guard

    return run_with_runtime(args, _body)


if __name__ == "__main__":
    raise SystemExit(_main())
