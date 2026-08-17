#!/usr/bin/env python3
"""Regression test 03: move_linear (blocking).

Exercises the single-robot Cartesian linear move through the legacy public
gateway API (``gateway.move_linear`` -> backend ``move_liner``). Same
safe-direction policy as test 02. Verifies displacement magnitude, constant
orientation, joint-state change and task-id recording.
"""

from __future__ import annotations

import math

import common
from common import (
    EXIT_FAIL,
    EXIT_PASS,
    build_parser,
    describe_motion_result,
    max_delta,
    max_orientation_delta_deg,
    run_with_runtime,
    context_joint_positions,
    settle,
    velocity_acceleration,
    wait_for_idle,
)

_ORIENTATION_EPS_DEG = 0.5
_JOINT_CHANGED_EPS_RAD = 1e-6
_POS_FRACTION_MIN = 0.5
_POS_FRACTION_MAX = 1.5


def _xyz_distance(a: list[float], b: list[float]) -> float:
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a[:3], b[:3])))


def _body(runtime, args) -> int:
    gateway = runtime.gateway

    if not runtime.wait_until_ready(args.ready_timeout):
        print("FAIL: runtime did not become ready")
        print(runtime.readiness())
        return EXIT_FAIL

    before = common.current_pose(gateway)
    if before is None:
        print("FAIL: no current Cartesian position")
        return EXIT_FAIL
    print("before:", before)

    joint_positions_before = context_joint_positions(gateway)
    vel, acc = velocity_acceleration(args)
    delta = float(args.move_mm)
    print(f"move_linear target delta={delta:.1f}mm vel={vel:g} acc={acc:g}")

    result = -1
    target = None
    for sign in (1, -1):
        candidate = list(before)
        candidate[0] += sign * delta
        print(f"attempt move_linear x{sign:+} -> {[round(v, 3) for v in candidate]}")
        result = gateway.move_linear(candidate, vel=vel, acc=acc, blocking=True)
        print("move_linear result:", describe_motion_result(result))
        if result == 0:
            target = candidate
            break
        if not wait_for_idle(gateway, args.ready_timeout):
            print("FAIL: gateway not idle after rejected move")
            return EXIT_FAIL

    if result != 0 or target is None:
        print("FAIL: move_linear rejected in both directions")
        return EXIT_FAIL

    if not wait_for_idle(gateway, args.ready_timeout):
        print("FAIL: gateway not idle after move_linear")
        return EXIT_FAIL

    settle(gateway, args.settle_s)

    after = common.current_pose(gateway)
    if after is None:
        print("FAIL: no current Cartesian position after move")
        return EXIT_FAIL
    print("after:", after)

    moved = _xyz_distance(before, after)
    print(f"cartesian displacement: {moved:.2f}mm (requested {delta:.1f}mm)")
    if not (_POS_FRACTION_MIN * delta <= moved <= _POS_FRACTION_MAX * delta):
        print("FAIL: displacement outside expected range")
        return EXIT_FAIL

    orientation_delta = max_orientation_delta_deg(before, after)
    print(f"orientation delta: {orientation_delta:.5f} deg")
    if orientation_delta > _ORIENTATION_EPS_DEG:
        print("FAIL: orientation changed during move_linear")
        return EXIT_FAIL

    joint_positions_after = context_joint_positions(gateway)
    joint_delta = max_delta(joint_positions_before, joint_positions_after)
    print(f"joint max delta: {joint_delta:.9f} rad")
    if joint_delta <= _JOINT_CHANGED_EPS_RAD:
        print("FAIL: joint state did not change during move_linear")
        return EXIT_FAIL

    task_id = gateway.last_submitted_task_id()
    print("last_submitted_task_id:", task_id)
    if task_id is None:
        print("FAIL: no task id recorded after move")
        return EXIT_FAIL

    print("PASS: move_linear executed (blocking)")
    return EXIT_PASS


def _main() -> int:
    args = build_parser("test 03: move_linear").parse_args()
    common.print_env()

    guard = common.require_fake_hardware(args)
    if guard is not None:
        return guard

    return run_with_runtime(args, _body)


if __name__ == "__main__":
    raise SystemExit(_main())
