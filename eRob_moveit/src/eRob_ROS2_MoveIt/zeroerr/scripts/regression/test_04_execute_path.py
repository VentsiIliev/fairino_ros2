#!/usr/bin/env python3
"""Regression test 04: execute_path (legacy public API).

Drives the old public path API (``gateway.execute_path`` -> backend
``execute_path``) with a short 4-waypoint Cartesian path starting at the
current position, using 3D waypoints + constant orientation (exercises the
orientation-fill preprocessing path). Verifies the path executed, the end
position moved the requested total distance, orientation stayed constant, the
joint state changed, and a task id was recorded. Retries along -X once if the
+safety validation rejects the path.
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
_WAYPOINTS = 4


def _build_path(start: list[float], delta_mm: float, waypoints: int) -> list[list[float]]:
    step = delta_mm / max(1, waypoints - 1)
    path = []
    for index in range(waypoints):
        path.append([
            start[0] + index * step,
            start[1],
            start[2],
        ])
    return path


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
    print(f"execute_path total delta={delta:.1f}mm "
          f"waypoints={_WAYPOINTS} vel={vel:g} acc={acc:g}")

    result = -1
    executed_delta = 0.0
    for sign in (1, -1):
        signed_delta = sign * delta
        path = _build_path(before, signed_delta, _WAYPOINTS)
        print(f"attempt execute_path x{sign:+} -> {[[round(v, 1) for v in wp] for wp in path]}")
        result = gateway.execute_path(
            path,
            vel=vel,
            acc=acc,
            blocking=True,
            orientation_mode="constant",
        )
        print("execute_path result:", describe_motion_result(result))
        if result == 0:
            executed_delta = signed_delta
            break
        if not wait_for_idle(gateway, args.ready_timeout):
            print("FAIL: gateway not idle after rejected path")
            return EXIT_FAIL

    if result != 0:
        print("FAIL: execute_path rejected in both directions")
        return EXIT_FAIL

    if not wait_for_idle(gateway, args.ready_timeout):
        print("FAIL: gateway not idle after execute_path")
        return EXIT_FAIL

    settle(gateway, args.settle_s)

    after = common.current_pose(gateway)
    if after is None:
        print("FAIL: no current Cartesian position after path")
        return EXIT_FAIL
    print("after:", after)

    moved = _xyz_distance(before, after)
    requested = abs(executed_delta)
    print(f"cartesian displacement: {moved:.2f}mm (requested {requested:.1f}mm)")
    if not (_POS_FRACTION_MIN * requested <= moved <= _POS_FRACTION_MAX * requested):
        print("FAIL: displacement outside expected range")
        return EXIT_FAIL

    orientation_delta = max_orientation_delta_deg(before, after)
    print(f"orientation delta: {orientation_delta:.5f} deg")
    if orientation_delta > _ORIENTATION_EPS_DEG:
        print("FAIL: orientation changed during execute_path")
        return EXIT_FAIL

    joint_positions_after = context_joint_positions(gateway)
    joint_delta = max_delta(joint_positions_before, joint_positions_after)
    print(f"joint max delta: {joint_delta:.9f} rad")
    if joint_delta <= _JOINT_CHANGED_EPS_RAD:
        print("FAIL: joint state did not change during execute_path")
        return EXIT_FAIL

    task_id = gateway.last_submitted_task_id()
    print("last_submitted_task_id:", task_id)
    if task_id is None:
        print("FAIL: no task id recorded after path")
        return EXIT_FAIL

    print("PASS: execute_path executed (public API, constant orientation)")
    return EXIT_PASS


def _main() -> int:
    args = build_parser("test 04: execute_path").parse_args()
    common.print_env()

    guard = common.require_fake_hardware(args)
    if guard is not None:
        return guard

    return run_with_runtime(args, _body)


if __name__ == "__main__":
    raise SystemExit(_main())
