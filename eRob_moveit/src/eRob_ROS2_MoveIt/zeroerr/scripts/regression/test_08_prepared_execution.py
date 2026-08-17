#!/usr/bin/env python3
"""Regression test 08: prepare_path + execute_prepared start policies.

Subcase A (live_anchor): prepare a short path offline, execute with
``live_anchor``. Verifies the prepared trajectory carries real motion
(``noop=False``, ``point_count > 0``), ``start_positions`` matches actual
joints at prepare time (hard FAIL if missing), robot did not move during
``prepare_path``, execution succeeds, and final joint positions
approximately match ``prepared.end_positions`` (hard FAIL if missing).

Subcase B (require_exact mismatch): prepare a fresh path, move the robot
away, verify displaced joint delta exceeds tolerance, execute with
``require_exact``. Verifies ``MOTION_ERROR_PREPARED_START_MISMATCH`` (-15),
robot did not move (joint immobility), idle after, stack ready, recovery
move succeeds.

Subcase C (require_exact success): prepare a fresh path, do not move,
execute with ``require_exact``. Verifies result == 0 and final joints
approximately match ``prepared.end_positions`` (hard FAIL if missing).
"""

from __future__ import annotations

import math

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
_JOINT_MATCH_EPS_RAD = 0.05
_TIGHT_EPS_RAD = 0.001
_PREPARE_IMMOBILITY_EPS_RAD = 0.01

MOTION_ERROR_PREPARED_START_MISMATCH = -15


def _build_path(start: list[float], delta_mm: float, axis: int = 0) -> list[list[float]]:
    path = []
    for index in range(3):
        waypoint = list(start)
        waypoint[axis] += delta_mm * index / 2.0
        path.append(waypoint[:3])
    return path


def _require_fields(prepared, label: str) -> tuple[list[float], list[float]]:
    """Extract start/end positions; hard FAIL on any missing field."""
    start_positions = list(getattr(prepared, "start_positions", None) or [])
    end_positions = list(getattr(prepared, "end_positions", None) or [])
    if not start_positions:
        raise AssertionError(
            f"FAIL ({label}): prepared.start_positions is missing or empty"
        )
    if not end_positions:
        raise AssertionError(
            f"FAIL ({label}): prepared.end_positions is missing or empty"
        )
    return start_positions, end_positions


def _check_noop(prepared, noop_allowed: bool, label: str) -> None:
    noop = getattr(prepared, "noop", None)
    if noop is not True and noop is not False:
        raise AssertionError(f"FAIL ({label}): prepared.noop is not a bool: {noop!r}")
    if noop and not noop_allowed:
        raise AssertionError(f"FAIL ({label}): prepare_path returned noop=True for a real path")


def _prepare_and_execute_live_anchor(runtime, gateway, args, vel, acc, delta) -> int:
    before = common.current_pose(gateway)
    if before is None:
        print("FAIL: no current Cartesian position (subcase A)")
        return EXIT_FAIL

    joints_before = context_joint_positions(gateway)

    prepared = gateway.prepare_path(
        _build_path(before, delta),
        vel=vel,
        acc=acc,
        orientation_mode="constant",
        timeout_s=_PREPARE_TIMEOUT_S,
    )
    if isinstance(prepared, int):
        print("FAIL: prepare_path returned error:", describe_motion_result(prepared))
        return EXIT_FAIL

    try:
        _check_noop(prepared, noop_allowed=False, label="A")
        start_positions, end_positions = _require_fields(prepared, "A")
    except AssertionError as exc:
        print(str(exc))
        return EXIT_FAIL

    point_count = getattr(prepared, "point_count", 0)
    print(f"prepared: noop={prepared.noop} points={point_count} "
          f"start_pos={[round(v, 4) for v in start_positions]} "
          f"end_pos={[round(v, 4) for v in end_positions]}")

    if point_count <= 0:
        print("FAIL (A): prepared.point_count <= 0")
        return EXIT_FAIL

    joints_after_prepare = context_joint_positions(gateway)
    if joints_before and joints_after_prepare and len(joints_before) == len(joints_after_prepare):
        prepare_delta = max_delta(joints_before, joints_after_prepare)
        print(f"joint delta during prepare_path: {prepare_delta:.6f} rad")
        if prepare_delta > _PREPARE_IMMOBILITY_EPS_RAD:
            print("FAIL (A): robot moved during prepare_path")
            return EXIT_FAIL
    else:
        print("WARN (A): joint state unavailable; skipping prepare immobility check")

    if len(start_positions) != len(joints_before):
        print(f"FAIL (A): start_positions length ({len(start_positions)}) "
              f"!= actual joints length ({len(joints_before)})")
        return EXIT_FAIL
    start_match = max_delta(start_positions, joints_before)
    print(f"start_positions vs actual joints delta: {start_match:.6f} rad")
    if start_match > _JOINT_MATCH_EPS_RAD:
        print("FAIL (A): prepared.start_positions does not match actual joints")
        return EXIT_FAIL

    result = gateway.execute_prepared(
        prepared,
        blocking=True,
        start_policy="live_anchor",
    )
    print("execute_prepared(live_anchor) result:", describe_motion_result(result))
    if result != 0:
        print("FAIL (A): execute_prepared(live_anchor) did not succeed")
        return EXIT_FAIL

    if not wait_for_idle(gateway, args.ready_timeout):
        print("FAIL (A): gateway not idle after prepared execution")
        return EXIT_FAIL
    settle(gateway, args.settle_s)

    joints_after = context_joint_positions(gateway)
    if len(joints_after) != len(end_positions):
        print(f"FAIL (A): end_positions length ({len(end_positions)}) "
              f"!= actual joints length ({len(joints_after)})")
        return EXIT_FAIL
    end_match = max_delta(end_positions, joints_after)
    print(f"final joints vs prepared.end_positions delta: {end_match:.6f} rad")
    if end_match > _JOINT_MATCH_EPS_RAD:
        print("FAIL (A): final joint positions do not match prepared.end_positions")
        return EXIT_FAIL

    return 0


def _body(runtime, args) -> int:
    gateway = runtime.gateway

    if not runtime.wait_until_ready(args.ready_timeout):
        print("FAIL: runtime did not become ready")
        print(runtime.readiness())
        return EXIT_FAIL

    vel, acc = velocity_acceleration(args)
    delta = float(args.move_mm)

    print("SUBCASE A: prepare + execute (live_anchor)")
    code = _prepare_and_execute_live_anchor(runtime, gateway, args, vel, acc, delta)
    if code != 0:
        return code

    print("\nSUBCASE B: require_exact with displaced live state")
    baseline = common.current_pose(gateway)
    if baseline is None:
        print("FAIL: no current Cartesian position (subcase B)")
        return EXIT_FAIL

    prepared_b = gateway.prepare_path(
        _build_path(baseline, delta),
        vel=vel,
        acc=acc,
        orientation_mode="constant",
        timeout_s=_PREPARE_TIMEOUT_S,
    )
    if isinstance(prepared_b, int):
        print("FAIL: prepare_path returned error (subcase B):", describe_motion_result(prepared_b))
        return EXIT_FAIL

    try:
        joint_at_prepare, _ = _require_fields(prepared_b, "B")
    except AssertionError as exc:
        print(str(exc))
        return EXIT_FAIL

    print("prepared (B): noop=", getattr(prepared_b, "noop", None),
          "start_positions=", [round(v, 4) for v in joint_at_prepare])

    away = list(baseline)
    away[1] += delta
    move_away = gateway.move_ptp(away, vel=vel, acc=acc, blocking=True)
    print("intermediate move_ptp result:", describe_motion_result(move_away))
    if move_away != 0:
        print("FAIL: intermediate move_ptp did not succeed")
        return EXIT_FAIL
    if not wait_for_idle(gateway, args.ready_timeout):
        print("FAIL: gateway not idle after intermediate move")
        return EXIT_FAIL
    settle(gateway, args.settle_s)

    joint_after_away = context_joint_positions(gateway)
    if len(joint_at_prepare) != len(joint_after_away):
        print("FAIL: prepared.start_positions length != actual joints length after move")
        return EXIT_FAIL
    displaced_joint_delta = max_delta(joint_at_prepare, joint_after_away)
    print(f"joint displacement from prepare state: {displaced_joint_delta:.6f} rad")

    tolerance = float(getattr(common.config, "EXECUTOR_PREPARED_START_TOL_RAD", 0.01))
    if displaced_joint_delta <= tolerance:
        print(
            f"SKIP-FAIL: displacement {displaced_joint_delta:.6f} rad does not "
            f"exceed the {tolerance} rad tolerance; increase --move-mm"
        )
        return EXIT_FAIL

    joints_before_reject = context_joint_positions(gateway)
    result = gateway.execute_prepared(
        prepared_b,
        blocking=True,
        start_policy="require_exact",
    )
    print("execute_prepared(require_exact) result:", describe_motion_result(result))
    if result != MOTION_ERROR_PREPARED_START_MISMATCH:
        print(
            f"FAIL: expected {MOTION_ERROR_PREPARED_START_MISMATCH} "
            "(start mismatch), got", describe_motion_result(result)
        )
        return EXIT_FAIL

    if not wait_for_idle(gateway, max(5.0, args.ready_timeout)):
        print("FAIL: gateway not idle after rejected prepared goal")
        return EXIT_FAIL

    joints_after_reject = context_joint_positions(gateway)
    if joints_before_reject and joints_after_reject:
        reject_delta = max_delta(joints_before_reject, joints_after_reject)
        print(f"joint delta during rejected goal: {reject_delta:.6f} rad")
        if reject_delta > _TIGHT_EPS_RAD:
            print("FAIL: robot moved during the rejected prepared goal")
            return EXIT_FAIL
    else:
        print("WARN: joint state unavailable; skipping immobility check")

    if not gateway.is_motion_stack_ready():
        print("FAIL: motion stack not ready after rejected prepared goal")
        print("fault:", gateway.get_motion_stack_fault_reason())
        return EXIT_FAIL

    recovery = list(baseline)
    recovery[0] += 2.0
    recovery_result = gateway.move_ptp(recovery, vel=vel, acc=acc, blocking=True)
    print("recovery move_ptp result:", describe_motion_result(recovery_result))
    if recovery_result != 0:
        print("FAIL: stack not usable after rejected prepared goal")
        return EXIT_FAIL

    print("\nSUBCASE C: require_exact success (no displacement)")
    baseline_c = common.current_pose(gateway)
    if baseline_c is None:
        print("FAIL: no current Cartesian position (subcase C)")
        return EXIT_FAIL

    prepared_c = gateway.prepare_path(
        _build_path(baseline_c, delta),
        vel=vel,
        acc=acc,
        orientation_mode="constant",
        timeout_s=_PREPARE_TIMEOUT_S,
    )
    if isinstance(prepared_c, int):
        print("FAIL: prepare_path returned error (subcase C):", describe_motion_result(prepared_c))
        return EXIT_FAIL

    try:
        _, end_positions_c = _require_fields(prepared_c, "C")
    except AssertionError as exc:
        print(str(exc))
        return EXIT_FAIL

    result_c = gateway.execute_prepared(
        prepared_c,
        blocking=True,
        start_policy="require_exact",
    )
    print("execute_prepared(require_exact) result:", describe_motion_result(result_c))
    if result_c != 0:
        print("FAIL: execute_prepared(require_exact) should succeed when not displaced")
        return EXIT_FAIL

    if not wait_for_idle(gateway, args.ready_timeout):
        print("FAIL: gateway not idle after successful prepared execution (subcase C)")
        return EXIT_FAIL
    settle(gateway, args.settle_s)

    joints_after_c = context_joint_positions(gateway)
    if len(joints_after_c) != len(end_positions_c):
        print(f"FAIL (C): end_positions length ({len(end_positions_c)}) "
              f"!= actual joints length ({len(joints_after_c)})")
        return EXIT_FAIL
    end_match_c = max_delta(end_positions_c, joints_after_c)
    print(f"final joints vs prepared.end_positions delta: {end_match_c:.6f} rad")
    if end_match_c > _JOINT_MATCH_EPS_RAD:
        print("FAIL (C): final joint positions do not match prepared.end_positions")
        return EXIT_FAIL

    print("PASS: prepared execution live_anchor + require_exact mismatch + require_exact success")
    return EXIT_PASS


def _main() -> int:
    args = build_parser("test 08: prepare_path + execute_prepared start policies").parse_args()
    common.print_env()

    guard = common.require_fake_hardware(args)
    if guard is not None:
        return guard

    return run_with_runtime(args, _body)


if __name__ == "__main__":
    raise SystemExit(_main())
