#!/usr/bin/env python3
"""Regression test 07: execute_ordered_motion_chain.

Drives the ordered-motion-chain public API with a two-segment batch
(ptp then linear) and verifies: result == 0, the chain status reached the
"completed" terminal phase with result 0, total Cartesian displacement, and
joint-state change. Retries along -X once if the primary direction is
rejected by safety validation.
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


def _build_chain(start: list[float], delta_mm: float, vel: float, acc: float) -> list[dict]:
    half = delta_mm / 2.0
    p1 = [start[0] + half, start[1], start[2], start[3], start[4], start[5]]
    p2 = [start[0] + delta_mm, start[1], start[2], start[3], start[4], start[5]]
    return [
        {
            "type": "ptp",
            "label": "seg_1_ptp",
            "position": p1,
            "vel": vel,
            "acc": acc,
            "blendR": 0.0,
        },
        {
            "type": "linear",
            "label": "seg_2_linear",
            "position": p2,
            "vel": vel,
            "acc": acc,
            "blendR": 0.0,
        },
    ]


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
    print(f"execute_ordered_motion_chain total delta={delta:.1f}mm vel={vel:g} acc={acc:g}")

    result = -1
    for sign in (1, -1):
        signed_delta = sign * delta
        segments = _build_chain(before, signed_delta, vel, acc)
        print(f"attempt execute_ordered_motion_chain x{sign:+} segments={len(segments)}")
        result = gateway.execute_ordered_motion_chain(segments, blocking=True)
        print("execute_ordered_motion_chain result:", describe_motion_result(result))
        if result == 0:
            break
        if not wait_for_idle(gateway, args.ready_timeout):
            print("FAIL: gateway not idle after rejected chain")
            return EXIT_FAIL

    if result != 0:
        print("FAIL: execute_ordered_motion_chain rejected in both directions")
        return EXIT_FAIL

    if not wait_for_idle(gateway, args.ready_timeout):
        print("FAIL: gateway not idle after ordered chain")
        return EXIT_FAIL

    settle(gateway, args.settle_s)

    chain_status = gateway.ordered_motion_chain_status()
    print("ordered_motion_chain_status:", chain_status)
    if not isinstance(chain_status, dict) or chain_status.get("phase") != "completed":
        print("FAIL: ordered chain status did not reach 'completed' phase")
        return EXIT_FAIL
    if chain_status.get("result") != 0:
        print("FAIL: ordered chain status result is not 0")
        return EXIT_FAIL

    after = common.current_pose(gateway)
    if after is None:
        print("FAIL: no current Cartesian position after ordered chain")
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
        print("FAIL: orientation changed during ordered chain")
        return EXIT_FAIL

    joint_positions_after = context_joint_positions(gateway)
    joint_delta = max_delta(joint_positions_before, joint_positions_after)
    print(f"joint max delta: {joint_delta:.9f} rad")
    if joint_delta <= _JOINT_CHANGED_EPS_RAD:
        print("FAIL: joint state did not change during ordered chain")
        return EXIT_FAIL

    task_id = gateway.last_submitted_task_id()
    print("last_submitted_task_id:", task_id)

    print("PASS: execute_ordered_motion_chain executed (ptp+linear, completed)")
    return EXIT_PASS


def _main() -> int:
    args = build_parser("test 07: execute_ordered_motion_chain").parse_args()
    common.print_env()

    guard = common.require_fake_hardware(args)
    if guard is not None:
        return guard

    return run_with_runtime(args, _body)


if __name__ == "__main__":
    raise SystemExit(_main())
