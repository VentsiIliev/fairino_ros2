#!/usr/bin/env python3
"""Regression test 06: stop_motion mid-active + queue semantics.

Starts a deliberately long path at low velocity, confirms the controller
entered the executing state, optionally fills the motion queue with
non-blocking submissions (queue-full -> -5 is a soft sub-check), then
issues ``stop_motion`` and verifies: the stop reported success (not the
``NO_ACTIVE_MOTION`` fallback), the robot returned to idle, the queue
was cleared, the motion stack is still ready, and a follow-up move_ptp
still works.

Uses a safe direction policy (tries +X then -X) for the long path.
"""

from __future__ import annotations

import math
import time

import common
from common import (
    EXIT_FAIL,
    EXIT_PASS,
    build_parser,
    describe_motion_result,
    run_with_runtime,
    settle,
    velocity_acceleration,
    wait_for_idle,
)

_QUEUE_OVERFLOW_TRIES = 3
_LONG_WAYPOINTS = 10


def _wait_until_executing(gateway, timeout_s: float) -> bool:
    deadline = time.monotonic() + max(0.0, float(timeout_s))
    while time.monotonic() < deadline:
        if bool(getattr(gateway.node, "is_executing", False)):
            return True
        time.sleep(0.02)
    return bool(getattr(gateway.node, "is_executing", False))


def _build_long_path(start: list[float], length_mm: float) -> list[list[float]]:
    step = length_mm / max(1, _LONG_WAYPOINTS - 1)
    return [
        [start[0] + index * step, start[1], start[2]]
        for index in range(_LONG_WAYPOINTS)
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

    vel, acc = velocity_acceleration(args)
    long_vel = max(1.0, min(vel, 3.0))
    length_mm = float(args.move_mm) * 3.0

    result = -1
    for sign in (1, -1):
        signed_delta = sign * length_mm
        path = _build_long_path(before, signed_delta)
        print(
            f"starting long execute_path (blocking=False, x{sign:+}): "
            f"length={abs(signed_delta):.0f}mm vel={long_vel:g} acc={acc:g}"
        )
        result = gateway.execute_path(
            path,
            vel=long_vel,
            acc=acc,
            blocking=False,
            orientation_mode="constant",
        )
        print("long execute_path result:", describe_motion_result(result))
        if result == 0:
            break
        if not wait_for_idle(gateway, args.ready_timeout):
            print("FAIL: gateway not idle after rejected long path")
            return EXIT_FAIL

    if result != 0:
        print("FAIL: long execute_path did not start in either direction")
        return EXIT_FAIL

    if not _wait_until_executing(gateway, max(10.0, args.ready_timeout)):
        print("FAIL: controller never entered executing state")
        return EXIT_FAIL
    print("controller is executing")

    max_queue = int(getattr(common.config, "MOTION_QUEUE_MAX_SIZE", 10))
    print(f"queue-max-size: {max_queue}")

    overflow_observed = False
    submissions = []
    for index in range(max_queue + _QUEUE_OVERFLOW_TRIES):
        target = list(before)
        target[1] += 1.0 + index * 1.0
        queued = gateway.move_ptp(target, vel=vel, acc=acc, blocking=False)
        submissions.append(queued)
        print(f"queue-fill submit #{index}: {describe_motion_result(queued)}")
        if int(queued) == -5:
            overflow_observed = True
        if not bool(getattr(gateway.node, "is_executing", False)):
            print("note: motion finished early; queue drained before overflow")
            break

    if overflow_observed:
        print("queue-full (-5) observed: PASS (soft sub-check)")
    else:
        print(
            "queue-full (-5) NOT observed: SKIP sub-check "
            "(fake-hardware motions completed before the queue filled)"
        )

    print("issuing stop_motion...")
    stop = gateway.stop_motion()
    print("stop_motion response:", stop)
    if not stop.get("stopped"):
        state = stop.get("stop_state")
        if state == "NO_ACTIVE_MOTION" and not bool(getattr(gateway.node, "is_executing", False)):
            print(
                "FAIL: stop_motion returned NO_ACTIVE_MOTION but this test "
                "requires an active motion to be present; the long path "
                "may have finished too quickly"
            )
            return EXIT_FAIL
        else:
            print("FAIL: stop_motion did not report stopped=True")
            return EXIT_FAIL
    if not stop.get("success"):
        print("FAIL: stop_motion success is False")
        return EXIT_FAIL

    if not wait_for_idle(gateway, max(10.0, args.ready_timeout)):
        print("FAIL: controller did not return to idle after stop")
        return EXIT_FAIL
    print("controller idle after stop")

    queue_status = gateway.node.motion_queue.get_status()
    queue_size = int(queue_status.get("queue_size", -1))
    print("queue_status after stop:", queue_status)
    if queue_size != 0:
        print("FAIL: motion queue not cleared after stop")
        return EXIT_FAIL

    if not gateway.is_motion_stack_ready():
        print("FAIL: motion stack not ready after stop")
        print("fault:", gateway.get_motion_stack_fault_reason())
        return EXIT_FAIL

    settle(gateway, args.settle_s)

    target = list(before)
    target[1] += float(args.move_mm) * 0.5
    follow = gateway.move_ptp(target, vel=vel, acc=acc, blocking=True)
    print("follow-up move_ptp result:", describe_motion_result(follow))
    if follow != 0:
        print("FAIL: follow-up move_ptp after stop did not succeed")
        return EXIT_FAIL

    print("PASS: stop_motion stopped execution and cleared the queue; stack usable")
    return EXIT_PASS


def _main() -> int:
    args = build_parser("test 06: stop_motion mid-active + queue semantics").parse_args()
    common.print_env()

    guard = common.require_fake_hardware(args)
    if guard is not None:
        return guard

    return run_with_runtime(args, _body)


if __name__ == "__main__":
    raise SystemExit(_main())
