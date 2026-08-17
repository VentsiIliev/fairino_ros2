#!/usr/bin/env python3
"""Regression test 10: startup status + runtime state snapshots.

Read-only verification of the status/snapshot surface exposed by the local
gateway: ``startup_status``, ``runtime_state_snapshot``, ``state_snapshot``,
``state_kinematics``, ``drive_operation_status``, ``motion_interlock_status``,
``ordered_motion_chain_status`` and safety-wall status. No motion is issued.
"""

from __future__ import annotations

import common
from common import (
    EXIT_FAIL,
    EXIT_PASS,
    build_parser,
    run_with_runtime,
)


def _body(runtime, args) -> int:
    gateway = runtime.gateway

    if not runtime.wait_until_ready(args.ready_timeout):
        print("FAIL: runtime did not become ready")
        print(runtime.readiness())
        return EXIT_FAIL

    startup = gateway.startup_status()
    print("startup_status:", startup)
    for key in ("ros2_active", "runtime_initialized", "motion_stack_ready"):
        if not isinstance(startup.get(key), bool):
            print(f"FAIL: startup_status[{key!r}] is not a bool")
            return EXIT_FAIL
    if not startup.get("motion_stack_ready"):
        print("FAIL: startup_status motion_stack_ready is False")
        return EXIT_FAIL

    snapshot = gateway.runtime_state_snapshot()
    print("runtime_state_snapshot:", snapshot)
    for key in ("runtime_ready", "runtime_initialized", "hardware_ready", "motion_stack_ready"):
        if not isinstance(snapshot.get(key), bool):
            print(f"FAIL: runtime_state_snapshot[{key!r}] is not a bool")
            return EXIT_FAIL
    if not snapshot.get("runtime_ready"):
        print("FAIL: runtime_state_snapshot runtime_ready is False")
        return EXIT_FAIL
    if not isinstance(snapshot.get("drive"), dict):
        print("FAIL: runtime_state_snapshot drive is not a dict")
        return EXIT_FAIL
    if not isinstance(snapshot.get("motion_interlock"), dict):
        print("FAIL: runtime_state_snapshot motion_interlock is not a dict")
        return EXIT_FAIL

    state = gateway.state_snapshot()
    print("state_snapshot:", state)
    if state.get("success") is not True:
        print("FAIL: state_snapshot success is not True")
        print("unavailable_fields:", state.get("unavailable_fields"))
        return EXIT_FAIL
    if not isinstance(state.get("status"), dict):
        print("FAIL: state_snapshot status is not a dict")
        return EXIT_FAIL
    if not isinstance(state.get("safety_walls"), dict):
        print("FAIL: state_snapshot safety_walls is not a dict")
        return EXIT_FAIL
    if not isinstance(state.get("active_tool"), str):
        print("FAIL: state_snapshot active_tool is not a string")
        return EXIT_FAIL

    kinematics = gateway.state_kinematics()
    print("state_kinematics:", kinematics)
    if kinematics.get("success") is not True:
        print("FAIL: state_kinematics success is not True")
        print("unavailable_fields:", kinematics.get("unavailable_fields"))
        return EXIT_FAIL
    expected_lengths = {"position": 6, "velocity": 3, "acceleration": 3}
    for key, length in expected_lengths.items():
        values = kinematics.get(key)
        if not values or len(values) != length:
            print(f"FAIL: state_kinematics {key!r} expected length {length}, got {len(values) if values else 'None'}")
            return EXIT_FAIL

    drive = gateway.drive_operation_status()
    print("drive_operation_status:", drive)
    if drive.get("hardware_ready") is not True:
        print("FAIL: drive_operation_status hardware_ready is False")
        return EXIT_FAIL
    if not isinstance(drive.get("actual_enabled"), bool):
        print("FAIL: drive_operation_status actual_enabled is not a bool")
        return EXIT_FAIL

    interlock = gateway.motion_interlock_status()
    print("motion_interlock_status:", interlock)
    if not isinstance(interlock, dict):
        print("FAIL: motion_interlock_status is not a dict")
        return EXIT_FAIL

    ordered = gateway.ordered_motion_chain_status()
    print("ordered_motion_chain_status:", ordered)
    if not isinstance(ordered, dict):
        print("FAIL: ordered_motion_chain_status is not a dict")
        return EXIT_FAIL
    if ordered.get("active") not in (False, True):
        print("FAIL: ordered_motion_chain_status has no 'active' field")
        return EXIT_FAIL

    walls = gateway.get_safety_walls_status()
    print("safety_walls_status:", walls)
    if not isinstance(walls, dict):
        print("FAIL: safety_walls_status is not a dict")
        return EXIT_FAIL

    print("PASS: startup/status/snapshot surface healthy")
    return EXIT_PASS


def _main() -> int:
    args = build_parser("test 10: startup status + runtime state snapshots").parse_args()
    common.print_env()

    guard = common.require_fake_hardware(args)
    if guard is not None:
        return guard

    return run_with_runtime(args, _body)


if __name__ == "__main__":
    raise SystemExit(_main())
