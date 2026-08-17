#!/usr/bin/env python3
"""Regression test 01: motion-stack readiness.

Proves a single-robot runtime constructs and becomes motion-ready on the
current branch through ``RobotRuntimeContext.from_config()``, including under
the paint/welding single-robot profiles (which inherit the default unscoped
``robot`` identity from the base config/runtime.yaml).
"""

from __future__ import annotations

import common
from common import (
    EXIT_FAIL,
    EXIT_PASS,
    build_parser,
    context_joint_positions,
    run_with_runtime,
)


def _finite(value) -> bool:
    return value == value and value not in (float("inf"), float("-inf"))


def _body(runtime, args) -> int:
    gateway = runtime.gateway
    node = gateway.node

    if not runtime.wait_until_ready(args.ready_timeout):
        print("FAIL: runtime did not become ready")
        print(runtime.readiness())
        return EXIT_FAIL

    if not gateway.is_runtime_initialized():
        print("FAIL: is_runtime_initialized() is False")
        return EXIT_FAIL

    startup = gateway.startup_status()
    print("startup_status:", startup)
    if not startup.get("ros2_active"):
        print("FAIL: startup ros2_active is False")
        return EXIT_FAIL
    if not startup.get("runtime_initialized"):
        print("FAIL: startup runtime_initialized is False")
        return EXIT_FAIL
    if not startup.get("motion_stack_ready"):
        print("FAIL: startup motion_stack_ready is False")
        print("fault:", startup.get("motion_stack_fault"))
        return EXIT_FAIL

    snapshot = gateway.runtime_state_snapshot()
    print("runtime_state_snapshot:", snapshot)
    if not snapshot.get("runtime_ready"):
        print("FAIL: runtime_state_snapshot runtime_ready is False")
        return EXIT_FAIL
    if not snapshot.get("motion_stack_ready"):
        print("FAIL: runtime_state_snapshot motion_stack_ready is False")
        return EXIT_FAIL

    state = gateway.state_snapshot()
    print("state_snapshot keys:", sorted(state.keys()))
    if state.get("success") is not True:
        print("FAIL: state_snapshot success is not True")
        print("unavailable_fields:", state.get("unavailable_fields"))
        return EXIT_FAIL
    position = state.get("position")
    print("position:", position)
    if not position or len(position) != 6:
        print("FAIL: state_snapshot position missing/invalid")
        return EXIT_FAIL

    context_joints = list(node.robot_context.joint_names)
    context_positions = context_joint_positions(gateway)
    print("context_joint_names:", context_joints)
    print("context_joint_positions:", context_positions)
    if len(context_positions) != len(context_joints):
        print("FAIL: could not resolve all context joints from /joint_states")
        return EXIT_FAIL
    if any(not _finite(value) for value in context_positions):
        print("FAIL: joint positions missing or non-finite")
        return EXIT_FAIL

    print("PASS: single-robot motion stack ready")
    return EXIT_PASS


def _main() -> int:
    args = build_parser("test 01: single-robot motion-stack readiness").parse_args()
    common.print_env()

    guard = common.require_fake_hardware(args)
    if guard is not None:
        return guard

    return run_with_runtime(args, _body)


if __name__ == "__main__":
    raise SystemExit(_main())
