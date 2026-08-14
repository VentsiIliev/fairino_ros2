from __future__ import annotations

import math

import config


def _active_joint_names(backend) -> list[str]:
    robot_context = getattr(backend.node, "robot_context", None)
    return list(
        getattr(robot_context, "joint_names", ())
        or getattr(config, "JOINT_NAMES", [])
        or []
    )


def _resolve_unwind_joint_name(joint_names: list[str]) -> str:
    configured_joint_name = str(
        getattr(config, "EXECUTOR_POST_UNWIND_JOINT_NAME", "Joint_6")
    ).strip()

    if configured_joint_name in joint_names:
        return configured_joint_name

    legacy_joint_names = list(getattr(config, "JOINT_NAMES", []) or [])
    if (
        configured_joint_name in legacy_joint_names
        and len(legacy_joint_names) == len(joint_names)
    ):
        return joint_names[legacy_joint_names.index(configured_joint_name)]

    if configured_joint_name in {"Joint_6", "j6"} and joint_names:
        return joint_names[-1]

    raise RuntimeError(
        f"Joint {configured_joint_name!r} cannot be resolved for active robot joints {joint_names}"
    )


def unwind_joint6_with_rotational_path(
    backend,
    vel=None,
    acc=None,
    queue_if_busy=True,
):
    """Context-aware implementation of the legacy standalone Joint 6 unwind."""
    joint_names = _active_joint_names(backend)
    if not joint_names:
        backend.node.get_logger().error(
            "[UNWIND_J6] No active robot joints configured"
        )
        return -1

    try:
        joint_name = _resolve_unwind_joint_name(joint_names)
    except RuntimeError as exc:
        backend.node.get_logger().error(f"[UNWIND_J6] {exc}")
        return -1

    axis_index = int(
        getattr(config, "EXECUTOR_POST_UNWIND_ROTATION_AXIS_INDEX", 5)
    )
    if axis_index < 3 or axis_index > 5:
        backend.node.get_logger().error(
            f"[UNWIND_J6] Invalid unwind rotation axis index: {axis_index}"
        )
        return -1

    joint_index = joint_names.index(joint_name)
    current_positions = (
        backend.node.trajectory_executor
        ._get_latest_joint_state_in_trajectory_order(joint_names)
    )
    if current_positions is None:
        backend.node.get_logger().error(
            "[UNWIND_J6] Latest joint state unavailable"
        )
        return -1

    initial_value = float(current_positions[joint_index])
    final_target = backend.node.trajectory_executor._canonical_angle(initial_value)
    min_delta = float(
        getattr(config, "EXECUTOR_POST_UNWIND_MIN_DELTA_RAD", 0.5)
    )
    remaining = final_target - initial_value

    if abs(remaining) < min_delta:
        backend.node.get_logger().info(
            "[UNWIND_J6] Rotational-path unwind skipped - no unwind needed "
            f"({joint_name} current={initial_value:.4f}rad "
            f"target={final_target:.4f}rad "
            f"delta={remaining:.4f}rad "
            f"min_delta={min_delta:.4f}rad)"
        )
        backend.node.last_move_result = 0
        return 0

    vel_percent = backend.node.trajectory_executor._clamp_percentage(vel)
    acc_percent = backend.node.trajectory_executor._clamp_percentage(acc)
    vel_scale = vel_percent / 100.0
    acc_scale = acc_percent / 100.0

    sign = float(
        getattr(config, "EXECUTOR_POST_UNWIND_ROTATION_AXIS_SIGN", 1.0)
    )
    if abs(sign) < 1e-9:
        sign = 1.0

    max_step_deg = max(
        1.0,
        abs(
            float(
                getattr(
                    config,
                    "EXECUTOR_POST_UNWIND_ROTATIONAL_SEGMENT_DEG",
                    180.0,
                )
            )
        ),
    )
    total_delta_deg = math.degrees(remaining) * sign
    segment_count = max(
        1,
        int(math.ceil(abs(total_delta_deg) / max_step_deg)),
    )

    backend.node.get_logger().info(
        "[UNWIND_J6] Executing rotational-path unwind: "
        f"{joint_name} {initial_value:.3f} -> {final_target:.3f} rad "
        f"delta={remaining:.3f} rad cart_axis={axis_index} "
        f"cart_delta={total_delta_deg:.3f}deg "
        f"segments={segment_count} max_segment={max_step_deg:.1f}deg "
        f"vel={vel_percent:.1f}% acc={acc_percent:.1f}%"
    )

    for segment_index in range(1, segment_count + 1):
        current_pos_wobj = backend.get_current_position()
        if current_pos_wobj is None or len(current_pos_wobj) < 6:
            backend.node.get_logger().error(
                "[UNWIND_J6] Current Cartesian pose unavailable"
            )
            return -1

        current_positions = (
            backend.node.trajectory_executor
            ._get_latest_joint_state_in_trajectory_order(joint_names)
        )
        if current_positions is None:
            backend.node.get_logger().error(
                "[UNWIND_J6] Latest joint state unavailable"
            )
            return -1

        current_value = float(current_positions[joint_index])
        remaining = final_target - current_value
        remaining_deg = math.degrees(remaining) * sign
        if abs(remaining) < min_delta:
            break

        segment_delta_deg = math.copysign(
            min(abs(remaining_deg), max_step_deg),
            remaining_deg,
        )
        target_pos_wobj = list(current_pos_wobj[:6])
        target_pos_wobj[axis_index] = (
            float(target_pos_wobj[axis_index]) + segment_delta_deg
        )

        backend.node.get_logger().info(
            f"[UNWIND_J6] Rotational unwind segment "
            f"{segment_index}/{segment_count}: "
            f"{current_value:.3f} -> {final_target:.3f} rad, "
            f"cart_delta={segment_delta_deg:.3f}deg"
        )

        result = backend._send_rotational_unwind_path(
            current_pos_wobj,
            target_pos_wobj,
            axis_index,
            vel_scale,
            acc_scale,
            joint_name=joint_name,
            joint_start=current_value,
            joint_target=(
                current_value + math.radians(segment_delta_deg) / sign
            ),
        )
        if result != 0:
            return result

        result = backend._wait_for_motion_idle_result(
            float(getattr(config, "BLOCKING_MOVE_TIMEOUT_S", 60.0)),
            "[UNWIND_J6]",
        )
        if result != 0:
            return result

    check = {
        "joint_names": joint_names,
        "joint_name": joint_name,
        "joint_index": joint_index,
        "target_value": final_target,
    }
    if backend.node.trajectory_executor._verify_explicit_unwind_complete(check):
        return 0
    return -6
