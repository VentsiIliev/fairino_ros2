#!/usr/bin/env python3
from __future__ import annotations

from copy import deepcopy
from typing import Any

from flask import jsonify
from motion.servo.cartesian_servo.i_cartesian_servo import CartesianServoFrame

import config
from enums import Direction, RobotAxis
from motion.planning.reachability import validate_pose_from_start

MOTION_ERROR_DESCRIPTIONS = {
    -1:  "Busy / invalid input / generic error",
    -2:  "MoveIt service unavailable",
    -3:  "Safety violation: target outside workspace",
    -4:  "No current robot position available",
    -5:  "Motion queue full",
    -6:  "Path planning failed: MoveIt returned no trajectory",
    -7:  "Time parameterization failed (TOTG/Ruckig)",
    -8:  "Jacobian fallback path planning failed",
    -9:  "Near-singularity detected",
    -10: "Collision detected during Jacobian check",
    -11: "Cartesian path planning failed: target unreachable, collision, or joint-limit constraint",
    -12: "Hardware not ready: EtherCAT slave not in OP",
    -13: "Drive operation is not enabled; call POST /drive/enable before motion",
    -14: "Controller execution failed: trajectory tolerance or controller action error",
}


def motion_error_response(result: int, **extra):
    description = MOTION_ERROR_DESCRIPTIONS.get(result, f"Unknown error code {result}")
    http_status = 503 if result in (-2, -5, -12) else 409 if result in (-13, -14) else 400 if result in (-3, -11) else 500
    body = {"result": result, "success": False, "error": description}
    body.update(extra)
    body["success"] = False
    return jsonify(body), http_status


def parse_jog_request(data: dict[str, Any] | None) -> dict[str, Any]:
    payload = data or {}

    axis_val = payload.get("axis")
    if axis_val is None:
        raise ValueError("Missing 'axis'")
    try:
        axis = RobotAxis.get_by_string(axis_val) if isinstance(axis_val, str) else RobotAxis(axis_val)
    except ValueError as exc:
        valid_axes = [axis.name for axis in RobotAxis]
        raise ValueError(f"Invalid 'axis': {axis_val}. Valid axes: {valid_axes}") from exc

    direction_val = payload.get("direction")
    if direction_val is None:
        raise ValueError("Missing 'direction'")
    try:
        if isinstance(direction_val, str):
            direction_name = direction_val.strip().upper()
            direction_aliases = {
                "POSITIVE": "PLUS",
                "NEGATIVE": "MINUS",
            }
            direction = Direction.get_by_string(direction_aliases.get(direction_name, direction_name))
        else:
            direction = Direction(direction_val)
    except ValueError as exc:
        valid_directions = [direction.name for direction in Direction]
        raise ValueError(f"Invalid 'direction': {direction_val}. Valid directions: {valid_directions}") from exc

    try:
        step = float(payload.get("step"))
        vel = float(payload.get("vel"))
        acc = float(payload.get("acc"))
    except (TypeError, ValueError) as exc:
        raise ValueError("Invalid step/vel/acc") from exc

    if axis == RobotAxis.Z:
        step = -step

    frame_value = payload.get("frame", "user")
    try:
        if isinstance(frame_value, int) or (
            isinstance(frame_value, str) and frame_value.strip().lstrip("-").isdigit()
        ):
            frame = CartesianServoFrame.USER
            user = int(frame_value)
        else:
            frame = CartesianServoFrame(str(frame_value).strip().lower())
            user = int(payload.get("user", 0))
    except (TypeError, ValueError) as exc:
        valid_frames = [frame.value for frame in CartesianServoFrame]
        raise ValueError(
            f"Invalid 'frame': {frame_value}. Valid frames: {valid_frames} or numeric user frame id"
        ) from exc

    try:
        tool = int(payload.get("tool", 0))
    except (TypeError, ValueError) as exc:
        raise ValueError("Invalid 'tool'") from exc

    return {
        "axis": axis,
        "direction": direction,
        "step": step,
        "vel": vel,
        "acc": acc,
        "frame": frame,
        "tool": tool,
        "user": user,
    }


def parse_joint_jog_request(data: dict[str, Any] | None) -> dict[str, Any]:
    payload = data or {}

    joint_value = payload.get("joint")
    if joint_value is None:
        raise ValueError("Missing 'joint'")
    joint = str(joint_value).strip()
    if not joint:
        raise ValueError("Invalid 'joint'")
    aliases = {f"J{index}": f"Joint_{index}" for index in range(1, 7)}
    joint = aliases.get(joint.upper(), joint)
    valid_joints = list(getattr(config, "JOINT_NAMES", []) or [])
    if joint not in valid_joints:
        raise ValueError(f"Invalid 'joint': {joint_value}. Valid joints: {valid_joints}")

    direction_val = payload.get("direction")
    if direction_val is None:
        raise ValueError("Missing 'direction'")
    try:
        if isinstance(direction_val, str):
            direction_name = direction_val.strip().upper()
            direction_aliases = {
                "POSITIVE": "PLUS",
                "NEGATIVE": "MINUS",
            }
            direction = Direction.get_by_string(direction_aliases.get(direction_name, direction_name))
        else:
            direction = Direction(direction_val)
    except ValueError as exc:
        valid_directions = [direction.name for direction in Direction]
        raise ValueError(f"Invalid 'direction': {direction_val}. Valid directions: {valid_directions}") from exc

    try:
        step = float(payload.get("step"))
        vel = float(payload.get("vel", config.DEFAULT_VEL_PERCENT))
        acc = float(payload.get("acc", config.DEFAULT_ACC_PERCENT))
    except (TypeError, ValueError) as exc:
        raise ValueError("Invalid step/vel/acc") from exc

    return {
        "joint": joint,
        "direction": direction,
        "step": step,
        "vel": vel,
        "acc": acc,
        "blocking": bool(payload.get("blocking", True)),
    }


def parse_servo_jog_start_request(data: dict[str, Any] | None) -> dict[str, Any]:
    payload = data or {}

    axis_val = payload.get("axis")
    if axis_val is None:
        raise ValueError("Missing 'axis'")
    try:
        axis = RobotAxis.get_by_string(axis_val) if isinstance(axis_val, str) else RobotAxis(axis_val)
    except ValueError as exc:
        valid_axes = [axis.name for axis in RobotAxis]
        raise ValueError(f"Invalid 'axis': {axis_val}. Valid axes: {valid_axes}") from exc

    direction_val = payload.get("direction")
    if direction_val is None:
        raise ValueError("Missing 'direction'")
    try:
        if isinstance(direction_val, str):
            direction_name = direction_val.strip().upper()
            direction_aliases = {
                "POSITIVE": "PLUS",
                "NEGATIVE": "MINUS",
            }
            direction = Direction.get_by_string(direction_aliases.get(direction_name, direction_name))
        else:
            direction = Direction(direction_val)
    except ValueError as exc:
        valid_directions = [direction.name for direction in Direction]
        raise ValueError(f"Invalid 'direction': {direction_val}. Valid directions: {valid_directions}") from exc

    linear_mm_s = payload.get("linear_mm_s")
    angular_deg_s = payload.get("angular_deg_s")
    vel = payload.get("vel")
    acc = payload.get("acc")
    try:
        linear_mm_s = None if linear_mm_s is None else float(linear_mm_s)
        angular_deg_s = None if angular_deg_s is None else float(angular_deg_s)
        vel = None if vel is None else float(vel)
        acc = None if acc is None else float(acc)
    except (TypeError, ValueError) as exc:
        raise ValueError("Invalid linear_mm_s/angular_deg_s/vel/acc") from exc

    if linear_mm_s is None and angular_deg_s is None and vel is None:
        raise ValueError("Missing speed; provide linear_mm_s/angular_deg_s or vel")

    frame_value = payload.get("frame", "user")
    try:
        if isinstance(frame_value, int) or (
            isinstance(frame_value, str) and frame_value.strip().lstrip("-").isdigit()
        ):
            frame = CartesianServoFrame.USER
            user = int(frame_value)
        else:
            frame = CartesianServoFrame(str(frame_value).strip().lower())
            user = int(payload.get("user", 0))
    except (TypeError, ValueError) as exc:
        valid_frames = [frame.value for frame in CartesianServoFrame]
        raise ValueError(
            f"Invalid 'frame': {frame_value}. Valid frames: {valid_frames} or numeric user frame id"
        ) from exc

    try:
        tool = int(payload.get("tool", 0))
    except (TypeError, ValueError) as exc:
        raise ValueError("Invalid 'tool'") from exc

    return {
        "axis": axis,
        "direction": direction,
        "vel": vel,
        "acc": acc,
        "linear_mm_s": linear_mm_s,
        "angular_deg_s": angular_deg_s,
        "frame": frame,
        "tool": tool,
        "user": user,
    }


def parse_move_linear_request(data: dict[str, Any] | None) -> dict[str, Any]:
    payload = data or {}
    position = payload.get("position")
    if not position or len(position) != 6:
        raise ValueError("Invalid position format")
    trajectory_optimizer = payload.get("trajectory_optimizer")
    if trajectory_optimizer is not None:
        trajectory_optimizer = str(trajectory_optimizer).strip().upper()
        if trajectory_optimizer not in {"TOTG", "RUCKIG"}:
            raise ValueError("Invalid trajectory_optimizer; expected TOTG or RUCKIG")
    return {
        "position": position,
        "tool": payload.get("tool", 0),
        "user": payload.get("user", 0),
        "vel": payload.get("vel", config.DEFAULT_VEL_PERCENT),
        "acc": payload.get("acc", config.DEFAULT_ACC_PERCENT),
        "blocking": payload.get("blocking", True),
        "trajectory_optimizer": trajectory_optimizer,
    }


def parse_execute_path_request(data: dict[str, Any] | None) -> dict[str, Any]:
    payload = data or {}
    path = payload.get("path")
    if not path:
        raise ValueError("No path provided")
    if isinstance(path, list) and path and isinstance(path[0], list) and path[0] and isinstance(path[0][0], list):
        path = path[0]
    trajectory_optimizer = payload.get("trajectory_optimizer")
    if trajectory_optimizer is not None:
        trajectory_optimizer = str(trajectory_optimizer).strip().upper()
        if trajectory_optimizer not in {"TOTG", "RUCKIG"}:
            raise ValueError("Invalid trajectory_optimizer; expected TOTG or RUCKIG")
    orientation_mode = str(payload.get("orientation_mode", "constant")).strip().lower()
    if orientation_mode not in {"constant", "per_waypoint"}:
        raise ValueError("Invalid orientation_mode; expected constant or per_waypoint")
    return {
        "path": path,
        "rx": payload.get("rx", payload.get("rx_degrees")),
        "ry": payload.get("ry", payload.get("ry_degrees")),
        "rz": payload.get("rz", payload.get("rz_degrees")),
        # Keep platform-provided 0-100 percentages intact here.
        # The backend normalizes once before passing values into MoveIt.
        "vel": payload.get("vel", config.DEFAULT_VEL_PERCENT),
        "acc": payload.get("acc", config.DEFAULT_ACC_PERCENT),
        "blocking": payload.get("blocking", False),
        "trajectory_optimizer": trajectory_optimizer,
        "orientation_mode": orientation_mode,
    }


def parse_execute_sequence_request(data: dict[str, Any] | None) -> dict[str, Any]:
    payload = data or {}
    raw_segments = payload.get("segments")
    if not isinstance(raw_segments, list) or not raw_segments:
        raise ValueError("Missing non-empty 'segments'")
    segments = []
    for index, raw_segment in enumerate(raw_segments):
        if not isinstance(raw_segment, dict):
            raise ValueError(f"Invalid segment {index}: expected object")
        position = raw_segment.get("position")
        if not position or len(position) != 6:
            raise ValueError(f"Invalid segment {index}: position must have 6 values")
        motion_type = str(raw_segment.get("motion_type", "linear")).strip().lower()
        if motion_type not in {"linear", "ptp"}:
            raise ValueError(f"Invalid segment {index}: motion_type must be linear or ptp")
        try:
            vel = float(raw_segment.get("vel"))
            acc = float(raw_segment.get("acc"))
            blend_radius = float(raw_segment.get("blend_radius", 0.0))
        except (TypeError, ValueError) as exc:
            raise ValueError(f"Invalid segment {index}: vel/acc/blend_radius must be numeric") from exc
        segments.append({
            "position": deepcopy(position),
            "vel": vel,
            "acc": acc,
            "motion_type": motion_type,
            "blend_radius": blend_radius,
        })
    return {
        "segments": segments,
        "tool": int(payload.get("tool", 0)),
        "user": int(payload.get("user", 0)),
        "blocking": bool(payload.get("blocking", False)),
    }


def parse_execute_ordered_motion_chain_request(data: dict[str, Any] | None) -> dict[str, Any]:
    payload = data or {}
    raw_segments = payload.get("segments")
    if not isinstance(raw_segments, list) or not raw_segments:
        raise ValueError("Missing non-empty 'segments'")
    trajectory_optimizer = payload.get("trajectory_optimizer")
    if trajectory_optimizer is not None:
        trajectory_optimizer = str(trajectory_optimizer).strip().upper()
        if trajectory_optimizer not in {"TOTG", "RUCKIG"}:
            raise ValueError("Invalid trajectory_optimizer; expected TOTG or RUCKIG")

    segments = []
    for index, raw_segment in enumerate(raw_segments):
        if not isinstance(raw_segment, dict):
            raise ValueError(f"Invalid segment {index}: expected object")
        segment_type = str(raw_segment.get("type") or raw_segment.get("kind") or "").strip().lower()
        if segment_type not in {"linear", "ptp", "path", "unwind_joint6"}:
            raise ValueError(f"Invalid segment {index}: unsupported type {segment_type!r}")
        segment = {"type": segment_type, "label": str(raw_segment.get("label") or f"segment_{index + 1}")}
        if segment_type in {"linear", "ptp"}:
            position = raw_segment.get("position")
            if not position or len(position) != 6:
                raise ValueError(f"Invalid segment {index}: linear position must have 6 values")
            blend_r = float(raw_segment.get("blendR", 0.0) or 0.0)

            if blend_r < 0.0:
                raise ValueError(
                    f"Invalid segment {index}: blendR must be >= 0"
                )

            segment.update({
                "position": deepcopy(position),
                "vel": float(
                    raw_segment.get(
                        "vel",
                        config.DEFAULT_VEL_PERCENT,
                    )
                ),
                "acc": float(
                    raw_segment.get(
                        "acc",
                        config.DEFAULT_ACC_PERCENT,
                    )
                ),
                "blendR": blend_r,
            })
        elif segment_type == "path":
            path = raw_segment.get("path")
            if not path:
                raise ValueError(f"Invalid segment {index}: path is required")
            segment.update({
                "path": deepcopy(path),
                "vel": float(raw_segment.get("vel", config.DEFAULT_VEL_PERCENT)),
                "acc": float(raw_segment.get("acc", config.DEFAULT_ACC_PERCENT)),
            })
        elif segment_type == "unwind_joint6":
            segment.update({
                "vel": float(raw_segment.get("vel", config.DEFAULT_VEL_PERCENT)),
                "acc": float(raw_segment.get("acc", config.DEFAULT_ACC_PERCENT)),
                "queue_if_busy": bool(raw_segment.get("queue_if_busy", True)),
            })
        segments.append(segment)
    return {
        "segments": segments,
        "tool": int(payload.get("tool", 0)),
        "user": int(payload.get("user", 0)),
        "blocking": bool(payload.get("blocking", True)),
        "trajectory_optimizer": trajectory_optimizer,
    }


def parse_prepare_ordered_motion_chain_request(data: dict[str, Any] | None) -> dict[str, Any]:
    payload = parse_execute_ordered_motion_chain_request(data)
    raw_start = (data or {}).get("start_position")
    if not isinstance(raw_start, (list, tuple)) or len(raw_start) != 6:
        raise ValueError("Missing 6-value 'start_position'")
    payload["start_position"] = [float(value) for value in raw_start]
    payload["blocking"] = True
    return payload


def parse_cartesian_servo_start_request(
    data: dict[str, Any] | None,
) -> dict[str, Any]:
    payload = data or {}

    frame_value = payload.get("frame")
    if frame_value is None:
        raise ValueError("Missing 'frame'")

    try:
        if isinstance(frame_value, int) or (
            isinstance(frame_value, str) and frame_value.strip().lstrip("-").isdigit()
        ):
            frame = CartesianServoFrame.USER
            user = int(frame_value)
        else:
            frame = CartesianServoFrame(str(frame_value).strip().lower())
            user = int(payload.get("user", 0))
    except (TypeError, ValueError) as exc:
        valid_frames = [frame.value for frame in CartesianServoFrame]
        raise ValueError(
            f"Invalid 'frame': {frame_value}. Valid frames: {valid_frames} or numeric user frame id"
        ) from exc

    try:
        tool = int(payload.get("tool", 0))
    except (TypeError, ValueError) as exc:
        raise ValueError("Invalid 'tool'") from exc

    return {
        "frame": frame,
        "tool": tool,
        "user": user,
    }


def parse_cartesian_servo_update_request(
    data: dict[str, Any] | None,
) -> dict[str, Any]:
    payload = data or {}

    linear = payload.get("linear_mm_s")
    angular = payload.get("angular_deg_s")

    if not isinstance(linear, (list, tuple)) or len(linear) != 3:
        raise ValueError(
            "Invalid 'linear_mm_s'; expected [vx, vy, vz]"
        )

    if not isinstance(angular, (list, tuple)) or len(angular) != 3:
        raise ValueError(
            "Invalid 'angular_deg_s'; expected [wx, wy, wz]"
        )

    try:
        linear = [float(value) for value in linear]
        angular = [float(value) for value in angular]
    except (TypeError, ValueError) as exc:
        raise ValueError(
            "Servo velocity values must be numeric"
        ) from exc

    return {
        "linear_mm_s": linear,
        "angular_deg_s": angular,
    }


def _state_validity_contact_pairs(validity_resp) -> list[str]:
    pairs = []
    for contact in list(getattr(validity_resp, "contacts", []) or []):
        body_1 = str(getattr(contact, "contact_body_1", "") or getattr(contact, "body_name_1", ""))
        body_2 = str(getattr(contact, "contact_body_2", "") or getattr(contact, "body_name_2", ""))
        if body_1 or body_2:
            pair = f"{body_1}<->{body_2}" if body_1 and body_2 else body_1 or body_2
            if pair not in pairs:
                pairs.append(pair)
    return pairs


def _coerce_pose6(value, label: str) -> list[float]:
    if not isinstance(value, (list, tuple)) or len(value) != 6:
        raise ValueError(f"Invalid {label}; expected [x,y,z,rx,ry,rz]")
    try:
        return [float(v) for v in value]
    except (TypeError, ValueError) as exc:
        raise ValueError(f"Invalid {label}; all values must be numeric") from exc
