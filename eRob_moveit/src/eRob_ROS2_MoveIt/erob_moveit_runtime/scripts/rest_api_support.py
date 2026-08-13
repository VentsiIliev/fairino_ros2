#!/usr/bin/env python3
from __future__ import annotations

from copy import deepcopy
import time
from typing import Any

from flask import jsonify
from moveit_msgs.msg import MoveItErrorCodes
from motion.servo.cartesian_servo.i_cartesian_servo import CartesianServoFrame

import config
from enums import Direction, RobotAxis
from motion.planning.planner_utils import _to_pose_list

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


def _wait_future(future, timeout_s: float = 10.0):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if future.done():
            return future.result()
        time.sleep(0.01)
    raise TimeoutError(f"Timed out waiting for MoveIt service response after {timeout_s:.1f}s")


def _request_ik_for_pose(node, pose, timeout_s: float = 3.0, seed_joint_state=None, avoid_collisions=True):
    ik_client = node.get_ik_client()
    if ik_client is None or not ik_client.wait_for_service(timeout_sec=1.0):
        raise TimeoutError("IK service unavailable")

    from moveit_msgs.srv import GetPositionIK

    req = GetPositionIK.Request()
    req.ik_request.group_name = config.PLANNING_GROUP
    req.ik_request.ik_link_name = config.EE_LINK
    req.ik_request.pose_stamped.header.frame_id = config.BASE_LINK
    req.ik_request.pose_stamped.header.stamp = node.get_clock().now().to_msg()
    req.ik_request.pose_stamped.pose = pose
    req.ik_request.avoid_collisions = bool(avoid_collisions)
    req.ik_request.timeout.sec = int(timeout_s)
    req.ik_request.timeout.nanosec = int((timeout_s - int(timeout_s)) * 1_000_000_000)

    if seed_joint_state is not None:
        state = deepcopy(seed_joint_state)
        state.header.stamp = node.get_clock().now().to_msg()
        req.ik_request.robot_state.joint_state = state
        req.ik_request.robot_state.is_diff = True
    elif node.current_joint_state is not None:
        state = deepcopy(node.current_joint_state)
        state.header.stamp = node.get_clock().now().to_msg()
        req.ik_request.robot_state.joint_state = state
        req.ik_request.robot_state.is_diff = True

    future = ik_client.call_async(req)
    return _wait_future(future, timeout_s=timeout_s + 1.0)


def _check_state_validity(node, joint_names, joint_positions, timeout_s: float = 2.0):
    state_validity_client = node.get_state_validity_client()
    if state_validity_client is None or not state_validity_client.wait_for_service(timeout_sec=1.0):
        raise TimeoutError("State validity service unavailable")

    from moveit_msgs.srv import GetStateValidity
    from sensor_msgs.msg import JointState

    req = GetStateValidity.Request()
    js = JointState()
    js.name = list(joint_names)
    js.position = list(joint_positions)
    req.robot_state.joint_state = js
    req.robot_state.is_diff = True
    req.group_name = config.PLANNING_GROUP

    future = state_validity_client.call_async(req)
    return _wait_future(future, timeout_s=timeout_s + 1.0)


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


def validate_pose_from_start(node, robot, start_position, target_position, tool=0, user=0, start_joint_state_payload=None):
    node.get_logger().info(
        "validate_pose_from_start request: "
        f"start={list(start_position) if isinstance(start_position, (list, tuple)) else start_position} "
        f"target={list(target_position) if isinstance(target_position, (list, tuple)) else target_position} "
        f"tool={int(tool)} user={int(user)} "
        f"seed_joint_state={bool(isinstance(start_joint_state_payload, dict) and start_joint_state_payload)}"
    )
    if len(start_position) != 6 or len(target_position) != 6:
        result = {"reachable": False, "reason": "invalid_pose_format", "fraction": 0.0}
        node.get_logger().warning(f"validate_pose_from_start rejected: {result}")
        return result

    tool_transform = node.get_tool_transform(int(tool))
    start_base = robot.apply_workobject(start_position, user_id=int(user))
    target_base = robot.apply_workobject(target_position, user_id=int(user))

    target_poses, err = _to_pose_list(node, [target_base], tool_transform, check_last_only=True)
    if err:
        result = {"reachable": False, "reason": "target_pose_safety_rejected", "fraction": 0.0, "result": err}
        node.get_logger().warning(f"validate_pose_from_start target pose rejected: {result}")
        return result

    start_joint_state = None
    if isinstance(start_joint_state_payload, dict):
        start_names = list(start_joint_state_payload.get("name", []) or [])
        start_positions = list(start_joint_state_payload.get("position", []) or [])
        if start_names and start_positions and len(start_names) == len(start_positions):
            from sensor_msgs.msg import JointState
            start_joint_state = JointState()
            start_joint_state.name = start_names
            start_joint_state.position = start_positions

    if start_joint_state is None:
        start_poses, err = _to_pose_list(node, [start_base], tool_transform, check_last_only=True)
        if err:
            result = {"reachable": False, "reason": "start_pose_safety_rejected", "fraction": 0.0, "result": err}
            node.get_logger().warning(f"validate_pose_from_start start pose rejected: {result}")
            return result

        try:
            start_ik_resp = _request_ik_for_pose(node, start_poses[0], timeout_s=2.0)
        except Exception as exc:
            result = {"reachable": False, "reason": f"ik_service_error: {exc}", "fraction": 0.0, "result": -2}
            node.get_logger().warning(f"validate_pose_from_start start IK exception: {result}")
            return result

        start_ik_error = int(getattr(getattr(start_ik_resp, "error_code", None), "val", 0))
        start_solution = getattr(start_ik_resp, "solution", None)
        start_joint_state = getattr(start_solution, "joint_state", None)
        start_positions = list(getattr(start_joint_state, "position", [])) if start_joint_state is not None else []
        start_names = list(getattr(start_joint_state, "name", [])) if start_joint_state is not None else []
        if start_ik_error != MoveItErrorCodes.SUCCESS or not start_positions or not start_names:
            result = {"reachable": False, "reason": "start_pose_ik_failed", "fraction": 0.0, "result": -11}
            node.get_logger().warning(f"validate_pose_from_start start IK failed: {result}")
            return result

    try:
        target_ik_resp = _request_ik_for_pose(
            node,
            target_poses[0],
            timeout_s=0.1,
            seed_joint_state=start_joint_state,
        )
    except Exception as exc:
        result = {"reachable": False, "reason": f"ik_service_error: {exc}", "fraction": 0.0, "result": -2}
        node.get_logger().warning(f"validate_pose_from_start target IK exception: {result}")
        return result

    target_ik_error = int(getattr(getattr(target_ik_resp, "error_code", None), "val", 0))
    target_solution = getattr(target_ik_resp, "solution", None)
    target_joint_state = getattr(target_solution, "joint_state", None)
    target_positions = list(getattr(target_joint_state, "position", [])) if target_joint_state is not None else []
    target_names = list(getattr(target_joint_state, "name", [])) if target_joint_state is not None else []
    if target_ik_error != MoveItErrorCodes.SUCCESS or not target_positions or not target_names:
        result = {"reachable": False, "reason": "target_pose_ik_failed", "fraction": 0.0, "result": -11}
        node.get_logger().warning(f"validate_pose_from_start target IK failed: {result}")
        return result

    try:
        validity_resp = _check_state_validity(node, target_names, target_positions, timeout_s=2.0)
    except Exception as exc:
        result = {"reachable": False, "reason": f"state_validity_error: {exc}", "fraction": 0.0, "result": -2}
        node.get_logger().warning(f"validate_pose_from_start state validity exception: {result}")
        return result

    reachable = bool(getattr(validity_resp, "valid", False))
    reason = "ok" if reachable else "target_state_in_collision"
    result = {
        "reachable": reachable,
        "reason": reason,
        "fraction": 1.0 if reachable else 0.0,
        "num_points": 1,
        "result": 0 if reachable else -11,
        "start_position": start_base,
        "target_position": target_base,
        "target_joint_state": {
            "name": target_names,
            "position": target_positions,
        } if reachable else None,
    }
    log_fn = node.get_logger().info if reachable else node.get_logger().warning
    log_fn(f"validate_pose_from_start result: {result}")
    return result
