#!/usr/bin/env python3
from __future__ import annotations

from copy import deepcopy
import time
from typing import Any

from flask import jsonify
from moveit_msgs.msg import MoveItErrorCodes

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
}


def motion_error_response(result: int):
    description = MOTION_ERROR_DESCRIPTIONS.get(result, f"Unknown error code {result}")
    http_status = 503 if result in (-2, -5) else 400 if result in (-3, -11) else 500
    return jsonify({"result": result, "success": False, "error": description}), http_status


def parse_jog_request(data: dict[str, Any] | None) -> tuple[RobotAxis, Direction, float, float, float]:
    payload = data or {}

    axis_val = payload.get("axis")
    if axis_val is None:
        raise ValueError("Missing 'axis'")
    try:
        axis = RobotAxis(axis_val)
    except ValueError as exc:
        raise ValueError(f"Invalid 'axis': {axis_val}") from exc

    direction_val = payload.get("direction")
    if direction_val is None:
        raise ValueError("Missing 'direction'")
    try:
        direction = Direction(direction_val)
    except ValueError as exc:
        raise ValueError(f"Invalid 'direction': {direction_val}") from exc

    try:
        step = float(payload.get("step"))
        vel = float(payload.get("vel"))
        acc = float(payload.get("acc"))
    except (TypeError, ValueError) as exc:
        raise ValueError("Invalid step/vel/acc") from exc

    if axis == RobotAxis.Z:
        step = -step

    return axis, direction, step, vel, acc


def _normalize_scaling(value, default_value: float) -> float:
    numeric = float(default_value if value is None else value)
    return numeric / 100.0 if numeric > 1.0 else numeric


def parse_move_linear_request(data: dict[str, Any] | None) -> dict[str, Any]:
    payload = data or {}
    position = payload.get("position")
    if not position or len(position) != 6:
        raise ValueError("Invalid position format")
    return {
        "position": position,
        "tool": payload.get("tool", 0),
        "user": payload.get("user", 0),
        "vel": payload.get("vel", config.DEFAULT_VEL_PERCENT),
        "acc": payload.get("acc", config.DEFAULT_ACC_PERCENT),
        "blocking": payload.get("blocking", True),
    }


def parse_execute_path_request(data: dict[str, Any] | None) -> dict[str, Any]:
    payload = data or {}
    path = payload.get("path")
    if not path:
        raise ValueError("No path provided")
    if isinstance(path, list) and path and isinstance(path[0], list) and path[0] and isinstance(path[0][0], list):
        path = path[0]
    return {
        "path": path,
        "rx": payload.get("rx"),
        "ry": payload.get("ry"),
        "rz": payload.get("rz"),
        "vel": _normalize_scaling(payload.get("vel"), config.DEFAULT_VEL_SCALING),
        "acc": _normalize_scaling(payload.get("acc"), config.DEFAULT_ACC_SCALING),
        "blocking": payload.get("blocking", False),
    }


def _wait_future(future, timeout_s: float = 10.0):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if future.done():
            return future.result()
        time.sleep(0.01)
    raise TimeoutError(f"Timed out waiting for MoveIt service response after {timeout_s:.1f}s")


def _request_ik_for_pose(node, pose, timeout_s: float = 3.0, seed_joint_state=None):
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
    req.ik_request.avoid_collisions = False
    req.ik_request.timeout.sec = int(timeout_s)
    req.ik_request.timeout.nanosec = int((timeout_s - int(timeout_s)) * 1_000_000_000)

    if seed_joint_state is not None:
        state = deepcopy(seed_joint_state)
        state.header.stamp = node.get_clock().now().to_msg()
        req.ik_request.robot_state.joint_state = state
        req.ik_request.robot_state.is_diff = False
    elif node.current_joint_state is not None:
        state = deepcopy(node.current_joint_state)
        state.header.stamp = node.get_clock().now().to_msg()
        req.ik_request.robot_state.joint_state = state
        req.ik_request.robot_state.is_diff = False

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
    req.group_name = config.PLANNING_GROUP

    future = state_validity_client.call_async(req)
    return _wait_future(future, timeout_s=timeout_s + 1.0)


def validate_pose_from_start(node, robot, start_position, target_position, tool=0, user=0, start_joint_state_payload=None):
    if len(start_position) != 6 or len(target_position) != 6:
        return {"reachable": False, "reason": "invalid_pose_format", "fraction": 0.0}

    tool_transform = node.get_tool_transform(int(tool))
    start_base = robot.apply_workobject(start_position, user_id=int(user))
    target_base = robot.apply_workobject(target_position, user_id=int(user))

    target_poses, err = _to_pose_list(node, [target_base], tool_transform, check_last_only=True)
    if err:
        return {"reachable": False, "reason": "target_pose_safety_rejected", "fraction": 0.0, "result": err}

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
            return {"reachable": False, "reason": "start_pose_safety_rejected", "fraction": 0.0, "result": err}

        try:
            start_ik_resp = _request_ik_for_pose(node, start_poses[0], timeout_s=2.0)
        except Exception as exc:
            return {"reachable": False, "reason": f"ik_service_error: {exc}", "fraction": 0.0, "result": -2}

        start_ik_error = int(getattr(getattr(start_ik_resp, "error_code", None), "val", 0))
        start_solution = getattr(start_ik_resp, "solution", None)
        start_joint_state = getattr(start_solution, "joint_state", None)
        start_positions = list(getattr(start_joint_state, "position", [])) if start_joint_state is not None else []
        start_names = list(getattr(start_joint_state, "name", [])) if start_joint_state is not None else []
        if start_ik_error != MoveItErrorCodes.SUCCESS or not start_positions or not start_names:
            return {"reachable": False, "reason": "start_pose_ik_failed", "fraction": 0.0, "result": -11}

    try:
        target_ik_resp = _request_ik_for_pose(
            node,
            target_poses[0],
            timeout_s=0.1,
            seed_joint_state=start_joint_state,
        )
    except Exception as exc:
        return {"reachable": False, "reason": f"ik_service_error: {exc}", "fraction": 0.0, "result": -2}

    target_ik_error = int(getattr(getattr(target_ik_resp, "error_code", None), "val", 0))
    target_solution = getattr(target_ik_resp, "solution", None)
    target_joint_state = getattr(target_solution, "joint_state", None)
    target_positions = list(getattr(target_joint_state, "position", [])) if target_joint_state is not None else []
    target_names = list(getattr(target_joint_state, "name", [])) if target_joint_state is not None else []
    if target_ik_error != MoveItErrorCodes.SUCCESS or not target_positions or not target_names:
        return {"reachable": False, "reason": "target_pose_ik_failed", "fraction": 0.0, "result": -11}

    try:
        validity_resp = _check_state_validity(node, target_names, target_positions, timeout_s=2.0)
    except Exception as exc:
        return {"reachable": False, "reason": f"state_validity_error: {exc}", "fraction": 0.0, "result": -2}

    reachable = bool(getattr(validity_resp, "valid", False))
    reason = "ok" if reachable else "target_state_in_collision"
    return {
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
