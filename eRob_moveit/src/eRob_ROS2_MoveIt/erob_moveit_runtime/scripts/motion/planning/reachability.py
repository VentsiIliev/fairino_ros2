#!/usr/bin/env python3
"""Transport-neutral reachability validation for the runtime gateway.

Extracted from ``rest_api_support.py`` so that gateway implementations can
validate pose reachability without importing Flask or other HTTP-only modules.
"""
from __future__ import annotations

from copy import deepcopy
import time

from moveit_msgs.msg import MoveItErrorCodes

import config
from motion.planning.planner_utils import _to_pose_list


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
