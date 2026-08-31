#!/usr/bin/env python3
from __future__ import annotations

from copy import deepcopy
from time import perf_counter

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    MotionPlanRequest,
    MotionSequenceItem,
    OrientationConstraint,
    PositionConstraint,
    MoveItErrorCodes,
)
from moveit_msgs.srv import GetMotionSequence
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectory

import config
from motion.planning.planner_utils import _begin_execution, _is_stale, _set_result, _to_pose_list
from motion.execution.trajectory_executor import _send_trajectory_to_controller


_MOVEIT_ERROR_NAMES = {
    int(value): name.lower().replace("_", " ")
    for name in dir(MoveItErrorCodes)
    if name.isupper() and isinstance((value := getattr(MoveItErrorCodes, name)), int)
}


def _moveit_error_message(error_code: int) -> str:
    reason = _MOVEIT_ERROR_NAMES.get(error_code, "unknown MoveIt error")
    return f"Pilz LIN planning failed: {reason} (MoveIt error_code={error_code})"


def _duration_to_sec(duration_msg):
    return float(duration_msg.sec) + float(duration_msg.nanosec) / 1e9


def _sec_to_duration(seconds):
    whole = int(seconds)
    nanos = int(round((float(seconds) - whole) * 1e9))
    if nanos >= 1_000_000_000:
        whole += 1
        nanos -= 1_000_000_000
    return Duration(sec=whole, nanosec=nanos)


def _pose_goal_constraints(pose: Pose) -> Constraints:
    constraints = Constraints()

    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    tolerance_m = max(1e-5, float(getattr(config, "SEQUENCE_POSITION_TOLERANCE_M", 0.0005)))
    box.dimensions = [tolerance_m, tolerance_m, tolerance_m]

    position_constraint = PositionConstraint()
    position_constraint.header.frame_id = config.BASE_LINK
    position_constraint.link_name = config.EE_LINK
    position_constraint.constraint_region = BoundingVolume()
    position_constraint.constraint_region.primitives.append(box)
    position_constraint.constraint_region.primitive_poses.append(pose)
    position_constraint.weight = 1.0

    orientation_constraint = OrientationConstraint()
    orientation_constraint.header.frame_id = config.BASE_LINK
    orientation_constraint.link_name = config.EE_LINK
    orientation_constraint.orientation = pose.orientation
    tolerance_rad = max(1e-5, float(getattr(config, "SEQUENCE_ORIENTATION_TOLERANCE_RAD", 0.01)))
    orientation_constraint.absolute_x_axis_tolerance = tolerance_rad
    orientation_constraint.absolute_y_axis_tolerance = tolerance_rad
    orientation_constraint.absolute_z_axis_tolerance = tolerance_rad
    orientation_constraint.parameterization = OrientationConstraint.ROTATION_VECTOR
    orientation_constraint.weight = 1.0

    constraints.position_constraints.append(position_constraint)
    constraints.orientation_constraints.append(orientation_constraint)
    return constraints


def _build_motion_plan_request(
    rc,
    pose: Pose,
    segment: dict,
    start_state=None,
    planning_group=None,
) -> MotionPlanRequest:
    req = MotionPlanRequest()
    req.group_name = str(planning_group or config.PLANNING_GROUP)
    req.pipeline_id = str(getattr(config, "SEQUENCE_PLANNING_PIPELINE", "pilz_industrial_motion_planner"))
    motion_type = str(segment.get("motion_type") or "linear").strip().lower()
    req.planner_id = "PTP" if motion_type == "ptp" else "LIN"
    req.num_planning_attempts = 1
    req.allowed_planning_time = float(getattr(config, "SEQUENCE_ALLOWED_PLANNING_TIME_S", 5.0))
    req.max_velocity_scaling_factor = max(0.0, min(1.0, float(segment["vel"]) / 100.0))
    req.max_acceleration_scaling_factor = max(0.0, min(1.0, float(segment["acc"]) / 100.0))
    req.goal_constraints.append(_pose_goal_constraints(pose))

    if start_state is not None:
        req.start_state = start_state
    return req


def _combine_joint_trajectories(trajectories) -> JointTrajectory | None:
    combined = JointTrajectory()
    offset_s = 0.0
    for trajectory_index, trajectory in enumerate(trajectories):
        joint_trajectory = getattr(trajectory, "joint_trajectory", None)
        points = list(getattr(joint_trajectory, "points", []) or [])
        if not points:
            continue
        if not combined.joint_names:
            combined.joint_names = list(joint_trajectory.joint_names)
        elif list(joint_trajectory.joint_names) != list(combined.joint_names):
            return None

        start_index = 1 if trajectory_index > 0 and combined.points else 0
        for point in points[start_index:]:
            adjusted = deepcopy(point)
            adjusted.time_from_start = _sec_to_duration(_duration_to_sec(point.time_from_start) + offset_s)
            combined.points.append(adjusted)
        offset_s = _duration_to_sec(combined.points[-1].time_from_start)
    return combined if combined.points else None


def _sequence_response(rc, future, generation, started_at):
    if _is_stale(rc, generation):
        rc.get_logger().info("[Sequence] Stale response discarded (preempted)")
        return

    try:
        response = future.result()
        sequence_response = getattr(response, "response", None)
        error_code = int(getattr(getattr(sequence_response, "error_code", None), "val", 0))
        planned = list(getattr(sequence_response, "planned_trajectories", []) or [])
        plan_elapsed_s = perf_counter() - started_at
        rc.get_logger().info(
            f"[TIMING] sequence_plan segments={len(planned)} error_code={error_code} "
            f"elapsed_s={plan_elapsed_s:.3f}"
        )
        try:
            from motion.move_linear_timing import mark as mark_move_linear_timing
            mark_move_linear_timing(
                rc,
                "planning_done",
                strategy="pilz_lin",
                error_code=error_code,
                segments=len(planned),
                plan_elapsed_s=plan_elapsed_s,
            )
        except Exception:
            pass
        if error_code != 1:
            message = _moveit_error_message(error_code)
            rc.last_motion_error = message
            rc.get_logger().error(f"[Sequence] {message}")
            _set_result(rc, -6)
            executor = getattr(rc, "trajectory_executor", None)
            if executor is not None:
                executor.process_next_queued_task()
            return
        joint_trajectory = _combine_joint_trajectories(planned)
        if joint_trajectory is None:
            message = "Pilz LIN planning returned no usable joint trajectory"
            rc.last_motion_error = message
            rc.get_logger().error(f"[Sequence] {message}")
            _set_result(rc, -6)
            executor = getattr(rc, "trajectory_executor", None)
            if executor is not None:
                executor.process_next_queued_task()
            return
        _send_trajectory_to_controller(rc, joint_trajectory)
    except Exception as exc:
        message = f"Pilz LIN planning service call failed: {exc}"
        rc.last_motion_error = message
        rc.get_logger().error(f"[Sequence] {message}")
        _set_result(rc, -2)
        executor = getattr(rc, "trajectory_executor", None)
        if executor is not None:
            executor.process_next_queued_task()


def send_motion_sequence(rc, segments, tool_transform=None, planning_group=None) -> int:
    if not segments:
        rc.last_motion_error = "Pilz LIN request contains no motion segments"
        rc.get_logger().error("[Sequence] Empty motion sequence")
        return -1
    if not rc.wait_for_motion_sequence_service(timeout_sec=1.0):
        rc.last_motion_error = "MoveIt motion-sequence service is unavailable"
        rc.get_logger().error("[Sequence] GetMotionSequence service not available")
        return -2
    if not rc.is_motion_stack_ready():
        rc.last_motion_error = f"Motion stack is not ready: {rc.get_motion_stack_fault_reason()}"
        rc.get_logger().error(f"[Sequence] Motion stack not ready: {rc.get_motion_stack_fault_reason()}")
        return config.MOTION_ERROR_HARDWARE_NOT_READY

    T_tool = tool_transform if tool_transform is not None else rc.T_tool
    waypoints = [list(segment["position"]) for segment in segments]
    poses, err = _to_pose_list(rc, waypoints, T_tool, check_last_only=False)
    if err:
        return err

    req = GetMotionSequence.Request()
    start_state = None
    if rc.current_joint_state is not None:
        start_state = deepcopy(rc.current_joint_state)
        start_state.header.stamp = rc.get_clock().now().to_msg()

    for index, (segment, pose) in enumerate(zip(segments, poses)):
        item = MotionSequenceItem()
        item.req = _build_motion_plan_request(
            rc,
            pose,
            segment,
            start_state=None if index > 0 else _robot_state_from_joint_state(start_state),
            planning_group=planning_group,
        )
        item.blend_radius = float(segment.get("blend_radius", 0.0)) / 1000.0
        req.request.items.append(item)

    rc.get_logger().info(
        f"[Sequence] Planning explicit motion sequence with {len(req.request.items)} segments "
        f"group={req.request.items[0].req.group_name} "
        f"vel_acc_blend="
        f"{[(round(float(segment.get('vel', 0.0)), 3), round(float(segment.get('acc', 0.0)), 3), round(float(segment.get('blend_radius', 0.0)), 3)) for segment in segments]}"
    )
    try:
        from motion.move_linear_timing import mark as mark_move_linear_timing
        mark_move_linear_timing(rc, "planning_start", strategy="pilz_lin", segments=len(req.request.items))
    except Exception:
        pass
    generation = _begin_execution(rc)
    started_at = perf_counter()
    future = rc.request_motion_sequence(req)
    future.add_done_callback(lambda f: _sequence_response(rc, f, generation, started_at))
    return 0


def _robot_state_from_joint_state(joint_state):
    if joint_state is None:
        return None
    from moveit_msgs.msg import RobotState

    clean_joint_state = deepcopy(joint_state)
    joint_count = len(clean_joint_state.name) or len(clean_joint_state.position)
    clean_joint_state.velocity = [0.0] * joint_count
    clean_joint_state.effort = []

    state = RobotState()
    state.joint_state = clean_joint_state
    state.is_diff = False
    return state
