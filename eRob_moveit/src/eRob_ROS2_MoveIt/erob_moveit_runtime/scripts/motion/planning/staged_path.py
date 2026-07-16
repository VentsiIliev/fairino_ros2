#!/usr/bin/env python3
from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
from time import perf_counter

from moveit_msgs.msg import RobotState

import config
from motion.execution.trajectory_executor import _send_trajectory_to_controller
from motion.planning.custom_sequence import (
    _optimize_sync,
    _plan_segment,
    _wait_execution_complete,
    _wait_future,
)
from motion.planning.direct_contour_ik import (
    _build_direct_contour_trajectory,
    _direct_ik_should_run,
    _log_report,
)
from motion.planning.planner_utils import _to_pose_list
from motion.planning.trajectory import _path_length_mm, _simplify_cartesian_waypoints
from motion.planning.trajectory_planner import _build_cartesian_request


def _clean_live_start_state(rc) -> RobotState:
    state = RobotState()
    clean_joint_state = deepcopy(rc.current_joint_state)
    clean_joint_state.header.stamp = rc.get_clock().now().to_msg()
    clean_joint_state.velocity = [0.0] * (len(clean_joint_state.name) or len(clean_joint_state.position))
    clean_joint_state.effort = []
    state.joint_state = clean_joint_state
    state.is_diff = False
    return state


def _build_follow_path_trajectory(
    rc,
    *,
    command_path: list[list[float]],
    start_state: RobotState,
    tool_transform,
    vel_scaling: float,
    acc_scaling: float,
    trajectory_optimizer_name=None,
):
    started = perf_counter()
    waypoints_6d = _simplify_cartesian_waypoints([list(point[:6]) for point in command_path])
    total_dist_mm = _path_length_mm(waypoints_6d)
    poses, err = _to_pose_list(rc, waypoints_6d, tool_transform, check_last_only=True)
    if err:
        raise RuntimeError(f"staged path pose conversion failed with result {err}")

    if _direct_ik_should_run(rc, waypoints_6d, total_dist_mm):
        result = _build_direct_contour_trajectory(rc, poses, seed_state=start_state)
        result.report.timings["total_before_optimizer_s"] = perf_counter() - started
        _log_report(rc, result.report)
        if result.report.ok:
            optimized, optimize_elapsed = _optimize_sync(
                rc,
                result.trajectory,
                vel_scaling,
                acc_scaling,
                optimizer_name=trajectory_optimizer_name,
            )
            rc.get_logger().info(
                f"[TIMING] staged_path_follow_plan method=direct_contour_ik "
                f"waypoints={len(waypoints_6d)} optimize_s={optimize_elapsed:.3f} "
                f"elapsed_s={perf_counter() - started:.3f}"
            )
            return optimized.joint_trajectory
        rc.get_logger().warning(
            f"[StagedPath] Direct contour IK rejected follow path; falling back to Cartesian path: "
            f"{result.report.failure_reason} {result.report.details}"
        )

    avg_spacing_mm = total_dist_mm / max(len(waypoints_6d) - 1, 1)
    max_step_scale = float(getattr(config, "PATH_EEF_STEP_SCALE", 2.5))
    max_step_min_m = float(getattr(config, "PATH_EEF_STEP_MIN_M", 0.005))
    max_step_max_m = float(getattr(config, "PATH_EEF_STEP_MAX_M", 0.03))
    max_step = min(max((avg_spacing_mm / 1000.0) * max_step_scale, max_step_min_m), max_step_max_m)
    request = _build_cartesian_request(
        rc,
        poses,
        max_step,
        vel_scaling,
        acc_scaling,
        start_state=start_state,
        avoid_collisions=config.resolve_avoid_collisions(None),
    )
    response = _wait_future(
        rc.request_cartesian_path(request),
        timeout_s=float(getattr(config, "CUSTOM_SEQUENCE_PLAN_TIMEOUT_S", 10.0)),
    )
    fraction = float(getattr(response, "fraction", 0.0))
    trajectory = getattr(response, "solution", None)
    joint_trajectory = getattr(trajectory, "joint_trajectory", None) if trajectory is not None else None
    point_count = len(getattr(joint_trajectory, "points", []) or [])
    if fraction < config.CARTESIAN_MIN_FRACTION or point_count <= 1:
        raise RuntimeError(f"staged path Cartesian planning failed: fraction={fraction:.4f} points={point_count}")

    optimized, optimize_elapsed = _optimize_sync(
        rc,
        trajectory,
        vel_scaling,
        acc_scaling,
        optimizer_name=trajectory_optimizer_name,
    )
    rc.get_logger().info(
        f"[TIMING] staged_path_follow_plan method=cartesian_path waypoints={len(waypoints_6d)} "
        f"fraction={fraction:.4f} points={point_count} optimize_s={optimize_elapsed:.3f} "
        f"elapsed_s={perf_counter() - started:.3f}"
    )
    return optimized.joint_trajectory


def execute_staged_path(
    rc,
    *,
    stage_position: list[float],
    command_path: list[list[float]],
    stage_vel: float,
    stage_acc: float,
    path_vel: float,
    path_acc: float,
    tool_transform=None,
    trajectory_optimizer_name=None,
) -> int:
    if not stage_position or len(stage_position) != 6:
        rc.get_logger().error("[StagedPath] Invalid stage position")
        return -1
    if not command_path:
        rc.get_logger().error("[StagedPath] Empty follow path")
        return -1
    if getattr(rc, "current_joint_state", None) is None:
        rc.get_logger().error("[StagedPath] No current joint state available")
        return -4
    if getattr(rc, "prev_cartesian", None) is None or len(rc.prev_cartesian) < 6:
        rc.get_logger().error("[StagedPath] No current Cartesian position available")
        return -4
    if not rc.is_motion_stack_ready():
        rc.get_logger().error(f"[StagedPath] Motion stack not ready: {rc.get_motion_stack_fault_reason()}")
        return config.MOTION_ERROR_HARDWARE_NOT_READY

    started = perf_counter()
    planning_rc = getattr(rc, "planner_context", rc)
    T_tool = tool_transform if tool_transform is not None else planning_rc.T_tool
    start_state = _clean_live_start_state(planning_rc)
    start_cartesian = list(planning_rc.prev_cartesian[:6])
    previous_execution_suppress = bool(getattr(rc, "_suppress_post_success_unwind", False))
    executor = ThreadPoolExecutor(max_workers=1)
    follow_path_future = None

    try:
        stage_segment = {
            "label": "stage",
            "position": list(stage_position),
            "vel": float(stage_vel),
            "acc": float(stage_acc),
            "motion_type": "linear",
        }
        stage = _plan_segment(
            planning_rc,
            index=0,
            segment=stage_segment,
            start_cartesian=start_cartesian,
            start_state=start_state,
            tool_transform=T_tool,
        )
        path_vel_scaling = max(0.0, min(1.0, float(path_vel) / 100.0))
        path_acc_scaling = max(0.0, min(1.0, float(path_acc) / 100.0))
        follow_path_future = executor.submit(
            _build_follow_path_trajectory,
            planning_rc,
            command_path=[list(point) for point in command_path],
            start_state=stage.final_state,
            tool_transform=T_tool,
            vel_scaling=path_vel_scaling,
            acc_scaling=path_acc_scaling,
            trajectory_optimizer_name=trajectory_optimizer_name,
        )

        setattr(rc, "_suppress_post_success_unwind", True)
        stage_exec_started = perf_counter()
        duration = stage.joint_trajectory.points[-1].time_from_start
        duration_s = float(duration.sec) + float(duration.nanosec) / 1e9
        timeout_s = max(
            float(getattr(config, "EXECUTOR_TIME_MIN_S", 5.0)),
            duration_s * float(getattr(config, "EXECUTOR_TIME_MULTIPLIER", 2.0)),
        )
        rc.get_logger().info(
            f"[StagedPath] Sending stage move points={len(stage.joint_trajectory.points)} "
            f"duration_s={duration_s:.3f} plan_s={stage.plan_elapsed_s:.3f} "
            f"optimize_s={stage.optimize_elapsed_s:.3f}"
        )
        _send_trajectory_to_controller(rc, stage.joint_trajectory)
        stage_result = _wait_execution_complete(rc, timeout_s=timeout_s + 2.0)
        rc.get_logger().info(
            f"[TIMING] staged_path_stage_execute result={stage_result} "
            f"elapsed_s={perf_counter() - stage_exec_started:.3f}"
        )
        if stage_result != 0:
            return stage_result

        wait_started = perf_counter()
        follow_joint_trajectory = follow_path_future.result(
            timeout=float(getattr(config, "CUSTOM_SEQUENCE_PLAN_TIMEOUT_S", 10.0)) + 5.0
        )
        rc.get_logger().info(
            f"[TIMING] staged_path_plan_ready wait_after_stage_s={perf_counter() - wait_started:.3f}"
        )

        setattr(rc, "_suppress_post_success_unwind", False)
        follow_exec_started = perf_counter()
        duration = follow_joint_trajectory.points[-1].time_from_start
        duration_s = float(duration.sec) + float(duration.nanosec) / 1e9
        timeout_s = max(
            float(getattr(config, "EXECUTOR_TIME_MIN_S", 5.0)),
            duration_s * float(getattr(config, "EXECUTOR_TIME_MULTIPLIER", 2.0)),
        )
        rc.get_logger().info(
            f"[StagedPath] Sending follow trajectory points={len(follow_joint_trajectory.points)} "
            f"duration_s={duration_s:.3f}"
        )
        _send_trajectory_to_controller(rc, follow_joint_trajectory)
        follow_result = _wait_execution_complete(rc, timeout_s=timeout_s + 2.0)
        rc.get_logger().info(
            f"[TIMING] staged_path_follow_execute result={follow_result} "
            f"elapsed_s={perf_counter() - follow_exec_started:.3f}"
        )
        return follow_result
    except Exception as exc:
        rc.get_logger().error(f"[StagedPath] Failed: {exc}")
        return -1
    finally:
        setattr(rc, "_suppress_post_success_unwind", previous_execution_suppress)
        executor.shutdown(wait=False)
        rc.get_logger().info(f"[TIMING] staged_path_total elapsed_s={perf_counter() - started:.3f}")
