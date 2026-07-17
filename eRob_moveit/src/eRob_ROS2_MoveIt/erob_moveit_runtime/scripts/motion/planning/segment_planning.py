#!/usr/bin/env python3
from __future__ import annotations

from dataclasses import dataclass
from threading import Event
import time
from time import perf_counter

from moveit_msgs.msg import RobotState

import config
from motion.execution.trajectory_optimizer import resolve_trajectory_optimizer
from motion.planning.direct_contour_ik import (
    _build_direct_contour_trajectory,
    _direct_ik_should_run,
    _log_report,
)
from motion.planning.planner_utils import _to_pose_list
from motion.planning.single_target import _compute_max_step, _wrapped_angle_delta_deg
from motion.planning.trajectory import _path_length_mm, _simplify_cartesian_waypoints
from motion.planning.trajectory_planner import _build_cartesian_request


@dataclass
class _PlannedSegment:
    index: int
    label: str
    target_position: list[float]
    joint_trajectory: object
    final_state: RobotState
    plan_elapsed_s: float
    optimize_elapsed_s: float


def _wait_future(future, timeout_s: float):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if future.done():
            return future.result()
        time.sleep(0.005)
    raise TimeoutError(f"Timed out after {timeout_s:.1f}s waiting for MoveIt response")


def _wait_execution_complete(rc, timeout_s: float) -> int:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if not bool(getattr(rc, "is_executing", False)):
            return int(getattr(rc, "last_move_result", 0))
        time.sleep(0.01)
    rc.get_logger().error(f"[SegmentPlan] Timed out waiting for segment execution after {timeout_s:.1f}s")
    return -1


def _robot_state_from_trajectory_end(joint_trajectory) -> RobotState:
    state = RobotState()
    state.joint_state.name = list(joint_trajectory.joint_names)
    last_point = joint_trajectory.points[-1]
    state.joint_state.position = list(last_point.positions)
    state.joint_state.velocity = [0.0] * len(state.joint_state.name)
    state.joint_state.effort = []
    state.is_diff = False
    return state


def _optimize_sync(rc, trajectory, vel_scaling: float, acc_scaling: float, optimizer_name=None):
    done = Event()
    result_holder = {"trajectory": None}
    optimizer = resolve_trajectory_optimizer(optimizer_name, node=rc, default_optimizer=rc.trajectory_optimizer)
    started = perf_counter()

    def on_done(result):
        result_holder["trajectory"] = result
        done.set()

    optimizer.optimize(rc, trajectory, vel_scaling, acc_scaling, on_done)
    if not done.wait(timeout=float(getattr(config, "CUSTOM_SEQUENCE_OPTIMIZE_TIMEOUT_S", 10.0))):
        raise TimeoutError("Timed out waiting for trajectory optimizer")
    optimized = result_holder["trajectory"]
    if optimized is None:
        raise RuntimeError("Trajectory optimizer failed")
    return optimized, perf_counter() - started


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
        raise RuntimeError(f"follow path pose conversion failed with result {err}")

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
                f"[TIMING] follow_path_plan method=direct_contour_ik "
                f"waypoints={len(waypoints_6d)} optimize_s={optimize_elapsed:.3f} "
                f"elapsed_s={perf_counter() - started:.3f}"
            )
            return optimized.joint_trajectory
        rc.get_logger().warning(
            f"[FollowPath] Direct contour IK rejected follow path; falling back to Cartesian path: "
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
        raise RuntimeError(f"follow path Cartesian planning failed: fraction={fraction:.4f} points={point_count}")

    optimized, optimize_elapsed = _optimize_sync(
        rc,
        trajectory,
        vel_scaling,
        acc_scaling,
        optimizer_name=trajectory_optimizer_name,
    )
    rc.get_logger().info(
        f"[TIMING] follow_path_plan method=cartesian_path waypoints={len(waypoints_6d)} "
        f"fraction={fraction:.4f} points={point_count} optimize_s={optimize_elapsed:.3f} "
        f"elapsed_s={perf_counter() - started:.3f}"
    )
    return optimized.joint_trajectory


def _plan_segment(
    rc,
    *,
    index: int,
    segment: dict,
    start_cartesian: list[float],
    start_state: RobotState | None,
    tool_transform,
) -> _PlannedSegment:
    started = perf_counter()
    target = list(segment["position"])
    label = str(segment.get("label") or f"segment_{index + 1}")
    vel_scaling = max(0.0, min(1.0, float(segment["vel"]) / 100.0))
    acc_scaling = max(0.0, min(1.0, float(segment["acc"]) / 100.0))

    poses, err = _to_pose_list(rc, [start_cartesian, target], tool_transform)
    if err:
        raise RuntimeError(f"pose conversion failed with result {err}")

    p0, p1 = poses[0].position, poses[-1].position
    delta_m = ((p1.x - p0.x) ** 2 + (p1.y - p0.y) ** 2 + (p1.z - p0.z) ** 2) ** 0.5
    orientation_delta_deg = max(
        _wrapped_angle_delta_deg(start_cartesian[3], target[3]),
        _wrapped_angle_delta_deg(start_cartesian[4], target[4]),
        _wrapped_angle_delta_deg(start_cartesian[5], target[5]),
    )
    max_step = _compute_max_step(delta_m, orientation_delta_deg)
    request = _build_cartesian_request(
        rc,
        poses,
        max_step,
        vel_scaling,
        acc_scaling,
        start_state=start_state,
        avoid_collisions=config.resolve_avoid_collisions(None),
    )

    rc.get_logger().info(
        f"[SegmentPlan] Planning segment {index + 1}: label='{label}' "
        f"vel={float(segment['vel']):.1f}% acc={float(segment['acc']):.1f}% "
        f"delta_mm={delta_m * 1000.0:.3f} start_seed={'planned' if start_state is not None else 'live'}"
    )
    future = rc.request_cartesian_path(request)
    response = _wait_future(
        future,
        timeout_s=float(getattr(config, "CUSTOM_SEQUENCE_PLAN_TIMEOUT_S", 10.0)),
    )
    fraction = float(getattr(response, "fraction", 0.0))
    trajectory = getattr(response, "solution", None)
    joint_trajectory = getattr(trajectory, "joint_trajectory", None) if trajectory is not None else None
    point_count = len(getattr(joint_trajectory, "points", []) or [])
    plan_elapsed = perf_counter() - started
    rc.get_logger().info(
        f"[TIMING] segment_plan segment={index + 1} label='{label}' "
        f"fraction={fraction:.4f} points={point_count} elapsed_s={plan_elapsed:.3f}"
    )
    if fraction < config.CARTESIAN_MIN_FRACTION or point_count <= 1:
        raise RuntimeError(f"cartesian planning failed: fraction={fraction:.4f} points={point_count}")

    optimized, optimize_elapsed = _optimize_sync(rc, trajectory, vel_scaling, acc_scaling)
    optimized_joint_trajectory = optimized.joint_trajectory
    return _PlannedSegment(
        index=index,
        label=label,
        target_position=target,
        joint_trajectory=optimized_joint_trajectory,
        final_state=_robot_state_from_trajectory_end(optimized_joint_trajectory),
        plan_elapsed_s=plan_elapsed,
        optimize_elapsed_s=optimize_elapsed,
    )
