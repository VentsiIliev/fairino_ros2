#!/usr/bin/env python3
from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
from dataclasses import dataclass
from threading import Event
import time
from time import perf_counter

from moveit_msgs.msg import RobotState

import config
from motion.execution.trajectory_executor import _send_trajectory_to_controller
from motion.execution.trajectory_optimizer import resolve_trajectory_optimizer
from motion.planning.planner_utils import _to_pose_list
from motion.planning.single_target import _compute_max_step, _wrapped_angle_delta_deg
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
    rc.get_logger().error(f"[CustomSeq] Timed out waiting for segment execution after {timeout_s:.1f}s")
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
        f"[CustomSeq] Planning segment {index + 1}: label='{label}' "
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
        f"[TIMING] custom_sequence_plan segment={index + 1} label='{label}' "
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


def execute_custom_sequence(rc, segments, tool_transform=None) -> int:
    if not segments:
        rc.get_logger().error("[CustomSeq] Empty sequence")
        return -1
    if getattr(rc, "current_joint_state", None) is None:
        rc.get_logger().error("[CustomSeq] No current joint state available")
        return -4
    if getattr(rc, "prev_cartesian", None) is None or len(rc.prev_cartesian) < 6:
        rc.get_logger().error("[CustomSeq] No current Cartesian position available")
        return -4
    if not rc.is_motion_stack_ready():
        rc.get_logger().error(f"[CustomSeq] Motion stack not ready: {rc.get_motion_stack_fault_reason()}")
        return config.MOTION_ERROR_HARDWARE_NOT_READY

    started = perf_counter()
    T_tool = tool_transform if tool_transform is not None else rc.T_tool
    start_cartesian = list(rc.prev_cartesian[:6])
    start_state = RobotState()
    clean_joint_state = deepcopy(rc.current_joint_state)
    clean_joint_state.header.stamp = rc.get_clock().now().to_msg()
    clean_joint_state.velocity = [0.0] * (len(clean_joint_state.name) or len(clean_joint_state.position))
    clean_joint_state.effort = []
    start_state.joint_state = clean_joint_state
    start_state.is_diff = False

    rc.get_logger().info(f"[CustomSeq] Executing custom motion sequence with {len(segments)} segments")
    executor = ThreadPoolExecutor(max_workers=1)
    next_future = None
    planned = None
    previous_target = start_cartesian
    previous_state = start_state
    previous_execution_suppress = bool(getattr(rc, "_suppress_post_success_unwind", False))

    try:
        for index, segment in enumerate(segments):
            if planned is None:
                planned = _plan_segment(
                    rc,
                    index=index,
                    segment=segment,
                    start_cartesian=previous_target,
                    start_state=previous_state,
                    tool_transform=T_tool,
                )

            if index + 1 < len(segments):
                next_segment = segments[index + 1]
                next_start_cartesian = planned.target_position
                next_start_state = planned.final_state
                next_future = executor.submit(
                    _plan_segment,
                    rc,
                    index=index + 1,
                    segment=next_segment,
                    start_cartesian=next_start_cartesian,
                    start_state=next_start_state,
                    tool_transform=T_tool,
                )
            else:
                next_future = None

            setattr(rc, "_suppress_post_success_unwind", index + 1 < len(segments))
            exec_started = perf_counter()
            duration = planned.joint_trajectory.points[-1].time_from_start
            duration_s = float(duration.sec) + float(duration.nanosec) / 1e9
            timeout_s = max(float(getattr(config, "EXECUTOR_TIME_MIN_S", 5.0)), duration_s * float(getattr(config, "EXECUTOR_TIME_MULTIPLIER", 2.0)))
            rc.get_logger().info(
                f"[CustomSeq] Sending segment {index + 1}/{len(segments)} label='{planned.label}' "
                f"points={len(planned.joint_trajectory.points)} duration_s={duration_s:.3f} "
                f"plan_s={planned.plan_elapsed_s:.3f} optimize_s={planned.optimize_elapsed_s:.3f}"
            )
            _send_trajectory_to_controller(rc, planned.joint_trajectory)
            result = _wait_execution_complete(rc, timeout_s=timeout_s + 2.0)
            exec_elapsed = perf_counter() - exec_started
            rc.get_logger().info(
                f"[TIMING] custom_sequence_execute segment={index + 1} label='{planned.label}' "
                f"result={result} elapsed_s={exec_elapsed:.3f}"
            )
            if result != 0:
                return result

            previous_target = planned.target_position
            previous_state = planned.final_state
            if next_future is not None:
                wait_started = perf_counter()
                planned = next_future.result(timeout=float(getattr(config, "CUSTOM_SEQUENCE_PLAN_TIMEOUT_S", 10.0)) + 5.0)
                rc.get_logger().info(
                    f"[TIMING] custom_sequence_next_ready segment={index + 2} "
                    f"wait_after_previous_s={perf_counter() - wait_started:.3f}"
                )
            else:
                planned = None
        return 0
    except Exception as exc:
        rc.get_logger().error(f"[CustomSeq] Failed: {exc}")
        return -1
    finally:
        setattr(rc, "_suppress_post_success_unwind", previous_execution_suppress)
        executor.shutdown(wait=False)
        rc.get_logger().info(f"[TIMING] custom_sequence_total elapsed_s={perf_counter() - started:.3f}")
