"""
Fast native joint-space PTP planner.

Endpoint IK selection, FK validation and collision checking are performed
inside the C++ ptp_helper using MoveIt RobotState and PlanningScene directly.
"""

from __future__ import annotations

import time

import config

from moveit_msgs.msg import RobotTrajectory
from sensor_msgs.msg import JointState

from .planner_utils import (
    _begin_execution,
    _set_result,
    _to_pose_list,
)
from .trajectory_planner import _apply_time_param


def _wait_future(
    future,
    timeout_s: float,
    description: str,
):
    deadline = time.monotonic() + float(timeout_s)

    while time.monotonic() < deadline:
        if future.done():
            return future.result()

        # 1 ms instead of the previous 10 ms polling.
        time.sleep(0.001)

    raise TimeoutError(
        f"{description} timed out after "
        f"{timeout_s:.3f}s"
    )


def _get_live_joint_state(robot_controller):
    joint_state = getattr(
        robot_controller,
        "current_joint_state",
        None,
    )

    if joint_state is None:
        return None

    names = list(
        getattr(
            joint_state,
            "name",
            [],
        )
        or []
    )

    positions = list(
        getattr(
            joint_state,
            "position",
            [],
        )
        or []
    )

    if not names:
        return None

    if len(names) != len(positions):
        return None

    result = JointState()

    result.header.stamp = (
        robot_controller
        .get_clock()
        .now()
        .to_msg()
    )

    result.name = names
    result.position = positions

    return result


def _request_native_ptp(
    robot_controller,
    target_pose,
    start_state,
):
    from erob_moveit_runtime.srv import ComputePtp

    client = robot_controller.get_ptp_client()

    if (
        client is None
        or not client.wait_for_service(
            timeout_sec=1.0
        )
    ):
        raise TimeoutError(
            "native PTP service unavailable"
        )

    request = ComputePtp.Request()

    request.target_pose = target_pose
    request.start_state = start_state

    request.group_name = str(
        config.PLANNING_GROUP
    )

    request.link_name = str(
        config.EE_LINK
    )

    #
    # Keep each IK attempt short.
    #
    # The C++ helper tries several seeds.
    #
    request.ik_timeout_s = float(
        getattr(
            config,
            "PTP_IK_TIMEOUT_S",
            0.01,
        )
    )

    request.ik_attempts = int(
        getattr(
            config,
            "PTP_IK_ATTEMPTS",
            9,
        )
    )

    request.interpolation_step_rad = float(
        getattr(
            config,
            "PTP_JOINT_INTERPOLATION_STEP_RAD",
            0.08,
        )
    )

    request.min_interpolation_segments = int(
        getattr(
            config,
            "PTP_MIN_INTERPOLATION_SEGMENTS",
            8,
        )
    )

    request.max_interpolation_segments = int(
        getattr(
            config,
            "PTP_MAX_INTERPOLATION_SEGMENTS",
            80,
        )
    )

    request.orientation_lock_tolerance_deg = float(
        getattr(
            config,
            "PTP_LOCK_ORIENTATION_TOL_DEG",
            2.0,
        )
    )

    request.locked_path_max_deviation_deg = float(
        getattr(
            config,
            "PTP_LOCKED_PATH_MAX_DRIFT_DEG",
            2.0,
        )
    )

    request.oriented_path_max_deviation_deg = float(
        getattr(
            config,
            "PTP_ORIENTED_PATH_MAX_DEVIATION_DEG",
            5.0,
        )
    )

    #
    # J6 deliberately has very low cost.
    #
    # This means rotating J6 is preferable to selecting
    # a bad elbow/wrist IK family.
    #
    request.joint_weights = [
        float(v)
        for v in getattr(
            config,
            "PTP_JOINT_WEIGHTS",
            [
                1.0,   # J1
                1.2,   # J2
                2.5,   # J3 - elbow
                1.5,   # J4
                4.0,   # J5 - wrist branch
                0.10,  # J6 - cheap
            ],
        )
    ]

    request.avoid_collisions = bool(
        getattr(
            config,
            "PTP_AVOID_COLLISIONS",
            True,
        )
    )

    future = client.call_async(
        request
    )

    timeout_s = (
        max(
            0.1,
            request.ik_timeout_s
            * request.ik_attempts,
        )
        + 1.0
    )

    return _wait_future(
        future,
        timeout_s=timeout_s,
        description="native PTP planning",
    )


def plan_ptp_trajectory(
    robot_controller,
    target_tcp_pose,
    start_joint_state,
    *,
    tool_transform=None,
):
    """
    Plan a native joint-space PTP trajectory from an explicitly supplied
    joint state.

    Used by:
    - normal move_ptp, with the live joint state
    - ordered motion chains, with the predicted end state of the previous segment

    Returns:
        ComputePtp.Response
    """
    if start_joint_state is None:
        raise RuntimeError(
            "PTP start joint state unavailable"
        )

    T_tool = (
        tool_transform
        if tool_transform is not None
        else robot_controller.T_tool
    )

    target_poses, err = _to_pose_list(
        robot_controller,
        [list(target_tcp_pose[:6])],
        T_tool,
        check_last_only=True,
    )

    if err:
        raise RuntimeError(
            f"PTP target pose conversion failed with result {err}"
        )

    target_pose = target_poses[0]

    return _request_native_ptp(
        robot_controller,
        target_pose,
        start_joint_state,
    )

def send_ptp_goal(
    robot_controller,
    x_mm,
    y_mm,
    z_mm,
    rx,
    ry,
    rz,
    vel_scale,
    acc_scale,
    tool_transform=None,
    trajectory_optimizer_name=None,
):
    robot_controller.force_safety_update()

    start_state = _get_live_joint_state(
        robot_controller
    )

    if start_state is None:
        robot_controller.get_logger().error(
            "[PTP] Current joint state unavailable"
        )
        return -4

    target_tcp_pose = [
        x_mm,
        y_mm,
        z_mm,
        rx,
        ry,
        rz,
    ]

    try:
        response = plan_ptp_trajectory(
            robot_controller,
            target_tcp_pose,
            start_state,
            tool_transform=tool_transform,
        )

    except TimeoutError as exc:
        robot_controller.get_logger().error(
            f"[PTP] {exc}"
        )

        return -2

    except Exception as exc:
        robot_controller.get_logger().error(
            "[PTP] Native planning failure: "
            f"{exc}"
        )

        return -1

    if not bool(response.success):
        robot_controller.get_logger().error(
            "[PTP] Planning rejected: "
            f"{response.message}"
        )

        return int(
            response.error_code
            or -11
        )

    robot_controller.get_logger().info(
        "[PTP] Native plan: "
        f"{response.message} | "
        f"total={response.total_time_ms:.2f}ms "
        f"IK={response.ik_time_ms:.2f}ms "
        f"validation={response.validation_time_ms:.2f}ms "
        f"attempts={response.ik_attempts_made} "
        f"solutions={response.ik_solutions_found} "
        f"validated={response.candidates_validated}"
    )

    if bool(response.noop):
        _set_result(
            robot_controller,
            0,
        )

        return 0

    joint_trajectory = response.trajectory

    if not joint_trajectory.points:
        robot_controller.get_logger().error(
            "[PTP] Native planner returned "
            "empty trajectory"
        )

        return -11

    moveit_trajectory = RobotTrajectory()

    moveit_trajectory.joint_trajectory = (
        joint_trajectory
    )

    generation = _begin_execution(
        robot_controller
    )

    _apply_time_param(
        robot_controller,
        moveit_trajectory,
        vel_scale,
        acc_scale,
        generation,
        log_prefix="[PTP]",
        trajectory_optimizer_name=(
            trajectory_optimizer_name
        ),
    )

    return 0