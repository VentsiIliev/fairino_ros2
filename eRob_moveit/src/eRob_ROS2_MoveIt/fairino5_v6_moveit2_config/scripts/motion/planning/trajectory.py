"""
Multi-Waypoint Cartesian Path
...
"""

from copy import deepcopy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetCartesianPath
from utils.transformation_utils import TransformationUtils
from .trajectory_planner import _cartesian_path_response
from .planner_diagnostics import _diagnose_fk_mismatch, _diagnose_start_collision
from .planner_utils import _set_result, _is_stale, TIME_PARAMETERIZATION
from .single_target import _execute_single_point
from ..execution.trajectory_optimization import apply_ipp_totg, apply_ruckig_service
from ..execution.trajectory_executor import _send_trajectory_to_controller
import numpy as np
import config


def send_path_cartesian(robot_controller, waypoints_mm, rx, ry, rz, vel_scaling, acc_scaling):
    robot_controller.safety_manager.force_update()
    if robot_controller.is_executing:
        result = robot_controller.motion_queue.submit(
            task_function=_execute_path,
            task_args=[robot_controller, waypoints_mm, rx, ry, rz, vel_scaling, acc_scaling]
        )
        if isinstance(result, tuple):
            task_id, position = result
            robot_controller.get_logger().info(f'[Queue] Motion queued at position {position} (task #{task_id})')
            return position
        else:
            robot_controller.get_logger().error(f'[Queue] Queue is full!')
            return result
    return _execute_path(robot_controller, waypoints_mm, rx, ry, rz, vel_scaling, acc_scaling)


def _execute_path(robot_controller, waypoints_mm, rx, ry, rz, vel_scaling, acc_scaling):
    robot_controller._last_requested_delta_mm = 0.0
    num_waypoints = len(waypoints_mm)
    robot_controller.get_logger().info(f'[EXECUTE_PATH] Received path with {num_waypoints} waypoints')
    robot_controller.get_logger().info(f'[EXECUTE_PATH] Transforming waypoints from work object to base frame')

    if not robot_controller.cart_path_client.wait_for_service(timeout_sec=1.0):
        robot_controller.get_logger().error('[EXECUTE_PATH] compute_cartesian_path service not available')
        return -2

    # Build ee_link poses + safety check every waypoint
    waypoints = []
    total_dist_mm = 0.0
    for i, wp in enumerate(waypoints_mm):
        tcp_pose = [wp[0], wp[1], wp[2], rx, ry, rz]
        T_tcp = TransformationUtils.pose_to_transform(tcp_pose)
        T_ee = TransformationUtils.remove_tcp_offset(T_tcp, robot_controller.T_tool)
        ee_pos = T_ee[:3, 3]
        ee_quat = TransformationUtils.matrix_to_quaternion(T_ee[:3, :3])

        is_safe, msg = robot_controller.safety_manager.check_position_safety(ee_pos[0], ee_pos[1], ee_pos[2])
        if not is_safe:
            robot_controller.get_logger().error(f'[SAFETY] Waypoint {i+1} rejected: {msg}')
            return -3
        if 'Warning' in msg and i == 0:
            robot_controller.get_logger().warning(f'[SAFETY] {msg}')

        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = ee_pos[0], ee_pos[1], ee_pos[2]
        pose.orientation.x, pose.orientation.y = ee_quat[0], ee_quat[1]
        pose.orientation.z, pose.orientation.w = ee_quat[2], ee_quat[3]
        waypoints.append(pose)

        if i > 0:
            prev = waypoints_mm[i - 1]
            total_dist_mm += np.linalg.norm(np.array(wp[:3]) - np.array(prev[:3]))

    avg_spacing_mm = total_dist_mm / max(num_waypoints - 1, 1)
    robot_controller.get_logger().info(f'[EXECUTE_PATH] {num_waypoints} waypoints, avg spacing: {avg_spacing_mm:.2f}mm')
    robot_controller.get_logger().info(f'[EXECUTE_PATH] First waypoint: {waypoints_mm[0][:3]}')

    # Step size based on avg spacing — MORE waypoints = path already dense = LARGER steps
    if avg_spacing_mm < 5:
        max_step = 0.003
    elif avg_spacing_mm < 15:
        max_step = 0.005
    else:
        max_step = 0.010

    # Check if robot is far from first waypoint
    current_cart = robot_controller.prev_cartesian
    if current_cart is not None and len(current_cart) >= 3:
        approach_dist = np.linalg.norm(np.array(waypoints_mm[0][:3]) - np.array(current_cart[:3]))
        robot_controller.get_logger().info(f'[EXECUTE_PATH] Distance to first waypoint: {approach_dist:.1f}mm')
        if approach_dist > config.PATH_APPROACH_THRESHOLD_MM:
            robot_controller.get_logger().info(
                f'[EXECUTE_PATH] Gap {approach_dist:.1f}mm > {config.PATH_APPROACH_THRESHOLD_MM}mm threshold — '
                f'planning first, then approaching')
            _plan_then_approach(robot_controller, waypoints, waypoints_mm[0], rx, ry, rz,
                                vel_scaling, acc_scaling, max_step)
            return 0

    # Normal flow: robot is close to first waypoint, plan from current state
    request = GetCartesianPath.Request()
    request.header.frame_id = config.BASE_LINK
    request.group_name = config.PLANNING_GROUP
    request.link_name = config.EE_LINK
    request.waypoints = waypoints
    request.max_step = max_step
    request.jump_threshold = 0.0
    request.avoid_collisions = True
    request.max_velocity_scaling_factor = vel_scaling
    request.max_acceleration_scaling_factor = acc_scaling

    if robot_controller.current_joint_state is not None:
        current_state = deepcopy(robot_controller.current_joint_state)
        current_state.header.stamp = robot_controller.get_clock().now().to_msg()
        request.start_state.joint_state = current_state
        request.start_state.is_diff = False
        robot_controller.get_logger().info(
            f'[Cartesian Path] Using current joint state as start: {[round(p, 3) for p in current_state.position[:6]]}')
    else:
        robot_controller.get_logger().warning('[Cartesian Path] No current joint state available')

    robot_controller.get_logger().info(f'[Cartesian Path] Using max_step={max_step*1000:.1f}mm')
    robot_controller.get_logger().info('[Cartesian Path] Requesting cartesian path computation...')

    with robot_controller.lock:
        robot_controller.is_executing = True
        robot_controller.plan_generation += 1
        gen = robot_controller.plan_generation
    future = robot_controller.cart_path_client.call_async(request)
    future.add_done_callback(lambda f: _cartesian_path_response(robot_controller, f, vel_scaling, acc_scaling, gen))
    return 0


def _plan_then_approach(robot_controller, waypoints, first_wp_mm, rx, ry, rz,
                        vel_scaling, acc_scaling, max_step):
    """
    Three-phase execution for when the robot is far from the path start:

    Phase 1 — IK query: compute_cartesian_path([wp[0]], max_step=0.1, no collision)
              → get joint state at wp[0] without moving
    Phase 2 — Plan:     compute_cartesian_path(all waypoints, start=IK@wp[0])
              → if fails: abort, robot stays put
              → if succeeds: store trajectory, queue _execute_pending_trajectory
    Phase 3 — Approach: _execute_single_point(current → wp[0])
              → on completion, motion queue fires _execute_pending_trajectory
    """

    # ── Phase 1: Quick IK for first waypoint ──────────────────────────────────
    ik_request = GetCartesianPath.Request()
    ik_request.header.frame_id = config.BASE_LINK
    ik_request.group_name = config.PLANNING_GROUP
    ik_request.link_name = config.EE_LINK
    ik_request.waypoints = [waypoints[0]]   # single waypoint IK
    ik_request.max_step = 0.1              # irrelevant for 1 point, just needs IK
    ik_request.jump_threshold = 0.0
    ik_request.avoid_collisions = False    # IK only, no collision check needed

    if robot_controller.current_joint_state is not None:
        ik_request.start_state.joint_state = deepcopy(robot_controller.current_joint_state)
        ik_request.start_state.is_diff = False

    with robot_controller.lock:
        robot_controller.is_executing = True
        robot_controller.plan_generation += 1
        gen = robot_controller.plan_generation

    robot_controller.get_logger().info('[EXECUTE_PATH] Phase 1: IK query for first waypoint...')
    ik_future = robot_controller.cart_path_client.call_async(ik_request)

    def on_ik_done(f):
        if _is_stale(robot_controller, gen):
            return
        try:
            ik_resp = f.result()
        except Exception as e:
            robot_controller.get_logger().error(f'[EXECUTE_PATH] IK service error: {e}')
            _set_result(robot_controller, -2)
            return

        traj = ik_resp.solution.joint_trajectory
        if ik_resp.fraction < 0.99 or not traj.points:
            robot_controller.get_logger().error(
                f'[EXECUTE_PATH] First waypoint unreachable (IK fraction={ik_resp.fraction:.2f}) — aborting, robot stays put')
            _set_result(robot_controller, -3)
            return

        ik_joints = traj.points[-1]
        robot_controller.get_logger().info(
            f'[EXECUTE_PATH] IK ok: {[round(p, 3) for p in ik_joints.positions[:6]]}')

        # Build virtual start state at first waypoint
        virtual_start = RobotState()
        virtual_start.joint_state.name = list(traj.joint_names)
        virtual_start.joint_state.position = list(ik_joints.positions)
        virtual_start.is_diff = False

        # ── Phase 2: Plan full path from virtual start ─────────────────────
        plan_request = GetCartesianPath.Request()
        plan_request.header.frame_id = config.BASE_LINK
        plan_request.group_name = config.PLANNING_GROUP
        plan_request.link_name = config.EE_LINK
        plan_request.waypoints = waypoints
        plan_request.max_step = max_step
        plan_request.jump_threshold = 0.0
        plan_request.avoid_collisions = True   # keep collision checking for path safety
        plan_request.start_state = virtual_start
        plan_request.max_velocity_scaling_factor = vel_scaling
        plan_request.max_acceleration_scaling_factor = acc_scaling

        robot_controller.get_logger().info(
            f'[EXECUTE_PATH] Phase 2: Planning {len(waypoints)}-waypoint path from wp[0] '
            f'(max_step={max_step*1000:.1f}mm)...')
        plan_future = robot_controller.cart_path_client.call_async(plan_request)
        plan_future.add_done_callback(on_plan_done)

    def on_plan_done(f):
        if _is_stale(robot_controller, gen):
            return
        try:
            resp = f.result()
        except Exception as e:
            robot_controller.get_logger().error(f'[EXECUTE_PATH] Plan service error: {e}')
            _set_result(robot_controller, -2)
            return

        fraction = resp.fraction
        robot_controller.get_logger().info(f'[EXECUTE_PATH] Path planned: {fraction*100:.1f}%')

        if fraction < config.CARTESIAN_MIN_FRACTION:
            robot_controller.get_logger().error(
                f'[EXECUTE_PATH] Planning failed ({fraction*100:.1f}%) — NOT moving to first waypoint')
            _diagnose_start_collision(robot_controller)
            _set_result(robot_controller, -3)
            return  # ← robot has not moved at all

        num_pts = len(resp.solution.joint_trajectory.points)
        robot_controller.get_logger().info(
            f'[EXECUTE_PATH] ✓ Plan succeeded ({num_pts} pts). '
            f'Queuing execution, approaching wp[0]: '
            f'[{first_wp_mm[0]:.1f}, {first_wp_mm[1]:.1f}, {first_wp_mm[2]:.1f}]')

        # Store trajectory for deferred execution
        robot_controller._pending_path_trajectory = resp.solution
        robot_controller._pending_path_vel_scaling = vel_scaling
        robot_controller._pending_path_acc_scaling = acc_scaling

        # Queue trajectory execution — fires after approach completes
        robot_controller.motion_queue.submit(
            task_function=_execute_pending_trajectory,
            task_args=[robot_controller]
        )

        # ── Phase 3: Approach first waypoint ───────────────────────────────
        current_cart = robot_controller.prev_cartesian
        if current_cart is None or len(current_cart) < 6:
            robot_controller.get_logger().error('[EXECUTE_PATH] Lost current position before approach — aborting')
            robot_controller.motion_queue.clear()
            _set_result(robot_controller, -4)
            return

        # Bypass send_cartesian_goal's is_executing check — we own is_executing here
        # _execute_single_point re-acquires the lock and increments plan_generation
        _execute_single_point(
            robot_controller,
            start_wp=list(current_cart[:6]),
            target_wp=[first_wp_mm[0], first_wp_mm[1], first_wp_mm[2], rx, ry, rz],
            vel_scaling=vel_scaling,
            acc_scaling=acc_scaling,
        )

    ik_future.add_done_callback(on_ik_done)


def _execute_pending_trajectory(robot_controller):
    """
    Execute the trajectory stored by _plan_then_approach.
    Called from the motion queue after the approach move completes.
    Aborts silently if the approach move failed.
    """
    if robot_controller.last_move_result != 0:
        robot_controller.get_logger().error(
            f'[EXECUTE_PATH] Approach failed (code={robot_controller.last_move_result}) — '
            f'discarding queued path execution')
        robot_controller._pending_path_trajectory = None
        return -1

    trajectory = getattr(robot_controller, '_pending_path_trajectory', None)
    vel_scaling = getattr(robot_controller, '_pending_path_vel_scaling', config.DEFAULT_VEL_SCALING)
    acc_scaling = getattr(robot_controller, '_pending_path_acc_scaling', config.DEFAULT_ACC_SCALING)

    if trajectory is None:
        robot_controller.get_logger().error('[EXECUTE_PATH] No pending trajectory!')
        return -1

    robot_controller._pending_path_trajectory = None  # consume

    num_pts = len(trajectory.joint_trajectory.points)
    robot_controller.get_logger().info(
        f'[EXECUTE_PATH] Phase 3: Executing pre-planned trajectory ({num_pts} pts)')

    with robot_controller.lock:
        robot_controller.is_executing = True
        robot_controller.plan_generation += 1
        gen = robot_controller.plan_generation

    def on_time_param_done(result_traj):
        if _is_stale(robot_controller, gen):
            robot_controller.get_logger().info('[EXECUTE_PATH] Stale TOTG response discarded')
            return
        if result_traj is None:
            robot_controller.get_logger().error('[EXECUTE_PATH] Time parameterization failed')
            _set_result(robot_controller, -7)
            return
        with robot_controller.lock:
            robot_controller.last_move_result = 0
        _send_trajectory_to_controller(robot_controller, result_traj.joint_trajectory)

    if TIME_PARAMETERIZATION == "RUCKIG":
        apply_ruckig_service(robot_controller, trajectory, vel_scaling, acc_scaling, callback=on_time_param_done)
    else:
        apply_ipp_totg(robot_controller, trajectory, vel_scaling, acc_scaling, callback=on_time_param_done)
    return 0
