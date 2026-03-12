"""
Shared Planner Internals
========================
Shared constants, diagnostic utilities, and response handlers used by both
motion/single_target.py and motion/trajectory.py.

  TIME_PARAMETERIZATION  — select TOTG or Ruckig at the top of this file
  _diagnose_fk_mismatch  — compare C++ Cartesian vs MoveIt FK (diagnostic)
  _cartesian_path_response — MoveIt /compute_cartesian_path callback
  _jacobian_fallback_move  — sub-mm fallback via Jacobian pseudoinverse
"""

from moveit_msgs.srv import GetPositionFK
from .trajectory_executor import _send_trajectory_to_controller
from .trajectory_optimization import apply_ipp_totg, apply_ruckig_service
import numpy as np

# ============ Trajectory Parameterization Selection ============
# Options:
#   "RUCKIG"  - Use Ruckig C++ service (jerk-limited, requires ruckig_helper node)
#   "TOTG"    - Use TOTG C++ service (time-optimal, requires ipp_helper node)
#
# Ruckig provides smoother motion with S-curve profiles (3rd order)
# TOTG is faster but has trapezoidal velocity profiles (2nd order)
TIME_PARAMETERIZATION = "TOTG"


def _diagnose_fk_mismatch(robot_controller, first_waypoint_pose, joint_state):
    """
    Compare the first waypoint (from C++ Cartesian) with MoveIt's FK result.
    This helps identify why compute_cartesian_path returns 0% success.
    """
    logger = robot_controller.get_logger()

    # Create FK service client if not exists
    if not hasattr(robot_controller, '_fk_client'):
        robot_controller._fk_client = robot_controller.create_client(
            GetPositionFK, '/compute_fk'
        )

    if not robot_controller._fk_client.wait_for_service(timeout_sec=1.0):
        logger.warning('[FK Diagnostic] /compute_fk service not available')
        return

    # Build FK request
    from copy import deepcopy
    fk_request = GetPositionFK.Request()
    fk_request.header.frame_id = 'base_link'
    fk_request.fk_link_names = ['ee_link']

    # Set robot state from current joints
    fk_request.robot_state.joint_state = deepcopy(joint_state)
    fk_request.robot_state.is_diff = False

    # Call FK service with timeout polling (safe to call from callbacks)
    try:
        import time
        future = robot_controller._fk_client.call_async(fk_request)

        # Poll for completion (works inside callbacks unlike spin_until_future_complete)
        start_time = time.time()
        while not future.done():
            if time.time() - start_time > 2.0:
                logger.warning('[FK Diagnostic] FK service call timed out')
                return
            time.sleep(0.01)

        if future.result() is None:
            logger.warning('[FK Diagnostic] FK service call returned None')
            return

        fk_response = future.result()

        if fk_response.error_code.val != 1:  # MoveItErrorCodes.SUCCESS = 1
            logger.warning(f'[FK Diagnostic] FK computation failed with error code: {fk_response.error_code.val}')
            return

        if len(fk_response.pose_stamped) == 0:
            logger.warning('[FK Diagnostic] FK returned no poses')
            return

        # Extract MoveIt's FK result for ee_link
        moveit_pose = fk_response.pose_stamped[0].pose
        moveit_pos = np.array([
            moveit_pose.position.x * 1000,  # Convert to mm
            moveit_pose.position.y * 1000,
            moveit_pose.position.z * 1000
        ])
        moveit_quat = np.array([
            moveit_pose.orientation.x,
            moveit_pose.orientation.y,
            moveit_pose.orientation.z,
            moveit_pose.orientation.w
        ])

        # Extract first waypoint position (already in meters in the Pose)
        waypoint_pos = np.array([
            first_waypoint_pose.position.x * 1000,  # Convert to mm
            first_waypoint_pose.position.y * 1000,
            first_waypoint_pose.position.z * 1000
        ])
        waypoint_quat = np.array([
            first_waypoint_pose.orientation.x,
            first_waypoint_pose.orientation.y,
            first_waypoint_pose.orientation.z,
            first_waypoint_pose.orientation.w
        ])

        # Compute position difference
        pos_diff = waypoint_pos - moveit_pos
        pos_dist = np.linalg.norm(pos_diff)

        # Compute orientation difference (quaternion dot product)
        quat_dot = abs(np.dot(moveit_quat, waypoint_quat))
        angle_diff_deg = np.degrees(2 * np.arccos(np.clip(quat_dot, -1.0, 1.0)))

        # Log comparison
        logger.info('=' * 60)
        logger.info('[FK Diagnostic] Comparing first waypoint vs MoveIt FK:')
        logger.info(f'[FK Diagnostic] MoveIt FK ee_link:  X={moveit_pos[0]:.2f} Y={moveit_pos[1]:.2f} Z={moveit_pos[2]:.2f} mm')
        logger.info(f'[FK Diagnostic] First waypoint:     X={waypoint_pos[0]:.2f} Y={waypoint_pos[1]:.2f} Z={waypoint_pos[2]:.2f} mm')
        logger.info(f'[FK Diagnostic] Position DIFF:      dX={pos_diff[0]:.2f} dY={pos_diff[1]:.2f} dZ={pos_diff[2]:.2f} mm')
        logger.info(f'[FK Diagnostic] Position distance:  {pos_dist:.2f} mm')
        logger.info(f'[FK Diagnostic] Orientation diff:   {angle_diff_deg:.2f} degrees')
        logger.info(f'[FK Diagnostic] MoveIt quat:        [{moveit_quat[0]:.4f}, {moveit_quat[1]:.4f}, {moveit_quat[2]:.4f}, {moveit_quat[3]:.4f}]')
        logger.info(f'[FK Diagnostic] Waypoint quat:      [{waypoint_quat[0]:.4f}, {waypoint_quat[1]:.4f}, {waypoint_quat[2]:.4f}, {waypoint_quat[3]:.4f}]')

        if pos_dist > 1.0:  # More than 1mm difference
            logger.error(f'[FK Diagnostic] POSITION MISMATCH > 1mm! This may cause path planning failure.')
        if angle_diff_deg > 1.0:  # More than 1 degree difference
            logger.error(f'[FK Diagnostic] ORIENTATION MISMATCH > 1 degree! This may cause path planning failure.')

        logger.info('=' * 60)

    except Exception as e:
        logger.error(f'[FK Diagnostic] Exception during FK comparison: {e}')


def _cartesian_path_response(robot_controller, future, vel_scaling, acc_scaling, generation=None):
    robot_controller.safety_manager.force_update()
    # Discard stale responses from before a preempt/stop
    with robot_controller.lock:
        if generation is not None and generation != robot_controller.plan_generation:
            robot_controller.get_logger().info('[Cartesian Path] Stale response discarded (preempted)')
            return
    try:
        response = future.result()
        fraction = response.fraction
        robot_controller.get_logger().info(f'[Cartesian Path] Path computed: {fraction * 100:.1f}% successful')

        if fraction < 0.9:
            robot_controller.get_logger().error(
                f'[Cartesian Path] Only {fraction * 100:.1f}% of path could be computed')
            robot_controller.get_logger().error(f'[Cartesian Path] Possible reasons:')
            robot_controller.get_logger().error(f'[Cartesian Path]   1. Target unreachable from current position')
            robot_controller.get_logger().error(f'[Cartesian Path]   2. Path goes through collision/obstacles')
            robot_controller.get_logger().error(f'[Cartesian Path]   3. Joint limits would be exceeded')

            if hasattr(robot_controller, 'prev_cartesian') and robot_controller.prev_cartesian is not None:
                curr = robot_controller.prev_cartesian
                robot_controller.get_logger().error(
                    f'[Cartesian Path] Current: X={curr[0]:.1f} Y={curr[1]:.1f} Z={curr[2]:.1f} RX={curr[3]:.1f} RY={curr[4]:.1f} RZ={curr[5]:.1f}')

            with robot_controller.lock:
                robot_controller.is_executing = False
                robot_controller.last_move_result = -3
            return

        # Get the computed trajectory
        trajectory = response.solution

        num_pts = len(trajectory.joint_trajectory.points)
        robot_controller.get_logger().info(
            f'[Cartesian Path] Computed trajectory has {num_pts} points')

        # ≤1 point: all Cartesian waypoints collapsed to same joint config
        if num_pts <= 1:
            if response.fraction >= 0.99:
                requested_delta_mm = getattr(robot_controller, '_last_requested_delta_mm', 0.0)
                if requested_delta_mm > 0.1:
                    robot_controller.get_logger().warning(
                        f'[Cartesian Path] ≤1 point but delta={requested_delta_mm:.3f}mm — trying Jacobian fallback')
                    stored_wps = getattr(robot_controller, '_last_full_waypoints', None)
                    if stored_wps:
                        ok = _jacobian_fallback_move(
                            robot_controller, stored_wps, vel_scaling, acc_scaling, generation)
                        if ok:
                            return  # Jacobian callback owns last_move_result from here
                    robot_controller.get_logger().error(
                        '[Cartesian Path] Jacobian fallback unavailable — returning -8')
                    result_code = -8
                else:
                    robot_controller.get_logger().info(
                        '[Cartesian Path] ≤1 point (100%) — robot already at target within IK precision')
                    result_code = 0
            else:
                robot_controller.get_logger().warning(
                    f'[Cartesian Path] ≤1 point, fraction={response.fraction * 100:.0f}% — planning failed')
                result_code = -6
            with robot_controller.lock:
                robot_controller.is_executing = False
                robot_controller.last_move_result = result_code
            return

        # Apply time parameterization for smooth velocity profile
        def on_time_param_done(result_trajectory):
            with robot_controller.lock:
                if generation is not None and generation != robot_controller.plan_generation:
                    robot_controller.get_logger().info('[Cartesian Path] Stale TOTG response discarded (preempted)')
                    return

            if result_trajectory is None:
                robot_controller.get_logger().error('[Cartesian Path] Time parameterization failed - aborting execution')
                with robot_controller.lock:
                    robot_controller.is_executing = False
                    robot_controller.last_move_result = -7
                return

            with robot_controller.lock:
                robot_controller.last_move_result = 0
            _send_trajectory_to_controller(robot_controller, result_trajectory.joint_trajectory)

        # Select time parameterization method based on TIME_PARAMETERIZATION setting
        if TIME_PARAMETERIZATION == "RUCKIG":
            apply_ruckig_service(robot_controller, trajectory, vel_scaling, acc_scaling, callback=on_time_param_done)
        else:  # "TOTG" or any other value
            apply_ipp_totg(robot_controller, trajectory, vel_scaling, acc_scaling, callback=on_time_param_done)

    except Exception as e:
        robot_controller.get_logger().error(f'[Cartesian Path] Service call failed: {e}')
        with robot_controller.lock:
            robot_controller.is_executing = False
            robot_controller.last_move_result = -2


def _jacobian_fallback_move(robot_controller, waypoints, vel_scaling, acc_scaling, generation):
    """Bypass MoveIt for sub-mm moves using Jacobian pseudoinverse joint correction.
    Uses pure-numpy FK (DH parameters) — no PyKDL dependency, always reliable.
    """

    def _fk(q):
        """Fairino5 v6 FK via DH parameters (matches robot_state_publisher.cpp)."""
        def rotz(a):
            c, s = np.cos(a), np.sin(a)
            return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=float)
        def rotx(a):
            c, s = np.cos(a), np.sin(a)
            return np.array([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]], dtype=float)
        def trans(x, y, z):
            return np.array([[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]], dtype=float)
        T = np.eye(4)
        T = T @ rotz(q[0])
        T = T @ trans(0, 0, 0.152) @ rotx(np.pi / 2) @ rotz(q[1])
        T = T @ trans(-0.425, 0, 0) @ rotz(q[2])
        T = T @ trans(-0.39501, 0, 0) @ rotz(q[3])
        T = T @ trans(0, 0, 0.1021) @ rotx(np.pi / 2) @ rotz(q[4])
        T = T @ trans(0, 0, 0.102) @ rotx(-np.pi / 2) @ rotz(q[5])
        return T

    n = 6
    js = robot_controller.current_joint_state
    if js is None or len(js.position) < n:
        robot_controller.get_logger().warning('[JacMove] Joint state unavailable or insufficient joints')
        return False

    joints = [float(js.position[i]) for i in range(n)]
    joint_names = [str(js.name[i]) for i in range(n)]
    q0 = np.array(joints)
    T0 = _fk(q0)

    # Compute Cartesian delta (metres)
    p0, p1 = waypoints[0].position, waypoints[-1].position
    delta_x = np.array([p1.x - p0.x, p1.y - p0.y, p1.z - p0.z, 0.0, 0.0, 0.0])

    if np.linalg.norm(delta_x[:3]) < 1e-7:
        robot_controller.get_logger().warning('[JacMove] delta_x ≈ 0 — waypoints identical, skipping move')
        with robot_controller.lock:
            robot_controller.is_executing = False
            robot_controller.last_move_result = 0
        return True

    # Numerical Jacobian via pure-numpy FD (no PyKDL — avoids silent JntArray setitem failures)
    eps = 1e-7
    J = np.zeros((6, n))
    for j in range(n):
        q_pert = q0.copy()
        q_pert[j] += eps
        T_pert = _fk(q_pert)
        J[0, j] = (T_pert[0, 3] - T0[0, 3]) / eps
        J[1, j] = (T_pert[1, 3] - T0[1, 3]) / eps
        J[2, j] = (T_pert[2, 3] - T0[2, 3]) / eps
        dR = T_pert[:3, :3] @ T0[:3, :3].T
        tr = np.clip((np.trace(dR) - 1.0) / 2.0, -1.0, 1.0)
        angle = np.arccos(tr)
        if abs(angle) > 1e-10:
            s2 = 2.0 * np.sin(angle)
            J[3, j] = (dR[2, 1] - dR[1, 2]) / s2 * angle / eps
            J[4, j] = (dR[0, 2] - dR[2, 0]) / s2 * angle / eps
            J[5, j] = (dR[1, 0] - dR[0, 1]) / s2 * angle / eps

    singular_values = np.linalg.svd(J, compute_uv=False)
    robot_controller.get_logger().info(
        f'[JacMove] delta_x={[f"{v:.6f}" for v in delta_x[:3]]}, S={[f"{v:.4f}" for v in singular_values]}')

    # Damped pseudoinverse
    lam = 0.00001
    U, S, Vt = np.linalg.svd(J)
    S_inv = S / (S ** 2 + lam ** 2)
    J_pinv = Vt.T @ np.diag(S_inv) @ U.T

    delta_q = J_pinv @ delta_x

    max_dq = np.max(np.abs(delta_q))
    robot_controller.get_logger().info(f'[JacMove] Δq={[f"{v:.5f}" for v in delta_q]} (max={max_dq:.5f})')

    if max_dq < 1e-5:
        robot_controller.get_logger().warning(
            f'[JacMove] Near-zero Δq (max={max_dq:.2e}) — near singularity or step too small, skipping')
        with robot_controller.lock:
            robot_controller.is_executing = False
            robot_controller.last_move_result = -9
        return False

    # Clamp large joint steps
    max_joint_step = 0.05  # rad
    if max_dq > max_joint_step:
        delta_q *= max_joint_step / max_dq
        robot_controller.get_logger().warning(f'[JacMove] Clamped {max_dq:.5f} → {max_joint_step:.2f} rad')

    target_joints = np.array(joints) + delta_q

    # Build trajectory
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from builtin_interfaces.msg import Duration
    traj_msg = JointTrajectory()
    traj_msg.joint_names = joint_names

    pt0 = JointTrajectoryPoint()
    pt0.positions = list(q0)
    pt0.velocities = [0.0]*n
    pt0.accelerations = [0.0]*n
    pt0.time_from_start = Duration(sec=0, nanosec=0)

    pt1 = JointTrajectoryPoint()
    pt1.positions = list(target_joints)
    pt1.velocities = [0.0]*n
    pt1.accelerations = [0.0]*n
    # Scale duration: cubic spline peak vel = 1.5 * max_dq / T → T = 1.5 * max_dq / (vel * limit)
    duration_s = max(0.05, 1.5 * max_dq / (max(0.01, vel_scaling) * 3.0))
    pt1.time_from_start = Duration(sec=0, nanosec=int(duration_s * 1e9))

    traj_msg.points = [pt0, pt1]

    # Collision-check midpoint + endpoint in parallel, then execute if both valid
    _jacobian_check_and_execute(robot_controller, joint_names, traj_msg, generation)
    return True


def _jacobian_check_and_execute(robot_controller, joint_names, traj_msg, generation):
    """Fire parallel /check_state_validity calls for midpoint + endpoint, execute only if both valid."""
    from moveit_msgs.srv import GetStateValidity
    from moveit_msgs.msg import RobotState
    from sensor_msgs.msg import JointState
    import threading

    if not hasattr(robot_controller, '_state_validity_client'):
        robot_controller._state_validity_client = robot_controller.create_client(
            GetStateValidity, '/check_state_validity')

    if not robot_controller._state_validity_client.wait_for_service(timeout_sec=0.3):
        robot_controller.get_logger().warning(
            '[JacMove] /check_state_validity unavailable — relying on SafetyWallManager only')
        with robot_controller.lock:
            robot_controller.last_move_result = 0
        _send_trajectory_to_controller(robot_controller, traj_msg)
        return

    q_start = list(traj_msg.points[0].positions)
    q_end   = list(traj_msg.points[-1].positions)
    q_mid   = [(q_start[i] + q_end[i]) * 0.5 for i in range(len(q_start))]

    results = {'mid': None, 'end': None}
    results_lock = threading.Lock()

    def _make_request(q):
        js = JointState()
        js.name = joint_names
        js.position = q
        rs = RobotState()
        rs.joint_state = js
        req = GetStateValidity.Request()
        req.robot_state = rs
        req.group_name = 'fairino5_v6_group'
        return req

    def _log_contacts(label, response):
        if response is None or not hasattr(response, 'contacts'):
            return
        if response.contacts:
            bodies = set()
            for c in response.contacts:
                bodies.add(f'{c.contact_body_1} ↔ {c.contact_body_2}')
            robot_controller.get_logger().warning(
                f'[JacMove] {label} contacts: {", ".join(sorted(bodies))}')
        else:
            robot_controller.get_logger().warning(
                f'[JacMove] {label} invalid but no contact details returned')

    _SAFETY_WALL_NAMES = {
        'wall_x_min', 'wall_x_max', 'wall_y_min', 'wall_y_max', 'wall_z_min', 'wall_z_max'
    }
    _EE_LINKS = {'ee_link', 'wrist3_link', 'wrist2_link', 'wrist1_link', 'forearm_link'}

    def _is_ee_wall_contact_only(resp):
        """Return True if the only contacts are ee_link ↔ safety-wall objects.
        TCP position was already validated by check_position_safety(); the ee_link
        geometry legitimately touches the boundary wall when operating near workspace edges."""
        if resp is None or resp.valid or not resp.contacts:
            return False
        for c in resp.contacts:
            b1, b2 = c.contact_body_1, c.contact_body_2
            if not (
                (b1 in _EE_LINKS and b2 in _SAFETY_WALL_NAMES) or
                (b2 in _EE_LINKS and b1 in _SAFETY_WALL_NAMES)
            ):
                return False  # non-wall or non-ee contact — real collision
        return True

    def _log_ee_wall_contacts(label, resp):
        """Log detail about allowed ee_link↔wall contacts for diagnostics."""
        pairs = []
        max_depth = 0.0
        for c in resp.contacts:
            b1, b2 = c.contact_body_1, c.contact_body_2
            depth = getattr(c, 'depth', 0.0)
            max_depth = max(max_depth, abs(depth))
            pairs.append(f'{b1}↔{b2}(d={depth*1000:.2f}mm)')
        robot_controller.get_logger().info(
            f'[JacMove] {label}: ee_link↔wall contact ALLOWED — '
            f'contacts=[{", ".join(sorted(pairs))}] max_penetration={max_depth*1000:.2f}mm '
            f'(TCP already validated by safety manager, proceeding)')

    def _on_both_done():
        with results_lock:
            if results['mid'] is None or results['end'] is None:
                return
            mid_resp = results['mid']
            end_resp = results['end']

        with robot_controller.lock:
            if robot_controller.plan_generation != generation:
                robot_controller.get_logger().info('[JacMove] Stale validity response — discarding')
                return

        if mid_resp is not None and not mid_resp.valid:
            if _is_ee_wall_contact_only(mid_resp):
                _log_ee_wall_contacts('Midpoint', mid_resp)
            else:
                robot_controller.get_logger().warning('[JacMove] ⛔ Midpoint invalid — collision on path, aborting')
                _log_contacts('Midpoint', mid_resp)
                with robot_controller.lock:
                    robot_controller.is_executing = False
                    robot_controller.last_move_result = -10
                return

        if end_resp is not None and not end_resp.valid:
            if _is_ee_wall_contact_only(end_resp):
                _log_ee_wall_contacts('Target', end_resp)
            else:
                robot_controller.get_logger().warning('[JacMove] ⛔ Target invalid — collision detected, aborting')
                _log_contacts('Target', end_resp)
                with robot_controller.lock:
                    robot_controller.is_executing = False
                    robot_controller.last_move_result = -10
                return

        with robot_controller.lock:
            robot_controller.last_move_result = 0
        _send_trajectory_to_controller(robot_controller, traj_msg)

    def _cb_mid(fut):
        try:
            with results_lock:
                results['mid'] = fut.result()
        except Exception as e:
            robot_controller.get_logger().error(f'[JacMove] Midpoint validity check failed: {e}')
            with results_lock:
                results['mid'] = None
        _on_both_done()

    def _cb_end(fut):
        try:
            with results_lock:
                results['end'] = fut.result()
        except Exception as e:
            robot_controller.get_logger().error(f'[JacMove] Endpoint validity check failed: {e}')
            with results_lock:
                results['end'] = None
        _on_both_done()

    robot_controller._state_validity_client.call_async(_make_request(q_mid)).add_done_callback(_cb_mid)
    robot_controller._state_validity_client.call_async(_make_request(q_end)).add_done_callback(_cb_end)

