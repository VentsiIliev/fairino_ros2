"""
Jacobian Fallback Move
======================
Sub-mm / short-move fallback via Jacobian pseudoinverse, used when MoveIt's
compute_cartesian_path returns ≤1 trajectory point for a non-trivial delta.

Public entry point: _jacobian_fallback_move()
Internal helper:    _jacobian_check_and_execute()
"""

import numpy as np
import config
from .planner_utils import _set_result, _is_stale
from ..execution.trajectory_executor import _send_trajectory_to_controller


_JACOBIAN_NOOP_POSITION_TOL_M = 1e-5  # 0.01 mm
_JACOBIAN_NOOP_ORIENTATION_TOL_RAD = 1e-5
_JACOBIAN_NOOP_JOINT_TOL_RAD = 1e-5


def _jacobian_fallback_move(robot_controller, waypoints, vel_scaling, acc_scaling, generation):
    """
    Execute a short Cartesian move directly via Jacobian pseudoinverse, bypassing
    the full MoveIt compute_cartesian_path pipeline.

    Used when MoveIt returns ≤1 trajectory point for a move that is non-trivial
    (delta > JACOBIAN_FALLBACK_MIN_DELTA_MM). MoveIt's IK solver collapses short
    paths to a single config; the Jacobian approach computes the exact joint-space
    delta analytically using the robot's DH parameters.

    Algorithm:
        1. Read current joint state
        2. Compute numerical Jacobian via finite differences on the DH FK
        3. Invert with damped pseudoinverse (avoids singularity blow-up)
        4. Compute Δq = J⁺ · Δx (Cartesian delta → joint delta)
        5. Clamp to JACOBIAN_MAX_JOINT_STEP to prevent large jumps
        6. Build a 2-point JointTrajectory (start → start+Δq)
        7. Collision-check midpoint + endpoint in parallel before executing

    After building the trajectory, control is passed to _jacobian_check_and_execute()
    which handles the async collision checks and final dispatch.

    Args:
        robot_controller: RobotController node
        waypoints:        list of geometry_msgs/Pose — start and end ee_link poses
        vel_scaling:      velocity scaling (0–1) for trajectory duration estimate
        acc_scaling:      acceleration scaling (not directly used, passed through)
        generation:       plan_generation for staleness detection

    Returns:
        bool: True if trajectory was submitted for execution (may still fail in collision check)
              False if computation failed (singular Jacobian, no joint state, etc.)
    """

    def _fk(q):
        def rotz(a):
            c, s = np.cos(a), np.sin(a)
            return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=float)
        def roty(a):
            c, s = np.cos(a), np.sin(a)
            return np.array([[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]], dtype=float)
        def rotx(a):
            c, s = np.cos(a), np.sin(a)
            return np.array([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]], dtype=float)
        def trans(x, y, z):
            return np.array([[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]], dtype=float)

        if getattr(config, 'ROBOT_BACKEND', 'fairino') == 'zeroerr':
            # FK from eRobo3 URDF joint chain (base_link → tcp)
            T = np.eye(4)
            T = T @ trans(0,       0,       0      ) @ rotz( q[0])   # Joint_1 axis +Z
            T = T @ trans(0,       0.053,   0.1405 ) @ roty(-q[1])   # Joint_2 axis -Y
            T = T @ trans(0,       0.0005,  0.3635 ) @ roty(-q[2])   # Joint_3 axis -Y
            T = T @ trans(0,      -0.014,   0.311  ) @ roty(-q[3])   # Joint_4 axis -Y
            T = T @ trans(0,       0.047,   0.039  ) @ rotz( q[4])   # Joint_5 axis +Z
            T = T @ trans(0,       0.0608,  0.047  ) @ roty( q[5])   # Joint_6 axis +Y
            T = T @ trans(0,       0.042,  -0.030  ) @ rotz(np.pi) @ rotx(-np.pi / 2)  # tool0 fixed
            T = T @ trans(0,      -0.0305, -0.083  )                  # tcp fixed
        else:
            # Fairino5 v6 DH FK
            T = np.eye(4)
            T = T @ rotz(q[0])
            T = T @ trans(0, 0, config.DH_D1) @ rotx(np.pi / 2) @ rotz(q[1])
            T = T @ trans(config.DH_A2, 0, 0) @ rotz(q[2])
            T = T @ trans(config.DH_A3, 0, 0) @ rotz(q[3])
            T = T @ trans(0, 0, config.DH_D4) @ rotx(np.pi / 2) @ rotz(q[4])
            T = T @ trans(0, 0, config.DH_D5) @ rotx(-np.pi / 2) @ rotz(q[5])
        return T

    def _rotation_angle(R):
        tr = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
        return float(np.arccos(tr))

    n = 6
    js = robot_controller.current_joint_state
    if js is None or len(js.position) < n:
        robot_controller.get_logger().warning('[JacMove] Joint state unavailable or insufficient joints')
        return False

    joints = [float(js.position[i]) for i in range(n)]
    joint_names = [str(js.name[i]) for i in range(n)]
    q0 = np.array(joints)
    T0 = _fk(q0)

    # Cartesian delta in metres (position only — orientation correction not applied)
    p0, p1 = waypoints[0].position, waypoints[-1].position
    delta_x = np.array([p1.x - p0.x, p1.y - p0.y, p1.z - p0.z, 0.0, 0.0, 0.0])

    if np.linalg.norm(delta_x[:3]) < 1e-7:
        robot_controller.get_logger().warning('[JacMove] delta_x ≈ 0 — waypoints identical, skipping move')
        _set_result(robot_controller, 0)
        return True

    # Numerical Jacobian via forward finite differences.
    # Each column j of J is the partial derivative of the end-effector pose
    # (position + axis-angle orientation) with respect to joint j.
    eps = config.JACOBIAN_NUM_DIFF_EPS
    J = np.zeros((6, n))
    for j in range(n):
        q_pert = q0.copy()
        q_pert[j] += eps
        T_pert = _fk(q_pert)
        # Position derivatives (linear rows)
        J[0, j] = (T_pert[0, 3] - T0[0, 3]) / eps
        J[1, j] = (T_pert[1, 3] - T0[1, 3]) / eps
        J[2, j] = (T_pert[2, 3] - T0[2, 3]) / eps
        # Orientation derivatives via rotation matrix log (angular rows)
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

    # Damped least-squares pseudoinverse: J⁺ = Vᵀ · diag(σ/(σ²+λ²)) · Uᵀ
    # λ (damping) prevents blow-up near singularities where σ ≈ 0
    lam = config.JACOBIAN_DAMPING
    U, S, Vt = np.linalg.svd(J)
    S_inv = S / (S ** 2 + lam ** 2)
    J_pinv = Vt.T @ np.diag(S_inv) @ U.T

    delta_q = J_pinv @ delta_x

    max_dq = np.max(np.abs(delta_q))
    robot_controller.get_logger().info(f'[JacMove] Δq={[f"{v:.5f}" for v in delta_q]} (max={max_dq:.5f})')

    if max_dq < _JACOBIAN_NOOP_JOINT_TOL_RAD:
        # Compare the requested target pose with the robot's current pose to distinguish:
        # - true no-op request (already at target) -> success
        # - meaningful micro move that collapsed numerically -> keep singularity/error
        p_start = np.array([p0.x, p0.y, p0.z], dtype=float)
        p_target = np.array([p1.x, p1.y, p1.z], dtype=float)
        requested_position_delta = float(np.linalg.norm(p_target - p_start))

        q_target = q0 + delta_q
        T_target = _fk(q_target)
        achieved_position_delta = float(np.linalg.norm(T_target[:3, 3] - T0[:3, 3]))
        achieved_orientation_delta = _rotation_angle(T_target[:3, :3] @ T0[:3, :3].T)

        if (
            requested_position_delta <= _JACOBIAN_NOOP_POSITION_TOL_M
            and achieved_position_delta <= _JACOBIAN_NOOP_POSITION_TOL_M
            and achieved_orientation_delta <= _JACOBIAN_NOOP_ORIENTATION_TOL_RAD
        ):
            robot_controller.get_logger().info(
                '[JacMove] Requested target already satisfied '
                f'(Δx={requested_position_delta * 1000.0:.6f}mm, maxΔq={max_dq:.2e}) — treating as no-op success')
            _set_result(robot_controller, 0)
            return True

        robot_controller.get_logger().warning(
            '[JacMove] Near-zero Δq for unresolved target '
            f'(reqΔx={requested_position_delta * 1000.0:.6f}mm, '
            f'achievedΔx={achieved_position_delta * 1000.0:.6f}mm, '
            f'achievedΔR={achieved_orientation_delta:.2e}rad, maxΔq={max_dq:.2e})')
        _set_result(robot_controller, -9)
        return False

    # Clamp: prevent large joint jumps that could exceed controller limits
    max_joint_step = config.JACOBIAN_MAX_JOINT_STEP  # rad
    if max_dq > max_joint_step:
        delta_q *= max_joint_step / max_dq
        robot_controller.get_logger().warning(f'[JacMove] Clamped {max_dq:.5f} → {max_joint_step:.2f} rad')

    target_joints = np.array(joints) + delta_q

    # Build a minimal 2-point JointTrajectory: start (t=0) → target (t=duration)
    # Duration is estimated from cubic spline peak velocity: T = 1.5·Δq_max / (vel·ω_max)
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from builtin_interfaces.msg import Duration
    traj_msg = JointTrajectory()
    traj_msg.joint_names = joint_names

    pt0 = JointTrajectoryPoint()
    pt0.positions = list(q0)
    pt0.velocities = [0.0] * n
    pt0.accelerations = [0.0] * n
    pt0.time_from_start = Duration(sec=0, nanosec=0)

    pt1 = JointTrajectoryPoint()
    pt1.positions = list(target_joints)
    pt1.velocities = [0.0] * n
    pt1.accelerations = [0.0] * n
    duration_s = max(config.JACOBIAN_MIN_DURATION_S, 1.5 * max_dq / (max(0.01, vel_scaling) * 3.0))
    pt1.time_from_start = Duration(sec=0, nanosec=int(duration_s * 1e9))

    traj_msg.points = [pt0, pt1]

    # Hand off to parallel collision check before executing
    _jacobian_check_and_execute(robot_controller, joint_names, traj_msg, generation)
    return True


def _jacobian_check_and_execute(robot_controller, joint_names, traj_msg, generation):
    """
    Validate a Jacobian trajectory by collision-checking midpoint and endpoint in
    parallel, then execute only if both states are valid.

    Two /check_state_validity requests are fired simultaneously:
      - q_mid = arithmetic mean of start and end joints (worst-case path point)
      - q_end = the target joint configuration

    The midpoint check catches configurations that are in collision at an intermediate
    position even though start and end are both valid (e.g. a short move whose
    joint-space arc clips a safety wall due to FK nonlinearity).

    Execution only proceeds when BOTH responses arrive and BOTH are valid
    (or contain only allowed ee_link↔safety-wall contacts).

    Triggered by: _jacobian_fallback_move(), after building the 2-point trajectory.

    Inner callbacks (all on the ROS2 executor thread):
      _cb_mid(fut)    — triggered when midpoint validity response arrives
      _cb_end(fut)    — triggered when endpoint validity response arrives
      _on_both_done() — called by whichever callback arrives second; fires
                        execution or aborts based on combined results

    Args:
        robot_controller: RobotController node
        joint_names:      list of joint name strings for the JointState messages
        traj_msg:         JointTrajectory with exactly 2 points (start, target)
        generation:       plan_generation for staleness detection
    """
    from moveit_msgs.srv import GetStateValidity
    from moveit_msgs.msg import RobotState
    from sensor_msgs.msg import JointState
    import threading

    # Reuse a single persistent validity client
    state_validity_client = robot_controller.get_state_validity_client()

    if not state_validity_client.wait_for_service(timeout_sec=0.3):
        # Service unavailable — fall back to SafetyWallManager pre-checks only
        robot_controller.get_logger().warning(
            '[JacMove] /check_state_validity unavailable — relying on SafetyWallManager only')
        with robot_controller.lock:
            robot_controller.last_move_result = 0
        _send_trajectory_to_controller(robot_controller, traj_msg)
        return

    q_start = list(traj_msg.points[0].positions)
    q_end   = list(traj_msg.points[-1].positions)
    # Arithmetic midpoint in joint space — approximates the worst-case intermediate pose
    q_mid   = [(q_start[i] + q_end[i]) * 0.5 for i in range(len(q_start))]

    # Shared results dict protected by a lock; both callbacks write here
    results = {'mid': None, 'end': None}
    results_lock = threading.Lock()

    def _make_request(q):
        """
        Build a GetStateValidity request for joint configuration q.
        The group name selects MoveIt's collision world for the planning group.
        """
        js = JointState()
        js.name = joint_names
        js.position = q
        rs = RobotState()
        rs.joint_state = js
        req = GetStateValidity.Request()
        req.robot_state = rs
        req.group_name = config.PLANNING_GROUP
        return req

    def _log_contacts(label, response):
        """Log all contact body pairs from a validity response for debugging."""
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

    # Safety wall names and bypass links from config (captured in closure for performance)
    _SAFETY_WALL_NAMES = config.SAFETY_WALL_NAMES
    _EE_LINKS = config.WALL_BYPASS_LINKS

    def _is_ee_wall_contact_only(resp):
        """
        Return True if every contact in resp is an allowed ee_link↔safety-wall pair.

        The SafetyWallManager pre-check (check_position_safety) already validated
        the TCP position before planning. The ee_link mesh may physically touch the
        boundary wall box when operating near workspace edges. These contacts are
        expected and safe — only real (non-wall) collisions should abort execution.

        Returns False if resp is None, valid, has no contacts, or contains any
        non-wall / non-ee_link contact pair.
        """
        if resp is None or resp.valid or not resp.contacts:
            return False
        for c in resp.contacts:
            b1, b2 = c.contact_body_1, c.contact_body_2
            if not (
                (b1 in _EE_LINKS and b2 in _SAFETY_WALL_NAMES) or
                (b2 in _EE_LINKS and b1 in _SAFETY_WALL_NAMES)
            ):
                return False  # At least one non-wall contact — treat as real collision
        return True

    def _log_ee_wall_contacts(label, resp):
        """Log detail about allowed ee_link↔wall contacts (penetration depth) for diagnostics."""
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

    def _handle_invalid(label, resp):
        """
        Evaluate a validity response and abort execution if a real collision is present.

        Returns True if execution should be aborted (sets result=-10).
        Returns False if the state is valid or contains only allowed ee_link↔wall contacts.
        """
        if resp is None or resp.valid:
            return False
        if _is_ee_wall_contact_only(resp):
            _log_ee_wall_contacts(label, resp)
            return False  # Allowed contact — do not abort
        robot_controller.get_logger().warning(f'[JacMove] ⛔ {label} invalid — collision, aborting')
        _log_contacts(label, resp)
        _set_result(robot_controller, -10)
        return True

    def _on_both_done():
        """
        Called by whichever of _cb_mid / _cb_end arrives second.

        Waits until both results are populated (returns early if either is still None),
        checks for staleness, evaluates both states for collision, and if both pass,
        dispatches the trajectory to the hardware controller.

        Triggered by: _cb_mid() or _cb_end() after writing their result to `results`.
        """
        with results_lock:
            if results['mid'] is None or results['end'] is None:
                return  # Other callback hasn't arrived yet — wait
            mid_resp, end_resp = results['mid'], results['end']

        if _is_stale(robot_controller, generation):
            robot_controller.get_logger().info('[JacMove] Stale validity response — discarding')
            return

        # Abort if either state has a real collision
        if _handle_invalid('Midpoint', mid_resp) or _handle_invalid('Target', end_resp):
            return

        with robot_controller.lock:
            robot_controller.last_move_result = 0
        _send_trajectory_to_controller(robot_controller, traj_msg)

    def _cb_mid(fut):
        """
        Callback triggered when the midpoint /check_state_validity response arrives.
        Stores the result and calls _on_both_done() to check if both are ready.
        """
        try:
            with results_lock:
                results['mid'] = fut.result()
        except Exception as e:
            robot_controller.get_logger().error(f'[JacMove] Midpoint validity check failed: {e}')
            with results_lock:
                results['mid'] = None
        _on_both_done()

    def _cb_end(fut):
        """
        Callback triggered when the endpoint /check_state_validity response arrives.
        Stores the result and calls _on_both_done() to check if both are ready.
        """
        try:
            with results_lock:
                results['end'] = fut.result()
        except Exception as e:
            robot_controller.get_logger().error(f'[JacMove] Endpoint validity check failed: {e}')
            with results_lock:
                results['end'] = None
        _on_both_done()

    # Fire both validity checks simultaneously — they run in parallel
    state_validity_client.call_async(_make_request(q_mid)).add_done_callback(_cb_mid)
    state_validity_client.call_async(_make_request(q_end)).add_done_callback(_cb_end)
