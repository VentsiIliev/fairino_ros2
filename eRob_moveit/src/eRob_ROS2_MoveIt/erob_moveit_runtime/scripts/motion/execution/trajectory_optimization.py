# Time Optimal Trajectory Generation
from erob_moveit_runtime.srv import ApplyIPP
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import yaml
import os
import config as cfg

# Cache for loaded joint limits
_joint_limits_cache = None


def _compute_reduced_fallback_scaling(vel_scaling, acc_scaling):
    if not bool(getattr(cfg, 'RUCKIG_FALLBACK_REDUCE_SCALING', True)):
        return float(vel_scaling), float(acc_scaling), False

    vel = float(vel_scaling)
    acc = float(acc_scaling)
    reduced_vel = max(float(getattr(cfg, 'RUCKIG_FALLBACK_MIN_VEL_SCALING', 0.1)),
                      vel * float(getattr(cfg, 'RUCKIG_FALLBACK_VEL_MULTIPLIER', 0.5)))
    reduced_acc = max(float(getattr(cfg, 'RUCKIG_FALLBACK_MIN_ACC_SCALING', 0.1)),
                      acc * float(getattr(cfg, 'RUCKIG_FALLBACK_ACC_MULTIPLIER', 0.5)))
    changed = (abs(reduced_vel - vel) > 1e-9) or (abs(reduced_acc - acc) > 1e-9)
    return reduced_vel, reduced_acc, changed


def _load_joint_limits_from_config():
    """Load joint limits from joint_limits.yaml config file."""
    global _joint_limits_cache

    if _joint_limits_cache is not None:
        return _joint_limits_cache

    # Prefer limits for the active robot profile, then fall back to the
    # package-wide joint_limits.yaml.  Profiles are optional overlays.
    config_paths = []

    # Try ament_index if available
    try:
        from ament_index_python.packages import get_package_share_directory
        pkg_dir = get_package_share_directory(os.environ.get('EROB_CONFIG_PACKAGE', 'fairino5_v6_moveit2_config'))
        active_profile = str(getattr(cfg, 'ACTIVE_PROFILE', '') or '').strip()
        if active_profile:
            config_paths.append(os.path.join(pkg_dir, 'config', active_profile, 'joint_limits.yaml'))
        config_paths.append(os.path.join(pkg_dir, 'config', 'joint_limits.yaml'))
    except Exception:
        pass

    # Source-tree fallback for development without an ament package index.
    source_config = os.path.normpath(
        os.path.join(os.path.dirname(__file__), '..', '..', '..', 'config')
    )
    active_profile = str(getattr(cfg, 'ACTIVE_PROFILE', '') or '').strip()
    if active_profile:
        config_paths.append(os.path.join(source_config, active_profile, 'joint_limits.yaml'))
    config_paths.append(os.path.join(source_config, 'joint_limits.yaml'))

    for config_path in config_paths:
        config_path = os.path.normpath(config_path)
        if os.path.exists(config_path):
            try:
                with open(config_path, 'r') as f:
                    config = yaml.safe_load(f)

                joint_limits = config.get('joint_limits', {})
                joint_names = cfg.JOINT_NAMES

                limits = {
                    'max_velocity': [],
                    'max_acceleration': [],
                    'max_jerk': [],
                    'default_vel_scaling': config.get('default_velocity_scaling_factor', cfg.DEFAULT_VEL_SCALING),
                    'default_acc_scaling': config.get('default_acceleration_scaling_factor', cfg.DEFAULT_ACC_SCALING),
                }

                for joint in joint_names:
                    if joint in joint_limits:
                        jl = joint_limits[joint]
                        limits['max_velocity'].append(jl.get('max_velocity', 3.14))
                        limits['max_acceleration'].append(jl.get('max_acceleration', 10.0))
                        limits['max_jerk'].append(jl.get('max_jerk', 50.0))
                    else:
                        # Fallback defaults
                        limits['max_velocity'].append(3.14)
                        limits['max_acceleration'].append(10.0)
                        limits['max_jerk'].append(50.0)

                _joint_limits_cache = limits
                return limits

            except Exception as e:
                print(f"[Ruckig] Warning: Could not load {config_path}: {e}")

    # Fallback if no config found
    _joint_limits_cache = {
        'max_velocity': [3.14, 3.14, 3.14, 3.14, 3.14, 3.14],
        'max_acceleration': [10.0, 10.0, 10.0, 10.0, 10.0, 10.0],
        'max_jerk': [50.0, 50.0, 50.0, 50.0, 60.0, 70.0],
        'default_vel_scaling': cfg.DEFAULT_VEL_SCALING,
        'default_acc_scaling': cfg.DEFAULT_ACC_SCALING,
    }
    return _joint_limits_cache


def apply_ruckig(robot_controller, trajectory, vel_scaling=None, acc_scaling=None, jerk_scaling=cfg.DEFAULT_JERK_SCALING, callback=None):
    """
    Apply Ruckig time-optimal jerk-limited trajectory parameterization.

    This is a drop-in replacement for apply_ipp_totg with smoother motion
    due to jerk limiting (3rd order vs 2nd order).

    Args:
        robot_controller: ROS2 node for logging
        trajectory: RobotTrajectory to time-parameterize
        vel_scaling: Velocity scaling factor [0.0-1.0], None = use config default
        acc_scaling: Acceleration scaling factor [0.0-1.0], None = use config default
        jerk_scaling: Jerk scaling factor [0.0-1.0]
        callback: Function to call with (trajectory_or_None) when done
    """
    try:
        from ruckig import InputParameter, OutputParameter, Result, Ruckig
    except ImportError:
        robot_controller.get_logger().error('[Ruckig] ruckig not installed! Run: pip install ruckig')
        if callback:
            callback(None)
        return

    # Load limits from config
    limits = _load_joint_limits_from_config()

    # Use config defaults if not specified
    if vel_scaling is None:
        vel_scaling = limits['default_vel_scaling']
    if acc_scaling is None:
        acc_scaling = limits['default_acc_scaling']

    robot_controller.get_logger().info(
        f'[Ruckig] Applying jerk-limited time parameterization (vel={vel_scaling}, acc={acc_scaling}, jerk={jerk_scaling})')
    robot_controller.get_logger().info(
        f'[Ruckig] Limits from config: max_vel={limits["max_velocity"]}, max_acc={limits["max_acceleration"]}, max_jerk={limits["max_jerk"]}')

    joint_traj = trajectory.joint_trajectory
    num_joints = len(joint_traj.joint_names)
    num_points = len(joint_traj.points)

    if num_points < 2:
        robot_controller.get_logger().warning('[Ruckig] Trajectory has < 2 points, nothing to parameterize')
        if callback:
            callback(trajectory)
        return

    # Apply scaling to limits from config
    max_vel = [v * vel_scaling for v in limits['max_velocity'][:num_joints]]
    max_acc = [a * acc_scaling for a in limits['max_acceleration'][:num_joints]]
    max_jerk = [j * jerk_scaling for j in limits['max_jerk'][:num_joints]]

    robot_controller.get_logger().info(f'[Ruckig] Processing {num_points} waypoints for {num_joints} joints')

    # Build new trajectory with proper time parameterization
    new_points = []
    current_time = 0.0
    sample_dt = cfg.RUCKIG_SAMPLE_DT_S  # 8ms sampling (125 Hz) - good for smooth motion

    for i in range(num_points - 1):
        start_pt = joint_traj.points[i]
        end_pt = joint_traj.points[i + 1]

        # Setup Ruckig for this segment
        otg = Ruckig(num_joints)
        inp = InputParameter(num_joints)
        out = OutputParameter(num_joints)

        # Current state
        inp.current_position = list(start_pt.positions[:num_joints])
        if start_pt.velocities and len(start_pt.velocities) >= num_joints:
            inp.current_velocity = list(start_pt.velocities[:num_joints])
        else:
            inp.current_velocity = [0.0] * num_joints
        if start_pt.accelerations and len(start_pt.accelerations) >= num_joints:
            inp.current_acceleration = list(start_pt.accelerations[:num_joints])
        else:
            inp.current_acceleration = [0.0] * num_joints

        # Target state
        inp.target_position = list(end_pt.positions[:num_joints])
        if end_pt.velocities and len(end_pt.velocities) >= num_joints:
            inp.target_velocity = list(end_pt.velocities[:num_joints])
        else:
            # For intermediate waypoints, allow non-zero velocity for smooth blending
            if i < num_points - 2:
                inp.target_velocity = [0.0] * num_joints  # Will be optimized by Ruckig
            else:
                inp.target_velocity = [0.0] * num_joints  # Final point: stop
        inp.target_acceleration = [0.0] * num_joints

        # Limits
        inp.max_velocity = max_vel
        inp.max_acceleration = max_acc
        inp.max_jerk = max_jerk

        # Calculate time-optimal trajectory for this segment
        result = otg.calculate(inp, out)

        if result == Result.ErrorInvalidInput:
            robot_controller.get_logger().error(f'[Ruckig] Invalid input at segment {i}')
            if callback:
                callback(None)
            return

        # Get segment duration
        segment_duration = out.trajectory.duration

        if segment_duration <= 0:
            # Skip zero-duration segments (identical positions)
            continue

        # Sample the trajectory at fixed intervals
        t = 0.0
        while t < segment_duration:
            out.trajectory.at_time(t, out.new_position, out.new_velocity, out.new_acceleration)

            point = JointTrajectoryPoint()
            point.positions = list(out.new_position)
            point.velocities = list(out.new_velocity)
            point.accelerations = list(out.new_acceleration)

            # Set timestamp
            total_time = current_time + t
            point.time_from_start = Duration(
                sec=int(total_time),
                nanosec=int((total_time % 1.0) * 1e9)
            )

            new_points.append(point)
            t += sample_dt

        current_time += segment_duration

    # Add final point
    final_pt = joint_traj.points[-1]
    final_point = JointTrajectoryPoint()
    final_point.positions = list(final_pt.positions[:num_joints])
    final_point.velocities = [0.0] * num_joints
    final_point.accelerations = [0.0] * num_joints
    final_point.time_from_start = Duration(
        sec=int(current_time),
        nanosec=int((current_time % 1.0) * 1e9)
    )
    new_points.append(final_point)

    # Update trajectory
    joint_traj.points = new_points
    trajectory.joint_trajectory = joint_traj

    robot_controller.get_logger().info(
        f'[Ruckig] Generated {len(new_points)} points, total duration: {current_time:.3f}s')

    if callback:
        callback(trajectory)


def _handle_apply_ipp_response(robot_controller, fut, trajectory, callback, tag):
    try:
        response = fut.result()

        if response is None:
            robot_controller.get_logger().error(f'{tag} ✗ Response is None')
            if callback: callback(None)
            return

        if not hasattr(response, 'trajectory'):
            robot_controller.get_logger().error(f'{tag} ✗ Response has no trajectory attribute')
            if callback: callback(None)
            return

        if not hasattr(response.trajectory, 'joint_trajectory'):
            robot_controller.get_logger().error(f'{tag} ✗ Response.trajectory has no joint_trajectory attribute')
            if callback: callback(None)
            return

        joint_traj = response.trajectory.joint_trajectory
        num_points = len(joint_traj.points)

        if num_points == 0:
            robot_controller.get_logger().error(f'{tag} ✗ Empty trajectory')
            if callback: callback(None)
            return

        robot_controller.get_logger().info(f'{tag} ✓ Generated {num_points} points')

        has_timestamps = any(
            pt.time_from_start.sec > 0 or pt.time_from_start.nanosec > 0
            for pt in joint_traj.points
        )
        if not has_timestamps:
            robot_controller.get_logger().error(f'{tag} ✗ Response has no timestamps - INVALID trajectory')
            if callback: callback(None)
            return

        trajectory.joint_trajectory = joint_traj
        if callback:
            callback(trajectory)

    except Exception as e:
        robot_controller.get_logger().error(f'{tag} ✗ Service call failed: {e}')
        import traceback
        robot_controller.get_logger().error(f'{tag} Traceback: {traceback.format_exc()}')
        if callback: callback(None)


def apply_ruckig_service(robot_controller, trajectory, vel_scaling=cfg.DEFAULT_VEL_SCALING, acc_scaling=cfg.DEFAULT_ACC_SCALING, callback=None):
    """Call the Ruckig service for jerk-limited trajectory smoothing (ASYNC).

    This uses the C++ ruckig_helper node which leverages MoveIt's built-in Ruckig integration.
    Provides smoother motion than TOTG due to jerk limiting (3rd order vs 2nd order).

    Args:
        robot_controller: ROS2 node with ruckig_client
        trajectory: RobotTrajectory to time-parameterize
        vel_scaling: Velocity scaling factor [0.0-1.0]
        acc_scaling: Acceleration scaling factor [0.0-1.0]
        callback: Function to call with (trajectory_or_None) when done
    """
    robot_controller.get_logger().info('[Ruckig] Checking if Ruckig service is available...')

    def _fallback_to_totg(reason: str):
        fallback_vel, fallback_acc, reduced = _compute_reduced_fallback_scaling(
            vel_scaling,
            acc_scaling,
        )
        if reduced:
            robot_controller.get_logger().warning(
                f'[Ruckig] Falling back to TOTG with reduced scaling: {reason} '
                f'(vel={vel_scaling:.3f}->{fallback_vel:.3f}, '
                f'acc={acc_scaling:.3f}->{fallback_acc:.3f})'
            )
        else:
            robot_controller.get_logger().warning(
                f'[Ruckig] Falling back to TOTG: {reason}'
            )
        apply_ipp_totg(
            robot_controller,
            trajectory,
            vel_scaling=fallback_vel,
            acc_scaling=fallback_acc,
            callback=callback,
        )

    # Create client if not exists
    ruckig_service_name = getattr(cfg, 'SERVICE_APPLY_RUCKIG', '/apply_ruckig')
    if not hasattr(robot_controller, 'ruckig_client'):
        robot_controller.ruckig_client = robot_controller.create_client(
            ApplyIPP,
            ruckig_service_name,
        )

    if not robot_controller.ruckig_client.wait_for_service(timeout_sec=cfg.OPT_SERVICE_TIMEOUT_S):
        robot_controller.get_logger().error(f'[Ruckig] ✗ Ruckig service {ruckig_service_name} NOT available after 5s!')
        robot_controller.get_logger().error('[Ruckig]    Is ruckig_helper node running? Check: ros2 node list | grep ruckig')
        _fallback_to_totg(f'{ruckig_service_name} service unavailable')
        return

    robot_controller.get_logger().info('[Ruckig] ✓ Ruckig service is available')

    request = _build_apply_ipp_request(trajectory=trajectory,
                                       vel_scaling=vel_scaling,
                                       acc_scaling=acc_scaling)
    robot_controller.get_logger().info(
        f'[Ruckig] Requesting jerk-limited smoothing (vel={vel_scaling}, acc={acc_scaling})')

    # Call ASYNC to avoid blocking the executor
    def _on_done(f):
        def _callback_with_fallback(result):
            if result is None:
                _fallback_to_totg('Ruckig returned no valid trajectory')
                return
            if callback:
                callback(result)

        _handle_apply_ipp_response(
            robot_controller, f, trajectory, _callback_with_fallback, '[Ruckig]')

    future = robot_controller.ruckig_client.call_async(request)
    future.add_done_callback(_on_done)


def apply_ipp_totg(robot_controller, trajectory, vel_scaling=cfg.DEFAULT_VEL_SCALING, acc_scaling=cfg.DEFAULT_ACC_SCALING, callback=None):
    """Call the IPP service to apply TOTG (ASYNC).

    Args:
        trajectory: RobotTrajectory to time-parameterize
        vel_scaling: Velocity scaling factor [0.0-1.0]
        acc_scaling: Acceleration scaling factor [0.0-1.0]
        callback: Function to call with (trajectory_or_None) when done
    """
    robot_controller.get_logger().info('[TOTG] Checking if IPP service is available...')

    if not robot_controller.ipp_client.wait_for_service(timeout_sec=cfg.OPT_SERVICE_TIMEOUT_S):
        robot_controller.get_logger().error('[TOTG] ✗ IPP service /apply_ipp NOT available after 5s!')
        robot_controller.get_logger().error('[TOTG]    Is ipp_helper node running? Check: ros2 node list | grep ipp')
        if callback:
            callback(None)
        return

    robot_controller.get_logger().info('[TOTG] ✓ IPP service is available')

    request = _build_apply_ipp_request(trajectory=trajectory,
                                       vel_scaling=vel_scaling,
                                       acc_scaling=acc_scaling)

    robot_controller.get_logger().info(
        f'[TOTG] Requesting time-optimal parameterization (vel={vel_scaling}, acc={acc_scaling})')

    # Call ASYNC to avoid blocking the executor
    future = robot_controller.ipp_client.call_async(request)
    future.add_done_callback(
        lambda f: _handle_apply_ipp_response(robot_controller, f, trajectory, callback, '[TOTG]'))


def _build_apply_ipp_request(trajectory, vel_scaling, acc_scaling) -> ApplyIPP.Request:
    """Helper to build ApplyIPP request from trajectory and scaling factors."""
    request = ApplyIPP.Request()
    request.trajectory = trajectory.joint_trajectory
    request.max_velocity_scaling = float(vel_scaling)
    request.max_acceleration_scaling = float(acc_scaling)
    return request
