"""
Planner Utilities
=================
Shared constants and exit-path helpers used by trajectory_planner.py and jacobian_move.py.

Both modules import from here to avoid circular dependencies.
"""

# ============ Trajectory Parameterization Selection ============
# Options:
#   "RUCKIG"  - Use Ruckig C++ service (jerk-limited, requires ruckig_helper node)
#   "TOTG"    - Use TOTG C++ service (time-optimal, requires ipp_helper node)
#
# Ruckig provides smoother motion with S-curve profiles (3rd order)
# TOTG is faster but has trapezoidal velocity profiles (2nd order)
TIME_PARAMETERIZATION = "TOTG"


def _set_result(rc, code):
    """
    Atomically clear the executing flag and record the final result code.

    Called at every early-exit point (planning failure, collision, timeout, etc.)
    and at the start of successful trajectory dispatch to record the outcome
    before _send_trajectory_to_controller takes ownership of is_executing.

    Args:
        rc:   RobotController node instance
        code: Integer result code (0 = success, negative = error — see error_codes.md)
    """
    with rc.lock:
        rc.is_executing = False
        rc.last_move_result = code
    if hasattr(rc, 'motion_queue'):
        rc.motion_queue.mark_current_complete(code)


def _begin_execution(rc):
    """
    Atomically mark execution as started and return a new generation token.

    Called immediately before submitting an async service request or dispatching
    a trajectory. Increments rc.plan_generation so that callbacks from any
    previous (now superseded) request detect staleness via _is_stale().

    Returns:
        int: The new plan_generation value to pass to stale-check callbacks.
    """
    with rc.lock:
        rc.is_executing = True
        rc.plan_generation += 1
        return rc.plan_generation


def _is_stale(rc, generation):
    """
    Return True if the incoming response belongs to a superseded plan.

    Each new move increments rc.plan_generation before submitting the async
    service call. If stop_motion() or a new move was issued between submission
    and response, the generation counter no longer matches and the response
    must be silently discarded to prevent a stale trajectory from executing.

    Args:
        rc:         RobotController node instance
        generation: The plan_generation value captured at submission time

    Returns:
        bool: True → discard response; False → response is current
    """
    with rc.lock:
        return generation is not None and rc.plan_generation != generation


def _require_cart_path_service(rc, tag):
    if not rc.cart_path_client.wait_for_service(timeout_sec=1.0):
        rc.get_logger().error(f'[{tag}] compute_cartesian_path service not available')
        return False
    return True


def _to_pose_list(robot_controller, waypoints_mm, T_tool, check_last_only=True):
    """
    Convert [x, y, z, rx, ry, rz] mm waypoints to geometry_msgs/Pose in EE frame.
    check_last_only=True  → safety-check only the last waypoint (single-target moves).
    check_last_only=False → safety-check every waypoint (multi-waypoint paths).
    Returns (poses, None) on success or (None, -3) on safety rejection.
    """
    from geometry_msgs.msg import Pose
    from utils.transformation_utils import TransformationUtils

    poses = []
    last = len(waypoints_mm) - 1
    for i, wp in enumerate(waypoints_mm):
        T_tcp = TransformationUtils.pose_to_transform(wp)
        T_ee  = TransformationUtils.remove_tcp_offset(T_tcp, T_tool)
        pos   = T_ee[:3, 3]
        quat  = TransformationUtils.matrix_to_quaternion(T_ee[:3, :3])

        if (not check_last_only) or (i == last):
            is_safe, msg = robot_controller.safety_manager.check_position_safety(
                pos[0], pos[1], pos[2])
            if not is_safe:
                robot_controller.get_logger().error(f'[SAFETY] Waypoint rejected: {msg}')
                return None, -3
            if 'Warning' in msg:
                robot_controller.get_logger().warning(f'[SAFETY] {msg}')

        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = pos[0], pos[1], pos[2]
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quat
        poses.append(pose)
    return poses, None

