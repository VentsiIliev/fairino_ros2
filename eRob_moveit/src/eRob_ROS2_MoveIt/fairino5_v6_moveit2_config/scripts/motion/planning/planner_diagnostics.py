"""
Planner Diagnostics
===================
FK mismatch and start-state collision diagnostic helpers.

Triggered from _cartesian_path_response when MoveIt returns 0% or fractional paths,
to distinguish between IK/reachability failures and actual collision states.
"""

import numpy as np
import config
from moveit_msgs.srv import GetPositionFK


def _diagnose_fk_mismatch(robot_controller, first_waypoint_pose, joint_state):
    """
    Compare the first MoveIt waypoint against MoveIt's own FK for the current joints.

    Triggered manually or from _cartesian_path_response when 0% path is returned,
    to determine whether the failure is caused by a position/orientation mismatch
    between the C++ Cartesian position publisher and MoveIt's internal FK model.

    A mismatch > 1 mm or > 1° means MoveIt thinks the robot is somewhere different
    from where the C++ FK says it is — the start state supplied to GetCartesianPath
    is wrong, causing immediate IK failure on the first waypoint.

    Calls /compute_fk synchronously (via busy-wait polling, safe from callbacks).

    Args:
        robot_controller:    RobotController node
        first_waypoint_pose: geometry_msgs/Pose — the ee_link pose of waypoint[0]
        joint_state:         sensor_msgs/JointState — current joint positions
    """
    logger = robot_controller.get_logger()

    if not hasattr(robot_controller, '_fk_client'):
        robot_controller._fk_client = robot_controller.create_client(
            GetPositionFK, '/compute_fk'
        )

    if not robot_controller._fk_client.wait_for_service(timeout_sec=1.0):
        logger.warning('[FK Diagnostic] /compute_fk service not available')
        return

    from copy import deepcopy
    fk_request = GetPositionFK.Request()
    fk_request.header.frame_id = config.BASE_LINK
    fk_request.fk_link_names = [config.EE_LINK]
    fk_request.robot_state.joint_state = deepcopy(joint_state)
    fk_request.robot_state.is_diff = False

    try:
        import time
        future = robot_controller._fk_client.call_async(fk_request)

        # Busy-wait poll: spin_until_future_complete() cannot be called from
        # within a ROS2 callback (would deadlock the single-threaded executor).
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
            logger.warning(f'[FK Diagnostic] FK failed: error_code={fk_response.error_code.val}')
            return

        if len(fk_response.pose_stamped) == 0:
            logger.warning('[FK Diagnostic] FK returned no poses')
            return

        # MoveIt's FK result for ee_link (metres → mm)
        moveit_pose = fk_response.pose_stamped[0].pose
        moveit_pos = np.array([
            moveit_pose.position.x * 1000,
            moveit_pose.position.y * 1000,
            moveit_pose.position.z * 1000
        ])
        moveit_quat = np.array([
            moveit_pose.orientation.x,
            moveit_pose.orientation.y,
            moveit_pose.orientation.z,
            moveit_pose.orientation.w
        ])

        # First waypoint as supplied to GetCartesianPath (metres → mm)
        waypoint_pos = np.array([
            first_waypoint_pose.position.x * 1000,
            first_waypoint_pose.position.y * 1000,
            first_waypoint_pose.position.z * 1000
        ])
        waypoint_quat = np.array([
            first_waypoint_pose.orientation.x,
            first_waypoint_pose.orientation.y,
            first_waypoint_pose.orientation.z,
            first_waypoint_pose.orientation.w
        ])

        pos_diff = waypoint_pos - moveit_pos
        pos_dist = np.linalg.norm(pos_diff)
        quat_dot = abs(np.dot(moveit_quat, waypoint_quat))
        angle_diff_deg = np.degrees(2 * np.arccos(np.clip(quat_dot, -1.0, 1.0)))

        logger.info('=' * 60)
        logger.info('[FK Diagnostic] Comparing first waypoint vs MoveIt FK:')
        logger.info(f'[FK Diagnostic] MoveIt FK ee_link:  X={moveit_pos[0]:.2f} Y={moveit_pos[1]:.2f} Z={moveit_pos[2]:.2f} mm')
        logger.info(f'[FK Diagnostic] First waypoint:     X={waypoint_pos[0]:.2f} Y={waypoint_pos[1]:.2f} Z={waypoint_pos[2]:.2f} mm')
        logger.info(f'[FK Diagnostic] Position DIFF:      dX={pos_diff[0]:.2f} dY={pos_diff[1]:.2f} dZ={pos_diff[2]:.2f} mm')
        logger.info(f'[FK Diagnostic] Position distance:  {pos_dist:.2f} mm')
        logger.info(f'[FK Diagnostic] Orientation diff:   {angle_diff_deg:.2f} degrees')
        logger.info(f'[FK Diagnostic] MoveIt quat:        [{moveit_quat[0]:.4f}, {moveit_quat[1]:.4f}, {moveit_quat[2]:.4f}, {moveit_quat[3]:.4f}]')
        logger.info(f'[FK Diagnostic] Waypoint quat:      [{waypoint_quat[0]:.4f}, {waypoint_quat[1]:.4f}, {waypoint_quat[2]:.4f}, {waypoint_quat[3]:.4f}]')

        if pos_dist > 1.0:
            logger.error(f'[FK Diagnostic] POSITION MISMATCH > 1mm! This may cause path planning failure.')
        if angle_diff_deg > 1.0:
            logger.error(f'[FK Diagnostic] ORIENTATION MISMATCH > 1 degree! This may cause path planning failure.')

        logger.info('=' * 60)

    except Exception as e:
        logger.error(f'[FK Diagnostic] Exception during FK comparison: {e}')


def _diagnose_start_collision(robot_controller):
    """
    Asynchronously check whether the current robot state is in collision and log
    all offending contact pairs.

    Triggered automatically from _cartesian_path_response whenever MoveIt returns
    fraction < CARTESIAN_MIN_FRACTION (including 0%), to distinguish between:
      - IK/reachability failure  → /check_state_validity returns valid=True
      - Self-collision or object collision → valid=False with contact body names

    The result arrives via _cb() on the ROS2 executor thread and is logged only —
    it does not influence the already-set result code.

    Common false-positive cause: SafetyWallManager ACM diff inadvertently
    re-enabling SRDF-disabled adjacent-link pairs (see safety_wall_manager.py).
    """
    from moveit_msgs.srv import GetStateValidity
    from moveit_msgs.msg import RobotState
    from sensor_msgs.msg import JointState

    logger = robot_controller.get_logger()

    # Reuse a single persistent client to avoid repeated service lookup overhead
    if not hasattr(robot_controller, '_state_validity_client'):
        robot_controller._state_validity_client = robot_controller.create_client(
            GetStateValidity, '/check_state_validity')

    if not robot_controller._state_validity_client.wait_for_service(timeout_sec=0.5):
        logger.warning('[CollisionDiag] /check_state_validity unavailable')
        return

    js = robot_controller.current_joint_state
    if js is None:
        logger.warning('[CollisionDiag] No joint state available')
        return

    from copy import deepcopy
    rs = RobotState()
    rs.joint_state = deepcopy(js)

    req = GetStateValidity.Request()
    req.robot_state = rs
    req.group_name = config.PLANNING_GROUP

    def _cb(fut):
        """
        Callback triggered when /check_state_validity responds.
        Logs whether the start state is valid or reports all colliding pairs.
        """
        try:
            resp = fut.result()
        except Exception as e:
            logger.error(f'[CollisionDiag] Service call failed: {e}')
            return

        if resp.valid:
            logger.info('[CollisionDiag] Start state is VALID — failure is IK/reachability, not collision')
            return

        if resp.contacts:
            pairs = sorted({f'{c.contact_body_1} ↔ {c.contact_body_2}' for c in resp.contacts})
            logger.error('[CollisionDiag] Start state IN COLLISION:')
            for p in pairs:
                logger.error(f'[CollisionDiag]   {p}')
        else:
            logger.error('[CollisionDiag] Start state invalid but no contact details returned')

    robot_controller._state_validity_client.call_async(req).add_done_callback(_cb)
