from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance


def _send_trajectory_to_controller(robot_controller, joint_trajectory):
    """Send trajectory DIRECTLY to the low-level controller for maximum smoothness.

    Bypasses MoveIt's ExecuteTrajectory action which can enforce unwanted path constraints.
    The controller's spline interpolation will smoothly blend through all waypoints.
    """
    if not robot_controller.execution_lock.acquire(blocking=False):
        robot_controller.get_logger().warning('[Controller] Trajectory already executing, ignoring')
        return

    if not robot_controller.controller_client.wait_for_server(timeout_sec=1.0):
        robot_controller.get_logger().error('[Controller] fairino5_controller not available')
        with robot_controller.lock:
            robot_controller.is_executing = False
        robot_controller.execution_lock.release()
        return

    if len(joint_trajectory.points) == 0:
        robot_controller.get_logger().error('[Controller] ✗ Empty trajectory - aborting')
        with robot_controller.lock:
            robot_controller.is_executing = False
        robot_controller.execution_lock.release()
        return

    has_valid_timestamps = False
    for pt in joint_trajectory.points:
        if pt.time_from_start.sec > 0 or pt.time_from_start.nanosec > 0:
            has_valid_timestamps = True
            break

    if not has_valid_timestamps:
        robot_controller.get_logger().error('[Controller] ✗ Trajectory has NO timestamps - aborting')
        with robot_controller.lock:
            robot_controller.is_executing = False
        robot_controller.execution_lock.release()
        return

    with robot_controller.lock:
        robot_controller.is_executing = True

    # Create controller goal with NO path tolerances
    # Only enforce goal position (final point)

    goal_tolerance = []
    for name in joint_trajectory.joint_names:
        tol = JointTolerance()
        tol.name = name
        tol.position = 0.01  # 0.01 rad final position tolerance
        tol.velocity = 0.0  # Must stop at end
        tol.acceleration = 0.0
        goal_tolerance.append(tol)

    controller_goal = FollowJointTrajectory.Goal()
    controller_goal.trajectory = joint_trajectory
    controller_goal.path_tolerance = []  # EMPTY = smooth blending, no stops
    controller_goal.goal_tolerance = goal_tolerance

    # Calculate trajectory duration and set VERY generous time tolerance
    # This prevents spurious aborts when the controller is slightly behind schedule
    if len(joint_trajectory.points) > 0:
        last_point = joint_trajectory.points[-1]
        traj_duration_sec = last_point.time_from_start.sec + last_point.time_from_start.nanosec / 1e9
        # Tolerance = 2x trajectory duration or 5s minimum
        # This gives controller plenty of room for execution delays
        time_tolerance_sec = max(5.0, traj_duration_sec * 2.0)

        # DEBUG: Log trajectory details
        robot_controller.get_logger().info(
            f'[Controller] Trajectory duration: {traj_duration_sec:.2f}s, timeout: {time_tolerance_sec:.1f}s')
        robot_controller.get_logger().info(
            f'[Controller] First point positions: {[round(p, 6) for p in joint_trajectory.points[0].positions]}')
        robot_controller.get_logger().info(
            f'[Controller] Last point positions: {[round(p, 6) for p in last_point.positions]}')
        robot_controller.get_logger().info(
            f'[Controller] First point time: {joint_trajectory.points[0].time_from_start.sec + joint_trajectory.points[0].time_from_start.nanosec / 1e9:.3f}s')
        robot_controller.get_logger().info(f'[Controller] Last point time: {traj_duration_sec:.3f}s')
    else:
        time_tolerance_sec = 5.0

    controller_goal.goal_time_tolerance.sec = int(time_tolerance_sec)
    controller_goal.goal_time_tolerance.nanosec = int((time_tolerance_sec % 1.0) * 1e9)

    robot_controller.get_logger().info(
        f'[Controller] Sending {len(joint_trajectory.points)} points directly to controller '
        f'(spline interpolation, no path tolerance, goal_time_tolerance={time_tolerance_sec:.1f}s)')

    future = robot_controller.controller_client.send_goal_async(controller_goal)
    robot_controller.active_execute_send_future = future
    future.add_done_callback(lambda f: _controller_goal_response(robot_controller, f))


def _controller_goal_response(robot_controller, future):
    """Handle controller goal acceptance."""
    robot_controller.active_execute_send_future = None
    try:
        goal_handle = future.result()
        if not goal_handle.accepted:
            robot_controller.get_logger().error('[Controller] Trajectory execution rejected by fairino5_controller')
            robot_controller.active_controller_goal = None
            with robot_controller.lock:
                robot_controller.is_executing = False
            robot_controller.execution_lock.release()
            return

        robot_controller.get_logger().info('[Controller] Trajectory accepted by fairino5_controller')
        robot_controller.active_controller_goal = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: _controller_goal_result(robot_controller, f))
    except Exception as e:
        robot_controller.get_logger().error(f'[Controller] Goal response error: {e}')
        with robot_controller.lock:
            robot_controller.is_executing = False
        robot_controller.execution_lock.release()


def _controller_goal_result(robot_controller, future):
    """Handle controller goal completion."""
    try:
        result = future.result().result
        if result.error_code == 0:
            robot_controller.get_logger().info('[Controller] ✓ Trajectory execution succeeded!')
            robot_controller.last_move_result = 0
        else:
            robot_controller.get_logger().error(f'[Controller] Trajectory execution failed with error: {result.error_code}')
            robot_controller.last_move_result = result.error_code
    except Exception as e:
        robot_controller.get_logger().error(f'[Controller] Result error: {e}')
        robot_controller.last_move_result = -1
    finally:
        robot_controller.active_controller_goal = None
        lock_released = False
        if robot_controller.execution_lock.locked():
            robot_controller.execution_lock.release()
            lock_released = True
        # Only clear is_executing and advance queue if WE held the lock.
        # If stop_motion() already released it, a new command may have started.
        if lock_released:
            with robot_controller.lock:
                robot_controller.is_executing = False
            robot_controller.motion_queue.mark_current_complete()
            _process_next_queued_task(robot_controller)


def _process_next_queued_task(robot_controller):
    """Process the next task from the motion queue."""
    task = robot_controller.motion_queue.get_next_task()
    if task is not None:
        robot_controller.get_logger().info(f'[Queue] Executing queued task #{task["id"]}')
        try:
            # Execute the queued task
            task['function'](*task['args'], **task['kwargs'])
        except Exception as e:
            robot_controller.get_logger().error(f'[Queue] Failed to execute queued task: {e}')
            with robot_controller.lock:
                robot_controller.is_executing = False
            # Try next task
            _process_next_queued_task(robot_controller)
    else:
        robot_controller.get_logger().info('[Queue] No more queued tasks')
