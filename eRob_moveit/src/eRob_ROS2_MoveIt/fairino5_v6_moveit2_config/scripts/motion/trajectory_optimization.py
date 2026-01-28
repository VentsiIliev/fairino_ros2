# Time Optimal Trajectory Generation
from fairino5_v6_moveit2_config.srv import ApplyIPP

def apply_ipp_totg(robot_controller, trajectory, vel_scaling=0.6, acc_scaling=0.4, callback=None):
    """Call the IPP service to apply TOTG (ASYNC).

    Args:
        trajectory: RobotTrajectory to time-parameterize
        vel_scaling: Velocity scaling factor [0.0-1.0]
        acc_scaling: Acceleration scaling factor [0.0-1.0]
        callback: Function to call with (trajectory_or_None) when done
    """
    robot_controller.get_logger().info('[TOTG] Checking if IPP service is available...')

    if not robot_controller.ipp_client.wait_for_service(timeout_sec=5.0):
        robot_controller.get_logger().error('[TOTG] ✗ IPP service /apply_ipp NOT available after 5s!')
        robot_controller.get_logger().error('[TOTG]    Is ipp_helper node running? Check: ros2 node list | grep ipp')
        if callback:
            callback(None)
        return

    robot_controller.get_logger().info('[TOTG] ✓ IPP service is available')

    request = ApplyIPP.Request()
    request.trajectory = trajectory.joint_trajectory
    request.max_velocity_scaling = float(vel_scaling)
    request.max_acceleration_scaling = float(acc_scaling)

    robot_controller.get_logger().info(
        f'[TOTG] Requesting time-optimal parameterization (vel={vel_scaling}, acc={acc_scaling})')

    # Call ASYNC to avoid blocking the executor
    future = robot_controller.ipp_client.call_async(request)

    # Handle response when ready
    def handle_ipp_response(fut):
        try:
            response = fut.result()

            if response is None:
                robot_controller.get_logger().error('[TOTG] ✗ Response is None')
                if callback:
                    callback(None)
                return

            # ✅ FIX: Extract JointTrajectory from RobotTrajectory response
            if not hasattr(response, 'trajectory'):
                robot_controller.get_logger().error('[TOTG] ✗ Response has no trajectory attribute')
                if callback:
                    callback(None)
                return

            # Response is RobotTrajectory, extract joint_trajectory
            if not hasattr(response.trajectory, 'joint_trajectory'):
                robot_controller.get_logger().error('[TOTG] ✗ Response.trajectory has no joint_trajectory attribute')
                if callback:
                    callback(None)
                return

            joint_traj = response.trajectory.joint_trajectory
            num_points = len(joint_traj.points)

            if num_points == 0:
                robot_controller.get_logger().error('[TOTG] ✗ Empty trajectory - TOTG failed')
                if callback:
                    callback(None)
                return

            robot_controller.get_logger().info(
                f'[TOTG] ✓ Generated {num_points} time-parameterized points')

            # Validate that timestamps are present
            has_timestamps = False
            for pt in joint_traj.points:
                if pt.time_from_start.sec > 0 or pt.time_from_start.nanosec > 0:
                    has_timestamps = True
                    break

            if not has_timestamps:
                robot_controller.get_logger().error('[TOTG] ✗ Response has no timestamps - INVALID trajectory')
                if callback:
                    callback(None)
                return

            # ✅ Success: Update trajectory with time-parameterized result
            trajectory.joint_trajectory = joint_traj
            if callback:
                callback(trajectory)

        except Exception as e:
            robot_controller.get_logger().error(f'[TOTG] ✗ Service call failed: {e}')
            import traceback
            robot_controller.get_logger().error(f'[TOTG] Traceback: {traceback.format_exc()}')
            if callback:
                callback(None)

    future.add_done_callback(handle_ipp_response)
