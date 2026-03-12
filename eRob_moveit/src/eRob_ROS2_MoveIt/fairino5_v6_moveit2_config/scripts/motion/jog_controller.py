def jog_cartesian(robot_controller, dx_mm=0.0, dy_mm=0.0, dz_mm=0.0, vel_scale=0.1, acc_scale=0.1):
    """
    Jog the robot by a small step in BASE frame.
    dx_mm, dy_mm, dz_mm are relative increments.

    Returns:
        int: 0 on success, -1 if robot is busy, -2 if service unavailable, -3 if safety rejected, -4 if no position data
    """
    robot_controller.safety_manager.force_update()

    # We need a valid current Cartesian position
    if robot_controller.prev_cartesian is None:
        robot_controller.get_logger().warning('[JOG] No Cartesian position available for jog')
        return -4

    # Current TCP position (already in mm from robot_monitor.py)
    x = robot_controller.prev_cartesian[0] + dx_mm
    y = robot_controller.prev_cartesian[1] + dy_mm
    z = robot_controller.prev_cartesian[2] + dz_mm
    rx = robot_controller.prev_cartesian[3]
    ry = robot_controller.prev_cartesian[4]
    rz = robot_controller.prev_cartesian[5]

    # Pre-validate jog target safety
    is_safe, msg = robot_controller.safety_manager.check_position_safety(
        x / 1000.0, y / 1000.0, z / 1000.0  # Convert mm to m
    )
    if not is_safe:
        robot_controller.get_logger().error(f'[SAFETY] Jog target rejected: {msg}')
        return -3

    return robot_controller.send_cartesian_goal(x, y, z, rx, ry, rz, vel_scale, acc_scale)
