#!/usr/bin/env python3
"""
Example: How to use FairinoRos2Client as a drop-in replacement for FairinoRobot

This shows three ways to integrate the ROS2 client into your existing code.
"""

# ============================================================
# Method 1: Direct Import (Recommended)
# ============================================================
from fairino_ros2_client import FairinoRos2Client

# Create robot instance (connects to ROS2 bridge)
robot = FairinoRos2Client(server_url="http://localhost:5000")

# Now use it exactly like FairinoRobot
position = robot.get_current_position()
print(f"Current position: {position}")

robot.move_cartesian([100, 200, 300, 180, 0, 0], vel=30, acc=30)


# ============================================================
# Method 2: Drop-in Replacement (No Code Changes)
# ============================================================
# If you have existing code using FairinoRobot, you can replace it:

# OLD CODE:
# from your_module import FairinoRobot
# robot = FairinoRobot(ip="192.168.1.100")

# NEW CODE (just change the import):
from fairino_ros2_client import FairinoRobot

robot = FairinoRobot(ip="localhost:5000")  # Now points to bridge server

# Everything else stays the same!
robot.move_cartesian([100, 200, 300, 180, 0, 0], vel=30, acc=30)
robot.move_liner([150, 200, 300, 180, 0, 0], vel=30, acc=30, blendR=0)


# ============================================================
# Method 3: Dynamic Selection (Real Robot vs ROS2)
# ============================================================
import os

# Choose robot type based on environment variable or config
USE_ROS2 = os.getenv("USE_ROS2_ROBOT", "false").lower() == "true"

if USE_ROS2:
    from fairino_ros2_client import FairinoRos2Client as RobotClass
    robot = RobotClass(server_url="http://localhost:5000")
    print("Using ROS2 Robot")
else:
    from your_module import FairinoRobot as RobotClass
    robot = RobotClass(ip="192.168.1.100")
    print("Using Real Robot")

# Now use robot with the same interface
robot.move_cartesian([100, 200, 300, 180, 0, 0], vel=30, acc=30)


# ============================================================
# Method 4: Abstract Factory Pattern (Best for Large Projects)
# ============================================================
class RobotFactory:
    """Factory to create the appropriate robot instance"""

    @staticmethod
    def create_robot(robot_type="real", **kwargs):
        """
        Create a robot instance.

        Args:
            robot_type: "real" for hardware, "ros2" for ROS2/MoveIt, "test" for mock
            **kwargs: Robot-specific parameters

        Returns:
            IRobot implementation
        """
        if robot_type == "ros2":
            from fairino_ros2_client import FairinoRos2Client
            server_url = kwargs.get("server_url", "http://localhost:5000")
            return FairinoRos2Client(server_url=server_url)

        elif robot_type == "real":
            from your_module import FairinoRobot
            ip = kwargs.get("ip", "192.168.1.100")
            return FairinoRobot(ip=ip)

        elif robot_type == "test":
            from your_module import TestRobotWrapper
            return TestRobotWrapper()

        else:
            raise ValueError(f"Unknown robot type: {robot_type}")


# Usage:
robot = RobotFactory.create_robot("ros2", server_url="http://localhost:5000")
robot.move_cartesian([100, 200, 300, 180, 0, 0], vel=30, acc=30)


# ============================================================
# Complete Example with Error Handling
# ============================================================
def main():
    """Complete example with proper error handling"""

    # Initialize robot with fallback
    try:
        robot = FairinoRos2Client(server_url="http://localhost:5000")
        print("✓ Connected to ROS2 bridge")
    except ConnectionError as e:
        print(f"✗ Failed to connect to ROS2 bridge: {e}")
        print("  Make sure bridge server is running:")
        print("  python3 fairino_bridge_server.py")
        return

    # Get current position
    current_pos = robot.get_current_position()
    if current_pos is None:
        print("✗ Failed to get current position")
        return
    print(f"✓ Current position: {current_pos}")

    # Move to new position
    target = [100, 200, 300, 180, 0, 0]
    result = robot.move_cartesian(target, tool=0, user=0, vel=30, acc=30)

    if result == 0:
        print(f"✓ Successfully moved to {target}")
    else:
        print(f"✗ Move failed with error code: {result}")

    # Execute a path
    path = [
        [100, 100, 300],
        [200, 100, 300],
        [200, 200, 300],
        [100, 200, 300]
    ]

    result = robot.execute_path(path, rx=180, ry=0, rz=0, vel=0.5, acc=0.4)

    if result == 0:
        print(f"✓ Successfully executed path with {len(path)} waypoints")
    else:
        print(f"✗ Path execution failed with error code: {result}")

    # Emergency stop example
    # robot.stop_motion()


if __name__ == "__main__":
    main()