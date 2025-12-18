#!/usr/bin/env python3

"""
Robot Factory - Creates robot instances based on configuration.

This factory allows switching between:
- Real robot (FairinoRobot with RPC)
- MoveIt adapter (MoveItRobotAdapter)
- Test robot (TestRobotWrapper)

Usage:
    from robot_factory import create_robot, RobotType

    # Create MoveIt adapter
    robot = create_robot(RobotType.MOVEIT)

    # Or create real robot
    robot = create_robot(RobotType.REAL, ip="192.168.1.100")

    # Or create test robot
    robot = create_robot(RobotType.TEST)
"""

import logging
from enum import Enum
from threading import Thread


class RobotType(Enum):
    """Robot type selection"""
    REAL = "real"       # Fairino Robot via RPC
    MOVEIT = "moveit"   # MoveIt adapter
    TEST = "test"       # Test mock robot


# Global ROS2 spin thread for MoveIt adapter
_ros2_spin_thread = None
_rclpy_initialized = False


def create_robot(robot_type: RobotType = RobotType.MOVEIT, **kwargs):
    """
    Factory function to create robot instances.

    Args:
        robot_type (RobotType): Type of robot to create
        **kwargs: Additional arguments passed to robot constructor
            - For REAL: ip (str) required
            - For MOVEIT: node_name, group_name, etc. optional
            - For TEST: no arguments needed

    Returns:
        IRobot: Robot instance implementing the IRobot interface

    Examples:
        # MoveIt adapter
        robot = create_robot(RobotType.MOVEIT)

        # Real robot
        robot = create_robot(RobotType.REAL, ip="192.168.1.100")

        # Test robot
        robot = create_robot(RobotType.TEST)
    """
    global _ros2_spin_thread, _rclpy_initialized

    if robot_type == RobotType.MOVEIT:
        # Import ROS2 modules
        try:
            import rclpy
            from moveit_robot_adapter import MoveItRobotAdapter
        except ImportError as e:
            logging.error(f"Failed to import ROS2 modules: {e}")
            logging.error("Make sure ROS2 is installed and sourced")
            raise

        # Initialize ROS2 if not already done
        if not _rclpy_initialized:
            try:
                rclpy.init()
                _rclpy_initialized = True
                logging.info("ROS2 initialized")
            except Exception as e:
                logging.warning(f"ROS2 already initialized or init failed: {e}")

        # Create MoveIt adapter
        robot = MoveItRobotAdapter(**kwargs)

        # Start spin thread if not already running
        if _ros2_spin_thread is None or not _ros2_spin_thread.is_alive():
            _ros2_spin_thread = Thread(
                target=rclpy.spin,
                args=(robot,),
                daemon=True,
                name="ROS2SpinThread"
            )
            _ros2_spin_thread.start()
            logging.info("Started ROS2 spin thread")

        # Wait for robot state to be available
        import time
        time.sleep(1.0)

        return robot

    elif robot_type == RobotType.REAL:
        # Import real robot
        try:
            from libs.fairino.linux.fairino import Robot
        except ImportError:
            logging.error("Failed to import Fairino robot library")
            raise

        if 'ip' not in kwargs:
            raise ValueError("IP address required for real robot. Use: create_robot(RobotType.REAL, ip='192.168.1.100')")

        # Import from user's code structure
        # Note: This assumes your FairinoRobot class is available
        # You may need to adjust the import path
        try:
            from core.model.robot.FairinoRobot import FairinoRobot
            return FairinoRobot(**kwargs)
        except ImportError:
            # Fall back to creating robot directly
            logging.warning("Could not import FairinoRobot, creating Robot.RPC directly")
            ip = kwargs.get('ip')
            return Robot.RPC(ip)

    elif robot_type == RobotType.TEST:
        # Import test robot
        try:
            from core.model.robot.TestRobot import TestRobotWrapper
            return TestRobotWrapper()
        except ImportError:
            logging.warning("Could not import TestRobotWrapper, using inline version")

            # Inline test robot
            class SimpleTestRobot:
                def __init__(self):
                    print("⚙️  Test Robot initialized (mock)")

                def move_cartesian(self, *args, **kwargs):
                    print(f"[MOCK] move_cartesian called")
                    return [0, 'Success']

                def move_liner(self, *args, **kwargs):
                    print(f"[MOCK] move_liner called")
                    return [0, 'Success']

                def get_current_position(self):
                    return [300.0, 0.0, 400.0, 180.0, 0.0, 0.0]

                def get_current_velocity(self):
                    return [0.0] * 6

                def get_current_acceleration(self):
                    return None

                def enable(self):
                    print("[MOCK] Robot enabled")

                def disable(self):
                    print("[MOCK] Robot disabled")

                def setDigitalOutput(self, *args, **kwargs):
                    print(f"[MOCK] setDigitalOutput called")
                    return [0, 'Success']

                def start_jog(self, *args, **kwargs):
                    print(f"[MOCK] start_jog called")
                    return [0, 'Success']

                def stop_motion(self):
                    print("[MOCK] stop_motion called")
                    return [0, 'Success']

                def resetAllErrors(self):
                    print("[MOCK] resetAllErrors called")
                    return [0, 'Success']

            return SimpleTestRobot()

    else:
        raise ValueError(f"Unknown robot type: {robot_type}")


def shutdown_robot(robot):
    """
    Properly shutdown robot and cleanup resources.

    Args:
        robot: Robot instance to shutdown
    """
    global _rclpy_initialized

    # Check if it's a MoveIt adapter (has destroy_node method)
    if hasattr(robot, 'destroy_node'):
        try:
            robot.disable()
            robot.destroy_node()
            logging.info("MoveIt adapter destroyed")
        except Exception as e:
            logging.error(f"Error destroying MoveIt adapter: {e}")

        # Shutdown ROS2 if initialized
        if _rclpy_initialized:
            try:
                import rclpy
                rclpy.shutdown()
                _rclpy_initialized = False
                logging.info("ROS2 shutdown")
            except Exception as e:
                logging.error(f"Error shutting down ROS2: {e}")
    else:
        # Other robot types - just disable
        if hasattr(robot, 'disable'):
            robot.disable()


# Example usage and testing
if __name__ == '__main__':
    import time

    # Setup logging
    logging.basicConfig(level=logging.INFO)

    print("=" * 60)
    print("Robot Factory Test")
    print("=" * 60)

    # Test 1: Create test robot
    print("\n1. Creating test robot...")
    test_robot = create_robot(RobotType.TEST)
    test_robot.enable()
    pos = test_robot.get_current_position()
    print(f"   Position: {pos}")
    test_robot.move_cartesian([350, 0, 400, 180, 0, 0], vel=30, acc=30)
    shutdown_robot(test_robot)

    # Test 2: Create MoveIt adapter (requires ROS2)
    print("\n2. Creating MoveIt adapter...")
    try:
        moveit_robot = create_robot(RobotType.MOVEIT)
        moveit_robot.enable()

        time.sleep(2.0)  # Wait for joint states

        pos = moveit_robot.get_current_position()
        if pos:
            print(f"   Current position: {pos}")

            # Move robot
            target = pos.copy()
            target[2] += 50  # Move 50mm up
            print(f"   Moving to: {target}")
            result = moveit_robot.move_cartesian(target, vel=30, acc=30)
            print(f"   Result: {result}")

        shutdown_robot(moveit_robot)
    except Exception as e:
        print(f"   Error: {e}")
        print("   (This is expected if ROS2/MoveIt is not running)")

    print("\n" + "=" * 60)
    print("Factory test complete!")
    print("=" * 60)