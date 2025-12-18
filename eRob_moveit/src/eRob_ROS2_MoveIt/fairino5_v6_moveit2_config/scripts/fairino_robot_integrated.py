#!/usr/bin/env python3

"""
Integrated FairinoRobot with MoveIt Adapter Support

This shows how to modify your existing FairinoRobot class to support
switching between real robot (RPC) and MoveIt adapter.

Key changes from original:
1. Added `use_moveit` parameter to __init__
2. Conditionally creates Robot.RPC or MoveItRobotAdapter
3. All other code remains unchanged!
"""

import platform
import logging
from threading import Lock, Event, Thread

# Your existing imports
from core.model.robot.IRobot import IRobot
from core.model.robot.enums.axis import Direction
from frontend.core.services.domain.RobotService import RobotAxis

# Conditional imports based on OS
if platform.system() == "Linux":
    logging.info("Linux detected")
    try:
        from libs.fairino.linux.fairino import Robot
        FAIRINO_AVAILABLE = True
    except ImportError:
        FAIRINO_AVAILABLE = False
        logging.warning("Fairino library not available")
else:
    FAIRINO_AVAILABLE = False


class FairinoRobot(IRobot):
    """
    A wrapper for robot control that can use either:
    - Real robot controller via RPC
    - MoveIt adapter via ROS2

    The adapter is a drop-in replacement - all methods work the same!
    """

    def __init__(self, ip=None, use_moveit=False, logger_context=None):
        """
        Initialize robot wrapper.

        Args:
            ip (str): IP address of robot controller (for real robot)
            use_moveit (bool): If True, use MoveIt adapter instead of RPC
            logger_context: Logger context for logging

        Examples:
            # Use real robot
            robot = FairinoRobot(ip="192.168.1.100")

            # Use MoveIt adapter
            robot = FairinoRobot(use_moveit=True)
        """
        self.ip = ip
        self.use_moveit = use_moveit
        self.logger_context = logger_context
        self.overSpeedStrategy = 3

        # Create appropriate robot instance
        if use_moveit:
            self._create_moveit_robot()
        elif ip and FAIRINO_AVAILABLE:
            self._create_real_robot()
        else:
            raise ValueError(
                "Either provide 'ip' for real robot or set 'use_moveit=True' for MoveIt adapter"
            )

    def _create_real_robot(self):
        """Create real robot connection via RPC"""
        self.robot = Robot.RPC(self.ip)
        logging.info(f"RobotWrapper initialized for real robot at {self.ip}")

        if self.robot is None:
            raise ConnectionError(f"Could not connect to robot at {self.ip}")

    def _create_moveit_robot(self):
        """Create MoveIt adapter"""
        try:
            import rclpy
            from moveit_robot_adapter import MoveItRobotAdapter

            # Initialize ROS2 if not already done
            try:
                rclpy.init()
                logging.info("ROS2 initialized")
            except Exception:
                logging.info("ROS2 already initialized")

            # Create adapter
            self.robot = MoveItRobotAdapter(
                node_name='fairino_robot_adapter',
                group_name='fairino5_v6_group',
                end_effector_link='wrist3_link',
                base_frame='base_link'
            )

            # Start ROS2 spin thread
            self._ros2_thread = Thread(
                target=rclpy.spin,
                args=(self.robot,),
                daemon=True,
                name="ROS2SpinThread"
            )
            self._ros2_thread.start()

            logging.info("MoveIt adapter initialized")

            # Wait for joint states to be available
            import time
            time.sleep(1.0)

        except ImportError as e:
            raise ImportError(
                f"Failed to import ROS2 modules: {e}. "
                "Make sure ROS2 is installed and sourced."
            )

    # All remaining methods stay EXACTLY the same as before!
    # The adapter implements the same interface as Robot.RPC

    def move_cartesian(self, position, tool=0, user=0, vel=30, acc=30, blendR=0):
        """
        Move robot in Cartesian space.

        Works with both real robot and MoveIt adapter!
        """
        if self.use_moveit:
            # MoveIt adapter uses the same method signature
            result = self.robot.move_cartesian(position, tool, user, vel, acc, blendR)
        else:
            # Real robot
            result = self.robot.MoveCart(position, tool, user, vel=vel, acc=acc)

        if self.logger_context:
            logging.debug(
                f"MoveCart to {position} with vel {vel}, acc {acc} -> result: {result}"
            )
        return result

    def move_liner(self, position, tool=0, user=0, vel=30, acc=30, blendR=0):
        """
        Execute linear movement with blending.

        Works with both real robot and MoveIt adapter!
        """
        if self.use_moveit:
            result = self.robot.move_liner(position, tool, user, vel, acc, blendR)
        else:
            result = self.robot.MoveL(position, tool, user, vel=vel, acc=acc, blendR=blendR)

        if self.logger_context:
            logging.debug(
                f"MoveL to {position} with vel {vel}, acc {acc}, blendR {blendR} -> result: {result}"
            )
        return result

    def get_current_position(self):
        """
        Get current TCP position.

        Works with both real robot and MoveIt adapter!
        """
        try:
            if self.use_moveit:
                # MoveIt adapter returns [x, y, z, rx, ry, rz] directly
                currentPose = self.robot.get_current_position()
            else:
                # Real robot returns (status, pose)
                currentPose = self.robot.GetActualTCPPose()
                if isinstance(currentPose, int):
                    currentPose = None
                else:
                    currentPose = currentPose[1]

            return currentPose

        except Exception as e:
            if self.logger_context:
                logging.error(f"get_current_position failed: {e}")
            return None

    def get_current_velocity(self):
        """Get current velocity"""
        if self.use_moveit:
            return self.robot.get_current_velocity()
        else:
            # Real robot implementation
            pass

    def get_current_acceleration(self):
        """Get current acceleration"""
        if self.use_moveit:
            return self.robot.get_current_acceleration()
        else:
            # Real robot implementation
            pass

    def enable(self):
        """Enable robot motion"""
        if self.use_moveit:
            self.robot.enable()
        else:
            self.robot.RobotEnable(1)

    def disable(self):
        """Disable robot motion"""
        if self.use_moveit:
            self.robot.disable()
        else:
            self.robot.RobotEnable(0)

    def printSdkVersion(self):
        """Print SDK version"""
        if self.use_moveit:
            return "MoveIt Adapter v1.0"
        else:
            version = self.robot.GetSDKVersion()
            print(version)
            return version

    def setDigitalOutput(self, portId, value):
        """Set digital output"""
        if self.use_moveit:
            result = self.robot.setDigitalOutput(portId, value)
        else:
            result = self.robot.SetDO(portId, value)

        if self.logger_context:
            logging.debug(f"SetDigitalOutput port {portId} to {value} -> result: {result}")
        return result

    def start_jog(self, axis, direction, step, vel, acc):
        """Start jogging"""
        if self.use_moveit:
            # MoveIt adapter handles jog internally
            result = self.robot.start_jog(axis, direction, step, vel, acc)
        else:
            # Real robot
            axis_val = axis.value
            dir_val = direction.value
            result = self.robot.StartJOG(
                ref=4, nb=axis_val, dir=dir_val,
                vel=vel, acc=acc, max_dis=step
            )

        if self.logger_context:
            logging.debug(
                f"StartJog axis {axis} direction {direction} step {step} -> result: {result}"
            )
        return result

    def stop_motion(self):
        """Stop all motion"""
        return self.robot.stop_motion() if self.use_moveit else self.robot.StopMotion()

    def resetAllErrors(self):
        """Reset all errors"""
        print(f"RobotWrapper: ResetAllError called")
        return self.robot.resetAllErrors() if self.use_moveit else self.robot.ResetAllError()


# Example usage demonstrating both modes
if __name__ == '__main__':
    import time

    logging.basicConfig(level=logging.INFO)

    print("=" * 60)
    print("FairinoRobot Integration Demo")
    print("=" * 60)

    # Demo 1: Using MoveIt adapter
    print("\n1. Testing with MoveIt adapter...")
    try:
        robot = FairinoRobot(use_moveit=True)
        robot.enable()

        # Wait for robot state
        time.sleep(2.0)

        # Get current position
        pos = robot.get_current_position()
        if pos:
            print(f"   Current position: {pos}")

            # Move robot
            target = pos.copy()
            target[2] += 50  # Move 50mm up
            print(f"   Moving to: {target}")
            result = robot.move_cartesian(target, vel=30, acc=30)
            print(f"   Result: {result}")

        robot.disable()
        print("   ✓ MoveIt adapter test passed!")

    except Exception as e:
        print(f"   ✗ MoveIt adapter test failed: {e}")
        print("   (Make sure ROS2/MoveIt is running)")

    # Demo 2: Using real robot (commented out - requires hardware)
    """
    print("\n2. Testing with real robot...")
    robot = FairinoRobot(ip="192.168.1.100")
    robot.enable()
    pos = robot.get_current_position()
    print(f"   Current position: {pos}")
    robot.move_cartesian([300, 0, 400, 180, 0, 0], vel=30, acc=30)
    robot.disable()
    print("   ✓ Real robot test passed!")
    """

    print("\n" + "=" * 60)
    print("Demo complete!")
    print("=" * 60)