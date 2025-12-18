#!/usr/bin/env python3

"""
Example usage of MoveItRobotAdapter as a drop-in replacement for FairinoRobot.

This demonstrates how to use the MoveIt adapter with code designed for FairinoRobot.
"""

import rclpy
from moveit_robot_adapter import MoveItRobotAdapter, Axis, Direction
from threading import Thread
import time


def run_robot_demo(robot):
    """
    Demo function that works with both FairinoRobot and MoveItRobotAdapter.

    Args:
        robot: Any object implementing the IRobot interface
    """
    print("=" * 60)
    print("Robot Demo - Works with FairinoRobot or MoveItRobotAdapter")
    print("=" * 60)

    # Enable robot
    print("\n1. Enabling robot...")
    robot.enable()
    time.sleep(1.0)

    # Get current position
    print("\n2. Getting current position...")
    current_pos = robot.get_current_position()
    if current_pos:
        print(f"   Current: X={current_pos[0]:.1f}mm, Y={current_pos[1]:.1f}mm, "
              f"Z={current_pos[2]:.1f}mm")
        print(f"   Orientation: RX={current_pos[3]:.1f}°, RY={current_pos[4]:.1f}°, "
              f"RZ={current_pos[5]:.1f}°")
    else:
        print("   Position not available yet, waiting...")
        time.sleep(2.0)
        current_pos = robot.get_current_position()

    if not current_pos:
        print("   ERROR: Could not get robot position!")
        return

    # Move to position using move_cartesian
    print("\n3. Moving 50mm up in Z axis...")
    target_pos = current_pos.copy()
    target_pos[2] += 50.0  # Move 50mm up

    result = robot.move_cartesian(
        position=target_pos,
        tool=0,
        user=0,
        vel=30,  # 30% velocity
        acc=30   # 30% acceleration
    )
    print(f"   Move result: {result}")

    # Wait for motion to complete
    time.sleep(3.0)

    # Jog in X direction
    print("\n4. Jogging 10mm in +X direction...")
    result = robot.start_jog(
        axis=Axis.X,
        direction=Direction.PLUS,
        step=10.0,  # 10mm
        vel=20,
        acc=20
    )
    print(f"   Jog result: {result}")

    time.sleep(3.0)

    # Move back to original position
    print("\n5. Returning to original position...")
    result = robot.move_liner(
        position=current_pos,
        tool=0,
        user=0,
        vel=30,
        acc=30,
        blendR=0
    )
    print(f"   Move result: {result}")

    time.sleep(3.0)

    # Get current velocity
    print("\n6. Getting current velocity...")
    velocity = robot.get_current_velocity()
    if velocity:
        print(f"   Joint velocities: {[f'{v:.3f}' for v in velocity]}")
    else:
        print("   Velocity not available")

    print("\n7. Demo complete!")
    print("=" * 60)


def main():
    """Main function demonstrating adapter usage"""

    # Initialize ROS2
    rclpy.init()

    # Create MoveIt adapter instance (drop-in replacement for FairinoRobot)
    print("Creating MoveItRobotAdapter...")
    robot = MoveItRobotAdapter(
        node_name='demo_robot_adapter',
        group_name='fairino5_v6_group',
        end_effector_link='wrist3_link',
        base_frame='base_link'
    )

    # Spin ROS2 node in background thread
    print("Starting ROS2 spin thread...")
    spin_thread = Thread(target=rclpy.spin, args=(robot,), daemon=True)
    spin_thread.start()

    # Wait for joint states to be available
    print("Waiting for robot state...")
    time.sleep(2.0)

    # Run demo
    try:
        run_robot_demo(robot)
    except KeyboardInterrupt:
        print("\nDemo interrupted by user")
    except Exception as e:
        print(f"\nError during demo: {e}")
        import traceback
        traceback.print_exc()

    # Cleanup
    print("\nCleaning up...")
    robot.disable()
    robot.destroy_node()
    rclpy.shutdown()
    print("Done!")


def example_with_existing_code():
    """
    Example showing how to use the adapter with existing FairinoRobot code.

    Just replace:
        robot = FairinoRobot(ip="192.168.1.100")

    With:
        robot = MoveItRobotAdapter()

    Everything else stays the same!
    """

    # Instead of:
    # from fairino_robot import FairinoRobot
    # robot = FairinoRobot(ip="192.168.1.100")

    # Use:
    from moveit_robot_adapter import MoveItRobotAdapter

    rclpy.init()
    robot = MoveItRobotAdapter()

    # Spin in background
    spin_thread = Thread(target=rclpy.spin, args=(robot,), daemon=True)
    spin_thread.start()

    # Your existing robot control code works unchanged!
    robot.enable()

    # Move to position
    position = [300.0, 0.0, 400.0, 180.0, 0.0, 0.0]  # [x, y, z, rx, ry, rz]
    robot.move_cartesian(position, vel=30, acc=30)

    # Linear move with blend radius
    position2 = [350.0, 50.0, 400.0, 180.0, 0.0, 0.0]
    robot.move_liner(position2, vel=30, acc=30, blendR=5.0)

    # Jog
    robot.start_jog(Axis.Z, Direction.PLUS, step=10.0, vel=20, acc=20)

    # Get position
    current = robot.get_current_position()
    print(f"Current position: {current}")

    # Cleanup
    robot.disable()
    robot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    # Run the demo
    main()

    # Uncomment to see the integration example:
    # example_with_existing_code()