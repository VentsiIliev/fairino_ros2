#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import sys


class CollisionTest(Node):
    def __init__(self):
        super().__init__('collision_test')

        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.collision_sub = self.create_subscription(
            Bool,
            '/collision_detected',
            self.collision_callback,
            10
        )

        self.joint_names = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']
        self.thresholds = [0.2, 0.2, 0.2, 0.15, 0.15, 0.1]

        print("\n" + "="*70)
        print("COLLISION DETECTION TEST - HIGH SENSITIVITY MODE")
        print("="*70)
        print("\nThresholds (Nm):")
        for name, thresh in zip(self.joint_names, self.thresholds):
            print(f"  {name}: {thresh:.1f} Nm")
        print("\nMonitoring effort values... (Press Ctrl+C to exit)")
        print("Try gently pushing joints during motion to test detection\n")

    def joint_state_callback(self, msg):
        efforts = {}
        for i, name in enumerate(msg.name):
            if name in self.joint_names and i < len(msg.effort):
                efforts[name] = abs(msg.effort[i])

        if len(efforts) == 6:
            output = "Effort: "
            warnings = []
            for name in self.joint_names:
                idx = self.joint_names.index(name)
                effort = efforts[name]
                threshold = self.thresholds[idx]

                if effort > threshold * 0.8:
                    warnings.append(f"{name}:{effort:.3f}")
                    output += f"\033[93m{name}:{effort:.3f}\033[0m  "
                elif effort > threshold * 0.5:
                    output += f"\033[33m{name}:{effort:.3f}\033[0m  "
                else:
                    output += f"{name}:{effort:.3f}  "

            print(f"\r{output}", end='', flush=True)

            if warnings:
                print(f"\n  ⚠ WARNING: Near threshold: {', '.join(warnings)}")

    def collision_callback(self, msg):
        if msg.data:
            print("\n" + "!"*70)
            print("!!! COLLISION DETECTED - ROBOT STOPPED !!!")
            print("!"*70 + "\n")
        else:
            print("\n✓ Collision reset - system ready\n")


def main(args=None):
    rclpy.init(args=args)
    node = CollisionTest()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nTest stopped.\n")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

