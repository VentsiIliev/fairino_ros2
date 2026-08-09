#!/usr/bin/env python3
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped


class FakeStateWaiter(Node):
    def __init__(self):
        super().__init__("zeroerr_fake_state_waiter")
        self.got_joint_state = False
        self.got_cartesian = False

        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)
        self.create_subscription(PoseStamped, "/cartesian_position", self._on_cartesian, 10)

    def _on_joint_state(self, msg):
        if len(msg.name) >= 6 and len(msg.position) >= 6:
            self.got_joint_state = True

    def _on_cartesian(self, _msg):
        self.got_cartesian = True


def main():
    rclpy.init()
    node = FakeStateWaiter()
    deadline = time.monotonic() + 15.0

    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.got_joint_state and node.got_cartesian:
                print("[ZEROERR] Fake joint and Cartesian state are available; starting runtime", flush=True)
                return 0

        missing = []
        if not node.got_joint_state:
            missing.append("/joint_states")
        if not node.got_cartesian:
            missing.append("/cartesian_position")
        print(
            "[ZEROERR] Timed out waiting for fake state: " + ", ".join(missing),
            file=sys.stderr,
            flush=True,
        )
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
