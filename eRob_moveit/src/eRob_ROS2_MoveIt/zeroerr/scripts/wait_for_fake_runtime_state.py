#!/usr/bin/env python3
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateWaiter(Node):
    def __init__(self):
        super().__init__("zeroerr_fake_joint_state_waiter")
        self.got_joint_state = False
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)

    def _on_joint_state(self, msg):
        if len(msg.name) >= 6 and len(msg.position) >= 6:
            self.got_joint_state = True


def main():
    rclpy.init()
    node = JointStateWaiter()
    deadline = time.monotonic() + 15.0

    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.got_joint_state:
                print("[ZEROERR] Fake joint state is available", flush=True)
                return 0

        print("[ZEROERR] Timed out waiting for /joint_states", file=sys.stderr, flush=True)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
