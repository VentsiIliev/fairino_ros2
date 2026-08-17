#!/usr/bin/env python3
"""Enable all twin CiA402 drives through the shared command controllers."""

import argparse
import time

import rclpy
from control_msgs.msg import DynamicJointState
from controller_manager_msgs.srv import ListControllers, SwitchController
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


JOINTS = [f"robot{robot}_Joint_{joint}" for robot in (1, 2) for joint in range(1, 7)]
ENABLE_CONTROLLER = "drive_enable_set_controller"
DISABLE_CONTROLLER = "drive_disable_set_controller"


class TwinDriveEnable(Node):
    def __init__(self):
        print("[twin_drive_enable] constructing ROS node", flush=True)
        super().__init__("twin_drive_enable")
        print("[twin_drive_enable] ROS node constructed", flush=True)
        self._statuswords = {}
        self._enable_pub = self.create_publisher(
            Float64MultiArray, "/drive_enable_set_controller/commands", 10
        )
        print("[twin_drive_enable] enable publisher created", flush=True)
        self._disable_pub = self.create_publisher(
            Float64MultiArray, "/drive_disable_set_controller/commands", 10
        )
        print("[twin_drive_enable] disable publisher created", flush=True)
        self._list_client = self.create_client(ListControllers, "/controller_manager/list_controllers")
        print("[twin_drive_enable] list service client created", flush=True)
        self._switch_client = self.create_client(SwitchController, "/controller_manager/switch_controller")
        print("[twin_drive_enable] switch service client created", flush=True)
        self.create_subscription(DynamicJointState, "/dynamic_joint_states", self._state_cb, 10)
        print("[twin_drive_enable] dynamic state subscription created", flush=True)

    def _state_cb(self, msg):
        for joint_name, states in zip(msg.joint_names, msg.interface_values):
            for interface_name, value in zip(states.interface_names, states.values):
                if interface_name == "statusword":
                    self._statuswords[joint_name] = int(round(value))

    def _spin_until(self, future, timeout_s):
        end = time.monotonic() + timeout_s
        while rclpy.ok() and not future.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        return future.result() if future.done() else None

    def _controllers(self):
        self.get_logger().info("Waiting for /controller_manager/list_controllers")
        if not self._list_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warning("/controller_manager/list_controllers is not available")
            return None
        self.get_logger().info("Calling /controller_manager/list_controllers")
        response = self._spin_until(self._list_client.call_async(ListControllers.Request()), 3.0)
        if response is None:
            self.get_logger().warning("Timed out calling /controller_manager/list_controllers")
            return None
        return {controller.name: controller.state for controller in response.controller}

    def _switch(self, activate, deactivate):
        self.get_logger().info(
            f"Switching controllers: activate={activate}, deactivate={deactivate}"
        )
        if not self._switch_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("/controller_manager/switch_controller is not available")
            return False
        request = SwitchController.Request()
        request.activate_controllers = activate
        request.deactivate_controllers = deactivate
        request.strictness = 2
        request.activate_asap = True
        response = self._spin_until(self._switch_client.call_async(request), 3.0)
        if response is None:
            self.get_logger().error("Timed out calling /controller_manager/switch_controller")
            return False
        return bool(response and response.ok)

    def run(self):
        self.get_logger().info("Twin drive enable helper started; waiting for command controllers")
        deadline = time.monotonic() + 30.0
        states = None
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            states = self._controllers()
            if states and ENABLE_CONTROLLER in states and DISABLE_CONTROLLER in states:
                break
            time.sleep(0.25)
        if not states or ENABLE_CONTROLLER not in states or DISABLE_CONTROLLER not in states:
            self.get_logger().error("Twin drive command controllers were not loaded")
            return 1

        self.get_logger().info(
            f"Command controllers loaded: {ENABLE_CONTROLLER}={states[ENABLE_CONTROLLER]}, "
            f"{DISABLE_CONTROLLER}={states[DISABLE_CONTROLLER]}"
        )

        if not self._switch([ENABLE_CONTROLLER, DISABLE_CONTROLLER], []):
            self.get_logger().error("Could not activate twin drive command controllers")
            return 1
        self.get_logger().info("Drive command controllers active; sending enable pulse")

        enable = Float64MultiArray(data=[1.0] * len(JOINTS))
        clear = Float64MultiArray(data=[0.0] * len(JOINTS))
        self._enable_pub.publish(enable)
        self._disable_pub.publish(clear)
        time.sleep(0.1)
        self._enable_pub.publish(clear)
        self._disable_pub.publish(clear)
        time.sleep(0.1)
        self._switch([], [ENABLE_CONTROLLER, DISABLE_CONTROLLER])
        self.get_logger().info("Enable pulse sent; verifying statuswords")

        verify_deadline = time.monotonic() + 5.0
        while rclpy.ok() and time.monotonic() < verify_deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if all((self._statuswords.get(joint, 0) & 0x6F) == 0x27 for joint in JOINTS):
                self.get_logger().info("All 12 twin drives report Operation Enabled")
                return 0

        missing = sorted(set(JOINTS) - set(self._statuswords))
        self.get_logger().error(
            "Enable command sent, but not all drives reached Operation Enabled: "
            + str({joint: self._statuswords.get(joint, 0) for joint in JOINTS})
            + (f"; missing statuswords: {missing}" if missing else "")
        )
        return 1


def main():
    print("[twin_drive_enable] process entered main()", flush=True)
    parser = argparse.ArgumentParser(description="Enable all twelve twin CiA402 drives")
    parser.parse_known_args()
    print("[twin_drive_enable] parsed arguments", flush=True)
    rclpy.init()
    print("[twin_drive_enable] rclpy initialized", flush=True)
    node = TwinDriveEnable()
    try:
        print("[twin_drive_enable] node created; starting enable sequence", flush=True)
        return node.run()
    except Exception as exc:
        node.get_logger().exception(f"Twin drive enable helper failed: {exc}")
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
