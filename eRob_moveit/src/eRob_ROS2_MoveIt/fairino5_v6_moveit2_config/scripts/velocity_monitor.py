#!/usr/bin/env python3

import sys
from threading import Thread

import rclpy
from PyQt6.QtWidgets import QApplication

from rclpy.node import Node

from monitor_window import MonitorWindow
from robot_controller import RobotController, FairinoRos2Robot, WorkObject


def ros_spin_thread(node):
    rclpy.spin(node)

def main():
    rclpy.init()
    app = QApplication(sys.argv)

    ros_node = RobotController()

    # Start spinning the node BEFORE waiting for monitor
    # This allows timer callbacks and TF transforms to be processed
    thread = Thread(target=ros_spin_thread, args=(ros_node,), daemon=True)
    thread.start()
    #
    # # Wait until monitor is initialized
    # if ros_node.wait_for_monitor(timeout_sec=5.0):
    #     ros_node.set_tool("TOOL_1")
    # else:
    #     ros_node.get_logger().error("Cannot set tool, RobotMonitor not initialized")

    # Define a work object (offset from base frame)
    work_object = WorkObject(x=0, y=0, z=0, rx=0, ry=0, rz=-10)
    fairino_robot = FairinoRos2Robot("192.168.58.2", ros_node, work_object)
    win = MonitorWindow(ros_node,fairino_robot)
    ros_node.ui_callback = win.ros_update

    win.show()
    sys.exit(app.exec())

if __name__ == '__main__':
    main()