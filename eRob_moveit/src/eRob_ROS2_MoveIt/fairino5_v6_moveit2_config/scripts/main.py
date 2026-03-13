#!/usr/bin/env python3
import sys

from threading import Thread

import rclpy
from PyQt6.QtWidgets import QApplication
import subprocess

from rclpy.node import Node

from utils.work_object import WorkObject
from monitor_window import MonitorWindow
from robot_controller import RobotController
from fairino_ros2_robot import FairinoRos2Robot
from rest_server import start_rest_server
def ros_spin_thread(node):
    rclpy.spin(node)

def open_rest_logs_terminal():
    subprocess.Popen([
        "gnome-terminal",
        "--",
        "bash",
        "-c",
        "echo 'Fairino REST Server Logs'; echo; tail -f /tmp/fairino_rest_server.log"
    ])


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

    rest_thread = Thread(
        target=start_rest_server,
        kwargs=dict(
            robot=fairino_robot,
            node=ros_node,
            start_ros=False,  # reuse existing ROS node # 👈 VERY IMPORTANT
            host="0.0.0.0",
            port=5000,
        ),
        daemon=True,
    )
    rest_thread.start()



    # win = MonitorWindow(ros_node,fairino_robot)
    # ros_node.ui_callback = win.ros_update

    # win.show()
    sys.exit(app.exec())

if __name__ == '__main__':
    main()