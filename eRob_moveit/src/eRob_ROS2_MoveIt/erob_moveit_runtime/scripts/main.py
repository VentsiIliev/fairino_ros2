#!/usr/bin/env python3
import sys
import time
import os

from threading import Thread

import rclpy
import subprocess

from rclpy.node import Node

from utils.work_object import WorkObject
from monitor_window import MonitorWindow
from robot_controller import RobotController
from backend.backend_factory import create_robot_backend
from rest_server import start_rest_server
import config
def ros_spin_thread(node):
    rclpy.spin(node)

def open_rest_logs_terminal():
    subprocess.Popen([
        "gnome-terminal",
        "--",
        "bash",
        "-c",
        f"echo 'Robot REST Server Logs'; echo; tail -f {config.REST_LOG}",
    ])



def main():
    rclpy.init()
    headless = os.environ.get("EROB_RUNTIME_HEADLESS", "0").lower() in {
        "1",
        "true",
        "yes",
        "on",
    }
    app = None
    if not headless:
        from PyQt6.QtWidgets import QApplication
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

    # Load default workobject from per-robot runtime.yaml (DEFAULT_WORKOBJECT: [x,y,z,rx,ry,rz])
    wo_params = config.DEFAULT_WORKOBJECT
    work_object = WorkObject(*wo_params) if any(v != 0 for v in wo_params) else None
    robot_backend = create_robot_backend(node=ros_node, workobject=work_object, ip="192.168.58.2")

    rest_thread = Thread(
        target=start_rest_server,
        kwargs=dict(
            robot=robot_backend,
            node=ros_node,
            start_ros=False,  # reuse existing ROS node # 👈 VERY IMPORTANT
            host="0.0.0.0",
            port=5000,
        ),
        daemon=True,
    )
    rest_thread.start()



    # win = MonitorWindow(ros_node, robot_backend)
    # ros_node.ui_callback = win.ros_update

    # win.show()
    if app is not None:
        sys.exit(app.exec())

    try:
        while rclpy.ok():
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
