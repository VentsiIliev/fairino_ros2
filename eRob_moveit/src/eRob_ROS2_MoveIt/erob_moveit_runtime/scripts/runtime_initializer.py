#!/usr/bin/env python3
from __future__ import annotations

from threading import Thread

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor

from backend.backend_factory import create_robot_backend
import config
from robot_controller import RobotController
from utils.work_object import WorkObject


def ros_spin_thread(node: RobotController) -> None:
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except ExternalShutdownException:
        pass
    finally:
        executor.remove_node(node)
        executor.shutdown()


def initialize_robot_runtime(update_status=None):
    def status(phase: str, message: str) -> None:
        if update_status is not None:
            update_status(phase, message)

    status("initializing_ros", "ROS client library is initializing")
    if not rclpy.ok():
        rclpy.init()

    status("constructing_robot_controller", "Robot controller is initializing")
    ros_node = RobotController()

    # Spin before backend setup so timers and TF callbacks are serviced during startup.
    status("spinning_ros", "ROS node is spinning")
    thread = Thread(target=ros_spin_thread, args=(ros_node,), daemon=True)
    thread.start()

    status("creating_backend", "Robot backend is being created")
    wo_params = config.DEFAULT_WORKOBJECT
    work_object = WorkObject(*wo_params) if any(v != 0 for v in wo_params) else None
    robot_backend = create_robot_backend(
        node=ros_node,
        workobject=work_object,
        ip=getattr(config, "ROBOT_IP", "192.168.58.2"),
    )
    ros_node.robot = robot_backend

    return robot_backend, ros_node
