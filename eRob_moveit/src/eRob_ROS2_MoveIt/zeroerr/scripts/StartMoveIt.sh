#!/bin/bash
cd ros2_ws
colcon build
source ./install/setup.bash
ros2 launch erobo3_control demo.launch.py
