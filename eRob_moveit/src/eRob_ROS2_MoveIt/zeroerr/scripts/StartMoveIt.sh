#!/bin/bash
cd ros2_ws
colcon build
source ./install/setup.bash
ros2 launch zeroerr full_stack.launch.py
