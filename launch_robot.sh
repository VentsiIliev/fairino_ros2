#!/bin/bash
# ROS2 Robot Launch Script with Environment Setup
# Ensures proper environment before launching

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Workspace directory
WS_DIR="/home/ilv/ros2_ws"

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}ROS2 Robot Launch Script${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# Change to workspace directory
cd "$WS_DIR"

# CRITICAL: Unset ws_moveit2 paths BEFORE sourcing anything
# This prevents incompatible moveit_msgs from loading
echo -e "${YELLOW}Setting up environment...${NC}"

# Clean any existing ROS environment to avoid ws_moveit2 contamination
unset ROS_PACKAGE_PATH
unset AMENT_PREFIX_PATH
unset PYTHONPATH
unset LD_LIBRARY_PATH
unset CMAKE_PREFIX_PATH

# Now source ROS2 with clean environment
source /opt/ros/rolling/setup.bash

# Additional safety: Strip any ws_moveit2 that might have leaked in
export PYTHONPATH=$(echo $PYTHONPATH | tr ':' '\n' | grep -v 'ws_moveit2' | tr '\n' ':' | sed 's/:$//')
export AMENT_PREFIX_PATH=$(echo $AMENT_PREFIX_PATH | tr ':' '\n' | grep -v 'ws_moveit2' | tr '\n' ':' | sed 's/:$//')
export CMAKE_PREFIX_PATH=$(echo $CMAKE_PREFIX_PATH | tr ':' '\n' | grep -v 'ws_moveit2' | tr '\n' ':' | sed 's/:$//')

source install/setup.bash

# Verify critical library exists
if [ ! -f "$WS_DIR/install/fairino_hardware/lib/libfairino.so.2" ]; then
    echo -e "${RED}✗ Error: libfairino.so.2 not found!${NC}"
    echo -e "${YELLOW}Please run ./clean_build.sh first${NC}"
    exit 1
fi

echo -e "${GREEN}✓ Environment ready${NC}"
echo ""

# Parse command line arguments
LAUNCH_FILE="demo.launch.py"
EXTRA_ARGS=""

if [ $# -gt 0 ]; then
    LAUNCH_FILE="$1"
    shift
    EXTRA_ARGS="$@"
fi

# Launch
echo -e "${YELLOW}Launching: ${LAUNCH_FILE} ${EXTRA_ARGS}${NC}"
echo ""
ros2 launch fairino5_v6_moveit2_config "$LAUNCH_FILE" $EXTRA_ARGS