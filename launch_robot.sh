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
PKG_NAME="fairino5_v6_moveit2_config"
LAUNCH_FILE="demo.launch.py"

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}ROS2 Robot Launch Script${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

cd "$WS_DIR"

echo -e "${YELLOW}Setting up environment...${NC}"

# Set DISPLAY for RViz
export DISPLAY=:1

# Clean incompatible paths (optional)
for VAR in ROS_PACKAGE_PATH AMENT_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH CMAKE_PREFIX_PATH; do
    export $VAR=$(echo ${!VAR} | tr ':' '\n' | grep -v 'ws_moveit2' | tr '\n' ':' | sed 's/:$//')
done

# Source ROS2 and workspace
source /opt/ros/rolling/setup.bash
source install/setup.bash

# NOTE: LD_LIBRARY_PATH is automatically configured by setup.bash
# Explicitly export it to ensure child processes inherit it
export LD_LIBRARY_PATH

# Verify critical libraries
if [ ! -f "$WS_DIR/install/fairino_hardware/lib/libfairino.so.2" ]; then
    echo -e "${RED}✗ Error: libfairino.so.2 not found!${NC}"
    echo -e "${YELLOW}Please run ./quick_build.sh first${NC}"
    exit 1
fi

# Full path to launch file
FULL_LAUNCH_PATH="$WS_DIR/install/$PKG_NAME/share/$PKG_NAME/launch/$LAUNCH_FILE"
if [ ! -f "$FULL_LAUNCH_PATH" ]; then
    echo -e "${RED}✗ Error: Launch file '$LAUNCH_FILE' not found!${NC}"
    exit 1
fi

echo -e "${GREEN}✓ Environment ready${NC}"
echo -e "${YELLOW}Launching: $FULL_LAUNCH_PATH${NC}"
echo ""

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  Initial startup may take ~40 seconds${NC}"
echo -e "${BLUE}  (Processing collision geometry)${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# Launch ROS2 (environment already configured by setup.bash)
ros2 launch "$FULL_LAUNCH_PATH"
