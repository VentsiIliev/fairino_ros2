#!/bin/bash
# ROS2 Workspace Quick Build Script
# Rebuilds without cleaning (faster for iterative development)

set -e  # Exit on any error

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Workspace directory
WS_DIR="/home/ilv/ros2_ws"

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}ROS2 Workspace Quick Build${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# Change to workspace directory
cd "$WS_DIR"

# Source ROS2 base installation
echo -e "${YELLOW}Sourcing ROS2 Rolling...${NC}"
source /opt/ros/rolling/setup.bash

# Check if specific packages are provided as arguments
if [ $# -eq 0 ]; then
    # No arguments - build all packages
    echo -e "${YELLOW}Building all packages...${NC}"
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
else
    # Build specific packages
    echo -e "${YELLOW}Building packages: $@${NC}"
    colcon build --symlink-install --packages-select "$@" --cmake-args -DCMAKE_BUILD_TYPE=Release
fi

# Source the workspace
echo ""
echo -e "${YELLOW}Sourcing workspace...${NC}"
source install/setup.bash

echo ""
echo -e "${GREEN}✓ Build complete!${NC}"
echo ""
echo -e "${YELLOW}To use this workspace, run:${NC}"
echo -e "  source $WS_DIR/install/setup.bash"
echo ""