#!/bin/bash
# ROS2 Workspace Quick Build Script with Library Checks

set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

WS_DIR="/home/ilv/ros2_ws"

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}ROS2 Workspace Quick Build${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

cd "$WS_DIR"

echo -e "${YELLOW}Sourcing ROS2 Rolling...${NC}"
source /opt/ros/rolling/setup.bash

export PYTHONPATH=$(echo $PYTHONPATH | tr ':' '\n' | grep -v 'ws_moveit2' | tr '\n' ':' | sed 's/:$//')

if [ $# -eq 0 ]; then
    echo -e "${YELLOW}Building all packages...${NC}"

    echo -e "${BLUE}Step 1/2: Building fairino_msgs...${NC}"
    colcon build --symlink-install --packages-select fairino_msgs --cmake-args -DCMAKE_BUILD_TYPE=Release --event-handlers console_cohesion+

    echo -e "${BLUE}Step 2/2: Building remaining packages...${NC}"
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-skip fairino_msgs --event-handlers console_cohesion+
else
    echo -e "${YELLOW}Building packages: $@${NC}"
    colcon build --symlink-install --packages-select "$@" --cmake-args -DCMAKE_BUILD_TYPE=Release --event-handlers console_cohesion+
fi

echo -e "${YELLOW}Sourcing workspace...${NC}"
source install/setup.bash

# Optional: check critical libraries exist
for lib in libfairino.so.2 libruckig.so libOgreMain.so.1.12.10; do
    if ! find "$WS_DIR/install" -name "$lib" | grep -q .; then
        echo -e "${YELLOW}⚠ Warning: $lib not found in workspace install${NC}"
    fi
done

echo -e "${GREEN}✓ Build complete!${NC}"
echo -e "${YELLOW}To use this workspace, run:${NC}"
echo -e "  source $WS_DIR/install/setup.bash"
