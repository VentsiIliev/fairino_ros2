#!/bin/bash
# ROS2 Workspace Clean Build Script
# This script performs a complete clean and rebuild of the entire ROS2 workspace

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
echo -e "${BLUE}ROS2 Workspace Clean Build Script${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# Change to workspace directory
cd "$WS_DIR"

# Step 1: Clean previous build artifacts
echo -e "${YELLOW}[1/6] Cleaning previous build artifacts...${NC}"
if [ -d "build" ]; then
    echo "  Removing build/ directory..."
    rm -rf build
fi
if [ -d "install" ]; then
    echo "  Removing install/ directory..."
    rm -rf install
fi
if [ -d "log" ]; then
    echo "  Removing log/ directory..."
    rm -rf log
fi
echo -e "${GREEN}✓ Clean complete${NC}"
echo ""

# Step 2: Source ROS2 base installation
echo -e "${YELLOW}[2/6] Sourcing ROS2 Rolling installation...${NC}"
if [ -f "/opt/ros/rolling/setup.bash" ]; then
    source /opt/ros/rolling/setup.bash
    echo -e "${GREEN}✓ ROS2 Rolling sourced${NC}"
else
    echo -e "${RED}✗ Error: /opt/ros/rolling/setup.bash not found!${NC}"
    exit 1
fi
echo ""

# Step 3: Build the workspace
echo -e "${YELLOW}[3/6] Building workspace with colcon...${NC}"
echo "  This may take several minutes..."
if colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release; then
    echo -e "${GREEN}✓ Build successful${NC}"
else
    echo -e "${RED}✗ Build failed! Check the errors above.${NC}"
    exit 1
fi
echo ""

# Step 4: Source the workspace
echo -e "${YELLOW}[4/6] Sourcing workspace...${NC}"
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
    echo -e "${GREEN}✓ Workspace sourced${NC}"
else
    echo -e "${RED}✗ Error: install/setup.bash not found!${NC}"
    exit 1
fi
echo ""

# Step 5: Verify critical packages
echo -e "${YELLOW}[5/6] Verifying critical packages...${NC}"
PACKAGES=("fairino_hardware" "fairino_msgs" "fairino5_v6_moveit2_config")
ALL_FOUND=true

for pkg in "${PACKAGES[@]}"; do
    if ros2 pkg list | grep -q "^${pkg}$"; then
        echo -e "  ${GREEN}✓${NC} $pkg"
    else
        echo -e "  ${RED}✗${NC} $pkg - NOT FOUND"
        ALL_FOUND=false
    fi
done

if [ "$ALL_FOUND" = true ]; then
    echo -e "${GREEN}✓ All critical packages verified${NC}"
else
    echo -e "${RED}✗ Some packages are missing${NC}"
fi
echo ""

# Step 6: Verify library paths
echo -e "${YELLOW}[6/6] Verifying library paths...${NC}"
if [ -d "$WS_DIR/install/fairino_hardware/lib" ]; then
    if [[ ":$LD_LIBRARY_PATH:" == *":$WS_DIR/install/fairino_hardware/lib:"* ]]; then
        echo -e "  ${GREEN}✓${NC} fairino_hardware lib in LD_LIBRARY_PATH"
    else
        echo -e "  ${YELLOW}⚠${NC} fairino_hardware lib may not be in LD_LIBRARY_PATH"
    fi

    # Check if libfairino.so.2 exists
    if [ -f "$WS_DIR/install/fairino_hardware/lib/libfairino.so.2" ]; then
        echo -e "  ${GREEN}✓${NC} libfairino.so.2 found"
    else
        echo -e "  ${RED}✗${NC} libfairino.so.2 NOT found"
    fi
fi
echo ""

# Final summary
echo -e "${BLUE}========================================${NC}"
echo -e "${GREEN}Build Complete!${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""
echo -e "${YELLOW}To use this workspace in a new terminal, run:${NC}"
echo -e "  source $WS_DIR/install/setup.bash"
echo ""
echo -e "${YELLOW}To launch your robot system:${NC}"
echo -e "  ros2 launch fairino5_v6_moveit2_config demo.launch.py"
echo ""