#!/bin/bash
# ROS2 Workspace Quick Build Script with Library Checks

set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OVERLAY_DIR="${WS_DIR}/eRob_moveit"
OVERLAY_PACKAGES=("erob_moveit_runtime" "fairino5_v6_moveit2_config" "zeroerr")

build_base_workspace() {
    echo -e "${BLUE}Step 1/3: Building fairino_msgs...${NC}"
    colcon build --symlink-install --packages-select fairino_msgs --cmake-args -DCMAKE_BUILD_TYPE=Release --event-handlers console_cohesion+

    echo -e "${BLUE}Step 2/3: Building remaining base workspace packages...${NC}"
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-skip fairino_msgs --event-handlers console_cohesion+
}

build_overlay_workspace() {
    echo -e "${BLUE}Step 3/3: Building eRob_moveit overlay packages...${NC}"
    cd "$OVERLAY_DIR"
    source "$WS_DIR/install/local_setup.bash"
    colcon build --symlink-install --packages-select "${OVERLAY_PACKAGES[@]}" --cmake-args -DCMAKE_BUILD_TYPE=Release --event-handlers console_cohesion+
    cd "$WS_DIR"
}

is_overlay_package() {
    local pkg="$1"
    local overlay_pkg=""
    for overlay_pkg in "${OVERLAY_PACKAGES[@]}"; do
        if [[ "$pkg" == "$overlay_pkg" ]]; then
            return 0
        fi
    done
    return 1
}

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}ROS2 Workspace Quick Build${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

cd "$WS_DIR"

echo -e "${YELLOW}Sourcing ROS2 Rolling...${NC}"
source /opt/ros/rolling/setup.bash

PYTHONPATH="$(
    printf '%s' "${PYTHONPATH:-}" \
        | tr ':' '\n' \
        | grep -v 'ws_moveit2' \
        | tr '\n' ':' \
        | sed 's/:$//' \
        || true
)"
export PYTHONPATH

if [ $# -eq 0 ]; then
    echo -e "${YELLOW}Building all packages...${NC}"
    build_base_workspace
    build_overlay_workspace
else
    base_packages=()
    overlay_packages=()
    for pkg in "$@"; do
        if is_overlay_package "$pkg"; then
            overlay_packages+=("$pkg")
        else
            base_packages+=("$pkg")
        fi
    done

    if [ ${#base_packages[@]} -gt 0 ]; then
        echo -e "${YELLOW}Building base workspace packages: ${base_packages[*]}${NC}"
        colcon build --symlink-install --packages-select "${base_packages[@]}" --cmake-args -DCMAKE_BUILD_TYPE=Release --event-handlers console_cohesion+
    fi

    if [ ${#overlay_packages[@]} -gt 0 ]; then
        echo -e "${YELLOW}Building overlay workspace packages: ${overlay_packages[*]}${NC}"
        cd "$OVERLAY_DIR"
        source "$WS_DIR/install/local_setup.bash"
        colcon build --symlink-install --packages-select "${overlay_packages[@]}" --cmake-args -DCMAKE_BUILD_TYPE=Release --event-handlers console_cohesion+
        cd "$WS_DIR"
    fi
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
