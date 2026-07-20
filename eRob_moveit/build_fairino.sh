#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_WS_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

source_setup() {
  local file="$1"
  local had_u=0
  if [[ $- == *u* ]]; then
    had_u=1
    set +u
  fi
  # shellcheck disable=SC1090
  source "${file}"
  if [[ ${had_u} -eq 1 ]]; then
    set -u
  fi
}

# Prevent a terminal that previously sourced Jazzy/another overlay from leaking
# paths into this Rolling build.
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH
unset PYTHONPATH
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION

source_setup /opt/ros/rolling/setup.bash
if [[ -f "${ROOT_WS_DIR}/install/local_setup.bash" ]]; then
  source_setup "${ROOT_WS_DIR}/install/local_setup.bash"
fi

cd "${SCRIPT_DIR}"
colcon build --packages-select erob_moveit_runtime fairino5_v6_moveit2_config
