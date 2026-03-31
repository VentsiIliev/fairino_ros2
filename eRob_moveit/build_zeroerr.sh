#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

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

source_setup /opt/ros/rolling/setup.bash
if [[ -f /home/ilv/ros2_ws/install/local_setup.bash ]]; then
  source_setup /home/ilv/ros2_ws/install/local_setup.bash
fi

cd "${SCRIPT_DIR}"
colcon build --packages-select erob_moveit_runtime zeroerr
