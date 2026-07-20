#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_WS_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

resolve_ros_setup_file() {
  if [[ -n "${ROS_SETUP_FILE:-}" ]]; then
    printf '%s\n' "${ROS_SETUP_FILE}"
    return
  fi
  if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    printf '/opt/ros/%s/setup.bash\n' "${ROS_DISTRO}"
    return
  fi
  if [[ -f /opt/ros/rolling/setup.bash ]]; then
    printf '%s\n' /opt/ros/rolling/setup.bash
    return
  fi
  if [[ -f /opt/ros/jazzy/setup.bash ]]; then
    printf '%s\n' /opt/ros/jazzy/setup.bash
    return
  fi
  return 1
}

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

ROS_SETUP="$(resolve_ros_setup_file)" || {
  echo "No ROS setup file found. Set ROS_SETUP_FILE=/opt/ros/<distro>/setup.bash" >&2
  exit 1
}

# Prevent a terminal that previously sourced another distro/overlay from leaking
# paths into this build.
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH
unset PYTHONPATH
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION

source_setup "${ROS_SETUP}"

if [[ -n "${MOVEIT_SETUP_FILE:-}" ]]; then
  if [[ ! -f "${MOVEIT_SETUP_FILE}" ]]; then
    echo "MOVEIT_SETUP_FILE not found: ${MOVEIT_SETUP_FILE}" >&2
    exit 1
  fi
  source_setup "${MOVEIT_SETUP_FILE}"
fi

if [[ -f "${ROOT_WS_DIR}/install/local_setup.bash" ]]; then
  source_setup "${ROOT_WS_DIR}/install/local_setup.bash"
fi

cd "${ROOT_WS_DIR}"

if [[ ! -f "${ROOT_WS_DIR}/install/local_setup.bash" ]]; then
  echo "Base workspace install not found; building dependencies up to erob_moveit_runtime and zeroerr..."
  colcon build --symlink-install --packages-up-to erob_moveit_runtime zeroerr --cmake-args -DCMAKE_BUILD_TYPE=Release
  source_setup "${ROOT_WS_DIR}/install/local_setup.bash"
fi

colcon build --symlink-install --packages-select erob_moveit_runtime zeroerr --cmake-args -DCMAKE_BUILD_TYPE=Release
