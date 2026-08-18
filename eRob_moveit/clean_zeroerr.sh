#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_WS_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
CONFIG_FILE="${ROBOT_LAUNCH_CONFIG:-${SCRIPT_DIR}/zeroerr_launch.conf}"

resolve_ros_setup_file() {
  if [[ -n "${ROS_SETUP_FILE:-}" ]]; then
    printf '%s\n' "${ROS_SETUP_FILE}"
    return
  fi
  if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    printf '/opt/ros/%s/setup.bash\n' "${ROS_DISTRO}"
    return
  fi
  if [[ -f /opt/ros/jazzy/setup.bash ]]; then
    printf '%s\n' /opt/ros/jazzy/setup.bash
    return
  fi
  if [[ -f /opt/ros/rolling/setup.bash ]]; then
    printf '%s\n' /opt/ros/rolling/setup.bash
    return
  fi
  return 1
}

DEFAULT_ROS_SETUP_FILE="$(resolve_ros_setup_file)" || {
  echo "No ROS setup file found. Set ROS_SETUP_FILE=/opt/ros/<distro>/setup.bash" >&2
  exit 1
}

if [[ -f "${CONFIG_FILE}" ]]; then
  # shellcheck disable=SC1090
  source "${CONFIG_FILE}"
fi

ROS_SETUP_FILE="${ROS_SETUP_FILE:-${DEFAULT_ROS_SETUP_FILE}}"
BASE_UNDERLAY_SETUP_FILE="${BASE_UNDERLAY_SETUP_FILE:-}"
WORKSPACE_SETUP_FILE="${WORKSPACE_SETUP_FILE:-${ROOT_WS_DIR}/install/local_setup.bash}"
ROBOT_ROS_DOMAIN_ID="${ROBOT_ROS_DOMAIN_ID:-42}"
ROBOT_ROS_AUTOMATIC_DISCOVERY_RANGE="${ROBOT_ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"
ROBOT_ROS_LOCALHOST_ONLY="${ROBOT_ROS_LOCALHOST_ONLY:-1}"

echo "Stopping ZeroErr stack..."
"${SCRIPT_DIR}/stop_zeroerr.sh" "$@" || true

echo "Stopping ROS 2 daemon..."
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH
unset PYTHONPATH
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION

source_setup_file() {
  local setup_file="$1"
  set +u
  # shellcheck disable=SC1090
  source "${setup_file}"
  set -u
}

source_setup_file "${ROS_SETUP_FILE}"
if [[ -n "${BASE_UNDERLAY_SETUP_FILE}" && -f "${BASE_UNDERLAY_SETUP_FILE}" ]]; then
  source_setup_file "${BASE_UNDERLAY_SETUP_FILE}"
fi
if [[ -f "${WORKSPACE_SETUP_FILE}" ]]; then
  source_setup_file "${WORKSPACE_SETUP_FILE}"
fi

export ROS_DOMAIN_ID="${ROBOT_ROS_DOMAIN_ID}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROBOT_ROS_AUTOMATIC_DISCOVERY_RANGE}"
export ROS_LOCALHOST_ONLY="${ROBOT_ROS_LOCALHOST_ONLY}"

timeout 5s ros2 daemon stop >/dev/null 2>&1 || true
pkill -TERM -f 'ros2cli.daemon.daemonize' 2>/dev/null || true
sleep 0.2
pkill -KILL -f 'ros2cli.daemon.daemonize' 2>/dev/null || true

echo "Clearing local ROS/FastDDS runtime leftovers..."
rm -f /tmp/zeroerr_runtime.pid /tmp/zeroerr_slave_monitor.pid
rm -rf /dev/shm/fastdds_* /dev/shm/sem.fastdds_* /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true

echo "Checking for remaining ZeroErr/ROS helper processes..."
remaining="$(
  pgrep -af 'ros2 launch zeroerr|zeroerr_runtime.py|ros2_control_node|move_group|zeroerr_servo_node|zeroerr_state_publisher|ipp_helper|ruckig_helper|contour_ik_helper|ptp_helper|linked_lin_helper|trajectory_state_validator|ethercat_sdo_srv_server|controller_manager/spawner|ros2cli.daemon.daemonize' || true
)"

if [[ -n "${remaining}" ]]; then
  echo "Warning: matching processes are still alive:"
  printf '%s\n' "${remaining}"
  exit 1
fi

echo "Clean ZeroErr/ROS launch state ready."
