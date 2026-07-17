#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_FILE="${ROBOT_LAUNCH_CONFIG:-${SCRIPT_DIR}/robot_launch.conf}"

if [[ ! -f "${CONFIG_FILE}" ]]; then
  echo "Config not found: ${CONFIG_FILE}" >&2
  exit 1
fi

# shellcheck disable=SC1090
source "${CONFIG_FILE}"

: "${ROBOT_TYPE:?ROBOT_TYPE is required}"
: "${WORKSPACE_DIR:?WORKSPACE_DIR is required}"
: "${ROS_SETUP_FILE:?ROS_SETUP_FILE is required}"
: "${WORKSPACE_SETUP_FILE:?WORKSPACE_SETUP_FILE is required}"

BASE_UNDERLAY_SETUP_FILE="${BASE_UNDERLAY_SETUP_FILE:-}"
USE_RVIZ="${USE_RVIZ:-true}"
ZEROERR_ROS_PID=""
ZEROERR_MONITOR_PID_FILE="/tmp/zeroerr_slave_monitor.pid"
FAIRINO_ROS_PID=""

cleanup_stale_zeroerr_processes() {
  local patterns=(
    "/opt/ros/.*/robot_state_publisher/robot_state_publisher"
    "/opt/ros/.*/controller_manager/ros2_control_node"
    "/opt/ros/.*/moveit_ros_move_group/move_group"
    "rest_server_main.py"
    "/opt/ros/.*/rviz2/rviz2.*zeroerr/share/zeroerr/config/moveit.rviz"
    "ros2 launch zeroerr"
    "WaitForSlavesOp.sh"
    "zeroerr_state_publisher.py"
    "/erob_moveit_runtime/.*/main.py"
    "/lib/erob_moveit_runtime/main.py"
    "ipp_helper"
    "ruckig_helper"
  )
  local pattern=""

  for pattern in "${patterns[@]}"; do
    pkill -f "${pattern}" 2>/dev/null || true
  done

  for _ in $(seq 1 10); do
    local alive=0
    for pattern in "${patterns[@]}"; do
      if pgrep -f "${pattern}" >/dev/null 2>&1; then
        alive=1
        break
      fi
    done
    if [[ "${alive}" -eq 0 ]]; then
      return
    fi
    sleep 1
  done

  for pattern in "${patterns[@]}"; do
    pkill -9 -f "${pattern}" 2>/dev/null || true
  done
  sleep 1
}

cleanup_stale_fairino_processes() {
  local patterns=(
    "/opt/ros/.*/robot_state_publisher/robot_state_publisher"
    "/opt/ros/.*/controller_manager/ros2_control_node"
    "/opt/ros/.*/moveit_ros_move_group/move_group"
    "/opt/ros/.*/rviz2/rviz2"
    "/erob_moveit_runtime/.*/main.py"
    "/lib/erob_moveit_runtime/main.py"
    "fairino_state_publisher.py"
    "ipp_helper"
    "ruckig_helper"
    "spawner"
    "ros2 launch fairino5_v6_moveit2_config"
    "ros2 launch zeroerr"
    "WaitForSlavesOp.sh"
  )
  local pattern=""

  for pattern in "${patterns[@]}"; do
    pkill -f "${pattern}" 2>/dev/null || true
  done

  for _ in $(seq 1 10); do
    local alive=0
    for pattern in "${patterns[@]}"; do
      if pgrep -f "${pattern}" >/dev/null 2>&1; then
        alive=1
        break
      fi
    done
    if [[ "${alive}" -eq 0 ]]; then
      return
    fi
    sleep 1
  done

  for pattern in "${patterns[@]}"; do
    pkill -9 -f "${pattern}" 2>/dev/null || true
  done
  sleep 1
}

cleanup_zeroerr() {
  if [[ -f "${ZEROERR_MONITOR_PID_FILE}" ]]; then
    monitor_pid=$(cat "${ZEROERR_MONITOR_PID_FILE}" 2>/dev/null || true)
    if [[ -n "${monitor_pid}" ]] && kill -0 "${monitor_pid}" 2>/dev/null; then
      kill "${monitor_pid}" 2>/dev/null || true
      wait "${monitor_pid}" 2>/dev/null || true
    fi
    rm -f "${ZEROERR_MONITOR_PID_FILE}"
  fi

  if [[ -n "${ZEROERR_ROS_PID}" ]] && kill -0 "${ZEROERR_ROS_PID}" 2>/dev/null; then
    kill -- -"${ZEROERR_ROS_PID}" 2>/dev/null || kill "${ZEROERR_ROS_PID}" 2>/dev/null || true
    wait "${ZEROERR_ROS_PID}" 2>/dev/null || true
  fi

  if [[ -n "${ZEROERR_ETHERCAT_SCRIPT:-}" && -x "${ZEROERR_ETHERCAT_SCRIPT}" ]]; then
    STOP_ONLY=1 "${ZEROERR_ETHERCAT_SCRIPT}" 2>/dev/null || true
  fi

  cleanup_stale_zeroerr_processes
  rm -rf /dev/shm/fastdds_* /dev/shm/sem.fastdds_* 2>/dev/null || true
}

cleanup_fairino() {
  if [[ -n "${FAIRINO_ROS_PID}" ]] && kill -0 "${FAIRINO_ROS_PID}" 2>/dev/null; then
    kill -- -"${FAIRINO_ROS_PID}" 2>/dev/null || kill "${FAIRINO_ROS_PID}" 2>/dev/null || true
    wait "${FAIRINO_ROS_PID}" 2>/dev/null || true
  fi

  if [[ -n "${FAIRINO_ETHERCAT_SCRIPT:-}" && -x "${FAIRINO_ETHERCAT_SCRIPT}" ]]; then
    STOP_ONLY=1 "${FAIRINO_ETHERCAT_SCRIPT}" 2>/dev/null || true
  fi

  cleanup_stale_fairino_processes
  rm -rf /dev/shm/fastdds_* /dev/shm/sem.fastdds_* 2>/dev/null || true
}

# Clear previously sourced ROS/colcon overlays so package resolution comes only from
# the configured workspace and the base ROS distro.
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH
unset PYTHONPATH
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION

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

if [[ -f "${ROS_SETUP_FILE}" ]]; then
  source_setup "${ROS_SETUP_FILE}"
fi

if [[ -n "${BASE_UNDERLAY_SETUP_FILE}" ]]; then
  if [[ ! -f "${BASE_UNDERLAY_SETUP_FILE}" ]]; then
    echo "Base underlay setup file not found: ${BASE_UNDERLAY_SETUP_FILE}" >&2
    exit 1
  fi
  source_setup "${BASE_UNDERLAY_SETUP_FILE}"
fi

if [[ "${BUILD_WORKSPACE:-0}" == "1" ]]; then
  (cd "${WORKSPACE_DIR}" && colcon build --packages-select erob_moveit_runtime fairino5_v6_moveit2_config zeroerr)
fi

if [[ ! -f "${WORKSPACE_SETUP_FILE}" ]]; then
  echo "Workspace setup file not found: ${WORKSPACE_SETUP_FILE}" >&2
  exit 1
fi
source_setup "${WORKSPACE_SETUP_FILE}"

echo "Using AMENT_PREFIX_PATH=${AMENT_PREFIX_PATH:-}"

launch_zeroerr() {
  local package="${ZEROERR_PACKAGE:?ZEROERR_PACKAGE is required}"
  local profile="${ZEROERR_PROFILE:-full}"
  local launch_file="${ZEROERR_LAUNCH_FILE:?ZEROERR_LAUNCH_FILE is required}"
  local minimal_launch_file="${ZEROERR_MINIMAL_LAUNCH_FILE:-ethercat_only.launch.py}"
  local ethercat_script="${ZEROERR_ETHERCAT_SCRIPT:-}"
  local slave_monitor_script="${ZEROERR_SLAVE_MONITOR_SCRIPT:-}"
  local collision_monitor_terminal="${ZEROERR_COLLISION_MONITOR_TERMINAL:-0}"
  local collision_monitor_gui="${ZEROERR_COLLISION_MONITOR_GUI:-0}"

  if [[ "${profile}" == "ethercat_only" ]]; then
    launch_file="${minimal_launch_file}"
  fi

  local launch_args=()
  if [[ "${profile}" != "ethercat_only" ]]; then
    launch_args+=("use_rviz:=${USE_RVIZ}")
  fi

  echo "Preparing ZeroErr stack (${profile}): ${package} ${launch_file}"
  cleanup_stale_zeroerr_processes

  if [[ -n "${ethercat_script}" ]]; then
    if [[ -x "${ethercat_script}" ]]; then
      echo "Resetting ZeroErr EtherCAT master: ${ethercat_script}"
      ZEROERR_ISOLATED_CORES="${ZEROERR_ISOLATED_CORES:-}" \
      ZEROERR_ISOLATED_MASK="${ZEROERR_ISOLATED_MASK:-}" \
      ZEROERR_PIN_NON_RT_AWAY="${ZEROERR_PIN_NON_RT_AWAY:-}" \
      ZEROERR_NON_RT_CORES="${ZEROERR_NON_RT_CORES:-}" \
      PREP_ONLY=1 "${ethercat_script}" || true
    else
      echo "ZeroErr EtherCAT script is not executable: ${ethercat_script}" >&2
    fi
  fi

  echo "Launching ZeroErr stack (${profile}): ${package} ${launch_file}"
  ZEROERR_ROS_PID=""
  ZEROERR_MONITOR_PID_FILE="/tmp/zeroerr_slave_monitor.pid"
  ZEROERR_COLLISION_MONITOR_TERMINAL="${collision_monitor_terminal}" \
    ZEROERR_COLLISION_MONITOR_GUI="${collision_monitor_gui}" \
    setsid ros2 launch "${package}" "${launch_file}" "${launch_args[@]}" &
  ZEROERR_ROS_PID=$!
  trap cleanup_zeroerr EXIT INT TERM HUP

  if [[ -n "${ethercat_script}" && -x "${ethercat_script}" ]]; then
    echo "Applying ZeroErr RT setup: ${ethercat_script}"
    ZEROERR_ISOLATED_CORES="${ZEROERR_ISOLATED_CORES:-}" \
    ZEROERR_ISOLATED_MASK="${ZEROERR_ISOLATED_MASK:-}" \
    ZEROERR_PIN_NON_RT_AWAY="${ZEROERR_PIN_NON_RT_AWAY:-}" \
    ZEROERR_NON_RT_CORES="${ZEROERR_NON_RT_CORES:-}" \
    POSTSTART_ONLY=1 "${ethercat_script}" || true
  fi

  if [[ -n "${slave_monitor_script}" ]]; then
    if [[ -x "${slave_monitor_script}" ]]; then
      echo "Opening ZeroErr slave monitor: ${slave_monitor_script}"
      MONITOR_PID_FILE="${ZEROERR_MONITOR_PID_FILE}" "${slave_monitor_script}" || true
    else
      echo "ZeroErr slave monitor script is not executable: ${slave_monitor_script}" >&2
    fi
  fi

  wait "${ZEROERR_ROS_PID}"
}

launch_fairino() {
  local package="${FAIRINO_PACKAGE:?FAIRINO_PACKAGE is required}"
  local launch_file="${FAIRINO_LAUNCH_FILE:?FAIRINO_LAUNCH_FILE is required}"
  local ethercat_script="${FAIRINO_ETHERCAT_SCRIPT:-}"
  local launch_args=("use_rviz:=${USE_RVIZ}")

  cleanup_stale_fairino_processes

  if [[ -n "${ethercat_script}" && -x "${ethercat_script}" ]]; then
    echo "Preparing Fairino EtherCAT master: ${ethercat_script}"
    PREP_ONLY=1 "${ethercat_script}" || true
  fi

  echo "Launching Fairino stack: ${package} ${launch_file}"
  if [[ -n "${ethercat_script}" && -x "${ethercat_script}" ]]; then
    FAIRINO_ROS_PID=""
    setsid ros2 launch "${package}" "${launch_file}" "${launch_args[@]}" &
    FAIRINO_ROS_PID=$!
    trap cleanup_fairino EXIT INT TERM HUP

    echo "Applying Fairino RT setup: ${ethercat_script}"
    POSTSTART_ONLY=1 "${ethercat_script}" || true

    wait "${FAIRINO_ROS_PID}"
  else
    exec ros2 launch "${package}" "${launch_file}" "${launch_args[@]}"
  fi
}

case "${ROBOT_TYPE}" in
  fairino)
    launch_fairino
    ;;
  zeroerr)
    launch_zeroerr
    ;;
  *)
    echo "Unsupported ROBOT_TYPE='${ROBOT_TYPE}'. Expected 'fairino' or 'zeroerr'." >&2
    exit 2
    ;;
esac
