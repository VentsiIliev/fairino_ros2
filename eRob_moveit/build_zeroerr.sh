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

source_setup /opt/ros/rolling/setup.bash

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
