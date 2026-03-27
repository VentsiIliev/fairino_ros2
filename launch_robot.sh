#!/usr/bin/env bash
set -euo pipefail

EROB_LAUNCHER="/home/ilv/ros2_ws/eRob_moveit/launch_robot.sh"

if [[ ! -x "${EROB_LAUNCHER}" ]]; then
  echo "Launcher not found or not executable: ${EROB_LAUNCHER}" >&2
  exit 1
fi

exec "${EROB_LAUNCHER}" "$@"
