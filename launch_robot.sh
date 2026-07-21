#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EROB_LAUNCHER="${SCRIPT_DIR}/eRob_moveit/launch_robot.sh"

if [[ ! -x "${EROB_LAUNCHER}" ]]; then
  echo "Launcher not found or not executable: ${EROB_LAUNCHER}" >&2
  exit 1
fi

exec "${EROB_LAUNCHER}" "$@"
