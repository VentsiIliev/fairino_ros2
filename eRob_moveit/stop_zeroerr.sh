#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ROBOT_LAUNCH_CONFIG="${SCRIPT_DIR}/zeroerr_launch.conf"
export ROBOT_LAUNCH_ACTION=stop

exec "${SCRIPT_DIR}/launch_robot.sh" "$@"
