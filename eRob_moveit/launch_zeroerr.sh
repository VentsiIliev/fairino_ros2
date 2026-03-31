#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ROBOT_LAUNCH_CONFIG="${SCRIPT_DIR}/zeroerr_launch.conf"

exec "${SCRIPT_DIR}/launch_robot.sh" "$@"
