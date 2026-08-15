#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ROBOT_LAUNCH_CONFIG="${SCRIPT_DIR}/zeroerr_launch.conf"

args=()
for arg in "$@"; do
  case "${arg}" in
    --twin-robots)
      export ZEROERR_TWIN_ROBOTS=1
      ;;
    *)
      args+=("${arg}")
      ;;
  esac
done

if [[ "${ZEROERR_TWIN_ROBOTS:-0}" == "1" ]]; then
  echo "Twin-robot mode enabled: shared infrastructure will start without the legacy REST runtime."
fi

exec "${SCRIPT_DIR}/launch_robot.sh" "${args[@]}"
