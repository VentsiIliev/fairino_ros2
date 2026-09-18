#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# This top-level workspace entrypoint is the ZeroErr/eRob launcher.  Keep the
# generic selector inside eRob_moveit for development, and use
# ./launch_fairino.sh explicitly when the Fairino backend is wanted.
EROB_LAUNCHER="${SCRIPT_DIR}/eRob_moveit/launch_zeroerr.sh"

if [[ ! -x "${EROB_LAUNCHER}" ]]; then
  echo "Launcher not found or not executable: ${EROB_LAUNCHER}" >&2
  exit 1
fi

exec "${EROB_LAUNCHER}" "$@"
