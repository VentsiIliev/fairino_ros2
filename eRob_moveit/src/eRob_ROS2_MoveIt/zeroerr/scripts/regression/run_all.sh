#!/usr/bin/env bash
# Sequential runner for the zeroerr single-robot regression suite.
#
# Usage:
#   source /opt/ros/jazzy/setup.bash
#   source /home/ventsi/fairino_ros2/install/setup.bash
#   export EROB_CONFIG_PACKAGE=zeroerr
#   export ZEROERR_USE_FAKE_HARDWARE=1      # required unless --allow-real-hardware
#   ./run_all.sh [--robot NAME] [test-args...]
#
# Exit codes (overall):
#   0  all tests passed
#   1  at least one test FAILED
#   2  environment/setup problem (abort)
#   3  usage error (abort)

set -u

cd "$(dirname "${BASH_SOURCE[0]}")"

: "${EROB_CONFIG_PACKAGE:=zeroerr}"
export EROB_CONFIG_PACKAGE

if [[ "${ZEROERR_USE_FAKE_HARDWARE:-}" == "" ]]; then
  echo "run_all.sh: ZEROERR_USE_FAKE_HARDWARE is not set." >&2
  echo "  Set ZEROERR_USE_FAKE_HARDWARE=1 for safe fake-hardware mode" >&2
  echo "  (or =0 with --allow-real-hardware for a real controller)." >&2
  exit 2
fi
export ZEROERR_USE_FAKE_HARDWARE

# Default robot resolves to the active profile's PRIMARY_ROBOT
# (robot under paint/welding, robot1 under twin_robots). --robot overrides.
DEFAULT_ROBOT="$(EROB_CONFIG_PACKAGE="${EROB_CONFIG_PACKAGE}" python3 - <<'PY'
import sys
from pathlib import Path
from ament_index_python.packages import get_package_prefix

sys.path.insert(
    0,
    str(
        Path(get_package_prefix("erob_moveit_runtime"))
        / "lib"
        / "erob_moveit_runtime"
    ),
)
import config

primary = config.get_primary_robot_name()
names = config.get_robot_names()
print(primary or (names[0] if names else "robot"))
PY
)"
ROBOT="${REGRESSION_ROBOT:-${DEFAULT_ROBOT:-robot}}"

# Static profile validation runs FIRST: it works even when the runtime cannot
# construct and documents any legacy-profile configuration gap.
TESTS=(
  test_11_static_profiles.py
  test_01_readiness.py
  test_02_move_ptp.py
  test_03_move_lin.py
  test_04_execute_path.py
  test_05_execute_sequence.py
  test_06_stop_motion.py
  test_07_ordered_motion_chain.py
  test_08_prepared_execution.py
  test_09_prepared_noop.py
  test_10_status_snapshots.py
  test_12_restart.py
)

overall=0
for t in "${TESTS[@]}"; do
  printf '\n%s\n' "================ $t ================"
  if printf '%s\n' "$@" | grep -q -- '--robot'; then
    python3 "$t" "$@"
  else
    python3 "$t" --robot "${ROBOT}" "$@"
  fi
  rc=$?
  case "${rc}" in
    0)  printf '[PASS] %s\n' "$t" ;;
    2)  printf '[SETUP] %s - aborting\n' "$t"
        exit 2 ;;
    3)  printf '[USAGE] %s - aborting\n' "$t"
        exit 3 ;;
    *)  printf '[FAIL] %s\n' "$t"
        overall=1 ;;
  esac
done

printf '\nrun_all.sh finished: '
if [[ "${overall}" -eq 0 ]]; then
  echo "ALL PASS"
else
  echo "FAILURES PRESENT (see output above)"
fi
exit "${overall}"
