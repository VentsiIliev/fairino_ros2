#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_WS_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
CONFIG_FILE="${ROBOT_LAUNCH_CONFIG:-${SCRIPT_DIR}/zeroerr_launch.conf}"
DEFAULT_ROS_SETUP_FILE="${ROS_SETUP_FILE:-/opt/ros/jazzy/setup.bash}"

if [ -f "$CONFIG_FILE" ]; then
  # shellcheck disable=SC1090
  source "$CONFIG_FILE"
fi

RT_CORES="${ZEROERR_ISOLATED_CORES:-2,3}"
ETHERCAT_CORES="${ZEROERR_ETHERCAT_CORES:-2}"
CONTROL_CORES="${ZEROERR_CONTROL_CORES:-3}"
NON_RT_CORES="${ZEROERR_NON_RT_CORES:-0,1}"
NIC="${ZEROERR_NIC:-enp3s0}"

cpu_list_intersects() {
  local lhs="$1"
  local rhs="$2"
  python3 - "$lhs" "$rhs" <<'PY'
import sys

def expand(value):
    cpus = set()
    for part in str(value).split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            start, end = part.split("-", 1)
            cpus.update(range(int(start), int(end) + 1))
        else:
            cpus.add(int(part))
    return cpus

raise SystemExit(0 if expand(sys.argv[1]) & expand(sys.argv[2]) else 1)
PY
}

echo "--- cmdline ---"
cat /proc/cmdline

echo
echo "--- expected layout ---"
echo "RT cores: $RT_CORES"
echo "EtherCAT cores: $ETHERCAT_CORES"
echo "ros2_control cores: $CONTROL_CORES"
echo "non-RT cores: $NON_RT_CORES"
echo "NIC: $NIC"

echo
echo "--- ZeroErr / ROS process threads on RT cores $RT_CORES ---"
ps -eLo pid,tid,class,rtprio,psr,pcpu,stat,comm,args \
  | awk -v rt_cores="$RT_CORES" '
      BEGIN {
        split(rt_cores, parts, ",")
        for (i in parts) {
          if (parts[i] ~ /-/) {
            split(parts[i], range, "-")
            for (cpu = range[1]; cpu <= range[2]; cpu++) rt[cpu] = 1
          } else {
            rt[parts[i]] = 1
          }
        }
      }
      rt[$5] && $0 ~ /EtherCAT|ros2_control|move_group|zeroerr|state_publisher|ruckig|ipp|contour|drive_diag|main.py|python/ {print}
    ' \
  | sort -k5,5n -k6,6nr

echo
echo "--- EtherCAT process affinity/scheduler ---"
for pid in $(pgrep -f '^\[EtherCAT-' 2>/dev/null || true); do
  [ -r "/proc/$pid/status" ] || continue
  printf 'PID %s ' "$pid"
  awk '/Name:|Cpus_allowed_list/ {printf "%s=%s ", $1, $2} END {print ""}' "/proc/$pid/status"
  chrt -p "$pid" 2>/dev/null || true
done

echo
echo "--- ros2_control affinity/scheduler ---"
for pid in $(pgrep -f ros2_control_node 2>/dev/null || true); do
  [ -r "/proc/$pid/status" ] || continue
  printf 'PID %s ' "$pid"
  awk '/Name:|Cpus_allowed_list/ {printf "%s=%s ", $1, $2} END {print ""}' "/proc/$pid/status"
  ps -eLo pid,tid,class,rtprio,psr,pcpu,stat,comm,args \
    | awk -v pid="$pid" '$1 == pid {print}'
done

echo
echo "--- NIC IRQs ---"
awk -v nic="$NIC" '$0 ~ nic || $0 ~ /EtherCAT/ {print}' /proc/interrupts

echo
echo "--- IRQs effective on RT cores $RT_CORES ---"
for d in /proc/irq/[0-9]*; do
  irq="${d##*/}"
  effective="$(cat "$d/effective_affinity_list" 2>/dev/null || true)"
  [ -n "$effective" ] || continue
  if cpu_list_intersects "$effective" "$RT_CORES"; then
    printf '%s effective=%s ' "$irq" "$effective"
    awk -v irq="$irq" '$1 == irq ":" {$1=""; print}' /proc/interrupts
  fi
done

echo
echo "--- rt cgroups ---"
for cg in /sys/fs/cgroup/rt_control /sys/fs/cgroup/rt_ethercat; do
  if [ -d "$cg" ]; then
    grep -H . "$cg/cpuset.cpus" "$cg/cgroup.procs" 2>/dev/null || true
  else
    echo "$cg: missing"
  fi
done
