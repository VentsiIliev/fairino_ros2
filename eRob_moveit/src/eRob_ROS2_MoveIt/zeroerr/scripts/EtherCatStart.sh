#!/bin/bash
set -euo pipefail
# EtherCAT RT startup — supports PREP_ONLY=1 or POSTSTART_ONLY=1.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_POLICY_FILE="$(cd "${SCRIPT_DIR}/../../../.." && pwd)/zeroerr_rt_policy.env"
ZEROERR_RT_POLICY_FILE="${ZEROERR_RT_POLICY_FILE:-$DEFAULT_POLICY_FILE}"
if [ -f "$ZEROERR_RT_POLICY_FILE" ]; then
  # shellcheck disable=SC1090
  source "$ZEROERR_RT_POLICY_FILE"
fi

ISOLATED_CORES="${ZEROERR_ISOLATED_CORES:-}"
ISOLATED_MASK="${ZEROERR_ISOLATED_MASK:-}"
IRQ_CORES="${ZEROERR_ETHERCAT_IRQ_CORES:-${ZEROERR_IRQ_CORES:-${ZEROERR_ETHERCAT_CORES:-}}}"
IRQ_MASK="${ZEROERR_IRQ_MASK:-}"
ETHERCAT_CORES="${ZEROERR_ETHERCAT_CORES:-}"
CONTROL_CORES="${ZEROERR_CONTROL_CORES:-15}"

PIN_NON_RT_AWAY="${ZEROERR_PIN_NON_RT_AWAY:-0}"
NON_RT_CORES="${ZEROERR_NON_RT_CORES:-}"
PLANNER_CORES="${ZEROERR_PLANNER_CORES:-$NON_RT_CORES}"
LOW_PRIORITY_CORES="${ZEROERR_LOW_PRIORITY_CORES:-$NON_RT_CORES}"

RT_PRIORITY="${ZEROERR_ETHERCAT_FIFO:-90}"
CONTROL_FIFO="${ZEROERR_CONTROL_FIFO:-90}"
AFFINITY_CHECK_PERIOD_S="${ZEROERR_AFFINITY_CHECK_PERIOD_S:-2}"
NON_RT_DISCOVERY_PERIOD_S="${ZEROERR_NON_RT_DISCOVERY_PERIOD_S:-6}"
NIC="${ZEROERR_ETHERCAT_IFACE:-${ZEROERR_NIC:-}}"
if [ -z "$NIC" ]; then
  NIC="$(${SCRIPT_DIR}/resolve_ethercat_nic.sh)"
fi
if [ ! -e "/sys/class/net/$NIC" ]; then
  echo "EtherCAT NIC '$NIC' does not exist; refusing RT setup." >&2
  exit 1
fi
ETHERCAT_DEVICE="${ZEROERR_ETHERCAT_DEVICE:-/dev/EtherCAT0}"
PREP_ONLY="${PREP_ONLY:-0}"
POSTSTART_ONLY="${POSTSTART_ONLY:-0}"
STOP_ONLY="${STOP_ONLY:-0}"
REUSE_ETHERCAT_MASTER="${ZEROERR_REUSE_ETHERCAT_MASTER:-1}"
MOVE_IRQS_AWAY_FROM="${ZEROERR_MOVE_IRQS_AWAY_FROM:-$ISOLATED_CORES}"
MOVE_IRQS_TO="${ZEROERR_MOVE_IRQS_TO:-$NON_RT_CORES}"
RT_POLICY_STRICT="${ZEROERR_RT_POLICY_STRICT:-1}"

declare -A CANONICAL_CPU_LISTS=()
CANONICAL_CPU_LIST_RESULT=""

canonical_cpu_list() {
  local target="$1"
  if [ -z "${CANONICAL_CPU_LISTS[$target]:-}" ]; then
    CANONICAL_CPU_LISTS["$target"]=$(
      taskset -c "$target" awk '/Cpus_allowed_list/ {print $2}' /proc/self/status
    )
  fi
  CANONICAL_CPU_LIST_RESULT="${CANONICAL_CPU_LISTS[$target]}"
}

ensure_affinity() {
  local target="$1"
  local task_id="$2"
  local expected=""
  local actual=""
  local key=""
  local value=""

  [ -r "/proc/$task_id/status" ] || return 0
  while IFS=$'\t' read -r key value; do
    if [ "$key" = "Cpus_allowed_list:" ]; then
      actual="${value//[[:space:]]/}"
      break
    fi
  done < "/proc/$task_id/status"

  canonical_cpu_list "$target"
  expected="$CANONICAL_CPU_LIST_RESULT"
  if [ "$actual" != "$expected" ]; then
    sudo taskset -cp "$target" "$task_id" > /dev/null 2>&1 || true
  fi
}

cpu_list_to_mask() {
  local cpus="$1"
  python3 - "$cpus" <<'PY'
import sys

mask = 0
for part in sys.argv[1].split(","):
    part = part.strip()
    if not part:
        continue
    if "-" in part:
        start, end = part.split("-", 1)
        for cpu in range(int(start), int(end) + 1):
            mask |= 1 << cpu
    else:
        mask |= 1 << int(part)
print(f"{mask:x}")
PY
}

cpu_lists_intersect() {
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

cpu_lists_equal() {
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

raise SystemExit(0 if expand(sys.argv[1]) == expand(sys.argv[2]) else 1)
PY
}

cpu_list_contains() {
  local superset="$1"
  local subset="$2"
  python3 - "$superset" "$subset" <<'PY'
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

superset = expand(sys.argv[1])
subset = expand(sys.argv[2])
raise SystemExit(0 if subset <= superset else 1)
PY
}

cpu_list_to_lines() {
  local value="$1"
  python3 - "$value" <<'PY'
import sys

cpus = set()
for part in str(sys.argv[1]).split(","):
    part = part.strip()
    if not part:
        continue
    if "-" in part:
        start, end = part.split("-", 1)
        cpus.update(range(int(start), int(end) + 1))
    else:
        cpus.add(int(part))

for cpu in sorted(cpus):
    print(cpu)
PY
}

rt_policy_issue() {
  local message="$1"
  if [ "$RT_POLICY_STRICT" = "1" ]; then
    echo "  Error: $message" >&2
    return 1
  fi
  echo "  Warning: $message"
  return 0
}

detect_kernel_isolated_cores() {
  tr ' ' '\n' < /proc/cmdline \
    | awk -F= '/^(isolcpus|nohz_full)=/ {print $2; exit}' \
    | sed 's/managed_irq,//g; s/domain,//g; s/nohz,//g' || true
}

validate_rt_core_config() {
  local kernel_isolated=""
  local online_cpus=""
  kernel_isolated="$(detect_kernel_isolated_cores)"
  online_cpus="$(cat /sys/devices/system/cpu/online 2>/dev/null || true)"

  if [ -n "$kernel_isolated" ] && [ -n "$ISOLATED_CORES" ] && ! cpu_lists_equal "$kernel_isolated" "$ISOLATED_CORES"; then
    rt_policy_issue "kernel isolates CPUs $kernel_isolated but ZEROERR_ISOLATED_CORES=$ISOLATED_CORES" || return 1
  fi
  if [ -n "$kernel_isolated" ] && [ -n "$ETHERCAT_CORES" ] && ! cpu_lists_intersect "$kernel_isolated" "$ETHERCAT_CORES"; then
    rt_policy_issue "EtherCAT cores ($ETHERCAT_CORES) are not in kernel isolated CPUs ($kernel_isolated)" || return 1
  fi
  if [ -n "$kernel_isolated" ] && [ -n "$CONTROL_CORES" ] && ! cpu_lists_intersect "$kernel_isolated" "$CONTROL_CORES"; then
    rt_policy_issue "ros2_control cores ($CONTROL_CORES) are not in kernel isolated CPUs ($kernel_isolated)" || return 1
  fi
  if [ -n "$online_cpus" ]; then
    for cpu_list_name in ISOLATED_CORES ETHERCAT_CORES CONTROL_CORES IRQ_CORES NON_RT_CORES PLANNER_CORES LOW_PRIORITY_CORES; do
      local cpu_list="${!cpu_list_name:-}"
      [ -n "$cpu_list" ] || continue
      if ! cpu_list_contains "$online_cpus" "$cpu_list"; then
        rt_policy_issue "$cpu_list_name=$cpu_list includes CPUs outside online set $online_cpus" || return 1
      fi
    done
  fi
  if [ -n "$ISOLATED_CORES" ] && [ -n "$NON_RT_CORES" ] && cpu_lists_intersect "$ISOLATED_CORES" "$NON_RT_CORES"; then
    rt_policy_issue "ZEROERR_NON_RT_CORES=$NON_RT_CORES overlaps isolated cores $ISOLATED_CORES" || return 1
  fi
  if [ -n "$CONTROL_CORES" ] && [ -n "$PLANNER_CORES" ] && cpu_lists_intersect "$CONTROL_CORES" "$PLANNER_CORES"; then
    rt_policy_issue "ZEROERR_PLANNER_CORES=$PLANNER_CORES overlaps control cores $CONTROL_CORES" || return 1
  fi
  if [ -n "${ZEROERR_RT_AUX_CORES:-}" ] && [ -n "$ETHERCAT_CORES" ] && [ -n "$CONTROL_CORES" ] && cpu_lists_intersect "$ETHERCAT_CORES" "$CONTROL_CORES"; then
    rt_policy_issue "ZEROERR_ETHERCAT_CORES=$ETHERCAT_CORES overlaps control cores $CONTROL_CORES even though ZEROERR_RT_AUX_CORES=$ZEROERR_RT_AUX_CORES is available" || return 1
  fi
  if [ -n "$ISOLATED_CORES" ] && [ -n "$LOW_PRIORITY_CORES" ] && cpu_lists_intersect "$ISOLATED_CORES" "$LOW_PRIORITY_CORES"; then
    rt_policy_issue "ZEROERR_LOW_PRIORITY_CORES=$LOW_PRIORITY_CORES overlaps isolated cores $ISOLATED_CORES" || return 1
  fi

  echo "  RT policy: isolated=${ISOLATED_CORES:-none} control=$CONTROL_CORES ethercat=$ETHERCAT_CORES irq=$IRQ_CORES non_rt=$NON_RT_CORES planner=$PLANNER_CORES low=$LOW_PRIORITY_CORES"
}

configure_cpu() {
  validate_rt_core_config
  echo "Setting CPU governor to performance..."
  for cpu in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
    [ -e "$cpu" ] || continue
    echo performance | sudo tee "$cpu" > /dev/null || true
  done
  if [ -r /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor ]; then
    echo "  → Governor: $(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor)"
  else
    echo "  → Governor: unavailable on this CPU/kernel"
  fi

  local idle_cores
  idle_cores=$(cpu_list_to_lines "${ISOLATED_CORES},${IRQ_CORES},${CONTROL_CORES}" | awk 'NF && !seen[$0]++')
  if [ -z "$idle_cores" ]; then
    echo "No RT-related cores configured for idle-state tuning."
    return 0
  fi
  echo "Disabling deep idle states on RT-related cores: $(echo "$idle_cores" | paste -sd, -)..."
  for core in $idle_cores; do
    for state in /sys/devices/system/cpu/cpu${core}/cpuidle/state*/disable; do
      [ -e "$state" ] || continue
      echo 1 | sudo tee "$state" > /dev/null 2>&1 || true
    done
  done
}

ethercat_master_pids() {
  local pids=""
  pids=$(pgrep -x "EtherCAT-OP" 2>/dev/null || true)
  [ -n "$pids" ] || pids=$(pgrep -f "EtherCAT" 2>/dev/null || true)
  printf "%s\n" "$pids" | sort -n | uniq
}

ethercat_master_healthy() {
  local pids=""
  pids="$(ethercat_master_pids)"
  [ -n "$pids" ] || return 1
  [ -e "$ETHERCAT_DEVICE" ] || return 1
  ethercat slaves > /dev/null 2>&1 || return 1
  return 0
}

start_ethercat_master() {
  echo "Starting EtherCAT..."
  sudo /etc/init.d/ethercat start
  sleep 2
}

restart_ethercat_master() {
  echo "Resetting EtherCAT..."
  sudo /etc/init.d/ethercat stop > /dev/null 2>&1 || true
  sleep 1
  sudo pkill -f "EtherCAT" 2>/dev/null || true
  sleep 1
  start_ethercat_master
}

ensure_ethercat_master_running() {
  if [ "$REUSE_ETHERCAT_MASTER" = "1" ] && ethercat_master_healthy; then
    echo "Reusing running EtherCAT master."
    return 0
  fi

  if [ -n "$(ethercat_master_pids)" ]; then
    echo "Running EtherCAT master is not healthy; restarting it..."
    restart_ethercat_master
  else
    start_ethercat_master
  fi
}

ensure_ethercat_device_access() {
  echo "Checking EtherCAT device access: $ETHERCAT_DEVICE"
  for _ in $(seq 1 50); do
    [ -e "$ETHERCAT_DEVICE" ] && break
    sleep 0.1
  done

  if [ ! -e "$ETHERCAT_DEVICE" ]; then
    echo "  Error: $ETHERCAT_DEVICE does not exist after EtherCAT master start."
    echo "  Check /etc/ethercat.conf MASTER0_DEVICE and run: sudo /etc/init.d/ethercat restart"
    return 1
  fi

  sudo chmod a+rw "$ETHERCAT_DEVICE" 2>/dev/null || true

  if [ ! -r "$ETHERCAT_DEVICE" ] || [ ! -w "$ETHERCAT_DEVICE" ]; then
    echo "  Error: current user cannot read/write $ETHERCAT_DEVICE."
    echo "  Current permissions: $(ls -l "$ETHERCAT_DEVICE" 2>/dev/null || true)"
    echo "  Fix with: sudo chmod a+rw $ETHERCAT_DEVICE"
    return 1
  fi

  echo "  EtherCAT device ready: $(ls -l "$ETHERCAT_DEVICE")"
}

pin_ethercat_master() {
  local ethercat_pids=""
  echo "Waiting for persistent EtherCAT worker to start..."
  for _ in $(seq 1 100); do
    ethercat_pids=$(pgrep -x "EtherCAT-OP" 2>/dev/null || true)
    if [ -n "$ethercat_pids" ]; then
      break
    fi
    ethercat_pids=$(pgrep -f "EtherCAT" 2>/dev/null || true)
    if [ -n "$ethercat_pids" ]; then
      break
    fi
    sleep 0.1
  done

  ETHERCAT_PIDS=$(printf "%s\n" "$ethercat_pids" | sort -n | uniq | tr '\n' ' ')
  if [ -n "$ETHERCAT_PIDS" ]; then
    for pid in $ETHERCAT_PIDS; do
      if sudo taskset -cp "$ETHERCAT_CORES" "$pid" > /dev/null 2>&1; then
        :
      else
        echo "  Warning: failed to pin EtherCAT PID $pid to cores $ETHERCAT_CORES"
      fi
      for tid in $(ls /proc/$pid/task/ 2>/dev/null); do
        sudo taskset -cp "$ETHERCAT_CORES" "$tid" > /dev/null 2>&1 || true
      done
      if sudo chrt -f -p $RT_PRIORITY $pid > /dev/null 2>&1; then
        :
      else
        echo "  Warning: failed to set FIFO priority on EtherCAT PID $pid"
      fi
      local affinity
      affinity=$(taskset -pc "$pid" 2>/dev/null | awk -F': ' '/current affinity list/ {print $2}')
      echo "  EtherCAT PID $pid → requested cores $ETHERCAT_CORES, actual affinity ${affinity:-unknown}, FIFO $RT_PRIORITY"

    done
  else
    echo "  Warning: EtherCAT process not found."
  fi
}

start_ethercat_repin_monitor() {
  (
    while true; do
      sleep "$AFFINITY_CHECK_PERIOD_S"
      local current_pids
      current_pids=$(pgrep -x "EtherCAT-OP" 2>/dev/null || true)
      [ -n "$current_pids" ] || current_pids=$(pgrep -f "EtherCAT" 2>/dev/null || true)
      [ -n "$current_pids" ] || continue

      local normalized
      normalized=$(printf "%s\n" "$current_pids" | sort -n | uniq | tr '\n' ' ' | sed 's/[[:space:]]*$//')
      [ -n "$normalized" ] || continue

      for pid in $normalized; do
        ensure_affinity "$ETHERCAT_CORES" "$pid"
        for task_path in /proc/"$pid"/task/*; do
          [ -d "$task_path" ] || continue
          ensure_affinity "$ETHERCAT_CORES" "${task_path##*/}"
        done
      done
    done
  ) &
}

pin_irqs() {
  echo "Pinning $NIC IRQs to cores $IRQ_CORES..."
  local irq_nums=""
  local affinity_mask="${IRQ_MASK#0x}"
  affinity_mask="${affinity_mask#0X}"
  if [ -z "$affinity_mask" ]; then
    affinity_mask="$(cpu_list_to_mask "$IRQ_CORES")"
  fi
  irq_nums=$(awk -v nic="$NIC" '$0 ~ nic {gsub(":", "", $1); print $1}' /proc/interrupts || true)

  if [ -z "$irq_nums" ]; then
    echo "  Warning: no IRQ found for $NIC in /proc/interrupts"
    return 0
  fi

  local irq_num=""
  for irq_num in $irq_nums; do
    if echo "$affinity_mask" | sudo tee "/proc/irq/${irq_num}/smp_affinity" > /dev/null 2>&1; then
      local effective=""
      effective=$(cat "/proc/irq/${irq_num}/effective_affinity_list" 2>/dev/null || true)
      echo "  IRQ $irq_num ($NIC) → mask $affinity_mask, effective CPUs ${effective:-unknown}"
    else
      echo "  Warning: failed to pin IRQ $irq_num ($NIC) to mask $affinity_mask"
    fi
  done
}

move_non_ethercat_irqs_off_isolated_cores() {
  [ -n "$MOVE_IRQS_AWAY_FROM" ] || return 0
  [ -n "$MOVE_IRQS_TO" ] || return 0

  local non_rt_mask
  non_rt_mask="$(cpu_list_to_mask "$MOVE_IRQS_TO")"
  local nic_irq_nums
  nic_irq_nums="$(awk -v nic="$NIC" '$0 ~ nic {gsub(":", "", $1); print $1}' /proc/interrupts || true)"

  echo "Moving non-EtherCAT IRQs off cores $MOVE_IRQS_AWAY_FROM (→ cores $MOVE_IRQS_TO)..."
  local irq_num=""
  while read -r irq_num; do
    [ -n "$irq_num" ] || continue
    if printf "%s\n" "$nic_irq_nums" | grep -qx "$irq_num"; then
      continue
    fi
    local effective=""
    effective=$(cat "/proc/irq/${irq_num}/effective_affinity_list" 2>/dev/null || true)
    if [ -n "$effective" ] && ! cpu_lists_intersect "$effective" "$MOVE_IRQS_AWAY_FROM"; then
      continue
    fi

    if echo "$non_rt_mask" | sudo tee "/proc/irq/${irq_num}/smp_affinity" > /dev/null 2>&1; then
      effective=$(cat "/proc/irq/${irq_num}/effective_affinity_list" 2>/dev/null || true)
      echo "  IRQ $irq_num → mask $non_rt_mask, effective CPUs ${effective:-unknown}"
    else
      local irq_name=""
      irq_name=$(awk -v irq="$irq_num" '$1 == irq ":" {$1=""; print}' /proc/interrupts 2>/dev/null || true)
      echo "  Warning: failed to move IRQ $irq_num off cores $MOVE_IRQS_AWAY_FROM${irq_name:+:$irq_name}"
    fi
  done < <(awk '/^[[:space:]]*[0-9]+:/ {gsub(":", "", $1); print $1}' /proc/interrupts)
}

stop_ethercat_master() {
  echo "Stopping EtherCAT..."
  sudo /etc/init.d/ethercat stop > /dev/null 2>&1 || true
  sudo pkill -f "EtherCAT" 2>/dev/null || true
}

pin_to_cpuset_cgroup() {
  local pid="$1"
  local cg_root="/sys/fs/cgroup"
  local cg_path="$cg_root/rt_control"

  # cpuset.cpus only appears in child cgroups if parent has cpuset in subtree_control
  if ! grep -q cpuset "$cg_root/cgroup.subtree_control" 2>/dev/null; then
    echo "+cpuset" | sudo tee "$cg_root/cgroup.subtree_control" > /dev/null 2>&1 || {
      echo "  [cpuset] cannot enable cpuset in root subtree_control — skipping"
      return 1
    }
  fi

  sudo mkdir -p "$cg_path" 2>/dev/null || true  # ignore if already exists

  if [ ! -f "$cg_path/cpuset.cpus" ]; then
    echo "  [cpuset] cpuset.cpus not available (subtree_control may be locked by systemd)"
    return 1
  fi

  echo "$CONTROL_CORES" | sudo tee "$cg_path/cpuset.cpus" > /dev/null 2>&1 || { echo "  [cpuset] cpus write failed"; return 1; }
  echo "0"               | sudo tee "$cg_path/cpuset.mems" > /dev/null 2>&1 || true

  # Writing PID to cgroup.procs moves ALL threads (current + future) — kernel-enforced
  if echo "$pid" | sudo tee "$cg_path/cgroup.procs" > /dev/null 2>&1; then
    echo "  [cpuset] PID $pid → rt_control cgroup (CPUs: $CONTROL_CORES) — kernel-enforced"
    return 0
  else
    echo "  [cpuset] cgroup.procs write failed for PID $pid — falling back to taskset loop"
    return 1
  fi
}

pin_ros2_control() {
  local ros2_ctrl_pid=""
  echo "Waiting for ros2_control_node to start..."
  for _ in $(seq 1 300); do
    ros2_ctrl_pid=$(pgrep -f "ros2_control_node" 2>/dev/null | sort -n | tail -1 || true)
    if [ -n "$ros2_ctrl_pid" ]; then
      break
    fi
    sleep 0.1
  done

  if [ -n "$ros2_ctrl_pid" ]; then
    sudo taskset -cp $CONTROL_CORES $ros2_ctrl_pid > /dev/null 2>&1
    echo "  ros2_control_node PID $ros2_ctrl_pid → cores $CONTROL_CORES"
    for tid in $(ls /proc/$ros2_ctrl_pid/task/ 2>/dev/null); do
      sudo taskset -cp $CONTROL_CORES $tid > /dev/null 2>&1
    done
    echo "  All threads of ros2_control_node set to cores $CONTROL_CORES"

    # Try kernel-enforced cpuset cgroup — prevents threads from self-reassigning cores
    pin_to_cpuset_cgroup "$ros2_ctrl_pid" || true

    # Background: verify affinity for late DDS/controller threads. Reads from
    # /proc are cheap; taskset is called only if a thread actually escaped.
    (
      local last_non_rt_scan=0
      while true; do
        sleep "$AFFINITY_CHECK_PERIOD_S"
        [ -d "/proc/$ros2_ctrl_pid" ] || break
        for task_path in /proc/"$ros2_ctrl_pid"/task/*; do
          [ -d "$task_path" ] || continue
          ensure_affinity "$CONTROL_CORES" "${task_path##*/}"
        done

        # Discover late non-RT processes at a lower rate. A single ps snapshot
        # replaces the previous 13 pgrep commands every 0.5 seconds.
        if (( SECONDS - last_non_rt_scan >= NON_RT_DISCOVERY_PERIOD_S )); then
          pin_non_rt_away 1
          last_non_rt_scan=$SECONDS
        fi
      done
      echo "  [RT] Thread re-pin loop finished"
    ) &
  else
    echo "  Warning: ros2_control_node not found after 30s"
  fi

  pin_non_rt_away

  echo ""
  echo "RT setup complete."
  echo "ros2_control_node status:"
  ps -eLo pid,tid,class,rtprio,psr,pcpu,stat,comm 2>/dev/null | grep -E "ros2_control|PID" | head -8 || true
}

pin_non_rt_away() {
  local quiet="${1:-0}"
  local non_rt_cores="$NON_RT_CORES"
  local planner_cores="$PLANNER_CORES"
  local low_priority_cores="$LOW_PRIORITY_CORES"
  local planner_procs=(
    move_group
    zeroerr_servo_node
    ipp_helper
    ruckig_helper
    contour_ik_helper
    ptp_helper
    linked_lin_helper
    trajectory_state_validator
  )
  local low_priority_procs=(
    rviz2
    zeroerr_state_publisher
    zeroerr_error_monitor.py
    zeroerr_drive_diagnostics.py
    ethercat_sdo_srv_server
    "main.py"
  )
  local procs=(
    move_group
    zeroerr_servo_node
    rviz2
    zeroerr_runtime.py
    zeroerr_state_publisher
    zeroerr_error_monitor.py
    zeroerr_drive_diagnostics.py
    ethercat_sdo_srv_server
    ipp_helper
    ruckig_helper
    contour_ik_helper
    ptp_helper
    linked_lin_helper
    trajectory_state_validator
    "main.py"
    spawner
    static_transform
  )
  if [ "$PIN_NON_RT_AWAY" != "1" ]; then
    if [ "$quiet" != "1" ]; then
      echo "Leaving non-RT ROS2 processes unpinned."
    fi
    return 0
  fi
  if [ "$quiet" != "1" ]; then
    echo "Pinning non-RT ROS2 processes away from RT cores (general=$non_rt_cores, planner=$planner_cores, low=$low_priority_cores)..."
  fi
  local pid=""
  local args=""
  while read -r pid args; do
    local target_cores="$non_rt_cores"
    local proc=""
    for proc in "${planner_procs[@]}"; do
      if [[ "$args" == *"$proc"* ]]; then
        target_cores="$planner_cores"
        break
      fi
    done
    for proc in "${low_priority_procs[@]}"; do
      if [[ "$args" == *"$proc"* ]]; then
        target_cores="$low_priority_cores"
        break
      fi
    done

    local proc=""
    for proc in "${procs[@]}"; do
      if [[ "$args" == *"$proc"* ]]; then
        for task_path in /proc/"$pid"/task/*; do
          [ -d "$task_path" ] || continue
          ensure_affinity "$target_cores" "${task_path##*/}"
        done
        break
      fi
    done
  done < <(ps -eo pid=,args=)
}

if [ "$STOP_ONLY" = "1" ]; then
  stop_ethercat_master
  exit 0
fi

if [ "$POSTSTART_ONLY" != "1" ]; then
  configure_cpu
  ensure_ethercat_master_running
  ensure_ethercat_device_access
  pin_ethercat_master
  if [ "${ZEROERR_MOVE_NON_ETHERCAT_IRQS:-0}" = "1" ]; then
    move_non_ethercat_irqs_off_isolated_cores
  fi
  pin_irqs
fi

if [ "$PREP_ONLY" = "1" ]; then
  exit 0
fi

start_ethercat_repin_monitor
pin_ethercat_master
pin_ros2_control
