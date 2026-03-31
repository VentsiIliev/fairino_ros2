#!/bin/bash
set -euo pipefail
# EtherCAT RT startup — supports PREP_ONLY=1 or POSTSTART_ONLY=1.

ISOLATED_CORES="${ZEROERR_ISOLATED_CORES:-14,15}"
ISOLATED_MASK="${ZEROERR_ISOLATED_MASK:-0xC000}"
IRQ_CORES="${ZEROERR_IRQ_CORES:-${ZEROERR_ETHERCAT_CORES:-14}}"
IRQ_MASK="${ZEROERR_IRQ_MASK:-0x4000}"
ETHERCAT_CORES="${ZEROERR_ETHERCAT_CORES:-14}"
CONTROL_CORES="${ZEROERR_CONTROL_CORES:-15}"
PIN_NON_RT_AWAY="${ZEROERR_PIN_NON_RT_AWAY:-0}"
NON_RT_CORES="${ZEROERR_NON_RT_CORES:-0-13}"
RT_PRIORITY=90
NIC="enp3s0"
PREP_ONLY="${PREP_ONLY:-0}"
POSTSTART_ONLY="${POSTSTART_ONLY:-0}"
STOP_ONLY="${STOP_ONLY:-0}"

configure_cpu() {
  echo "Setting CPU governor to performance..."
  for cpu in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
    echo performance | sudo tee "$cpu" > /dev/null
  done
  echo "  → Governor: $(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor)"

  local idle_cores
  idle_cores=$(printf "%s\n%s\n%s\n" "$ISOLATED_CORES" "$IRQ_CORES" "$CONTROL_CORES" | tr ',' '\n' | awk 'NF && !seen[$0]++')
  echo "Disabling deep idle states on RT-related cores: $(echo "$idle_cores" | paste -sd, -)..."
  for core in $idle_cores; do
    for state in /sys/devices/system/cpu/cpu${core}/cpuidle/state*/disable; do
      echo 1 | sudo tee "$state" > /dev/null 2>&1
    done
  done
}

restart_ethercat_master() {
  echo "Resetting EtherCAT..."
  sudo /etc/init.d/ethercat stop > /dev/null 2>&1 || true
  sleep 1
  sudo pkill -f "EtherCAT" 2>/dev/null || true
  sleep 1
  echo "Starting EtherCAT..."
  sudo /etc/init.d/ethercat start
  sleep 2
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

      (
        while true; do
          sleep 0.5
          [ -d "/proc/$pid" ] || break
          sudo taskset -cp "$ETHERCAT_CORES" "$pid" > /dev/null 2>&1 || true
          for tid in $(ls /proc/$pid/task/ 2>/dev/null); do
            sudo taskset -cp "$ETHERCAT_CORES" "$tid" > /dev/null 2>&1 || true
          done
        done
        echo "  [RT] EtherCAT re-pin loop finished for PID $pid"
      ) &
    done
  else
    echo "  Warning: EtherCAT process not found."
  fi
}

start_ethercat_repin_monitor() {
  (
    local seen=""
    while true; do
      sleep 0.5
      local current_pids
      current_pids=$(pgrep -x "EtherCAT-OP" 2>/dev/null || true)
      [ -n "$current_pids" ] || current_pids=$(pgrep -f "EtherCAT" 2>/dev/null || true)
      [ -n "$current_pids" ] || continue

      local normalized
      normalized=$(printf "%s\n" "$current_pids" | sort -n | uniq | tr '\n' ' ' | sed 's/[[:space:]]*$//')
      [ -n "$normalized" ] || continue

      if [ "$normalized" != "$seen" ]; then
        echo "Refreshing EtherCAT affinity for PID set: $normalized"
        for pid in $normalized; do
          sudo taskset -cp "$ETHERCAT_CORES" "$pid" > /dev/null 2>&1 || true
          for tid in $(ls /proc/$pid/task/ 2>/dev/null); do
            sudo taskset -cp "$ETHERCAT_CORES" "$tid" > /dev/null 2>&1 || true
          done
          local affinity
          affinity=$(taskset -pc "$pid" 2>/dev/null | awk -F': ' '/current affinity list/ {print $2}')
          echo "  EtherCAT monitor PID $pid → actual affinity ${affinity:-unknown}"
        done
        seen="$normalized"
      fi
    done
  ) &
}

pin_irqs() {
  echo "Pinning $NIC IRQs to cores $IRQ_CORES..."
  for irq_dir in /proc/irq/*/; do
    irq_num=$(basename "$irq_dir")
    if ls "$irq_dir" 2>/dev/null | grep -q "$NIC"; then
      echo $IRQ_MASK | sudo tee "/proc/irq/${irq_num}/smp_affinity" > /dev/null 2>&1
      echo "  IRQ $irq_num ($NIC) → mask $IRQ_MASK"
    fi
  done
}

stop_ethercat_master() {
  echo "Stopping EtherCAT..."
  sudo /etc/init.d/ethercat stop > /dev/null 2>&1 || true
  sudo pkill -f "EtherCAT" 2>/dev/null || true
}

pin_to_cpuset_cgroup() {
  local pid="$1"
  local cg_root="/sys/fs/cgroup"
  local cg_path="$cg_root/rt_ethercat"

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

    # Background: keep re-pinning in case ROS 2 / DDS spawns threads late or threads escape.
    (
      while true; do
        sleep 0.5
        [ -d "/proc/$ros2_ctrl_pid" ] || break
        for tid in $(ls /proc/$ros2_ctrl_pid/task/ 2>/dev/null); do
          sudo taskset -cp $CONTROL_CORES $tid > /dev/null 2>&1
        done

        # Optionally keep known non-RT processes off the isolated cores as they come and go.
        pin_non_rt_away 1
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
  local procs=(move_group rviz2 zeroerr_state_publisher ipp_helper ruckig_helper "main.py" spawner static_transform)
  if [ "$PIN_NON_RT_AWAY" != "1" ]; then
    if [ "$quiet" != "1" ]; then
      echo "Leaving non-RT ROS2 processes unpinned."
    fi
    return 0
  fi
  if [ "$quiet" != "1" ]; then
    echo "Pinning non-RT ROS2 processes away from RT cores (→ cores $non_rt_cores)..."
  fi
  local proc=""
  for proc in "${procs[@]}"; do
    for pid in $(pgrep -f "$proc" 2>/dev/null || true); do
      taskset -cp "$non_rt_cores" "$pid" > /dev/null 2>&1 || true
    done
  done
}

if [ "$STOP_ONLY" = "1" ]; then
  stop_ethercat_master
  exit 0
fi

if [ "$POSTSTART_ONLY" != "1" ]; then
  configure_cpu
  restart_ethercat_master
  pin_ethercat_master
  pin_irqs
fi

if [ "$PREP_ONLY" = "1" ]; then
  exit 0
fi

start_ethercat_repin_monitor
pin_ethercat_master
pin_ros2_control
