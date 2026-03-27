# RT CPU Isolation Fix — EtherCAT Missed Cycles

## Problem

After startup, `ros2_control_node` on the ZeroErr robot was missing EtherCAT cycles with overruns up to **187ms** on a `SCHED_FIFO-90` thread. Timing showed the missed cycles started ~0.55 seconds after safety walls were published — the moment `move_group` processes the `PlanningScene` collision objects update.

`SCHED_FIFO` prevents normal *processes* from preempting RT threads, but not kernel activities: timer interrupts, softirqs, RCU callbacks, and scheduler load-balancing. A 187ms overrun on a FIFO-90 thread is a **kernel-level preemption**, not userspace. `isolcpus` is the only proper fix.

---

## Fix: Kernel CPU Isolation

Isolate cores 14 and 15 from the kernel scheduler so only explicitly pinned processes run on them.

### 1. Edit GRUB

```bash
sudo nano /etc/default/grub
```

Find `GRUB_CMDLINE_LINUX_DEFAULT` and add the isolation parameters:

```
GRUB_CMDLINE_LINUX_DEFAULT="quiet splash threadirqs mitigations=off nohz_full=all rcu_nocbs=all isolcpus=14,15"
```

- `isolcpus=14,15` — removes cores 14,15 from the kernel scheduler
- `nohz_full=all` — suppresses timer ticks on non-housekeeping cores
- `rcu_nocbs=all` — offloads RCU callbacks away from isolated cores
- `threadirqs` — moves IRQ handlers to kernel threads (so they can be pinned)
- `mitigations=off` — disables Spectre/Meltdown mitigations for lower latency

### 2. Update GRUB config

```bash
sudo update-grub
```

### 3. Fix the RT kernel entry (one-time)

The RT kernel (`6.14.0-rt3`) uses a custom GRUB entry in `/etc/grub.d/proxifiedScripts/custom` that does **not** inherit `GRUB_CMDLINE_LINUX_DEFAULT`. Patch it manually:

```bash
sudo sed -i 's|ro  quiet splash \$vt_handoff|ro  quiet splash threadirqs mitigations=off nohz_full=all rcu_nocbs=all isolcpus=14,15 $vt_handoff|' /etc/grub.d/proxifiedScripts/custom
sudo update-grub
```

Verify the RT kernel entry has the parameters:
```bash
sudo grep "6.14.0-rt3" /boot/grub/grub.cfg | grep "^.*linux.*vmlinuz"
```

### 4. Reboot into the RT kernel

Select **"Ubuntu 24.04 Realtime"** from the GRUB menu.

---

## Verification

After reboot, confirm isolation is active:

```bash
cat /sys/devices/system/cpu/isolated    # → 14-15
cat /sys/devices/system/cpu/nohz_full   # → 1-15 (or 14-15)
cat /proc/cmdline | grep isolcpus       # → should contain isolcpus=14,15
```

Confirm RT pinning is correct after launching the robot stack:

```bash
# EtherCAT master: should show cores 14 or 15, FIFO policy
ps -eLo pid,tid,class,rtprio,psr,comm | grep -i ethercat

# ros2_control_node: should show cores 14 or 15, FIFO-90
ps -eLo pid,tid,class,rtprio,psr,comm | grep ros2_control

# NIC IRQ affinity (enp3s0): should show C000 (mask for cores 14,15)
grep -l enp3s0 /proc/irq/*/node 2>/dev/null | while read f; do
  irq=$(echo $f | cut -d/ -f4)
  echo "IRQ $irq: $(cat /proc/irq/$irq/smp_affinity)"
done
```

---

## How the RT Pinning Works

The script `eRob_moveit/src/eRob_ROS2_MoveIt/zeroerr/scripts/EtherCatStart.sh` handles all RT setup:

| Process | Cores | Policy | Priority |
|---------|-------|--------|----------|
| EtherCAT master | 14,15 | FIFO | 90 |
| ros2_control_node (all threads) | 14,15 | FIFO | 90 |
| enp3s0 NIC IRQs | 14,15 (mask 0xC000) | — | — |
| move_group, rviz2, ipp_helper, etc. | 0–13 | normal | — |

The script is called automatically by `launch_robot.sh` for the ZeroErr robot.

---

## If the Issue Recurs

**Check 1 — Are the cores still isolated after reboot?**
```bash
cat /sys/devices/system/cpu/isolated
```
If empty: the wrong kernel booted (generic instead of RT), or GRUB defaulted to a different entry.

**Check 2 — Which kernel is running?**
```bash
uname -r    # should show 6.14.0-rt3
```

**Check 3 — Did the GRUB custom entry lose its parameters?** (happens after kernel/grub package updates)
```bash
sudo grep "6.14.0-rt3" /boot/grub/grub.cfg | grep "^.*linux.*vmlinuz"
```
If `isolcpus` is missing, re-run the sed patch in step 3 above and `sudo update-grub`.

**Check 4 — Is ros2_control actually on the isolated cores?**
```bash
ps -eLo pid,tid,class,rtprio,psr,comm | grep ros2_control
```
`psr` column should show 14 or 15. If not, run `EtherCatStart.sh` with `POSTSTART_ONLY=1`:
```bash
POSTSTART_ONLY=1 ./eRob_moveit/src/eRob_ROS2_MoveIt/zeroerr/scripts/EtherCatStart.sh
```

**Check 5 — Is the planning scene update still causing spikes?**
Missed cycles immediately after wall publication suggest `move_group` is leaking onto isolated cores — confirm with:
```bash
ps -eLo pid,psr,comm | grep move_group   # should never show 14 or 15
```