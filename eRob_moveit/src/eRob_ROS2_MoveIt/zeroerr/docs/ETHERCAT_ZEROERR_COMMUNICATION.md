# ZeroErr EtherCAT Communication Log

This document tracks ZeroErr EtherCAT communication issues, suspected causes, fixes tried, and test results. Keep entries chronological and include the exact log file, launch command, code/config changes, and physical setup notes whenever possible.

## Current System Notes

- EtherCAT master: reported as `EtherCAT master 1.6.8` in the startup log.
- Robot package: `zeroerr`
- Expected slaves: `6`
- EtherCAT startup script: `zeroerr/scripts/EtherCatStart.sh`
- OP wait script: `zeroerr/scripts/WaitForSlavesOp.sh`
- Slave monitor script: `zeroerr/scripts/PollEthercat.sh`
- Drive configs:
  - `zeroerr/config/zeroErr.yaml`
  - `zeroerr/config/zeroErr1.yaml`
- ROS 2 control config:
  - `zeroerr/config/urdfs/eRobo3.ros2_control.xacro`
  - `zeroerr/config/ros2_controllers.yaml`
- Runtime watchdog:
  - `erob_moveit_runtime/scripts/robot_controller.py`

## Investigation Entry Template

Use this template for each new issue or fix attempt.

```md
### YYYY-MM-DD - Short Title

**Setup**
- Robot/profile:
- Launch command:
- EtherCAT cabling/power notes:
- Code/config commit or local diff:
- Log file:

**Observed Symptoms**
- 

**Relevant Log Evidence**
- 

**Suspected Cause**
- 

**Fix Applied**
- 

**Test Procedure**
- 

**Result**
- Pass/fail:
- What changed:
- Remaining issue:

**Next Action**
- 
```

## 2026-06-26 - Startup OP Instability and Drive Velocity Error

**Setup**

- Robot/profile: ZeroErr, `ACTIVE_PROFILE: paint`
- Log file: `zeroerr/config/paint/temp_logs.txt`
- Expected EtherCAT slaves: `6`
- Physical event during test: EtherCAT cable was unplugged and plugged back in by the operator, so later recovery to OP should not be interpreted as pure software recovery.

**Observed Symptoms**

- EtherCAT initially starts and `ros2_control_node` reports all modules operational.
- `WaitForSlavesOp.sh` later reports all 6 slaves in OP.
- Before the runtime node starts, multiple drives report `0x8400 velocity error exceeds the limit value`.
- Runtime watchdog later reports slaves not fully OP, then OP count recovers progressively after the physical cable replug.

**Relevant Log Evidence**

- Startup:
  - `Starting EtherCAT master 1.6.8 done`
  - `EthercatDriver`: `Initialization progress: 6/6 modules operational`
  - `EthercatDriver`: `System started (modules operational)`
- OP wait:
  - `WaitForSlavesOp.sh`: `Waiting for 6 EtherCAT slaves to reach OP...`
  - `WaitForSlavesOp.sh`: `All 6 EtherCAT slaves are in OP.`
- Drive faults before runtime:
  - Slave 0: `error_code=0x8400`, `target_position_0x607A=0`, `actual_position_0x6064=136300`
  - Slave 1: `error_code=0x8400`, `target_position_0x607A=0`, `actual_position_0x6064=4297`
  - Slave 2: `error_code=0x8400`, `target_position_0x607A=0`, `actual_position_0x6064=-181269`
  - Slave 3: `error_code=0x8400`, `target_position_0x607A=0`, `actual_position_0x6064=-56067`
  - Slave 4: `error_code=0x8400`, `target_position_0x607A=0`, `actual_position_0x6064=-130717`
  - Slave 5: `error_code=0x8400`, `target_position_0x607A=0`, `actual_position_0x6064=-18351`
- Runtime watchdog after startup/cable event:
  - `EtherCAT not fully OP (0/6 OP, expected 6)`
  - then `1/6`, `2/6`, `3/6`, `5/6`
  - finally `All slaves back in OP - motion interlock cleared`

**Interpretation**

There are two separate effects in this log:

1. The later OP recovery is explained by the cable being unplugged and plugged back in. That recovery should not be treated as proof that the software recovery path is good.
2. The earlier drive faults are independent of the cable replug. The strongest clue is that `0x607A` target position is `0` while actual positions are far from zero. That can produce a large immediate following/velocity error when the drives enter cyclic synchronous position mode.

**Suspected Cause**

Primary suspected cause: startup command-position synchronization is incomplete. The position command interface can publish or retain `0`, which then reaches drive object `0x607A` before the command has been synchronized to the actual drive position from `0x6064`.

Secondary suspected cause: launch sequencing does not fully gate startup on stable OP. `WaitForSlavesOp.sh` runs, but many nodes/controllers are already started while the OP wait is still active. This means the wait script is partly observational rather than a hard startup barrier.

Distributed clocks are already enabled in the drive YAMLs:

- `dc_mode: true`
- `assign_activate: 0x0300`

So DC synchronization may still need tuning later, but this log points first to target-position synchronization and launch ordering.

**Relevant Code/Config Findings**

- `zeroerr/config/zeroErr.yaml` maps `0x607A` as the position command:
  - `command_interface: position`
  - `default: .nan`
- `zeroerr/config/zeroErr1.yaml` does the same with the opposite factor sign.
- `ethercat_generic_cia402_drive.cpp` tries to set the default target position from the last actual position, but command-interface values can still override this when they are non-NaN.
- `full_stack.launch.py` starts `WaitForSlavesOp.sh`, but controller spawners, SDO monitor, helpers, and other nodes are not all gated on its completion.

**Fix Candidates**

1. Synchronize command position to actual position before allowing CSP enable.
   - On hardware activation/startup, read `0x6064` actual position.
   - Write the corresponding value into the ROS position command interface.
   - Only then allow `0x607A` command updates to pass through.

2. In the CiA402 plugin, hold target position at actual position until a valid controller command has been intentionally initialized.
   - Avoid treating `0` as a valid initial command immediately after startup.
   - Keep using actual position as the target during non-initialized or mode-transition cycles.

3. Make the OP wait a real launch barrier.
   - Start the minimum required EtherCAT/ros2_control path.
   - Wait for stable OP.
   - Then spawn controllers and start runtime monitors.
   - Delay SDO polling/error monitor until after the bus is stable, because SDO failures during bus transitions add noise and may increase startup load.

4. Add better logging around target-position initialization.
   - Log first actual positions from `0x6064`.
   - Log first command positions sent to `0x607A`.
   - Log when command positions are considered synchronized.
   - Log OP state transitions with timestamps and physical-event notes when possible.

**Testing Plan**

1. Baseline without applying fixes:
   - Start with robot powered and EtherCAT cable untouched.
   - Launch the ZeroErr demo.
   - Confirm whether `0x8400` appears before any physical cable event.
   - Record OP state count every second for at least 60 seconds.

2. Apply command-position sync fix.
   - Relaunch without touching the cable.
   - Expected result: no startup `0x8400` caused by `target_position_0x607A=0`.
   - Expected result: runtime watchdog should not report `0/6 OP` unless there is a real bus/cable event.

3. Apply launch sequencing fix.
   - Confirm controllers and runtime monitors start only after stable OP.
   - Expected result: fewer SDO read failures during startup and clearer OP transition logs.

4. Physical cable test, only after software startup is stable.
   - Unplug/replug EtherCAT cable intentionally.
   - Confirm watchdog detects loss of OP.
   - Confirm system blocks motion while not fully OP.
   - Decide whether automatic motion recovery is allowed or whether operator acknowledgement is required.

**Result**

- No software fix applied yet in this entry.
- Current result: issue documented from `temp_logs.txt`.
- Next action: implement command-position synchronization and improve launch gating, then add a new dated test entry below with results.

## Open Questions

- Does `ros2_control` initialize command interfaces to `0` for this hardware path before the first valid joint state is read?
- Should the CiA402 plugin reject or ignore `0` position commands during the first startup cycles unless explicitly initialized?
- Should `auto_enable_set: true` remain enabled, or should enable operation be delayed until command position sync is confirmed?
- Should the runtime require manual acknowledgement after a real EtherCAT cable disconnect, instead of clearing the interlock automatically when OP returns?

## 2026-06-26 - Fix Attempt: Gate CSP Enable Until Command Is Valid

**Setup**

- Latest log: `zeroerr/config/paint/temp_logs.txt`
- Relevant symptom: startup still reports `target_position_0x607A=0` with nonzero actual positions. After the first trajectory command, the errors quiet out and the operator hears a click, suggesting the drives only settle into enable/hold behavior when the first real controller command arrives.

**Observed Symptoms**

- Before first command, drives can be in `switched_on`, `switch_on_disabled`, or `fault`, not consistently `operation_enabled`.
- `mode_display=[8, 8, 8, 8, 8, 8]`, so the drives are in CSP mode while the target is still zero.
- First runtime trajectory starts from the live joint state, for example `First point positions: [1.633465, -0.051496, -2.172277, 0.671921, -1.566509, -0.219947]`.

**Fix Applied**

- Changed the CiA402 plugin to block the final automatic `STATE_SWITCH_ON -> STATE_OPERATION_ENABLED` transition in CSP mode until the ROS position command is finite and within `0.005 rad` of the measured `0x6064` position.
- During that pre-enable window, `0x607A` is held at the measured actual position via the PDO default. This does not modify the ROS controller command interface and does not command torque/drag mode.

**Expected Result**

- Startup should no longer send raw `0` into `0x607A` while actual position is far from zero.
- The robot should not enter operation-enabled until the controller has produced a sane hold/motion command close to the actual joint position.
- The first command may still produce an enable click, but it should not be preceded by repeated `0x8400` target/actual mismatch errors or buzzing caused by zero target.

**Test Procedure**

- Rebuild `ethercat_generic_cia402_drive`.
- Launch ZeroErr without touching the EtherCAT cable.
- Before the first command, check `temp_logs.txt` for any `target_position_0x607A=0` with large `target_minus_actual`.
- Send the same small first command and confirm there is no startup drift and no torque/drag-mode activation.

**Result**

- Build passed:
  - `colcon build --packages-select ethercat_generic_cia402_drive --cmake-args -DBUILD_TESTING=OFF`
- First hardware result: partial improvement only. Slaves 0, 2, and 3 showed `0x607A` close to actual position, but slaves 1 and 5 still sent `0` because their actual offsets were inside the original loose `0.25 rad` threshold.
- Follow-up change: tightened the threshold to `0.005 rad` so the startup zero command is rejected for slaves 1 and 5 too.
- Follow-up hardware result pending.

**Additional Finding From Same Log**

- `controller_manager` reports `Joint_3` current/command position near `-2.172 rad` being limited to `-1.570 rad`.
- Source URDF files checked so far show Joint_3 limits around `[-3.14, 3.14]`, so this may come from a stale installed robot description or another generated limit source.
- This is separate from the EtherCAT `0x607A=0` problem, but it is safety-relevant because a controller-side clamp can command a position away from the measured startup pose.

## 2026-06-26 - Fix Attempt: REST-Controlled Drive Operation Enable

**Reason**

- After the CSP enable gate, the drives can intentionally remain in CiA402 `switched_on` with no active errors.
- The platform needs an explicit way to request final `operation_enabled` after it has selected the active tool and is ready to hold/move.

**Fix Applied**

- Added CiA402 plugin support for `enable_set` and `disable_set` command interfaces.
- Default startup request is disabled, so drives remain in `switched_on`.
- `enable_set` rising edge requests Operation Enabled.
- Until a valid near-current controller command appears, `0x607A` remains held at measured actual position. The plugin hands over to the controller after the first valid command is accepted.
- `disable_set` rising edge requests return to `switched_on`.
- Added REST endpoints:
  - `POST /drive/enable`
  - `POST /drive/disable`
- Runtime activates only dedicated `drive_enable_set_controller` / `drive_disable_set_controller` for this path.
- ZeroErr launch spawns only the dedicated drive enable/disable set controllers for this path.

**Test Procedure**

- Build `ethercat_generic_cia402_drive`, `erob_moveit_runtime`, and `zeroerr`.
- Launch and confirm startup snapshot is `switched_on` with `0x0000` errors.
- Call `POST /tool/active` as usual.
- Call `POST /drive/enable`.
- Confirm status moves from `switched_on` to `operation_enabled` without startup drift.
- Call `POST /drive/disable` and confirm status returns to `switched_on`.

**Result**

- Build passed:
  - `colcon build --packages-select ethercat_generic_cia402_drive erob_moveit_runtime zeroerr --cmake-args -DBUILD_TESTING=OFF`
  - `python3 -m py_compile erob_moveit_runtime/scripts/rest/server.py erob_moveit_runtime/scripts/robot_controller.py`
- Hardware result pending.

## 2026-06-26 - Fix Attempt: Watchdog OP Recovery Request

**Observed Symptom**

- After `80+` consecutive missed EtherCAT cycles, the runtime watchdog reports:
  - `EtherCAT not fully OP (5/6 OP, expected 6)`
  - later `4/6 OP`
- The runtime only stopped motion and interlocked; it did not request recovery.

**Cause**

- Runtime watchdog was intentionally motion-safety only: it polled `ethercat slaves`, stopped motion, and kept the interlock active until all slaves returned to OP.
- The low-level master printed `Requesting immediate state recovery to OP`, but that path only re-read the slave config state and did not issue `ethercat states OP`.

**Fix Applied**

- Added a rate-limited recovery request in the runtime watchdog:
  - command: `ethercat states OP`
  - default enabled by `ETHERCAT_RECOVERY_ENABLED: true`
  - minimum interval: `ETHERCAT_RECOVERY_MIN_INTERVAL_S: 2.0`
  - timeout: `ETHERCAT_RECOVERY_CMD_TIMEOUT_S: 2.0`
- Motion remains interlocked until the watchdog later observes all expected slaves back in OP.

**Result**

- Build passed:
  - `python3 -m py_compile erob_moveit_runtime/scripts/config.py erob_moveit_runtime/scripts/robot_controller.py`
  - `colcon build --packages-select erob_moveit_runtime zeroerr --cmake-args -DBUILD_TESTING=OFF`
- Hardware result pending.

## 2026-06-26 - Fix Attempt 1: Seed Position Command From Actual Position

**Setup**

- Package changed: `ethercat_generic_cia402_drive`
- Files changed:
  - `ethercat_generic_cia402_drive/include/ethercat_generic_plugins/generic_ec_cia402_drive.hpp`
  - `ethercat_generic_cia402_drive/src/generic_ec_cia402_drive.cpp`
  - `ethercat_generic_cia402_drive/test/test_generic_ec_cia402_drive.cpp`
  - `ethercat_generic_cia402_drive/test/test_generic_ec_cia402_drive.hpp`

**Fix Applied**

The CiA402 plugin now tracks whether the position command has been synchronized after the drive reaches operation-enabled state.

Before allowing the ROS position command interface to drive `0x607A`, the plugin:

- waits until the drive is initialized/operation-enabled,
- requires a valid last actual position from `0x6064`,
- writes that actual position into the ROS position command interface,
- keeps overriding the target position with the actual-position default until this synchronization has happened.

If the drive enters fault or loses operation-enabled state long enough to reinitialize, the sync flag is cleared so the next OP entry repeats the same safe startup synchronization.

**Expected Effect**

This should prevent the startup condition seen in `temp_logs.txt` where:

- `target_position_0x607A=0`
- actual position is nonzero
- the drive raises `0x8400 velocity error exceeds the limit value`

**Verification**

- Added a unit test for the exact unsafe case:
  - command interface starts at `0`
  - actual position is `123456`
  - plugin must write `123456` to `0x607A`, not `0`
- Full test build with default settings could not run because this environment is missing `ament_lint_auto`.
- Runtime library build passed from the main workspace:
  - command: `colcon build --packages-select ethercat_generic_cia402_drive --cmake-args -DBUILD_TESTING=OFF`
  - result: passed

**Hardware Test Needed**

Run the normal ZeroErr launch without touching the EtherCAT cable and check:

- no startup `0x8400` velocity errors,
- no `target_position_0x607A=0` while actual position is nonzero,
- all 6 slaves remain OP after startup,
- runtime watchdog does not report `0/6 OP` unless there is a real cable/bus event.

## 2026-06-26 - Test Result: First Fix Partially Works, Real Command Stops Errors

**Setup**

- Log file: `zeroerr/config/paint/temp_logs.txt`
- Test condition: launched with Fix Attempt 1 applied.
- Operator finding: after sending a motion command, the repeated drive error logs stop.

**Observed Result**

Fix Attempt 1 improved the target-position mismatch:

- Most reported `target_position_0x607A` values are now near `actual_position_0x6064`.
- The old pattern of every slave starting with `target_position_0x607A=0` is mostly gone.

However, there are still isolated startup zero-target events:

- Slave 5: `target_position_0x607A=0`, `actual_position_0x6064=-18350`
- Slave 4: `target_position_0x607A=0`, `actual_position_0x6064=-130714`
- Later log: slave 1 also showed `target_position_0x607A=0`, `actual_position_0x6064=4297`

When the first motion command is sent:

- `manipulator_controller` reports `Received new action goal`
- `manipulator_controller` reports `Accepted new action goal`
- after that, error logs reduce to warnings with small target-minus-actual values

**Interpretation**

The command path becomes healthy after a real trajectory goal because the trajectory controller starts publishing coherent position targets. Before any user command, one or more controllers can still write a default/stale `0` command into the position command interface after the plugin's initial seed.

This means startup still needs protection against discontinuous position commands written by controllers during the pre-motion window.

The same log also shows a separate realtime issue:

- `Missed cycle! Expected: 1000000 ns, Actual: 90850935 ns`
- `90 consecutive missed cycles`
- controller manager overrun around `90 ms`

That can drop slaves out of OP independently of target-position sync.

## 2026-06-26 - Fix Attempt 2: Reject Discontinuous Startup Position Commands

**Fix Applied**

The CiA402 plugin now rejects discontinuous position commands relative to the latest actual position:

- If the command interface value differs from `last_position_` by more than `0.25 rad`, the plugin overwrites the command back to `last_position_`.
- The plugin sends the safe actual-position default to `0x607A` for that cycle.
- This catches late controller defaults such as `0` after initial plugin sync.

This is intended to protect startup and fault-recovery transitions. Normal trajectory execution should still work because a properly generated trajectory starts near the current joint state and advances in small increments.

**Additional Realtime Fix**

The EtherCAT driver no longer hard-pins its control thread to CPU 0 by default.

Previously, the driver thread forced:

- `CPU_SET(0, &cpuset)`

This could fight the launch scripts/cgroups that are trying to isolate EtherCAT and `ros2_control` on dedicated CPUs. The driver now only pins if the hardware parameter `control_thread_cpu` is explicitly set. Otherwise it inherits the process/cgroup affinity.

**Verification**

Built from the main workspace:

```bash
colcon build --packages-select ethercat_driver ethercat_generic_cia402_drive --cmake-args -DBUILD_TESTING=OFF
```

Result: build passed.

Full package tests still require installing `ament_lint_auto`.

**Next Hardware Test**

Relaunch and check:

- no `target_position_0x607A=0` entries after OP startup,
- no `0x8400` velocity errors before the first user motion command,
- missed cycle count is reduced or absent,
- OP remains stable before and after sending the first command.

## 2026-06-26 - Test Result: Slave 6 Stuck Below OP

**Setup**

- Log file: `zeroerr/config/paint/temp_logs.txt`
- Observed by operator: slave 6 remains in `PREOP + E` while the other slaves are OP.

**Relevant Log Evidence**

- `EthercatDriver` initially reports: `Initialization progress: 6/6 modules operational`
- Later, runtime watchdog reports: `EtherCAT not fully OP (5/6 OP, expected 6)`
- Runtime drive-state snapshot:
  - statusword: `[5815, 4744, 5815, 5815, 5815, 0]`
  - status state: `['operation_enabled', 'fault', 'operation_enabled', 'operation_enabled', 'operation_enabled', 'not_ready_to_switch_on']`
  - mode display: `[8, 8, 8, 8, 8, 0]`

Index 5 / slave 6 has statusword `0` and mode display `0`, meaning the runtime is not receiving a normal CiA402 PDO state from that drive. This is consistent with the slave not reaching OP or dropping out of OP.

**Important Launch Finding**

`WaitForSlavesOp.sh` did not log `All 6 EtherCAT slaves are in OP` in this run, but `zeroerr_runtime.py` still started and issued an EtherCAT motion interlock.

That means the launch sequence was still allowing runtime and diagnostic nodes to start while the OP wait was unresolved.

**Fix Applied**

Updated `zeroerr/launch/full_stack.launch.py` so these components start only after `WaitForSlavesOp.sh` exits:

- `zeroerr_error_monitor.py`
- drive enable/disable controller spawners
- `zeroerr_runtime.py`
- collision monitor and optional collision GUI were already gated on the OP wait

Built from the main workspace:

```bash
colcon build --packages-select zeroerr --cmake-args -DBUILD_TESTING=OFF
```

Result: build passed.

**Next Hardware Test**

Relaunch and verify:

- If slave 6 remains `PREOP + E`, `zeroerr_runtime.py` should not start.
- `zeroerr_error_monitor.py` and drag controllers should not start before all 6 slaves are OP.
- The log should stop before runtime startup and make the slave-6 OP failure easier to isolate.

If slave 6 still stays `PREOP + E`, capture `ethercat slaves -v` while the launch is waiting. The exact AL status code/text for slave 6 is needed to decide whether this is:

- PDO/Sync Manager configuration rejection,
- distributed-clock/sync issue,
- watchdog/working-counter issue,
- drive-side application fault preventing OP,
- physical link/power problem at the last slave.

## 2026-06-26 - Test Result: OP Gate Passes, Slave 6 Still Gets Zero Target Before First Command

**Setup**

- Log file: `zeroerr/config/paint/temp_logs.txt`
- Test condition: launch gating applied so runtime/monitor nodes start after `WaitForSlavesOp.sh`.
- Operator finding: when the first motion command is sent, the repeated drive error logging quiets down.

**Relevant Log Evidence**

- `EthercatDriver`: `Initialization progress: 6/6 modules operational`
- `EthercatDriver`: `System started (modules operational)`
- `WaitForSlavesOp.sh`: `All 6 EtherCAT slaves are in OP.`
- After gated startup, runtime still reports an OP drop:
  - `EtherCAT not fully OP (0/6 OP, expected 6)`
  - then `1/6`, `2/6`, `3/6`, `4/6`, `5/6`
  - then `All slaves back in OP - motion interlock cleared`
- Runtime startup snapshot shows the drives are already faulted or disabled:
  - status state: `['fault', 'switch_on_disabled', 'fault', 'fault', 'fault', 'fault']`
  - error code: `['0xA000', '0x8400', '0xA000', '0xA000', '0xA000', '0x8400']`
- Slave 6 still shows the bad pre-command target case:
  - `target_position_0x607A=0`
  - `actual_position_0x6064=-18351`
  - `target_minus_actual=18351`

**Interpretation**

The OP wait is now working as a launch barrier: all six slaves reached OP before the gated monitor/runtime nodes started.

The remaining issue is the pre-command position target. The CiA402 guard from Fix Attempt 2 was still too narrow because it only rejected discontinuous commands after the drive was marked initialized and position-synced. In the observed failing window, slave 6 can still be transitioning, fault-recovering, or reporting `mode_display=0`, so a default/stale `0` can still be sent or retained before the first real trajectory command arrives.

The fact that the error logging quiets after the first command supports this: once the trajectory controller receives a real goal, it starts publishing coherent targets near the actual joint position.

**Fix Applied**

Tightened the CiA402 target-position guard in `ethercat_generic_cia402_drive`:

- Whenever a valid actual position from `0x6064` is known, use it as the default target for `0x607A`, even if mode display is still `0`.
- Before the drive is fully initialized/synced, reject a NaN or discontinuous position command and overwrite the command interface with the latest actual position.
- Continue overriding the RPDO command with the safe actual-position default until CSP position command synchronization is complete.

Added a regression test for the observed slave-6 case:

- mode display is still `0`,
- command interface value is `0`,
- actual position is `-18351`,
- expected target written to `0x607A` is `-18351`.

**Expected Effect**

This should prevent the remaining startup case where slave 6 sees:

- `target_position_0x607A=0`
- `actual_position_0x6064=-18351`

before the first user motion command.

**Next Hardware Test**

Relaunch without sending a motion command for at least 30 seconds and check:

- no slave 6 `target_position_0x607A=0` while actual is nonzero,
- no startup `0x8400` caused by target/actual mismatch,
- runtime does not see a post-gate `0/6 OP` drop,
- after the first command, errors remain quiet as before.

## 2026-06-26 - Safety Revert: Do Not Rewrite Position Command During Initialization

**Observed Result**

- The CiA402 target-position guard that overwrote the position command from actual position caused robot drift during launch/initialization.
- This is unsafe because initialization must not create motion.

**Reverted**

- Removed the code that writes `last_position_` into the ROS position command interface.
- Removed the code that rejects/overwrites startup position commands based on actual position delta.
- Removed the tests added for that behavior.

**Current Safety Rule**

Do not modify the commanded joint position in the EtherCAT plugin during initialization. Future fixes for the pre-command `0x8400` issue should hold enable/motion state, delay controller activation, or prevent operation-enabled transition until the controller has a valid hold command, rather than changing target position inside the EtherCAT PDO write path.

## 2026-06-26 - Fix Attempt 3: Keep Trajectory Controller Inactive Until First Real Command

**Setup**

- Log file: `zeroerr/config/paint/temp_logs.txt`
- Operator note: slaves only recovered to OP after unplugging/replugging the EtherCAT cable.
- Safety constraint from previous test: do not rewrite position commands from actual position inside the EtherCAT plugin, because that caused drift during launch.

**Observed Symptoms**

- EtherCAT driver reports all modules operational.
- `WaitForSlavesOp.sh` reports all 6 slaves in OP.
- Before the first user motion command, every drive repeatedly reports `target_position_0x607A=0` while actual position is nonzero.
- Runtime later reports EtherCAT not fully OP, then OP count recovers after the physical cable replug.
- After the first real trajectory command is accepted by `manipulator_controller`, target positions become sane and the repeated errors quiet down.

**Relevant Log Evidence**

- `manipulator_controller` is loaded and activated before the OP wait completes:
  - `Loading controller : 'manipulator_controller'`
  - `Activating controllers: [ manipulator_controller ]`
  - `Configured and activated manipulator_controller`
- Later, after OP wait succeeds, the error monitor shows all drives with zero target:
  - slave 0: `target_position_0x607A=0`, `actual_position_0x6064=34737`
  - slave 1: `target_position_0x607A=0`, `actual_position_0x6064=4305`
  - slave 2: `target_position_0x607A=0`, `actual_position_0x6064=-157798`
  - slave 3: `target_position_0x607A=0`, `actual_position_0x6064=-30776`
  - slave 4: `target_position_0x607A=0`, `actual_position_0x6064=-105759`
  - slave 5: `target_position_0x607A=0`, `actual_position_0x6064=-18350`
- Before the first motion, runtime still sees drive faults:
  - status state: `['ready_to_switch_on', 'ready_to_switch_on', 'fault', 'switch_on_disabled', 'switched_on', 'fault']`
  - error code: `['0x0000', '0x0000', '0x8400', '0x0000', '0x0000', '0x8400']`
- First real motion command:
  - `manipulator_controller`: `Received new action goal`
  - `manipulator_controller`: `Accepted new action goal`
  - later target/actual differences drop to small values, for example `target_minus_actual=2`.

**Interpretation**

The active trajectory controller is the likely source of the zero position targets before the first real command. The default MoveIt full-stack launch spawns and activates `manipulator_controller` immediately. That leaves a long boot window where an active position controller exists but has not received a valid trajectory yet.

The fix should control controller activation, not rewrite EtherCAT PDO position targets.

**Fix Applied**

- Replaced `zeroerr/launch/spawn_controllers.launch.py` default MoveIt controller spawner wrapper with explicit spawners:
  - `joint_state_broadcaster` starts active as before.
  - `manipulator_controller` is loaded/configured with `--inactive`.
- Updated runtime trajectory execution:
  - before sending a real trajectory, overwrite the first point with the live joint state as before,
  - require `/joint_states` to be available,
  - activate `manipulator_controller` through `/controller_manager/switch_controller`,
  - then wait for the FollowJointTrajectory action server and send the trajectory.

**Verification**

- Python syntax check passed:
  - `robot_controller.py`
  - `trajectory_executor.py`
  - `spawn_controllers.launch.py`
- Build passed:
  - `colcon build --packages-select erob_moveit_runtime zeroerr --cmake-args -DBUILD_TESTING=OFF`

**Next Hardware Test**

Relaunch without unplugging/replugging the EtherCAT cable and check:

- `spawner_manipulator_controller` should say it loaded/configured inactive, not activated.
- Before the first motion command, there should be no repeated `target_position_0x607A=0` with nonzero actual positions.
- Runtime should log `Activating manipulator_controller before trajectory` only when the first real command is sent.
- All 6 slaves should remain OP without needing a cable replug.

## 2026-06-26 - Safety Constraint: Keep Startup in Position Control

**Operator Safety Note**

- Keep the robot in position control during startup.
- Avoid activating alternate torque-control paths unintentionally because normal position holding is no longer active.

**Fix Applied**

- `full_stack.launch.py` and `ethercat_only.launch.py` keep the manipulator controller as the position-control path.
- The experimental torque-control path was archived and removed from active launch/runtime configuration.
  - normal trajectory activation refuses to proceed if any drag/torque controller is still active.

**Expected Startup State**

- `joint_state_broadcaster`: active
- `manipulator_controller`: inactive until the first real trajectory command
- alternate torque-control controllers: inactive

This preserves position-control startup safety and prevents accidental torque mode at launch.

## 2026-06-26 - Safety Revert: Do Not Leave Position Controller Inactive at Startup

**Observed Result**

- Leaving `manipulator_controller` inactive until the first runtime trajectory caused robot motion during initialization.
- This is unsafe on this hardware because the drives can enable before an active position controller is holding a valid command.

**Reverted**

- Restored `zeroerr/launch/spawn_controllers.launch.py` to the standard MoveIt controller spawner behavior.
- Restored normal `TrajectoryExecutor` behavior: it expects the FollowJointTrajectory action server to already be available.
- Removed the runtime helper that activated `manipulator_controller` only on first command.
- Restored `ethercat_only.launch.py` so `manipulator_controller` starts active.

**Kept**

- Alternate torque-control controllers remain out of the active startup path.
- Normal trajectory mode stays owned by `manipulator_controller`.

**Current Safety Rule**

The position trajectory controller must be active for startup holding. Do not leave the robot with drives enabled and no active position controller. Do not activate alternate torque-control paths at startup.

## 2026-06-26 - Fix Attempt 4: Initialize JTC Hold From State, Not Command

**Setup**

- Log file: `zeroerr/config/paint/temp_logs.txt`
- Operator observation: all slaves enter OP, but before the first command the robot makes a buzzing noise.

**Observed Symptoms**

- `WaitForSlavesOp.sh` reports all 6 slaves in OP.
- `manipulator_controller` is loaded and activated before the first user command.
- Before the first user command, the error monitor repeatedly reports `target_position_0x607A=0` while actual positions are nonzero.
- Drives report `0x8400 velocity error exceeds the limit value`.

**Relevant Log Evidence**

- `manipulator_controller`: `Configured and activated manipulator_controller`
- `WaitForSlavesOp.sh`: `All 6 EtherCAT slaves are in OP.`
- Pre-command target mismatch:
  - slave 0: `target_position_0x607A=0`, `actual_position_0x6064=130478`
  - slave 2: `target_position_0x607A=0`, `actual_position_0x6064=-181122`
  - slave 4: `target_position_0x607A=0`, `actual_position_0x6064=-130030`
  - slave 5: `target_position_0x607A=0`, `actual_position_0x6064=-17828`
- Runtime startup snapshot still shows mode display 8 for all joints, meaning the drives are in CSP while the target mismatch is present.

**Interpretation**

The likely cause is the JointTrajectoryController activation behavior. In ROS 2 Rolling, `set_last_command_interface_value_as_state_on_activation` defaults to `true`. If the hardware command interface starts at `0`, the controller can use that stale command value as its activation hold state instead of the measured state interface.

That explains the pre-command buzzing: the position controller is active, but its initial hold target is zero for one or more joints.

**Fix Applied**

Updated `zeroerr/config/ros2_controllers.yaml`:

```yaml
manipulator_controller:
  ros__parameters:
    set_last_command_interface_value_as_state_on_activation: false
```

This tells `manipulator_controller` to initialize from the measured state interfaces during activation, not from stale command interface values.

**Verification**

- YAML parse check confirmed the parameter is `False`.
- Build passed:
  - `colcon build --packages-select zeroerr --cmake-args -DBUILD_TESTING=OFF`

**Next Hardware Test**

Relaunch and check before sending any motion command:

- no buzzing during the startup hold window,
- no repeated `target_position_0x607A=0` while actual positions are nonzero,
- no startup `0x8400` velocity-limit errors,
- all slaves remain OP without cable replug.

## 2026-06-26 - Fix: Reject Motion Until Drive Enable Endpoint Is Used

**Issue**

After adding `/drive/enable` and `/drive/disable`, REST motion endpoints still accepted commands while the drives were intentionally left not enabled. The robot did not physically move, but the API behavior was misleading and allowed motion requests into the planning/submission path.

**Fix Applied**

- Added runtime motion error `-13`: drive operation is not enabled.
- `move/linera`, `move/ptp`, `execute/path`, `jog`, and explicit Joint 6 unwind now reject before planning/execution unless `/drive/enable` has succeeded.
- `RobotController.execute()` also rejects direct non-REST motion calls when drive operation has not been enabled.
- `/drive/disable` clears the runtime enable request state.
- Added `GET /drive/status` to report whether drive enable has been requested.

**Expected Result**

Before `/drive/enable`, motion endpoints return HTTP `409` with:

```json
{"result": -13, "success": false, "error": "Drive operation is not enabled; call POST /drive/enable before motion"}
```

After `/drive/enable`, motion requests may proceed to the existing EtherCAT OP and motion-stack checks. If slaves are not fully OP, the existing hardware interlock still rejects motion separately.

## 2026-06-26 - Reverted Unsafe Fix: Do Not Reassert Drive Enable Inside ROS2 Recovery

**Issue**

When the robot had already been enabled and EtherCAT slaves dropped out of OP, the runtime recovery brought slaves back to OP but the drives remained only `switched_on`. The platform did not resend `/drive/enable` because the REST connection was still alive and its cached enable state had not changed.

**Unsafe Result**

- Re-pulsing drive enable automatically inside ROS2 recovery caused the robot to float/drift.
- The log showed `target_position_0x607A=0` while actual position was nonzero on slave 1, followed by `0x8400` velocity-limit errors.
- This means enabling immediately after EtherCAT recovery can happen before the drive/controller target is in a safe hold state.

**Fix Applied**

- Removed automatic ROS2 drive-enable reassert after EtherCAT recovery.
- ROS2 now rejects `/drive/enable` while EtherCAT is not fully OP.
- ROS2 clears its `requested_enabled` flag when EtherCAT faults, so `/drive/status` reports disabled after a slave drop.
- The platform refreshes drive status before motion and calls `set_active_tool()` before linear/PTP/cartesian motion, which attempts `/drive/enable` only after the tool request succeeds.

**Expected Result**

- During EtherCAT fault: `/drive/enable` returns `HARDWARE_NOT_READY`; no drive enable pulse is sent.
- After all slaves are OP: the platform may re-run `set_active_tool()` and then `/drive/enable`.
- Motion is not posted unless the platform sees drive enable accepted.

**Safety Rule**

Do not automatically enable drives from EtherCAT recovery callbacks. Drive enable must be initiated from the platform flow after active tool sync and only while EtherCAT is fully OP.

## 2026-06-26 - Fix: Do Not Enable From Passive Active-Tool Sync

**Issue**

The platform state monitor re-syncs the active tool after startup/reconnect. The ROS2 client was changed to call `/drive/enable` after every successful `set_active_tool()`. That meant passive tool sync enabled the drives before any real motion command was ready.

**Log Evidence**

- `Switched active tool to TOOL_1`
- `[DriveEnable] Enable operation requested via enable_set/disable_set`
- First `[MOVE] ...` log occurred much later.
- Operator reported buzzing until the first move command.

**Fix Applied**

- `set_active_tool()` now only sets the tool; it does not enable drives.
- Motion methods set the active tool, then call `/drive/enable` immediately before checking drive status and posting the motion request.

**Expected Result**

Platform reconnect/state-sync can update the active tool without enabling the robot. Drive enable should happen only as part of an actual motion/jog/unwind command path.
