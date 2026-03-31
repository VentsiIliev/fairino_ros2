# ZeroErr Drag Mode

## Purpose

This document explains:

- how ZeroErr drag mode is implemented
- what failed during the CST transition work
- what was tested
- what finally worked
- why the robot can still sag or free-fall even after CST is working
- which parameters affect compensation and feel

This is the current source-of-truth for the ZeroErr hand-guiding path.

## Current Architecture

Drag mode is implemented in:

- [robot_controller.py](/home/ilv/ros2_ws/eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/robot_controller.py)

The runtime exposes:

- `POST /drag/enable`
- `POST /drag/disable`
- `GET /drag/status`

The collision monitor GUI calls those endpoints directly from:

- [zeroerr_collision_monitor_gui.py](/home/ilv/ros2_ws/eRob_moveit/src/eRob_ROS2_MoveIt/zeroerr/scripts/zeroerr_collision_monitor_gui.py)

The actual hardware/control interfaces used for drag mode are defined in:

- [eRobo3.ros2_control.xacro](/home/ilv/ros2_ws/eRob_moveit/src/eRob_ROS2_MoveIt/zeroerr/config/eRobo3.ros2_control.xacro)
- [zeroErr.yaml](/home/ilv/ros2_ws/eRob_moveit/src/eRob_ROS2_MoveIt/zeroerr/config/zeroErr.yaml)
- [zeroErr1.yaml](/home/ilv/ros2_ws/eRob_moveit/src/eRob_ROS2_MoveIt/zeroerr/config/zeroErr1.yaml)
- [ros2_controllers.yaml](/home/ilv/ros2_ws/eRob_moveit/src/eRob_ROS2_MoveIt/zeroerr/config/ros2_controllers.yaml)
- [runtime.yaml](/home/ilv/ros2_ws/eRob_moveit/src/eRob_ROS2_MoveIt/zeroerr/config/runtime.yaml)

## Drag Mode Signal Path

When drag mode is enabled:

1. `manipulator_controller` is deactivated.
2. Forward controllers remain active for:
   - `mode_of_operation`
   - `effort`
   - `torque_offset`
   - `enable_set`
   - `disable_set`
3. The runtime requests:
   - `mode_of_operation = 10` (`CST`)
4. During transition it pulses:
   - `disable_set`
   - then `enable_set`
5. Once the drives report `mode_display = 10`, the runtime sends:
   - `torque_offset` for gravity/friction compensation
   - `effort` for damping

While drag mode is active:

- `torque_offset` is the main compensation term
- `effort` is a damping term
- `position` control is not the active hand-guiding path
- `0x6071` and `0x60B2` are commanded in per-thousand of rated current, not Nm
- the runtime converts Nm to drive units before publishing

The command scaling is:

- `torque_offset = compensation_scale * joint_compensation_scale[i] * (expected_tau + friction_tau)`
- `raw_command = torque_nm * 1000000 / (rated_current_mA * output_torque_constant_Nm_per_A)`

When drag mode is disabled:

1. drag commands are zeroed
2. mode is returned to `8` (`CSP`)
3. `manipulator_controller` is reactivated
4. the runtime sends a hold-current-position trajectory so the robot does not snap back to an older target

## What Was Wrong Initially

At first, the software reported drag mode as enabled, but the drives stayed in:

- `mode_display = 8`

That meant:

- the runtime thought it was in `CST`
- the actual drives were still in `CSP`
- real torque-mode drag was not active yet

### Problems Found During Debugging

#### 1. Controller ownership

Originally, drag mode tried to coexist with:

- `manipulator_controller`

That was wrong because the position trajectory controller must not remain active while trying to take over the robot in torque mode.

Fix:

- drag enable now deactivates `manipulator_controller`
- drag disable reactivates it

#### 2. GUI/backend issues

There were two unrelated runtime issues:

- the GUI drag poll path briefly crashed because `_poll_drag_status()` treated a `(payload, error)` tuple as a dict
- `/drag/enable` initially returned HTTP 500 because the Flask thread tried to call `rclpy.spin_once()` on a node already spinning in the main executor

Fixes:

- GUI now unpacks `(payload, error)` correctly
- service waits in the REST path now use a thread-safe `threading.Event` callback wait

These were not the real CST problem, but they had to be fixed first.

#### 3. Hardcoded startup mode suspicion

The hardware xacro originally had a plugin param that could force startup mode:

- `<param name="mode_of_operation">8</param>`

That was removed from:

- [eRobo3.ros2_control.xacro](/home/ilv/ros2_ws/eRob_moveit/src/eRob_ROS2_MoveIt/zeroerr/config/eRobo3.ros2_control.xacro)

The startup default was left only in the PDO YAMLs:

- `default: 8` on `0x6060`

This was the correct cleanup, but it was not the final blocker.

#### 4. The real blocker: EtherCAT CiA402 plugin behavior

The actual root cause was in:

- [generic_ec_cia402_drive.cpp](/home/ilv/ros2_ws/src/ethercat_driver_ros2/ethercat_driver_ros2/ethercat_generic_plugins/ethercat_generic_cia402_drive/src/generic_ec_cia402_drive.cpp)

The plugin was doing this after initialization:

- forcing `override_command = true` for `mode_of_operation`

That meant:

- startup default mode was written successfully
- but all later runtime writes to `mode_of_operation` were ignored

So every request to switch:

- `8 -> 10`

was being blocked inside the plugin before reaching the drive.

Fix:

- keep startup mode as default
- but allow runtime `mode_of_operation` commands after initialization
- if no command is written, hold the current displayed mode rather than re-forcing the startup value

This was the change that finally allowed `CST` to engage.

## What Was Tested

### Test 1: Runtime drag request only

Observed:

- `requested_mode_value = 10`
- `mode_display = 8`

Conclusion:

- runtime requested `CST`
- drives stayed in `CSP`

### Test 2: Deactivate trajectory controller

Observed:

- `manipulator_controller` successfully became `inactive`
- `mode_display` still stayed `8`

Conclusion:

- controller ownership was necessary to fix
- but not sufficient

### Test 3: `disable_set` / `enable_set` transition pulses

Observed:

- drag transition path clearly published `disable_set`
- drives still stayed in `8`

Conclusion:

- transition pulsing alone did not solve the blocked mode switch

### Test 4: Patch the EtherCAT CiA402 plugin

Observed on the next enable:

- first mismatch frame:
  - `mode_display=[8, 8, 8, 8, 8, 8]`
  - one joint briefly reported a nonzero `error_code`
- one second later:
  - `mode_display=[10, 10, 10, 10, 10, 10]`
  - `statusword=[5815, ...]`
  - `error_code=[0, 0, 0, 0, 0, 0]`

Conclusion:

- the robot successfully entered `CST`
- the runtime is now controlling the drives in torque mode during drag enable

## What Worked

The working path is:

1. deactivate `manipulator_controller`
2. request `mode_of_operation = 10`
3. pulse `disable_set`
4. complete re-enable transition
5. wait for `mode_display = 10`
6. apply:
   - `torque_offset`
   - `effort`

Successful evidence:

- `mode_display=[10,10,10,10,10,10]`
- `error_code=[0,0,0,0,0,0]`

That means the system is now in real `CST` drag mode, not the earlier fake position-jog drag mode.

## Why The Robot Went Into Free Fall

A successful transition to `CST` does not mean the compensation is correct.

If the robot goes limp or drops, the usual reason is:

- `torque_offset` compensation is too low for gravity

In the current implementation:

- `torque_offset = compensation_scale * (expected_tau + friction_tau)`

with:

- `expected_tau` from the KDL/URDF model including gravity
- `friction_tau` from the simple friction model

If that model underestimates gravity torque, the arm will sag or free-fall.

### Practical meaning

- `compensation too low`
  - arm drops
- `compensation too high`
  - arm rises or feels pushy
- `damping too low`
  - arm feels loose and keeps moving
- `damping too high`
  - arm feels sticky and hard to guide

So the free-fall test means:

- `CST` is working
- compensation tuning is not finished

## Parameters That Affect Drag Feel

All of these are currently defined in:

- [runtime.yaml](/home/ilv/ros2_ws/eRob_moveit/src/eRob_ROS2_MoveIt/zeroerr/config/runtime.yaml)

### `DRAG_MODE_COMPENSATION_SCALE`

Global multiplier on gravity/friction compensation.

Effect:

- higher value = more support against gravity
- lower value = more sag

This is the first parameter to change after a free-fall test.

### `DRAG_MODE_DAMPING_NM_PER_RAD_S`

Per-joint damping term used for:

- `effort_cmd = -damping * velocity`

Effect:

- higher value = more resistance to motion
- lower value = more loose, less stable feel

### `DRAG_MODE_MAX_TORQUE_OFFSET_NM`

Per-joint clamp on compensation torque.

Effect:

- if too low, compensation saturates and the arm still drops
- if high enough, the model can actually support the arm

This must be large enough for joints carrying most of the gravity load.

### `DRAG_MODE_MAX_EFFORT_NM`

Per-joint clamp on damping torque.

Effect:

- too low = damping has little effect
- too high = can feel resistive or unsafe if tuning is wrong

### Friction parameters

These come from the collision/monitor config path:

- `friction_coulomb_nm`
- `friction_viscous_nm_per_rad_s`

Effect:

- better friction modeling improves compensation quality
- but free-fall is usually dominated by gravity compensation first, not friction

## Recommended Compensation Tuning Procedure

Do this carefully in a safe posture.

1. Start in a mechanically favorable pose.
   - keep the arm low
   - avoid a fully extended horizontal pose at first

2. Increase `DRAG_MODE_COMPENSATION_SCALE` gradually.
   - the current value may simply be too low
   - test whether the arm still drops

3. Check for saturation.
   - if the arm still drops even with higher scale, inspect whether `torque_offset` is hitting `DRAG_MODE_MAX_TORQUE_OFFSET_NM`
   - if yes, raise the limit carefully

4. Add damping only after gravity support is close.
   - tune `DRAG_MODE_DAMPING_NM_PER_RAD_S` once the arm no longer free-falls

5. Improve friction later.
   - friction matters for feel
   - gravity support matters first for safety

## Immediate Interpretation Of The Free-Fall Test

The free-fall result does **not** mean CST drag mode failed.

It means:

- mode transition succeeded
- the compensation model is still too weak for the real robot/load

So the next engineering task is:

- increase and validate gravity compensation
- then tune damping
- then refine friction compensation

## Useful Runtime Checks

Current status endpoint:

```bash
curl http://localhost:5000/drag/status | python3 -m json.tool
```

Important fields:

- `enabled`
- `requested_mode_value`
- `mode_display`
- `statusword`
- `error_code`
- `last_effort_command_nm`
- `last_torque_offset_command_nm`

Successful CST drag mode should show:

- `enabled: true`
- `mode_display: [10, 10, 10, 10, 10, 10]`
- `error_code: [0, 0, 0, 0, 0, 0]`

## Relevant Files

- [robot_controller.py](/home/ilv/ros2_ws/eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/robot_controller.py)
- [config.py](/home/ilv/ros2_ws/eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/config.py)
- [runtime.yaml](/home/ilv/ros2_ws/eRob_moveit/src/eRob_ROS2_MoveIt/zeroerr/config/runtime.yaml)
- [ros2_controllers.yaml](/home/ilv/ros2_ws/eRob_moveit/src/eRob_ROS2_MoveIt/zeroerr/config/ros2_controllers.yaml)
- [eRobo3.ros2_control.xacro](/home/ilv/ros2_ws/eRob_moveit/src/eRob_ROS2_MoveIt/zeroerr/config/eRobo3.ros2_control.xacro)
- [zeroErr.yaml](/home/ilv/ros2_ws/eRob_moveit/src/eRob_ROS2_MoveIt/zeroerr/config/zeroErr.yaml)
- [zeroErr1.yaml](/home/ilv/ros2_ws/eRob_moveit/src/eRob_ROS2_MoveIt/zeroerr/config/zeroErr1.yaml)
- [generic_ec_cia402_drive.cpp](/home/ilv/ros2_ws/src/ethercat_driver_ros2/ethercat_driver_ros2/ethercat_generic_plugins/ethercat_generic_cia402_drive/src/generic_ec_cia402_drive.cpp)
