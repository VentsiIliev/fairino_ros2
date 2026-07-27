# ZeroErr Collision Monitor

## Purpose

The ZeroErr collision monitor estimates unexpected external joint torque and exposes it in a GUI for tuning. It is a diagnostic and tuning tool first. It does not stop the robot by itself.

The current implementation is a practical, paper-inspired observer:

- measured joint torque is reconstructed from actuator current
- a simple friction model is subtracted
- expected joint torque comes from the URDF/KDL dynamics model
- a momentum-observer style estimator filters the residual
- thresholds are applied per joint

This is not yet a full 1:1 implementation of the paper model because friction compensation is still simple and the observer is not yet using a full identified friction/gravity model.

## Data Flow

The monitor runs in [zeroerr_collision_monitor.py](/home/ilv/ros2_ws/eRob_moveit/src/eRob_ROS2_MoveIt/zeroerr/scripts/zeroerr_collision_monitor.py).

Signal path:

1. EtherCAT state interfaces provide:
   - joint `position`
   - joint `velocity`
   - drive `effort`
   - `motor_actual_current`
   - `following_error_actual`
   - status/error interfaces
2. `motor_actual_current` is converted to `Cur A`
3. `Cur A` is converted to `CurTau Nm`
4. friction compensation is subtracted:
   - `MeasTau Nm = CurTau Nm - FricTau Nm`
5. KDL inverse dynamics computes expected torque:
   - `ExpTau Nm`
6. the estimator produces:
   - `DiffTau Nm = MeasTau Nm - ExpTau Nm`
   - `ExtTau Nm` = filtered residual used for dynamics detection
7. thresholds and confirmation cycles decide:
   - `ACTIVE`
   - `LATCHED`
   - `CLEAR`

## What The GUI Shows

The GUI runs in [zeroerr_collision_monitor_gui.py](/home/ilv/ros2_ws/eRob_moveit/src/eRob_ROS2_MoveIt/zeroerr/scripts/zeroerr_collision_monitor_gui.py).

Main table columns:

- `DrvTau Nm`
  - torque reconstructed from the drive torque feedback channel
  - useful for comparing drive torque reporting against current-based torque
- `Cur A`
  - motor current in amps
- `CurTau Nm`
  - output torque reconstructed from current and actuator constants
- `FricTau Nm`
  - friction torque currently subtracted by the monitor
- `MeasTau Nm`
  - torque actually fed into the estimator
- `ExpTau Nm`
  - expected torque from the URDF/KDL model
- `DiffTau Nm`
  - raw residual before observer filtering
- `ExtTau Nm`
  - filtered external torque estimate used for dynamics triggering
- `FollowErr`
  - drive following error
- `Contact`
  - simple contact heuristic state
- `Dyn`
  - dynamics detector state
- `Reason`
  - short trigger explanation for the joint

## Detector Modes

There are two detector families:

- `contact`
  - simple heuristic
  - checks measured torque and following error together
- `dynamics`
  - observer-based
  - uses `ExtTau Nm`

State meaning:

- `ACTIVE`
  - currently over threshold
- `LATCHED`
  - stayed over threshold long enough to confirm
- `CLEAR`
  - below threshold again

## Runtime Parameters

These are the main tuning parameters exposed in launch and now editable live in the GUI.

### General

- `poll_period_sec`
  - publish/update period for monitor snapshots
  - lower value = faster updates, more CPU load
- `confirm_cycles`
  - number of consecutive cycles required before latching
  - higher value = less chatter, slower reaction
- `print_table`
  - prints the text table to terminal
  - useful for debugging, adds console noise

### Estimator Selection

- `use_inverse_dynamics`
  - enables the model-based estimator path
  - if false, only the simple contact heuristic is meaningful
- `dynamics_estimator_mode`
  - current values:
    - `momentum_observer`
    - `inverse_dynamics`
  - `momentum_observer` is the preferred mode currently
  - changing it affects how `ExtTau Nm` is computed
- `measured_torque_source`
  - current values:
    - `current_based_torque`
    - `drive_torque`
  - decides whether the measured side of the estimator uses current-reconstructed torque or drive torque feedback

### Contact Heuristic

- `effort_thresholds`
  - shown in GUI as `ContactTau`
  - per-joint threshold for contact detection
  - higher value = less sensitive contact detection
- `following_error_thresholds`
  - per-joint following error threshold
  - higher value = less sensitive contact detection

Contact detection requires both thresholds to be exceeded.

### Dynamics Thresholds

- `external_torque_thresholds`
  - per-joint threshold on `ExtTau Nm`
  - this is the main threshold for model-based collision detection
  - higher value = fewer dynamics triggers

Important:

- do not set this equal to idle baseline
- it should be above both:
  - rest residual
  - normal no-contact motion residual

### Friction Model

The current friction model is:

`tau_friction = kc * sign(v) + kv * v`

with a deadband around zero speed.

- `friction_coulomb_nm`
  - per-joint Coulomb friction term
  - affects constant offset during motion
- `friction_viscous_nm_per_rad_s`
  - per-joint viscous friction term
  - affects residual growing with speed
- `friction_velocity_deadband_rad_s`
  - small velocity zone where sign friction is suppressed
  - helps reduce chatter around zero velocity

Effect on the signal chain:

- higher friction compensation reduces `MeasTau Nm`
- if tuned correctly, `DiffTau Nm` and `ExtTau Nm` drop during normal motion
- if overtuned, real disturbances may be hidden

### Actuator Model

These parameters define the current-to-torque conversion.

- `joint_models`
  - per-joint actuator type
  - currently used to map joint to rated current and torque constant
- `model_names`
  - list of known actuator model labels
- `model_rated_current_ma`
  - rated current for each model
- `model_output_torque_constant_nm_per_a`
  - output torque constant for each model

Current ZeroErr mapping:

- `Joint_1`, `Joint_2`, `Joint_3` -> `eRob80H100T`
- `Joint_4`, `Joint_5`, `Joint_6` -> `eRob70H100T`

These values directly affect:

- `Cur A`
- `CurTau Nm`
- therefore the measured torque used by the estimator when `measured_torque_source=current_based_torque`

### Model Geometry

The expected torque depends on the URDF:

- link geometry/joint origins affect kinematics and Jacobians
- link mass, center of mass, and inertia affect expected torque

Relevant files:

- [erob_arm.urdf](/home/ilv/ros2_ws/eRob_moveit/src/eRob_ROS2_MoveIt/zeroerr/config/urdfs/erob_arm.urdf)
- [eRobo3.urdf.xacro](/home/ilv/ros2_ws/eRob_moveit/src/eRob_ROS2_MoveIt/zeroerr/config/urdfs/eRobo3.urdf.xacro)
- [runtime.yaml](/home/ilv/ros2_ws/eRob_moveit/src/eRob_ROS2_MoveIt/zeroerr/config/runtime.yaml)

If the URDF inertials are wrong, `ExpTau Nm` will be wrong and the observer residual will carry that model error.

## Tuning Procedure

Recommended sequence:

1. Start with the correct URDF for the real robot.
2. Verify `CurTau Nm` is in the right range and tracks `DrvTau Nm` reasonably.
3. Capture baseline at rest.
4. Run known-safe motion with no contact.
5. Tune friction:
   - first `friction_viscous_nm_per_rad_s`
   - then `friction_coulomb_nm`
   - then `friction_velocity_deadband_rad_s`
6. Watch:
   - `MeasTau Nm`
   - `ExpTau Nm`
   - `DiffTau Nm`
   - `ExtTau Nm`
7. Set `external_torque_thresholds` above normal no-contact peaks.
8. Test with controlled manual disturbance.

What good tuning looks like:

- `DiffTau Nm` and `ExtTau Nm` stay modest during normal motion
- idle residual does not chatter at threshold
- manual push creates a clear separation from nominal motion

## Interpreting Common Problems

### `ACTIVE` or `LATCHED` while robot is still

Likely causes:

- thresholds too close to idle baseline
- current scaling wrong
- URDF inertials wrong
- friction compensation missing or too small

### Large `DiffTau Nm` on small Cartesian moves

Possible causes:

- model mismatch in URDF
- link extension not reflected in inertials or joint origins
- friction/transmission losses not modeled
- current-to-torque constants inaccurate

### `CurTau Nm` and `DrvTau Nm` disagree strongly

Likely causes:

- wrong actuator model per joint
- wrong rated current
- wrong torque constant

### `ExtTau Nm` still noisy after baseline capture

Baseline capture is only a rest measurement. It is not enough by itself. Thresholds must be set above normal motion residual, not just idle residual.

## Current Limitations

- friction model is still simple
- no identified gravity/friction compensation table yet
- detector does not stop motion by itself
- full paper-equivalent implementation still needs better friction identification

## Main Files

- monitor:
  - [zeroerr_collision_monitor.py](/home/ilv/ros2_ws/eRob_moveit/src/eRob_ROS2_MoveIt/zeroerr/scripts/zeroerr_collision_monitor.py)
- GUI:
  - [zeroerr_collision_monitor_gui.py](/home/ilv/ros2_ws/eRob_moveit/src/eRob_ROS2_MoveIt/zeroerr/scripts/zeroerr_collision_monitor_gui.py)
- launch defaults:
  - [demo.launch.py](/home/ilv/ros2_ws/eRob_moveit/src/eRob_ROS2_MoveIt/zeroerr/launch/demo.launch.py)
  - [ethercat_only.launch.py](/home/ilv/ros2_ws/eRob_moveit/src/eRob_ROS2_MoveIt/zeroerr/launch/ethercat_only.launch.py)
- shared dynamics code:
  - [external_torque_estimator.py](/home/ilv/ros2_ws/eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/safety/collision_detection/external_torque_estimator.py)
  - [inverse_dynamics_model.py](/home/ilv/ros2_ws/eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/safety/collision_detection/inverse_dynamics_model.py)
