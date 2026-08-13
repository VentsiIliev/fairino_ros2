# Servo Pickup / Paint Contact Context - 2026-08-13

## Scope

Work covered MoveIt Servo integration, ServoJog support, generic servo-until-condition pickup, paint pickup strategy wiring, and the initial scaffold for height-measured pickup Z.

Main repositories:

- ROS2 backend: `/home/ilv/ros2_ws/eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime`
- Platform: `/home/ilv/Desktop/robot_app_platform`

## MoveIt Servo / ROS2 Runtime

MoveIt Servo is used through REST endpoints and ServoJog support.

Key behavior:

- Servo node should stay alive and be paused/resumed rather than repeatedly relaunched.
- Stop must publish zero velocity and pause Servo.
- Servo node cleanup must happen with the rest of runtime services/nodes when stopping.
- No Panda data/config should be used.
- Collision checking should remain enabled.
- Collision escape behavior was explored because Servo can stop at collision and then block recovery unless the command moves away from collision.

Important runtime symptoms already seen:

- `Waiting to receive robot state update` meant Servo was not receiving robot state.
- `Requested pause state is already active` was not necessarily fatal.
- Singularity warnings can decelerate commands:
  - `Moving closer to a singularity, decelerating`
  - `Moving away from a singularity, decelerating`
- `Collision monitor could not be started` can happen if collision monitor state is stale/duplicated.
- `failed to select TWIST command type` / `failed to resume MoveIt Servo` were seen during repeated starts and needed robust pause/resume handling.

REST endpoints involved:

- `/servo/cartesian/start`
- `/servo/cartesian/update`
- `/servo/cartesian/stop`
- `/servojog/start`
- `/servojog/stop`

## Platform ServoJog

ServoJog was added as a separate capability rather than replacing planned jog.

Design decisions:

- Keep planned step jog behavior.
- Add ServoJog explicitly for continuous press/release UI behavior.
- Use `/servojog/start` and `/servojog/stop`, not overload `/jog`.
- UI needs separate modes:
  - Step mode: fixed distance planned jog.
  - Servo mode: continuous jog while button is held.
- Platform sends actual jog speed directly as `linear_mm_s` / `angular_deg_s`, not velocity percentage.
- GUI speed controls drive ServoJog speed.
- Direction discrepancies were fixed so ServoJog direction matches Step mode, especially Y and Z behavior.

Important files touched:

- `src/engine/robot/interfaces/i_robot.py`
- `src/engine/robot/interfaces/i_motion_service.py`
- `src/engine/robot/services/robot_service.py`
- `src/engine/robot/services/motion_service.py`
- `src/engine/robot/drivers/ros2_robot.py`
- `src/engine/robot/drivers/client_adapters/http_websocket.py`
- `src/applications/base/robot_jog_widget.py`
- `src/applications/base/robot_jog_service.py`
- `src/applications/base/robot_jog_service_builder.py`

## Generic Servo-Until-Condition Pickup

Added generic procedure for sensor-driven pickup.

Purpose:

- Move to approach pose.
- Start Servo descent.
- Poll any condition/sensor.
- Stop Servo immediately when condition is active.
- Fail safely if condition cannot be read.

Key files:

- `src/engine/robot/procedures/servo_until_condition.py`
- `src/engine/robot/procedures/vacuum_pickup_condition.py`
- `src/engine/robot/procedures/dummy_pickup_condition.py`
- `src/engine/robot/procedures/__init__.py`
- `scripts/test_servo_pickup_procedure.py`
- `scripts/test_servo_pickup_procedure.md`

Safety behavior:

- Preflight condition readability is checked before motion.
- Condition is checked again after approach and before Servo start.
- If condition read fails repeatedly while Servo is active, Servo is stopped.
- Servo is stopped in `finally` whenever it was started.
- Invalid speed/axis config blocks Servo start.

Important result messages:

- `condition_detected`
- `condition_already_active`
- `condition_unreadable_before_motion`
- `condition_unreadable_after_approach`
- `condition_unreadable_during_servo`
- `servo_start_failed:<ret>`
- `invalid_linear_speed`
- `invalid_angular_speed`
- `timeout`
- `cancelled`

Dummy sensor:

- `TimedDummyPickupCondition`
- Test-only.
- Now arms only after Servo actually starts via optional `on_servo_start()`.
- Preflight reads return readable/inactive and do not start the dummy timer.
- If `Dummy Detect After` is greater than `Detection Timeout`, expected result is `timeout`.

## Paint Pickup Integration

Paint pickup can now use planned pickup or Servo contact pickup.

Key files:

- `src/robot_systems/paint/processes/paint/config.py`
- `src/robot_systems/paint/processes/paint/execute/pickup_executor.py`
- `src/robot_systems/paint/processes/paint/execute/workpiece_path_executor.py`
- `src/robot_systems/paint/processes/paint/execution_machine/handlers/workflow/pickup_handler.py`
- `src/robot_systems/paint/processes/paint/execution_machine/handlers/magazine_load/magazine_execute_pickup_release_handler.py`
- `src/robot_systems/paint/paint_robot_system.py`
- `src/robot_systems/paint/application_wiring.py`

Paint pickup sequence:

- Build pickup/staging plan.
- If planned mode: execute ordered pickup segments.
- If Servo contact mode:
  - Execute approach.
  - Run Servo until condition.
  - Continue from lift/staging after the planned descend segment.

Magazine pickup sequence:

- Build magazine approach, descend, lift, release waypoints.
- If Servo contact mode:
  - Execute approach segment.
  - Run Servo until condition.
  - Continue with lift/release segments.

Terminal blend handling:

- Single-segment planned chains must force terminal `blendR=0.0`.
- This avoids backend error:
  - `Segment 1 requests blendR=20.000 but there is no next segment`

Failure message handling:

- Magazine Servo contact path now returns specific messages instead of collapsing everything to `Move to ... release pose failed`.
- Expected failures include:
  - `Servo contact pickup condition is not configured`
  - `Magazine servo contact pickup failed: timeout`
  - `Magazine servo contact pickup failed: condition_unreadable...`
  - `Magazine lift/release after servo contact failed for ...`

## Paint Pickup Contact Mode Scaffold

Latest change replaced pickup Servo booleans with explicit modes.

Config constants:

- `PICKUP_CONTACT_MODE_PLANNED = "planned"`
- `PICKUP_CONTACT_MODE_SERVO_CONTACT = "servo_contact"`
- `PICKUP_CONTACT_MODE_HEIGHT_MEASURE = "height_measure"`
- `PICKUP_CONTACT_MODES = ("planned", "servo_contact", "height_measure")`

`PickupMotionConfig` fields:

- `pickup_contact_mode: str = "planned"`
- `magazine_pickup_contact_mode: str = "planned"`

Removed source references to:

- `servo_contact_enabled`
- `servo_contact_magazine_enabled`
- `pickup_servo_contact_enabled`
- `pickup_servo_contact_magazine_enabled`

Current behavior:

- `planned`: current planned pickup behavior.
- `servo_contact`: existing Servo contact behavior.
- `height_measure`: scaffold only; fails fast and loudly.
- Invalid mode string also fails fast.

Fail-fast height scaffold messages:

- Calibration pickup:
  - `Height-measured pickup Z mode is not wired yet`
- Magazine pickup:
  - `Magazine height-measured pickup Z mode is not wired yet`

UI:

- Settings section renamed from `Servo Contact Pickup` to `Pickup Contact Strategy`.
- UI now exposes combo fields:
  - `pickup_contact_mode`
  - `magazine_pickup_contact_mode`
- Servo-specific tuning fields remain because they are used when mode is `servo_contact`.

Default settings updated:

- `src/robot_systems/paint/storage/settings/paint/process.json`
  - `pickup_contact_mode: "planned"`
  - `magazine_pickup_contact_mode: "planned"`

## Height Measuring Direction

Height measuring service exists in paint system startup:

- `PaintRobotSystem.on_start()` builds:
  - `_height_measuring_service`
  - `_height_measuring_calibration_service`
  - `_laser_detection_service`

But height measuring is not wired into paint pickup execution yet.

Clean intended next design:

- Keep height measuring optional.
- Do not silently fallback.
- If mode is `height_measure` and service is missing/unavailable/not calibrated, fail fast.
- Height mode and Servo contact mode are mutually exclusive by construction because there is one mode field.
- Resolve pickup Z centrally before pickup planning, probably through existing `_pickup_z_mm`.

Existing central pickup-Z integration point:

- Normal paint pickup uses `_pickup_z_mm` in:
  - `src/robot_systems/paint/processes/paint/plan/pickup_transfer_planner.py`
- Magazine pickup uses `_pickup_z_mm` in:
  - `src/robot_systems/paint/processes/paint/execution_machine/handlers/magazine_load/magazine_execute_pickup_release_handler.py`

Important semantic question for future wiring:

- Confirm whether `height_service.measure_at(x, y)` returns:
  - absolute robot Z, or
  - measured height above calibrated zero plane.

This determines the formula:

- If absolute robot Z:
  - `pickup_z = measured_z + contact_offset_mm + height_measure_z_adjustment_mm`
- If height above zero plane:
  - `pickup_z = pickup_safety_z_min_mm + measured_height + contact_offset_mm + height_measure_z_adjustment_mm`

## Tests / Verification

Focused tests run and passed:

```bash
python3 -m unittest \
  tests/robot_systems/paint/processes/paint/execute/test_servo_contact_pickup_executor.py \
  tests.engine.robot.procedures.test_servo_until_condition
```

Latest result:

```text
Ran 11 tests in 0.065s
OK
```

The test output includes intentional exception traces from simulated disconnected sensor tests.

Compile checks were run for touched paint/procedure modules using:

```bash
python3 -m py_compile ...
```

## Operational Notes

After platform code changes:

- Restart the platform process so settings schema and runtime classes reload.
- Old `servo_contact_enabled` keys in existing JSON are ignored by the dataclass serializer once the fields are removed.
- After opening/saving Paint Process Settings, new mode fields become the persisted source of truth.

For dummy Servo contact testing:

- `pickup_contact_mode = "servo_contact"` or `magazine_pickup_contact_mode = "servo_contact"`
- `servo_contact_dummy_sensor_enabled = true`
- `servo_contact_dummy_detect_after_s < servo_contact_timeout_s`
- Example:
  - detect after: `1.0s`
  - timeout: `5.0s`

Expected success log:

```text
Servo contact descent result success=True detected=True timeout=False message=condition_detected
```

Expected timeout if detect-after is longer than timeout:

```text
success=False detected=False timeout=True message=timeout
```

## Key Current Uncommitted Files

Likely modified in platform:

- `src/engine/robot/procedures/dummy_pickup_condition.py`
- `src/engine/robot/procedures/servo_until_condition.py`
- `src/robot_systems/paint/processes/paint/config.py`
- `src/robot_systems/paint/processes/paint/execute/pickup_executor.py`
- `src/robot_systems/paint/processes/paint/execution_machine/handlers/workflow/pickup_handler.py`
- `src/robot_systems/paint/processes/paint/execution_machine/handlers/magazine_load/magazine_execute_pickup_release_handler.py`
- `src/robot_systems/paint/applications/paint_process_settings/mapper.py`
- `src/robot_systems/paint/applications/paint_process_settings/view/paint_process_settings_schema.py`
- `src/robot_systems/paint/storage/settings/paint/process.json`
- `tests/engine/robot/procedures/test_servo_until_condition.py`
- `tests/robot_systems/paint/processes/paint/execute/test_servo_contact_pickup_executor.py`

