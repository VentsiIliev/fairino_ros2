# ROS2 MoveIt Backend Refactoring Plan

Canonical plan: use this document as the working refactoring plan. The older
`refactoring_plan.md` and `refactoring_plan_updated.md` are retained only as
historical drafts.

## Implementation Handoff Log

### 2026-08-10 - Slice 68: restore proven LIN blend routing

Status: completed and build checked. Ordered LIN blend groups now use the same
individually planned trajectory -> `BlendBuilder` -> optimizer path as PTP.

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/ordered_planning_worker.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/planning/linked_lin_client.py
eRob_moveit/src/eRob_ROS2_MoveIt/zeroerr/config/runtime.yaml
```

What was done:

- Removed the ordered-planning worker fork that sent all-LIN blend groups
  through Python Cartesian blend-pose generation and `/compute_linked_lin`.
- Removed the Python post-helper closed-loop cleanup pass that was compensating
  for CartesianInterpolator joint loops.
- Disabled `LINKED_LIN_HELPER_ENABLED` in the ZeroErr runtime config.

Reason:

- Successful PTP blending and the previously working LIN path both rely on
  already-valid per-segment trajectories plus the existing `BlendBuilder`.
- The experimental linked-LIN path changed behavior by creating a single
  anonymous Cartesian pose stream before IK, which exposed IK branch reversals
  that the optimizer rejects.

Next pickup point:

1. Redesign `ComputeLinkedLin.srv` around per-segment LIN targets, labels,
   velocities, accelerations and blend radii.
2. In C++, plan each LIN segment sequentially with the previous segment's final
   joint state as the next seed.
3. Return validated per-segment trajectories or a concatenated trajectory with
   exact segment boundary indices.
4. Feed the returned segment trajectories/boundaries into the existing
   `BlendBuilder`; do not generate Cartesian blend geometry before IK.

Verification:

```text
python3 -m py_compile \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/ordered_planning_worker.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/planning/linked_lin_client.py

/bin/bash -lc "env -i HOME=\"$HOME\" TERM=\"${TERM:-xterm}\" PATH=\"/usr/bin:/bin:/usr/sbin:/sbin\" bash --noprofile --norc -lc 'source /opt/ros/rolling/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws/eRob_moveit; colcon build --packages-select erob_moveit_runtime zeroerr'"
```

Verification notes:

- Python compile passed.
- `colcon build --packages-select erob_moveit_runtime zeroerr` passed.
- The build still prints the existing missing `/opt/ros/rolling/setup.bash`
  warning before using the Jazzy underlay, and the existing Jazzy
  `tl_expected` deprecation warning from MoveIt dependencies.

### 2026-08-10 - Slice 1: typed models and ordered-chain adapter

Status: completed and compile-checked. No runtime behavior is intentionally
changed yet.

Files added:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/planning/planning_types.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/__init__.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/scheduling_types.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/motion_batch.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/motion_group.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/motion_adapters.py
```

What was done:

- Added `MotionSegment`, `LinearSegment`, `PtpSegment`, `PathSegment`,
  `UnwindSegment` and `PlannedTrajectory`.
- Added `MotionBatch`, `MotionGroup`, `PlannedMotionGroup` and
  `MotionGroupState`.
- Added ordered-chain dictionary adapters that convert the current normalized
  request shape into typed motion models.
- Kept the adapter unwired from execution so the first slice is structural only.

Issues / repo facts found:

- The public linear method is `move_liner(...)`, not `move_lin(...)`.
- The public path method is `execute_path(...)`, not `move_path(...)`.
- Ordered-chain request parsing already normalizes segment dictionaries before
  calling `MoveItRobotBackend.execute_ordered_motion_chain(...)`.
- `blendR` currently belongs to the segment and describes the transition after
  that segment.
- The existing `motion/execution/motion_queue.py` is task-oriented only; it
  does not yet model `PENDING / PLANNING / READY / EXECUTING` group states.

Verification:

```text
python3 -m py_compile \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/planning/planning_types.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/__init__.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/scheduling_types.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/motion_batch.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/motion_group.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/motion_adapters.py
```

Next pickup point:

1. Add a small unit-style smoke test or compile-time usage check for
   `ordered_motion_batch_from_mappings(...)`.
2. Wire the adapter into `execute_ordered_motion_chain(...)` only for
   validation/logging first, while still passing original dictionaries into the
   current pipelined executor.
3. Then extract `_build_blended_group()` into `motion/blending` with no
   behavior change.

### 2026-08-10 - Slice 2: diagnostic MotionBatch adapter wiring

Status: completed and compile/smoke-checked. Runtime execution still uses the
legacy ordered-chain dictionaries.

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/motion_adapters.py
```

What was done:

- `execute_ordered_motion_chain(...)` now builds a typed `MotionBatch` through
  `ordered_motion_batch_from_mappings(...)` for validation/timing/logging.
- The existing `_execute_ordered_motion_chain_pipelined(...)` call still
  receives the original `segments` dictionaries.
- Adapter failures are logged as warnings and recorded in timing, then the
  legacy executor continues. This keeps the slice diagnostic-only.
- Removed the adapter's top-level `config` import. Runtime config loading
  requires `EROB_CONFIG_PACKAGE`, which makes simple import/smoke tests fail
  outside a launched robot runtime. The backend now passes
  `config.DEFAULT_VEL_PERCENT` and `config.DEFAULT_ACC_PERCENT` into the
  adapter explicitly.

Issues / repo facts found:

- Importing `config.py` directly outside the launched runtime fails when
  `EROB_CONFIG_PACKAGE` is unset. New low-level model/adapter modules should
  avoid top-level runtime config imports where practical.
- The adapter is currently stricter than the diagnostic path in one useful way:
  it rejects negative `blendR` instead of clamping.

Verification:

```text
python3 -m py_compile \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/planning/planning_types.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/__init__.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/scheduling_types.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/motion_batch.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/motion_group.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/motion_adapters.py

python3 -c "import sys; sys.path.insert(0, 'eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts'); from motion.scheduling.motion_adapters import ordered_motion_batch_from_mappings; batch = ordered_motion_batch_from_mappings([{'type':'linear','label':'a','position':[1,2,3,4,5,6],'vel':30,'acc':30,'blendR':0.0},{'type':'ptp','label':'b','position':[6,5,4,3,2,1],'vel':40,'acc':40,'blendR':0.0}], blocking=True, tool=0, user=0); print(len(batch.segments), batch.segments[0].label, batch.segments[1].__class__.__name__)"
```

Next pickup point:

1. Extract `_build_blended_group()` and its pure helper functions into
   `motion/blending`.
2. Keep the first blend extraction behavior-preserving: call the new
   `BlendBuilder` from inside `_execute_ordered_motion_chain_pipelined(...)`
   and compare logs/results before deleting legacy nested code.
3. Avoid moving scheduling or execution behavior in the same slice.

### 2026-08-10 - Slice 3: extracted BlendBuilder

Status: completed and compile/import-checked. Runtime blend construction now
routes through the extracted builder, but the old nested function remains
temporarily as reference/fallback code.

Files added:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/blending/__init__.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/blending/blend_builder.py
```

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py
```

What was done:

- Added `BlendBuilder`, `BlendBuilderConfig`, `xyz_distance_mm(...)` and
  `joint_path_distances(...)` under `motion/blending`.
- Moved the ordered blend group construction logic into
  `BlendBuilder.build(...)` with dependencies passed in explicitly:
  logger, state-validity callback and blend config values.
- Updated `_execute_ordered_motion_chain_pipelined(...)` to construct one
  `ordered_blend_builder` and call `ordered_blend_builder.build(planned_group)`
  instead of the nested `_build_blended_group(...)`.
- Kept scheduling, planning and execution flow unchanged.

Temporary condition:

- The nested legacy `_build_blended_group(...)` and its related nested helper
  functions still exist in `moveit_robot_backend.py`. This is intentional for
  one validation pass only. Remove the nested legacy blend function after
  comparing runtime logs/results from the extracted builder against the old
  behavior.

Issues / repo facts found:

- The blend builder depends on state-validity checks for generated blend
  samples, so extraction needs a callback rather than direct service ownership.
- The original blend code mutates short input trajectories by densifying them.
  The extracted builder preserves that behavior.
- The builder still uses the current planned-segment dictionary shape. Typed
  `PlannedTrajectory` integration is a later slice.

Verification:

```text
python3 -m py_compile \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/blending/__init__.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/blending/blend_builder.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/planning/planning_types.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/__init__.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/scheduling_types.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/motion_batch.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/motion_group.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/motion_adapters.py

python3 -c "import sys; sys.path.insert(0, 'eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts'); from motion.blending.blend_builder import BlendBuilder, BlendBuilderConfig, joint_path_distances, xyz_distance_mm; print(BlendBuilderConfig().sample_count, round(xyz_distance_mm([0,0,0],[3,4,0]), 1))"
```

Next pickup point:

1. Run a real ordered-chain blend case and compare `[OrderedBlend]` logs with
   expected behavior.
2. If the extracted builder behaves correctly, remove the nested legacy
   `_build_blended_group(...)` and any nested helpers used only by it.
3. Then begin extracting `_plan_ordered_segment(...)` into `motion/planning`.

### 2026-08-10 - Slice 4: removed nested legacy blend function

Status: completed and compile/import-checked. The active blend path is now only
the extracted `motion.blending.BlendBuilder`.

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py
```

What was done:

- Removed nested `_build_blended_group(...)` from
  `_execute_ordered_motion_chain_pipelined(...)`.
- Removed nested helper functions used only by that legacy blend function:
  `_xyz_distance_mm(...)`, `_joint_path_distances(...)` and
  `_blend_trim_fraction(...)`.
- Confirmed the only remaining backend blend call is
  `ordered_blend_builder.build(planned_group)`.

Issues / repo facts found:

- `_wait_state_validity(...)` remains nested in the backend because it is now
  passed into `BlendBuilder` as the validation callback.
- No real robot ordered-chain blend run was performed in this slice; validation
  was limited to syntax/import checks.

Verification:

```text
python3 -m py_compile \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/blending/__init__.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/blending/blend_builder.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/planning/planning_types.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/__init__.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/scheduling_types.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/motion_batch.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/motion_group.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/motion_adapters.py

python3 -c "import sys; sys.path.insert(0, 'eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts'); from motion.blending.blend_builder import BlendBuilderConfig, xyz_distance_mm; print(BlendBuilderConfig().sample_count, round(xyz_distance_mm([0,0,0],[3,4,0]), 1))"
```

Next pickup point:

1. Run a real ordered-chain blend case when a robot/sim runtime is available.
2. Begin extracting `_plan_ordered_segment(...)` into `motion/planning` in small
   pieces. Keep the first extraction as a wrapper around the current logic or a
   narrow split by segment type.
3. Do not alter scheduling, queueing or controller execution in the same slice.

### 2026-08-10 - Slice 5: ordered segment parameter parser

Status: completed and compile/smoke-checked. This is the first small
`_plan_ordered_segment(...)` extraction slice.

Files added:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/planning/ordered_segment_parameters.py
```

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py
```

What was done:

- Added `OrderedSegmentParameters` and
  `parse_ordered_segment_parameters(...)`.
- Moved common legacy ordered segment parsing out of
  `_plan_ordered_segment(...)`: `type`, `label`, `blendR`, velocity percent,
  acceleration percent, velocity scale, acceleration scale and `protected`.
- Updated `_plan_ordered_segment(...)` to use the parsed values.
- Kept all segment-specific LIN/PTP/PATH/unwind planning behavior in the
  backend for now.

Issues / repo facts found:

- This parser intentionally preserves the backend planner's legacy `blendR`
  behavior: negative values are clamped to `0.0` here. The public request
  parser/adapter can remain stricter.
- The blend grouping worker still has separate `segment.get("type")` and
  `segment.get("blendR")` reads for group detection. That is scheduling logic,
  not part of this planner parameter parsing slice.
- The ordered planner still returns legacy dictionaries. Typed
  `PlannedTrajectory` integration remains future work.

Verification:

```text
python3 -m py_compile \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/planning/ordered_segment_parameters.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/blending/__init__.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/blending/blend_builder.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/planning/planning_types.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/__init__.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/scheduling_types.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/motion_batch.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/motion_group.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/motion_adapters.py

python3 -c "import sys; sys.path.insert(0, 'eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts'); from motion.planning.ordered_segment_parameters import parse_ordered_segment_parameters; p = parse_ordered_segment_parameters({'type':'Linear','label':'a','vel':50,'acc':25,'blendR':-3,'protected':1}, 0, default_velocity_percent=30, default_acceleration_percent=30); print(p.segment_type, p.label, p.blend_radius, p.velocity_scale, p.acceleration_scale, p.protected)"
```

Next pickup point:

1. Extract one segment-specific planner path next, preferably LIN because it
   already delegates most work to `motion.planning.segment_planning._plan_segment`.
2. Keep the extracted LIN planner returning the same legacy planned dictionary
   shape at first.
3. Do not extract PTP/PATH/unwind in the same slice.

### 2026-08-10 - Runtime regression note: drive set controller startup race

Status: fixed in source and rebuilt into `install/erob_moveit_runtime`.
This was not caused by the motion refactor slices.

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/runtime_adapter.py
```

What happened:

- Startup auto-enable repeatedly failed with:
  `Could not activate controller with name 'drive_disable_set_controller'
  because no controller with this name exists`.
- Both source and installed `zeroerr/config/ros2_controllers.yaml` already
  define `drive_enable_set_controller` and `drive_disable_set_controller`.
- `zeroerr/launch/full_stack.launch.py` spawns `drive_enable_set_controller` at 3s
  and `drive_disable_set_controller` at 4s. Runtime auto-enable could call the
  STRICT controller switch before the second controller was loaded.

What was done:

- `ZeroErrRuntimeAdapter._switch_drive_set_controllers(..., activate=True)` now
  waits up to 8 seconds for both set controllers to appear in
  `/controller_manager/list_controllers` before sending the STRICT switch.
- Deactivation still uses the immediate controller state and only deactivates
  loaded active controllers.
- Rebuilt `erob_moveit_runtime`, so the installed launch path has the fix.

Verification:

```text
python3 -m py_compile \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/runtime_adapter.py

cd /home/ilv/ros2_ws/eRob_moveit
colcon build --packages-select erob_moveit_runtime

python3 -m py_compile \
  eRob_moveit/install/erob_moveit_runtime/lib/erob_moveit_runtime/backend/runtime_adapter.py
```

Next pickup point:

1. Relaunch ZeroErr and confirm startup logs show either both set controllers
   loaded before activation or temporary
   `[DriveEnable] Waiting for set controllers to load: ...` messages.
2. If the spawner still fails to load `drive_disable_set_controller`, inspect
   controller manager service responsiveness and spawner timing rather than the
   controller YAML; the YAML already contains the controller.

### 2026-08-10 - Slice 6: extracted ordered LIN planner wrapper

Status: completed and compile/smoke-checked. The ordered LIN branch now lives
in `motion/planning`, while PTP/PATH/unwind remain in the backend.

Files added:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/planning/ordered_lin_planner.py
```

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py
```

What was done:

- Added `plan_ordered_linear_segment(...)`.
- Moved the LIN branch from nested `_plan_ordered_segment(...)` into the new
  helper.
- The helper still delegates to existing
  `motion.planning.segment_planning._plan_segment(...)`.
- The helper returns the same legacy planned dictionary shape as before:
  `type`, `label`, `start_position`, `target_position`, `final_state`,
  `trajectory`, timing fields, `protected`, `blendR`, velocity/acceleration
  scales and `optimization_deferred`.
- Kept timing event names and fields unchanged.

Issues / repo facts found:

- `_plan_segment(...)` already supports `defer_optimization` in this repo, but
  the extracted helper preserves the previous compatibility fallback for older
  versions.
- The backend still imports `_plan_segment` and
  `_robot_state_from_trajectory_end` because they are passed into the helper as
  dependencies for this behavior-preserving slice.
- No real ordered LIN or blend run was performed in this slice.

Verification:

```text
python3 -m py_compile \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/planning/ordered_lin_planner.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/planning/ordered_segment_parameters.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/blending/__init__.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/blending/blend_builder.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/planning/planning_types.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/__init__.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/scheduling_types.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/motion_batch.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/motion_group.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/motion_adapters.py

python3 -c "import sys; sys.path.insert(0, 'eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts'); from motion.planning.ordered_lin_planner import plan_ordered_linear_segment; from motion.planning.ordered_segment_parameters import parse_ordered_segment_parameters; p = parse_ordered_segment_parameters({'type':'linear','label':'lin','vel':30,'acc':40,'blendR':5}, 0, default_velocity_percent=30, default_acceleration_percent=30); print(callable(plan_ordered_linear_segment), p.segment_type, p.velocity_scale, p.acceleration_scale)"
```

Next pickup point:

1. Extract the ordered PTP branch next into `motion/planning`, keeping the same
   legacy planned dictionary shape.
2. Keep PTP extraction separate from PATH/unwind.
3. After PTP extraction, consider introducing a small shared helper for
   `ordered_segment_plan_done` timing payloads if duplication becomes obvious.

### 2026-08-10 - Slice 7: extracted ordered PTP planner wrapper

Status: completed and compile/smoke-checked. Ordered LIN and PTP branches now
live in `motion/planning`; PATH and unwind remain in the backend.

Files added:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/planning/ordered_ptp_planner.py
```

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py
```

What was done:

- Added `plan_ordered_ptp_segment(...)`.
- Moved the ordered PTP branch from nested `_plan_ordered_segment(...)` into
  the new helper.
- The helper still delegates to existing
  `motion.planning.ptp_target.plan_ptp_trajectory(...)`.
- The helper returns the same legacy planned dictionary shape as before,
  including noop handling, native PTP timing fields, optional optimization,
  `protected`, `blendR`, velocity/acceleration scales and
  `optimization_deferred`.
- Kept timing event names and fields unchanged.

Issues / repo facts found:

- `motion.planning.ptp_target` imports runtime `config.py`, which fails outside
  a launched runtime if `EROB_CONFIG_PACKAGE` is unset. To keep the new wrapper
  import/smoke-testable, `ordered_ptp_planner.py` imports
  `plan_ptp_trajectory` lazily inside `plan_ordered_ptp_segment(...)`.
- `RobotTrajectory` remains needed in the PTP helper for optimization wrapping.
- No real ordered PTP run was performed in this slice.

Verification:

```text
python3 -m py_compile \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/planning/ordered_ptp_planner.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/planning/ordered_lin_planner.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/planning/ordered_segment_parameters.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/blending/__init__.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/blending/blend_builder.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/planning/planning_types.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/__init__.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/scheduling_types.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/motion_batch.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/motion_group.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/motion_adapters.py

python3 -c "import sys; sys.path.insert(0, 'eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts'); from motion.planning.ordered_ptp_planner import plan_ordered_ptp_segment; from motion.planning.ordered_segment_parameters import parse_ordered_segment_parameters; p = parse_ordered_segment_parameters({'type':'ptp','label':'ptp','vel':60,'acc':30,'blendR':2}, 0, default_velocity_percent=30, default_acceleration_percent=30); print(callable(plan_ordered_ptp_segment), p.segment_type, p.velocity_scale, p.acceleration_scale)"
```

Next pickup point:

1. Extract the ordered PATH branch next, keeping the same legacy planned
   dictionary shape.
2. Keep PATH extraction separate from unwind because unwind has substantially
   different control flow and live fallback behavior.
3. After PATH extraction, `_plan_ordered_segment(...)` should mostly dispatch
   LIN/PTP/PATH helpers and retain only unwind locally.

### 2026-08-10 - Slice 8: extracted ordered PATH planner wrapper

Status: completed and compile/smoke-checked. Ordered LIN, PTP and PATH
branches now live in `motion/planning`; unwind remains in the backend.

Files added:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/planning/ordered_path_planner.py
```

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py
```

What was done:

- Added `plan_ordered_path_segment(...)`.
- Moved the ordered PATH branch from nested `_plan_ordered_segment(...)` into
  the new helper.
- The helper still delegates to existing
  `motion.planning.segment_planning._build_follow_path_trajectory(...)`.
- The helper returns the same legacy planned dictionary shape as before:
  `type`, `label`, `target_position`, `final_state`, `trajectory`,
  `plan_elapsed_s`, `optimize_elapsed_s`, `protected`, `blendR`, velocity and
  acceleration scales and `optimization_deferred`.
- Kept path blending rejection behavior unchanged: `blendR > 0` still raises.
- Kept timing event names and fields unchanged.

Issues / repo facts found:

- PATH planning prepends `current_cartesian` to the workobject-transformed path
  before calling `_build_follow_path_trajectory(...)`; this behavior is
  preserved in the helper.
- `math` remains needed in `moveit_robot_backend.py` for unwind/jog code after
  PATH extraction.
- No real ordered PATH run was performed in this slice.

Verification:

```text
python3 -m py_compile \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/planning/ordered_path_planner.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/planning/ordered_ptp_planner.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/planning/ordered_lin_planner.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/planning/ordered_segment_parameters.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/blending/__init__.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/blending/blend_builder.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/planning/planning_types.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/__init__.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/scheduling_types.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/motion_batch.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/motion_group.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/motion_adapters.py

python3 -c "import sys; sys.path.insert(0, 'eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts'); from motion.planning.ordered_path_planner import plan_ordered_path_segment; from motion.planning.ordered_segment_parameters import parse_ordered_segment_parameters; p = parse_ordered_segment_parameters({'type':'path','label':'path','vel':20,'acc':30,'blendR':0}, 0, default_velocity_percent=30, default_acceleration_percent=30); print(callable(plan_ordered_path_segment), p.segment_type, p.velocity_scale, p.acceleration_scale)"
```

Next pickup point:

1. Do not rush unwind extraction. First consider whether to split unwind into
   smaller helper functions because it has live-final-unwind behavior, no-op
   behavior, multi-piece direct IK planning and explicit joint branch forcing.
2. A safe next slice is to extract the simple unwind result builders or config
   parsing, not the whole unwind branch.
3. Keep controller execution and scheduler behavior unchanged.

### 2026-08-10 - Slice 9: extracted ordered unwind config and result helpers

Status: completed and compile-checked. The ordered unwind trajectory planning
loop still remains in `moveit_robot_backend.py` intentionally.

Files added:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/planning/ordered_unwind_planner.py
```

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py
docs/refactoring/refactoring_plan_cpp_validation.md
```

What was done:

- Added `OrderedUnwindConfig` and `parse_ordered_unwind_config(...)`.
- Centralized ordered unwind settings for joint name, rotation axis, minimum
  delta, velocity/acceleration percentages, scales, sign, segment size and
  final live-execution mode.
- Added small builders for the three legacy unwind planned dictionary shapes:
  live runtime unwind, no-op unwind and preplanned unwind.
- Wired `_plan_ordered_segment(...)` to use those helpers while keeping the
  direct IK planning loop and execution behavior in the backend.
- Removed common LIN/PTP/PATH velocity locals left unused after earlier branch
  extraction.

Issues / repo facts found:

- Ordered unwind has substantially more execution coupling than LIN/PTP/PATH:
  final live execution, explicit wrap preservation, multi-piece direct IK and
  post-execution joint verification.
- The standalone unwind path still has a separate implementation below the
  ordered-chain code. Do not delete or merge it until ordered unwind behavior
  is validated under runtime.
- `colcon build --packages-select erob_moveit_runtime` still reports that
  `/opt/ros/rolling/setup.bash` is missing in this environment, but the package
  builds successfully through the available underlay.
- No real ordered unwind run was performed in this slice.

Verification:

```text
python3 -m py_compile \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/planning/ordered_unwind_planner.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/planning/ordered_path_planner.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/planning/ordered_ptp_planner.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/planning/ordered_lin_planner.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/planning/ordered_segment_parameters.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/blending/__init__.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/blending/blend_builder.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/planning/planning_types.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/__init__.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/scheduling_types.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/motion_batch.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/motion_group.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/motion_adapters.py

python3 -c "import sys; sys.path.insert(0, 'eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts'); from motion.planning.ordered_unwind_planner import parse_ordered_unwind_config, build_ordered_unwind_noop_result; C = type('C', (), {'DEFAULT_VEL_PERCENT': 30, 'DEFAULT_ACC_PERCENT': 40, 'EXECUTOR_POST_UNWIND_ROTATION_AXIS_INDEX': 5, 'EXECUTOR_POST_UNWIND_JOINT_NAME': 'Joint_6', 'EXECUTOR_POST_UNWIND_MIN_DELTA_RAD': 0.5, 'EXECUTOR_POST_UNWIND_ROTATION_AXIS_SIGN': 0.0, 'EXECUTOR_POST_UNWIND_ROTATIONAL_SEGMENT_DEG': 180.0, 'EXECUTOR_ORDERED_FINAL_UNWIND_LIVE_EXECUTION': True}); cfg = parse_ordered_unwind_config({'vel': 20}, config_obj=C, clamp_percentage=lambda v: float(v), is_final_segment=True); print(cfg.joint_name, cfg.axis_index, cfg.velocity_scale, cfg.acceleration_scale, cfg.sign, cfg.live_final_execution, build_ordered_unwind_noop_result(segment_type='unwind_joint6', label='u', current_cartesian=[1,2,3,4,5,6], current_state='s', plan_started=0.0, protected=False)['blendR'])"

/bin/bash -lc "env -i HOME=\"$HOME\" TERM=\"${TERM:-xterm}\" PATH=\"/usr/bin:/bin:/usr/sbin:/sbin\" bash --noprofile --norc -lc 'source /opt/ros/rolling/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws/eRob_moveit; colcon build --packages-select erob_moveit_runtime'"
```

Next pickup point:

1. Runtime-test an ordered chain with LIN/PTP/PATH plus a no-op unwind and a
   required unwind.
2. If runtime behavior matches, extract the ordered unwind direct IK piece
   planner as a helper while keeping live final unwind execution in the backend.
3. Keep the standalone nested legacy unwind implementation in place as
   fallback/reference until ordered unwind is validated and the user confirms
   removal timing.

### 2026-08-10 - Runtime validation after Slice 9: launch and real moves

Status: runtime log reviewed from
`eRob_moveit/src/eRob_ROS2_MoveIt/zeroerr/docs/temp_logs`.

Files changed after review:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/execution/trajectory_executor.py
docs/refactoring/refactoring_plan_cpp_validation.md
```

What was validated:

- The earlier `drive_disable_set_controller` startup regression did not
  reproduce. The controller was loaded/configured, both set controllers
  activated successfully and the STRICT switch succeeded.
- The MotionBatch diagnostic adapter validated both tested ordered chains:
  a 4-segment chain (`LinearSegment:2,PtpSegment:2`) and a 15-segment chain
  (`LinearSegment:6,PathSegment:1,PtpSegment:7,UnwindSegment:1`).
- Extracted ordered LIN/PTP/PATH wrappers were exercised through runtime.
- Extracted `BlendBuilder` built runtime blend groups:
  a 2-segment group, a 6-segment group and a 7-segment group.
- Ordered PATH ran for `paint_contact_1:Workpiece`, planned 763 waypoints into
  a 150-point trajectory, executed successfully and matched the end state.
- Final live ordered unwind ran through the new unwind helper path and reached
  the live no-op case successfully:
  `Rotational-path unwind skipped - no unwind needed`.
- Both ordered chains completed with `result=0`.

Issues found:

- Startup drive enable initially sent a short hold-position trajectory whose
  absolute `header.stamp` was created before controller preparation. The
  controller rejected it because `header.stamp + 0.260s` was already in the
  past by the time the goal arrived.
- Patched `_execute_trajectory(...)` to refresh
  `controller_goal.trajectory.header.stamp` immediately before
  `send_goal_async(...)`. This preserves relative `time_from_start` values but
  prevents short trajectories from expiring during preparation.
- Ruckig reported one implausible output on the first linear segment and fell
  back to the seeded TOTG trajectory. The move still executed successfully.
- The PATH diagnostic detected a near-180 degree Joint 6 reversal candidate,
  but TOTG still generated a trajectory and the PATH segment executed
  successfully.
- MoveIt still logs expected octomap warnings when no 3D sensor plugin is
  configured.

Verification:

```text
python3 -m py_compile \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/execution/trajectory_executor.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/runtime_adapter.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/planning/ordered_unwind_planner.py

/bin/bash -lc "env -i HOME=\"$HOME\" TERM=\"${TERM:-xterm}\" PATH=\"/usr/bin:/bin:/usr/sbin:/sbin\" bash --noprofile --norc -lc 'source /opt/ros/rolling/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws/eRob_moveit; colcon build --packages-select erob_moveit_runtime'"
```

Next pickup point:

1. Re-launch once and confirm the startup hold trajectory is accepted after the
   header-stamp refresh.
2. Runtime-test a non-no-op ordered unwind before extracting more unwind logic.
3. Keep the standalone legacy unwind implementation in place as
   fallback/reference until that non-no-op unwind path is validated.

### 2026-08-10 - Startup readiness gate tightened

Status: completed and compile/build-checked. Needs one launch validation to
confirm GUI/client behavior.

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/robot_controller.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/rest_server.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/runtime_api/handlers.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/runtime_websockets/state_server.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/runtime_websockets/execution_server.py
docs/refactoring/refactoring_plan_cpp_validation.md
```

What was done:

- Expanded `RobotController.is_motion_stack_ready()` so readiness includes the
  services and servers used by runtime moves:
  joint state, Cartesian pose, Cartesian path service, IK, FK, state validity,
  PTP helper, contour IK helper, IPP/TOTG optimizer and FollowJointTrajectory.
- Added a shared motion-readiness gate in `RuntimeApi`. Motion endpoints now
  return HTTP 503 with `motion_stack_fault` instead of accepting a request
  while dependencies are still warming up.
- Applied the gate to LIN, PTP, PATH, sequence, ordered motion chain, explicit
  Joint 6 unwind and jog.
- Changed `/startup/status` and `/health` readiness semantics: after the
  runtime object exists they report `motion_stack_warming` with a specific
  missing dependency until `is_motion_stack_ready()` passes.
- Updated state/execution WebSocket payloads so `runtime_initialized` means
  Python objects exist, while `runtime_ready` now means the motion stack is
  actually ready.
- Added standing rule: every new motion service/helper dependency must be
  added to motion-stack readiness, fault reporting and the startup/WebSocket
  state contract in the same slice.
- Compatibility correction: legacy GUI/client probes treat
  `/health.status == "ok"` as the interaction gate, so `/health` must only
  return `status: "ok"` once the motion stack is ready. During warmup it
  should still return JSON with HTTP 200 and `ready=false`, but `status` must
  be the phase, for example `motion_stack_warming`.
- Regression fix: readiness checks must never raise through Flask. If a
  service/client readiness probe fails internally, it now becomes
  `motion_stack_ready=false` with a JSON `motion_stack_fault` response instead
  of Flask's default HTML 500 page. REST also has a JSON fallback error handler
  so GUI `response.json()` calls do not fail on backend exceptions.
- Regression fix: `RobotController` now exposes `get_ptp_client()` and
  `get_contour_ik_client()` as thin delegates to `PlannerContext`, matching
  the existing FK/IK/state-validity client accessors. This fixes `/health`
  getting stuck in `motion_stack_warming` with
  `RobotController object has no attribute 'get_ptp_client'`.
- Startup regression fix from master comparison: restored the
  `RobotController` property delegates for `current_joint_state`,
  `prev_cartesian`, `latest_data`, `active_execute_send_future`,
  `active_controller_goal` and `plan_generation`. The fake-hardware branch had
  removed these while moving state into `RobotStateStore`, which caused early
  readiness probes to report attribute errors and caused later assignments to
  bypass the store.

Issues / repo facts found:

- The old startup path set `ready=True` as soon as `runtime_initializer`
  returned robot/node objects, which could happen before helper services and
  controller action server availability caught up.
- Backend motion functions had their own readiness checks, but API clients
  could still see a ready state and submit too early. The transport layer now
  blocks this earlier and reports the missing dependency.
- Readiness probes must call methods that exist on the same API surface as
  the motion paths. PTP planning already expected
  `RobotController.get_ptp_client()`, but only `PlannerContext` had the method,
  so the readiness probe exposed an existing wrapper gap.
- Master startup was faster mostly because it reported REST ready once the
  runtime object existed. The current readiness slice intentionally waits for
  MoveIt/helper/controller services before `/health.status` becomes `ok`.
  This is safer for the GUI but makes the visible ready time no faster than the
  slowest required helper, currently launched at fixed timers around 5.0s,
  6.5s, 8.0s and 8.5s.
- Branch-level launch difference from master: `zeroerr/launch/full_stack.launch.py`
  replaced MoveIt's `generate_demo_launch(moveit_config)` with a manually
  assembled launch graph for fake-hardware support. Real-hardware timer values
  for runtime/state/helper nodes stayed roughly the same, but the launch graph,
  RViz default config path and ros2_control setup are no longer identical to
  master.
- This patch intentionally does not block status, state, tool, safety or drive
  endpoints during warmup; those remain useful for UI diagnostics.
- Rule for future agents: do not add a service/client/helper that a move path
  depends on without also updating `RobotController.is_motion_stack_ready()`,
  `RobotController.get_motion_stack_fault_reason()`, `/startup/status`,
  `/health`, `/status` and WebSocket readiness payloads as needed.
- Client compatibility rule: do not weaken the meaning of `/health.status`.
  Existing GUI/client probes treat `status == "ok"` as permission to enable
  interaction, so `ok` must mean full motion readiness. Use `/startup/status`,
  `/status`, `ready`, `phase`, `motion_stack_ready` and `motion_stack_fault`
  for detailed warmup diagnostics.
- REST/API compatibility rule: all REST endpoints, especially motion endpoints,
  must return JSON on expected unready/error paths. Do not allow startup or
  readiness exceptions to escape as Flask HTML error pages.

Verification:

```text
python3 -m py_compile \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/robot_controller.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/rest_server.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/runtime_api/handlers.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/runtime_websockets/state_server.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/runtime_websockets/execution_server.py

/bin/bash -lc "env -i HOME=\"$HOME\" TERM=\"${TERM:-xterm}\" PATH=\"/usr/bin:/bin:/usr/sbin:/sbin\" bash --noprofile --norc -lc 'source /opt/ros/rolling/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws/eRob_moveit; colcon build --packages-select erob_moveit_runtime'"
```

Next pickup point:

1. Re-launch and watch `/startup/status`, `/health`, `/status` and both
   WebSockets during startup. Confirm `runtime_ready` stays false until helper
   services and controller action server are actually available.
2. Try an early move request during warmup; expected response is HTTP 503 with
   `motion_stack_fault`.
3. After readiness goes true, rerun a short LIN/PTP plus ordered chain smoke
   test.

### 2026-08-10 - Slice 10: scheduler grouping skeleton

Status: completed and compile/smoke-checked. This slice does not change live
motion execution behavior. It introduces the first generalized
`MotionScheduler` grouping pass and wires it only into ordered-chain diagnostic
logging.

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/motion_scheduler.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/__init__.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py
docs/refactoring/refactoring_plan_cpp_validation.md
```

What was done:

- Added `MotionScheduler.group_batch(...)` and `group_motion_batch(...)`.
- Added contiguous compatibility rules:
  LIN+LIN can group, PTP+PTP can group, PATH and unwind stay standalone.
- Added hard boundary detection:
  PATH/unwind always hard-stop, and `blend_radius <= 0` creates a hard
  boundary after that segment.
- Added planner keys for scheduler groups:
  `linked_lin`, `lin`, `ptp`, `path`, `unwind`.
- Extended ordered-chain adapter diagnostics to log the computed group summary
  without changing the legacy ordered-chain executor path.

Issues / repo facts found:

- `MotionBatch` and `MotionGroup` models already existed from earlier slices,
  so this slice only needed the deterministic grouping pass.
- The active ordered-chain implementation is still the legacy pipelined path.
  The scheduler grouping is currently observability-only and safe to compare
  against runtime behavior before switching execution over.

Verification:

```text
python3 -m py_compile \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/motion_scheduler.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/__init__.py

/bin/bash -lc "source /opt/ros/jazzy/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws; PYTHONPATH=eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts:$PYTHONPATH python3 -c \"from motion.planning.planning_types import LinearSegment, PtpSegment, PathSegment; from motion.scheduling.scheduling_types import MotionBatch; from motion.scheduling.motion_scheduler import group_motion_batch; b=MotionBatch((LinearSegment('l1',10,10,5), LinearSegment('l2',10,10,0), PtpSegment('p1',10,10,5), PtpSegment('p2',10,10,0), PathSegment('path',10,10,0, waypoints=((1,2,3),(4,5,6))))); print([(g.start_index, g.planner_name, len(g.segments), g.hard_stop_after) for g in group_motion_batch(b)])\""
# output: [(0, 'linked_lin', 2, True), (2, 'ptp', 2, True), (4, 'path', 1, True)]

/bin/bash -lc "env -i HOME=\"$HOME\" TERM=\"${TERM:-xterm}\" PATH=\"/usr/bin:/bin:/usr/sbin:/sbin\" bash --noprofile --norc -lc 'source /opt/ros/rolling/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws/eRob_moveit; colcon build --packages-select erob_moveit_runtime'"
```

Next pickup point:

1. Run an ordered-chain request and inspect the new
   `[OrderedChain] MotionBatch adapter validated ... groups=...` log.
2. Compare the group summary against the legacy ordered-chain planning groups.
3. If the grouping matches expected behavior, start extracting group execution
   status into scheduler-owned compatibility status while still using the
   legacy executor.

### 2026-08-10 - Slice 11: scheduler-owned ordered status adapter

Status: completed and compile/smoke-checked. Live ordered-chain execution is
still unchanged. This slice starts moving the legacy ordered-chain status shape
behind scheduler-owned compatibility helpers.

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/status_adapter.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/__init__.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py
docs/refactoring/refactoring_plan_cpp_validation.md
```

What was done:

- Added `normalize_ordered_chain_status(...)` to centralize legacy
  ordered-chain status merging and inactive-field resets.
- Added `ordered_chain_group_status(...)` to expose JSON-safe scheduler group
  metadata in the existing ordered-chain status payload.
- Routed `MoveItRobotBackend._set_ordered_motion_chain_status(...)` through the
  scheduling adapter.
- Added `scheduler_groups` and `scheduler_group_count` to the initial
  ordered-chain status after `MotionBatch` adapter validation succeeds.

Issues / repo facts found:

- The existing status payload is widely consumed through REST and execution
  WebSocket paths, so this slice preserves field names and only adds scheduler
  metadata.
- `_preplanned_snapshot(...)` is still nested inside the legacy executor. That
  is the next status-related extraction candidate.

Verification:

```text
python3 -m py_compile \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/status_adapter.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/__init__.py

/bin/bash -lc "source /opt/ros/jazzy/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws; PYTHONPATH=eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts:$PYTHONPATH python3 -c \"from motion.planning.planning_types import LinearSegment; from motion.scheduling.scheduling_types import MotionBatch; from motion.scheduling.motion_scheduler import group_motion_batch; from motion.scheduling.status_adapter import normalize_ordered_chain_status, ordered_chain_group_status; groups=group_motion_batch(MotionBatch((LinearSegment('l1',10,10,5), LinearSegment('l2',10,10,0)))); print(ordered_chain_group_status(groups)); print(normalize_ordered_chain_status({'active': True, 'current_segment_number': 2}, {'active': False, 'phase': 'completed'}, updated_at=123.0))\""

/bin/bash -lc "env -i HOME=\"$HOME\" TERM=\"${TERM:-xterm}\" PATH=\"/usr/bin:/bin:/usr/sbin:/sbin\" bash --noprofile --norc -lc 'source /opt/ros/rolling/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws/eRob_moveit; colcon build --packages-select erob_moveit_runtime'"
```

Next pickup point:

1. Run an ordered-chain request and inspect `/execute/ordered_motion_chain/status`
   for `scheduler_groups` and `scheduler_group_count`.
2. Extract `_preplanned_snapshot(...)` into `motion/scheduling/status_adapter.py`
   once the added scheduler status fields are confirmed compatible with the
   GUI/client.
3. Keep the legacy nested unwind function in place for now as requested; remove
   it later only after the extracted unwind helper is fully validated.

### 2026-08-10 - Slice 12: extracted preplanned status snapshot

Status: completed and compile/smoke-checked. Live execution behavior is still
unchanged. This slice moves another piece of ordered-chain status calculation
out of the backend and into scheduler-owned compatibility helpers.

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/status_adapter.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/__init__.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py
docs/refactoring/refactoring_plan_cpp_validation.md
```

What was done:

- Added `ordered_chain_preplanned_snapshot(...)` to
  `motion/scheduling/status_adapter.py`.
- Replaced the nested `_preplanned_snapshot(...)` implementation in the legacy
  ordered-chain executor with a locked call to the new helper.
- Preserved all legacy payload fields:
  `planned_segments_count`, `preplanned_ready_count`,
  `next_preplanned_segment_*` and `last_planned_segment_*`.

Issues / repo facts found:

- The backend still owns `planned_by_index` and the planning lock; only the
  JSON-compatible status projection moved in this slice.
- This is another behavior-preserving bridge toward a scheduler-owned queue.

Verification:

```text
python3 -m py_compile \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/status_adapter.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/__init__.py

/bin/bash -lc "source /opt/ros/jazzy/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws; PYTHONPATH=eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts:$PYTHONPATH python3 -c \"from motion.scheduling.status_adapter import ordered_chain_preplanned_snapshot; planned={0:{'label':'a','type':'linear'},2:{'label':'c','type':'ptp'},4:{'label':'e','type':'path'}}; print(ordered_chain_preplanned_snapshot(planned, current_index=1))\""

/bin/bash -lc "env -i HOME=\"$HOME\" TERM=\"${TERM:-xterm}\" PATH=\"/usr/bin:/bin:/usr/sbin:/sbin\" bash --noprofile --norc -lc 'source /opt/ros/rolling/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws/eRob_moveit; colcon build --packages-select erob_moveit_runtime'"
```

Next pickup point:

1. Run an ordered-chain request and confirm the preplanned status fields still
   update as before.
2. Start introducing scheduler-owned queue/group state in parallel with
   `planned_by_index`, still as observability-only fields.
3. Keep the legacy nested unwind function in place for now as fallback/reference
   until the extracted unwind path has more runtime validation.

### 2026-08-10 - Slice 13: scheduler segment-state observability

Status: completed and compile/smoke-checked. Live execution behavior is still
unchanged. This slice adds scheduler-owned segment state metadata alongside the
legacy ordered-chain executor.

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/status_adapter.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/__init__.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py
docs/refactoring/refactoring_plan_cpp_validation.md
```

What was done:

- Added `ordered_chain_initial_segment_states(...)` and
  `ordered_chain_segment_state_status(...)`.
- Added `scheduler_segment_states` to ordered-chain status.
- Existing legacy events now update scheduler segment state:
  - initial request: `PENDING`
  - `_mark_planned(...)`: `READY`
  - `_execute_planned_segment(...)` start: `EXECUTING`
  - `_execute_planned_segment(...)` completion: `DONE` or `FAILED`

Issues / repo facts found:

- This is segment-state observability only. It does not yet replace
  `planned_by_index`, `planned_queue`, or the legacy execution loop.
- Blended logical segments keep the existing legacy contract: the physical
  blended trajectory is queued at the first segment, and later logical members
  are represented as `blend_consumed`.

Verification:

```text
python3 -m py_compile \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/status_adapter.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/__init__.py

/bin/bash -lc "source /opt/ros/jazzy/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws; PYTHONPATH=eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts:$PYTHONPATH python3 -c \"from motion.scheduling.status_adapter import ordered_chain_segment_state_status; states={1:{'segment_index':1,'state':'READY'},0:{'segment_index':0,'state':'DONE'}}; print(ordered_chain_segment_state_status(states))\""

/bin/bash -lc "env -i HOME=\"$HOME\" TERM=\"${TERM:-xterm}\" PATH=\"/usr/bin:/bin:/usr/sbin:/sbin\" bash --noprofile --norc -lc 'source /opt/ros/rolling/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws/eRob_moveit; colcon build --packages-select erob_moveit_runtime'"
```

Next pickup point:

1. Runtime-check ordered-chain status and confirm `scheduler_segment_states`
   transitions match legacy `phase/current_segment/preplanned` fields.
2. Add group-level state transitions for `scheduler_groups` after segment-state
   telemetry is validated.
3. Keep the legacy nested unwind function in place for now as fallback/reference
   until the extracted unwind path has more runtime validation.

### 2026-08-10 - Slice 14: scheduler group-state observability

Status: completed and compile/smoke-checked. Live execution behavior is still
unchanged. This slice adds scheduler group state transitions derived from the
segment-state telemetry introduced in Slice 13.

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/status_adapter.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/__init__.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py
docs/refactoring/refactoring_plan_cpp_validation.md
```

What was done:

- Added scheduler group state helpers:
  `ordered_chain_initial_group_states(...)`,
  `update_ordered_chain_group_states_from_segments(...)` and
  `ordered_chain_group_state_status(...)`.
- `scheduler_groups` now include `state` and `result`.
- Group state is derived from member segment states:
  `PENDING`, `READY`, `EXECUTING`, `DONE` or `FAILED`.
- Passed initial scheduler group/segment metadata from
  `execute_ordered_motion_chain(...)` into the legacy pipelined executor so the
  runtime status observes the same grouping that the new scheduler computed.

Issues / repo facts found:

- This remains observability-only. The legacy queue and execution loop still
  decide what actually runs and when.
- Group state currently mirrors logical segment state. It does not yet model
  independently queued planned groups.

Verification:

```text
python3 -m py_compile \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/status_adapter.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/__init__.py

/bin/bash -lc "source /opt/ros/jazzy/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws; PYTHONPATH=eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts:$PYTHONPATH python3 -c \"from motion.scheduling.status_adapter import ordered_chain_initial_group_states, ordered_chain_group_state_status, update_ordered_chain_group_states_from_segments; groups=ordered_chain_initial_group_states(({'group_index':0,'start_segment_index':0,'end_segment_index':1,'planner_name':'linked_lin','segment_count':2,'hard_stop_after':True},)); segments={0:{'state':'DONE','result':0},1:{'state':'EXECUTING','result':None}}; update_ordered_chain_group_states_from_segments(groups, segments); print(ordered_chain_group_state_status(groups)); segments[1]={'state':'DONE','result':0}; update_ordered_chain_group_states_from_segments(groups, segments); print(ordered_chain_group_state_status(groups))\""

/bin/bash -lc "env -i HOME=\"$HOME\" TERM=\"${TERM:-xterm}\" PATH=\"/usr/bin:/bin:/usr/sbin:/sbin\" bash --noprofile --norc -lc 'source /opt/ros/rolling/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws/eRob_moveit; colcon build --packages-select erob_moveit_runtime'"
```

Next pickup point:

1. Runtime-check ordered-chain status and confirm `scheduler_groups` and
   `scheduler_segment_states` transition consistently during real moves.
2. Start extracting the legacy `planned_queue`/`planned_by_index` state holder
   into a scheduler-owned observation object, still without changing execution.
3. Keep the legacy nested unwind function in place for now as fallback/reference
   until the extracted unwind path has more runtime validation.

### 2026-08-10 - Slice 15: scheduler ordered-chain observation object

Status: completed and compile/smoke-checked. Live execution behavior is still
unchanged. This slice moves the legacy planned-segment observation map and lock
behind a scheduler-owned helper object.

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/ordered_observation.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/__init__.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py
docs/refactoring/refactoring_plan_cpp_validation.md
```

What was done:

- Added `OrderedChainObservation`.
- Moved planned-segment observation state into that class:
  `mark_planned(...)`, `mark_consumed(...)` and
  `preplanned_snapshot(...)`.
- Removed the backend-local `planning_lock` and `planned_by_index` variables.
- Kept the legacy `planned_queue` and ordered-chain execution loop unchanged.

Issues / repo facts found:

- This slice only extracts observation/status state. It does not yet make the
  scheduler own the planning queue.
- The `planned_queue` remains the next migration target.

Verification:

```text
python3 -m py_compile \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/ordered_observation.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/__init__.py

/bin/bash -lc "source /opt/ros/jazzy/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws; PYTHONPATH=eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts:$PYTHONPATH python3 -c \"from motion.scheduling.ordered_observation import OrderedChainObservation; obs=OrderedChainObservation(); print(obs.mark_planned(2, {'label':'third','type':'ptp'})); print(obs.preplanned_snapshot(current_index=1)); obs.mark_consumed(2); print(obs.preplanned_snapshot(current_index=1))\""

/bin/bash -lc "env -i HOME=\"$HOME\" TERM=\"${TERM:-xterm}\" PATH=\"/usr/bin:/bin:/usr/sbin:/sbin\" bash --noprofile --norc -lc 'source /opt/ros/rolling/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws/eRob_moveit; colcon build --packages-select erob_moveit_runtime'"
```

Next pickup point:

1. Runtime-check ordered-chain status after this extraction.
2. Extract a scheduler-owned planned queue wrapper around the legacy
   `Queue` operations while preserving the same producer/consumer behavior.
3. Keep the legacy nested unwind function in place for now as fallback/reference
   until the extracted unwind path has more runtime validation.

### 2026-08-10 - Slice 16: scheduler planned-queue wrapper

Status: completed and compile/smoke-checked. Live execution behavior is still
unchanged. This slice wraps the legacy ordered-chain planned queue tuple
protocol in a scheduler-owned helper.

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/ordered_planned_queue.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/__init__.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py
docs/refactoring/refactoring_plan_cpp_validation.md
```

What was done:

- Added `OrderedPlannedQueue`.
- Replaced direct backend `Queue()` construction with `OrderedPlannedQueue()`.
- Replaced direct `planned_queue.put((index, planned, None))` calls with
  `put_planned(...)`.
- Replaced terminal queue tuples with `put_done()` and `put_error(...)`.
- Preserved the consumer-side tuple contract:
  `(planned_index, planned, exc) = planned_queue.get(timeout=...)`.

Issues / repo facts found:

- This is still a wrapper around the same producer/consumer behavior. It does
  not yet change planning order, blocking semantics, timeout handling or
  execution ordering.
- The backend still catches `queue.Empty` because the wrapper intentionally
  exposes the same timeout behavior.

Verification:

```text
python3 -m py_compile \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/ordered_planned_queue.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/__init__.py

/bin/bash -lc "source /opt/ros/jazzy/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws; PYTHONPATH=eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts:$PYTHONPATH python3 -c \"from motion.scheduling.ordered_planned_queue import OrderedPlannedQueue; q=OrderedPlannedQueue(); q.put_planned(3, {'label':'s4'}); q.put_done(); print(q.get(timeout=0.1)); print(q.get(timeout=0.1))\""

/bin/bash -lc "env -i HOME=\"$HOME\" TERM=\"${TERM:-xterm}\" PATH=\"/usr/bin:/bin:/usr/sbin:/sbin\" bash --noprofile --norc -lc 'source /opt/ros/rolling/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws/eRob_moveit; colcon build --packages-select erob_moveit_runtime'"
```

Next pickup point:

1. Runtime-check ordered-chain execution after the queue wrapper.
2. Start moving planned-queue status/observation into `OrderedPlannedQueue`
   itself, or introduce a combined scheduler bridge object that owns both
   `OrderedChainObservation` and `OrderedPlannedQueue`.
3. Keep the legacy nested unwind function in place for now as fallback/reference
   until the extracted unwind path has more runtime validation.

### 2026-08-10 - Slice 17: combined ordered scheduler bridge

Status: completed and compile/smoke-checked. Live execution behavior is still
unchanged. This slice creates one scheduler bridge object that owns both the
planned queue wrapper and planned-segment observation object.

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/ordered_scheduler_bridge.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/__init__.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py
docs/refactoring/refactoring_plan_cpp_validation.md
```

What was done:

- Added `OrderedSchedulerBridge`.
- The bridge owns:
  - `OrderedPlannedQueue`
  - `OrderedChainObservation`
- `MoveItRobotBackend._execute_ordered_motion_chain_pipelined(...)` now creates
  one `OrderedSchedulerBridge()` and uses `scheduler_bridge.queue`,
  `scheduler_bridge.mark_planned(...)`,
  `scheduler_bridge.mark_consumed(...)` and
  `scheduler_bridge.preplanned_snapshot(...)`.
- The existing queue tuple protocol and execution behavior are preserved.

Issues / repo facts found:

- This is still a bridge around the legacy ordered-chain implementation, not a
  full scheduler switchover.
- `OrderedChainObservation` and `OrderedPlannedQueue` remain exported for now;
  they are useful independently for focused tests and incremental migration.

Verification:

```text
python3 -m py_compile \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/ordered_scheduler_bridge.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/__init__.py

/bin/bash -lc "source /opt/ros/jazzy/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws; PYTHONPATH=eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts:$PYTHONPATH python3 -c \"from motion.scheduling.ordered_scheduler_bridge import OrderedSchedulerBridge; b=OrderedSchedulerBridge(); b.queue.put_planned(1, {'label':'s2'}); print(b.queue.get(timeout=0.1)); print(b.mark_planned(1, {'label':'s2','type':'linear'})); b.mark_consumed(1); print(b.preplanned_snapshot())\""

/bin/bash -lc "env -i HOME=\"$HOME\" TERM=\"${TERM:-xterm}\" PATH=\"/usr/bin:/bin:/usr/sbin:/sbin\" bash --noprofile --norc -lc 'source /opt/ros/rolling/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws/eRob_moveit; colcon build --packages-select erob_moveit_runtime'"
```

Next pickup point:

1. Runtime-check ordered-chain execution after the combined bridge.
2. Start moving producer/consumer wait validation into `OrderedSchedulerBridge`
   so backend code no longer directly validates `(planned_index, planned, exc)`.
3. Keep the legacy nested unwind function in place for now as fallback/reference
   until the extracted unwind path has more runtime validation.

### 2026-08-10 - Slice 18: planned-item validation moved into bridge

Status: completed and compile/smoke-checked. Live execution behavior is still
unchanged. This slice moves consumer-side validation of planned queue items into
`OrderedSchedulerBridge`.

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/ordered_scheduler_bridge.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py
docs/refactoring/refactoring_plan_cpp_validation.md
```

What was done:

- Added `OrderedSchedulerBridge.wait_for_planned(...)`.
- Moved these legacy consumer checks out of the backend:
  - queue timeout -> `TimeoutError`
  - planner worker exception -> `RuntimeError`
  - planner ended early
  - planned index mismatch
- Backend now receives only a validated `planned` segment from the bridge.

Issues / repo facts found:

- Error messages were preserved to keep debugging behavior stable.
- The bridge still uses the same underlying tuple queue protocol.

Verification:

```text
python3 -m py_compile \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/ordered_scheduler_bridge.py

/bin/bash -lc "source /opt/ros/jazzy/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws; PYTHONPATH=eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts:$PYTHONPATH python3 -c \"from motion.scheduling.ordered_scheduler_bridge import OrderedSchedulerBridge; b=OrderedSchedulerBridge(); b.queue.put_planned(0, {'label':'s1'}); print(b.wait_for_planned(0, timeout=0.1)); b.queue.put_planned(2, {'label':'s3'}); ns={'b': b}; exec('err=None\\ntry:\\n    b.wait_for_planned(1, timeout=0.1)\\nexcept RuntimeError as exc:\\n    err=str(exc)', ns); print(ns['err'])\""

/bin/bash -lc "env -i HOME=\"$HOME\" TERM=\"${TERM:-xterm}\" PATH=\"/usr/bin:/bin:/usr/sbin:/sbin\" bash --noprofile --norc -lc 'source /opt/ros/rolling/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws/eRob_moveit; colcon build --packages-select erob_moveit_runtime'"
```

Next pickup point:

1. Runtime-check ordered-chain execution after moving validation into the
   bridge.
2. Start moving producer-side queue publication into bridge methods that also
   mark observation/status in one place.
3. Keep the legacy nested unwind function in place for now as fallback/reference
   until the extracted unwind path has more runtime validation.

### 2026-08-10 - Slice 19: producer-side publication moved into bridge

Status: completed and compile/smoke-checked. Live execution behavior is still
unchanged. This slice moves producer-side planned queue publication into
`OrderedSchedulerBridge`.

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/ordered_scheduler_bridge.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py
docs/refactoring/refactoring_plan_cpp_validation.md
```

What was done:

- Added bridge producer methods:
  - `publish_planned(...)`
  - `publish_done()`
  - `publish_error(...)`
- Backend no longer calls `OrderedPlannedQueue.put_*` directly.
- `publish_planned(...)` now queues the planned segment and updates
  preplanned observation in one scheduler-owned operation.
- The backend still updates scheduler segment/group state before publishing the
  preplanned snapshot, preserving current status timing.

Issues / repo facts found:

- The backend still has a local `_publish_planned(...)` wrapper because it also
  updates `scheduler_segment_states` and REST/WebSocket status. That wrapper can
  move into a richer scheduler bridge after runtime validation.

Verification:

```text
python3 -m py_compile \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/ordered_scheduler_bridge.py

/bin/bash -lc "source /opt/ros/jazzy/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws; PYTHONPATH=eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts:$PYTHONPATH python3 -c \"from motion.scheduling.ordered_scheduler_bridge import OrderedSchedulerBridge; b=OrderedSchedulerBridge(); print(b.publish_planned(1, {'label':'s2','type':'linear'})); print(b.wait_for_planned(1, timeout=0.1)); b.publish_done(); print(b.queue.get(timeout=0.1))\""

/bin/bash -lc "env -i HOME=\"$HOME\" TERM=\"${TERM:-xterm}\" PATH=\"/usr/bin:/bin:/usr/sbin:/sbin\" bash --noprofile --norc -lc 'source /opt/ros/rolling/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws/eRob_moveit; colcon build --packages-select erob_moveit_runtime'"
```

Next pickup point:

1. Runtime-check ordered-chain execution after producer-side publication moved
   into the bridge.
2. Consider moving scheduler segment/group state update callbacks into
   `OrderedSchedulerBridge`, or pause for runtime validation before deeper
   migration.
3. Keep the legacy nested unwind function in place for now as fallback/reference
   until the extracted unwind path has more runtime validation.

### 2026-08-10 - Slice 20: scheduler segment/group state moved into bridge

Status: completed and compile/smoke/build-checked. Live execution behavior is
intended to remain unchanged. This slice moves scheduler segment/group state
mutation into `OrderedSchedulerBridge`; the backend still publishes the returned
status payload through the existing ordered-chain status mechanism.

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/ordered_scheduler_bridge.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py
docs/refactoring/refactoring_plan_cpp_validation.md
```

What was done:

- `OrderedSchedulerBridge` now accepts initial `scheduler_group_status` and
  `scheduler_segment_states`.
- Added `OrderedSchedulerBridge.set_segment_state(...)`.
- Group state derivation now happens inside the bridge using the existing
  status adapter helpers.
- Backend `_set_scheduler_segment_state(...)` is reduced to delegating to the
  bridge and publishing any returned status fields.
- Removed direct backend imports of the group/segment status mutation helpers.

Issues / repo facts found:

- Status publication still belongs to `moveit_robot_backend.py`; the bridge
  returns JSON-safe updates but does not call backend status APIs.
- The local backend `_publish_planned(...)` wrapper still exists because it
  publishes both readiness/status and queue/observation updates in the current
  ordering.
- The legacy nested unwind implementation remains intentionally in place as
  fallback/reference per operator request.

Verification:

```text
python3 -m py_compile \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/ordered_scheduler_bridge.py

PYTHONPATH=eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts:$PYTHONPATH python3 -c "from motion.scheduling.ordered_scheduler_bridge import OrderedSchedulerBridge; b=OrderedSchedulerBridge(scheduler_group_status=({'group_index':0,'start_segment_index':0,'end_segment_index':1,'planner_name':'linked_lin','segment_count':2,'hard_stop_after':True},), scheduler_segment_states={0:{'segment_index':0,'state':'PENDING'},1:{'segment_index':1,'state':'PENDING'}}); print(b.set_segment_state(0, 'READY')); print(b.set_segment_state(1, 'EXECUTING')); print(b.set_segment_state(1, 'DONE', result=0)); print(b.set_segment_state(0, 'DONE', result=0))"

/bin/bash -lc "env -i HOME=\"$HOME\" TERM=\"${TERM:-xterm}\" PATH=\"/usr/bin:/bin:/usr/sbin:/sbin\" bash --noprofile --norc -lc 'source /opt/ros/rolling/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws/eRob_moveit; colcon build --packages-select erob_moveit_runtime'"
```

Build note:

- The build still prints
  `/opt/ros/rolling/setup.bash: No such file or directory` in this environment,
  then completes `erob_moveit_runtime` successfully.

Next pickup point:

1. Runtime-check ordered-chain execution after moving segment/group state into
   the bridge.
2. Consider moving the backend `_publish_planned(...)` wrapper into the bridge
   via a returned combined status update or a narrow status-publisher callback.
3. Do not remove the legacy nested unwind function yet; keep it available as
   fallback/reference until extracted unwind has more runtime validation.

### 2026-08-10 - Slice 21: ready segment publication combined in bridge

Status: completed and compile/smoke/build-checked. Live execution behavior is
intended to remain unchanged except that the READY segment state and preplanned
snapshot are now returned as one bridge update payload.

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/ordered_scheduler_bridge.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py
docs/refactoring/refactoring_plan_cpp_validation.md
```

What was done:

- Added `OrderedSchedulerBridge.publish_ready_segment(...)`.
- The new bridge method marks the logical segment `READY`, updates derived
  group state, queues the planned segment, and updates preplanned observation.
- Backend `_publish_planned(...)` now delegates to
  `scheduler_bridge.publish_ready_segment(...)` and only publishes the returned
  status payload.

Issues / repo facts found:

- Backend `_publish_planned(...)` still exists as a thin status-publication
  wrapper because `OrderedSchedulerBridge` does not call backend status APIs.
- Execution lifecycle state transitions (`EXECUTING`, `DONE`, `FAILED`) already
  delegate state mutation to the bridge, but the calls are still made directly
  from `_execute_planned_segment(...)`.
- The legacy nested unwind implementation remains intentionally in place as
  fallback/reference per operator request.

Verification:

```text
python3 -m py_compile \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/ordered_scheduler_bridge.py

PYTHONPATH=eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts:$PYTHONPATH python3 -c "from motion.scheduling.ordered_scheduler_bridge import OrderedSchedulerBridge; b=OrderedSchedulerBridge(scheduler_group_status=({'group_index':0,'start_segment_index':0,'end_segment_index':0,'planner_name':'segment','segment_count':1,'hard_stop_after':True},), scheduler_segment_states={0:{'segment_index':0,'segment_number':1,'label':'s1','type':'linear','state':'PENDING','result':None}}); print(b.publish_ready_segment(0, {'label':'s1','type':'linear'})); print(b.wait_for_planned(0, timeout=0.1))"

/bin/bash -lc "env -i HOME=\"$HOME\" TERM=\"${TERM:-xterm}\" PATH=\"/usr/bin:/bin:/usr/sbin:/sbin\" bash --noprofile --norc -lc 'source /opt/ros/rolling/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws/eRob_moveit; colcon build --packages-select erob_moveit_runtime'"
```

Build note:

- The build still prints
  `/opt/ros/rolling/setup.bash: No such file or directory` in this environment,
  then completes `erob_moveit_runtime` successfully.

Next pickup point:

1. Runtime-check ordered-chain execution with the combined
   `publish_ready_segment(...)` update.
2. Consider moving execution lifecycle event helpers into the bridge, still
   returning status payloads for backend publication.
3. Do not remove the legacy nested unwind function yet; keep it available as
   fallback/reference until extracted unwind has more runtime validation.

### 2026-08-10 - Slice 22: execution lifecycle scheduler helpers

Status: completed and compile/smoke/build-checked. Live execution behavior is
intended to remain unchanged. This slice gives the bridge named lifecycle
methods for consumer-side scheduler state transitions.

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/ordered_scheduler_bridge.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py
docs/refactoring/refactoring_plan_cpp_validation.md
```

What was done:

- Added `OrderedSchedulerBridge.consume_planned(...)`, which consumes a planned
  observation entry and returns the updated preplanned snapshot.
- Added `OrderedSchedulerBridge.mark_executing(...)`.
- Added `OrderedSchedulerBridge.mark_finished(...)`.
- Removed the backend's local scheduler state mutation wrapper.
- Backend execution now publishes status from bridge-returned lifecycle
  payloads instead of passing raw scheduler state strings around.

Issues / repo facts found:

- Backend still publishes ordered-chain status through
  `_set_ordered_motion_chain_status(...)`; the bridge only produces payloads.
- Backend still owns actual controller execution, stop handling, and
  timing/logging around execution.
- Runtime launch was not run in this slice; only syntax, focused lifecycle smoke,
  and package build were checked.
- The legacy nested unwind implementation remains intentionally in place as
  fallback/reference per operator request.

Verification:

```text
python3 -m py_compile \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/ordered_scheduler_bridge.py

PYTHONPATH=eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts:$PYTHONPATH python3 -c "from motion.scheduling.ordered_scheduler_bridge import OrderedSchedulerBridge; b=OrderedSchedulerBridge(scheduler_group_status=({'group_index':0,'start_segment_index':0,'end_segment_index':0,'planner_name':'segment','segment_count':1,'hard_stop_after':True},), scheduler_segment_states={0:{'segment_index':0,'segment_number':1,'label':'s1','type':'linear','state':'PENDING','result':None}}); print(b.publish_ready_segment(0, {'label':'s1','type':'linear'})); print(b.consume_planned(0, current_index=1)); print(b.mark_executing(0, {'label':'s1','type':'linear'})); print(b.mark_finished(0, {'label':'s1','type':'linear'}, 0))"

/bin/bash -lc "env -i HOME=\"$HOME\" TERM=\"${TERM:-xterm}\" PATH=\"/usr/bin:/bin:/usr/sbin:/sbin\" bash --noprofile --norc -lc 'source /opt/ros/rolling/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws/eRob_moveit; colcon build --packages-select erob_moveit_runtime'"
```

Build note:

- The build still prints
  `/opt/ros/rolling/setup.bash: No such file or directory` in this environment,
  then completes `erob_moveit_runtime` successfully.

Next pickup point:

1. Runtime-check ordered-chain execution after moving lifecycle helpers into
   the bridge.
2. Consider extracting the backend execution status payload construction
   (`phase=executing`, `phase=segment_completed/failed`) into a status adapter
   helper while keeping controller execution in the backend.
3. Do not remove the legacy nested unwind function yet; keep it available as
   fallback/reference until extracted unwind has more runtime validation.

### 2026-08-10 - Fake-system runtime validation after Slice 22

Status: operator-tested on fake hardware using log capture at:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/zeroerr/docs/temp_logs
```

Result: passed for the covered fake-system workflow.

Runtime evidence:

- `RobotController` reached ready state:
  `[Init] RobotController ready (4.34s total)`.
- Fake hardware correctly skipped drive auto-enable:
  `[DriveEnable] Startup auto-enable skipped in fake hardware mode`.
- Contour IK helper and PTP helper both reported ready with local
  `PlanningScene`.
- A single PTP move completed successfully:
  `backend_move_ptp submitted blocking=True result=0`.
- Ordered-chain request 1 completed:
  `segments=4`, `ordered_chain_done`, `result=0`,
  `total_elapsed_s=4.829`.
- Ordered-chain request 2 completed:
  `segments=15`, `ordered_chain_done`, `result=0`,
  `total_elapsed_s=25.651`.
- Scheduler bridge lifecycle telemetry did not block execution: blended groups,
  `blend_consumed` logical segments, PATH segment, and final
  `unwind_joint6` segment all reached `result=0`.
- Final unwind path stayed safe:
  `[UNWIND_J6] Rotational-path unwind skipped - no unwind needed`.

Issues / repo facts found:

- MoveIt still emits octomap monitor errors:
  `No 3D sensor plugin(s) defined for octomap updates`. This appears
  environmental/configuration-related and did not block fake-system execution.
- Helpers and `move_group` still emit repeated `Publisher already registered`
  warnings for duplicate node names. This did not block this run, but should be
  kept in mind if logs become noisy or rosout behavior matters.
- Ruckig helper still reports implausible output for two early trajectories and
  rejects those outputs; the workflow continued successfully through the
  existing optimizer fallback/path.
- The log contains `TOTG_PATH_DIAG reversal_candidate` warnings where the
  vectors are effectively collinear (`cos=1.000000000`); these look diagnostic,
  not runtime failures.

Next pickup point:

1. Treat Slice 22 as fake-system runtime validated for the logged workflow.
2. Continue with a narrow extraction of ordered execution status payload
   construction (`executing`, `segment_completed`, `segment_failed`) into
   `motion/scheduling/status_adapter.py`.
3. Keep controller execution, stop handling, and trajectory sends in the backend
   until a clearer executor boundary is introduced.
4. Do not remove the legacy nested unwind function yet; keep it available as
   fallback/reference until extracted unwind has more runtime validation.

### 2026-08-10 - Slice 23: ordered execution status payload adapter

Status: completed and compile/smoke/build-checked. Live execution behavior is
intended to remain unchanged. This slice extracts two inline ordered-chain
execution status dictionaries from the backend into `status_adapter.py`.

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/status_adapter.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py
docs/refactoring/refactoring_plan_cpp_validation.md
```

What was done:

- Added `ordered_chain_executing_status(...)`.
- Added `ordered_chain_segment_finished_status(...)`.
- Backend `_execute_planned_segment(...)` now delegates construction of
  `phase=executing`, `phase=segment_completed`, and `phase=segment_failed`
  payloads to the status adapter.
- Controller execution, state matching, stop handling, timing, and logging
  remain in the backend.

Issues / repo facts found:

- The status adapter is still a compatibility layer for legacy ordered-chain
  fields, not the final scheduler state model.
- No live launch was run after this slice; it builds on the fake-system runtime
  validation captured immediately before it.
- The legacy nested unwind implementation remains intentionally in place as
  fallback/reference per operator request.

Verification:

```text
python3 -m py_compile \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/status_adapter.py

PYTHONPATH=eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts:$PYTHONPATH python3 -c "from motion.scheduling.status_adapter import ordered_chain_executing_status, ordered_chain_segment_finished_status; planned={'label':'s1','type':'linear','protected':True}; print(ordered_chain_executing_status(index=2,total=4,planned_segment=planned,preplanned_ready_count=1)); print(ordered_chain_segment_finished_status(index=2,planned_segment=planned,result=0)); print(ordered_chain_segment_finished_status(index=2,planned_segment=planned,result=-5))"

/bin/bash -lc "env -i HOME=\"$HOME\" TERM=\"${TERM:-xterm}\" PATH=\"/usr/bin:/bin:/usr/sbin:/sbin\" bash --noprofile --norc -lc 'source /opt/ros/rolling/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws/eRob_moveit; colcon build --packages-select erob_moveit_runtime'"
```

Build note:

- The build still prints
  `/opt/ros/rolling/setup.bash: No such file or directory` in this environment,
  then completes `erob_moveit_runtime` successfully.

Next pickup point:

1. Runtime-check ordered-chain execution after Slice 23 if possible, especially
   GUI status fields during `executing` and `segment_completed/failed`.
2. Consider extracting the stop status payload (`phase=stopped`) into
   `status_adapter.py`.
3. Keep controller execution, stop handling, and trajectory sends in the backend
   until a clearer executor boundary is introduced.
4. Do not remove the legacy nested unwind function yet; keep it available as
   fallback/reference until extracted unwind has more runtime validation.

### 2026-08-10 - Slice 24: ordered stop status payload adapter

Status: completed and compile/smoke/build-checked. Live execution behavior is
intended to remain unchanged. This slice extracts the inline ordered-chain
`phase=stopped` compatibility payload from the backend into `status_adapter.py`.

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/status_adapter.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py
docs/refactoring/refactoring_plan_cpp_validation.md
```

What was done:

- Added `ordered_chain_stopped_status(...)`.
- Backend stop-before-execution handling now delegates status payload
  construction to the status adapter.
- Stop detection, `stop_planning.set()`, return code `-14`, controller
  execution, and trajectory handling remain in the backend.

Issues / repo facts found:

- `active=False` still goes through `normalize_ordered_chain_status(...)`, which
  applies compatibility resets for current/preplanned fields after the stop
  payload is merged. This matches existing behavior.
- No live launch was run after this slice; only syntax, focused payload smoke,
  and package build were checked.
- The legacy nested unwind implementation remains intentionally in place as
  fallback/reference per operator request.

Verification:

```text
python3 -m py_compile \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/status_adapter.py

PYTHONPATH=eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts:$PYTHONPATH python3 -c "from motion.scheduling.status_adapter import ordered_chain_stopped_status; print(ordered_chain_stopped_status(index=3, planned_segment={'label':'s3','type':'path','protected':True}, result=-14))"

/bin/bash -lc "env -i HOME=\"$HOME\" TERM=\"${TERM:-xterm}\" PATH=\"/usr/bin:/bin:/usr/sbin:/sbin\" bash --noprofile --norc -lc 'source /opt/ros/rolling/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws/eRob_moveit; colcon build --packages-select erob_moveit_runtime'"
```

Build note:

- The build still prints
  `/opt/ros/rolling/setup.bash: No such file or directory` in this environment,
  then completes `erob_moveit_runtime` successfully.

Next pickup point:

1. Runtime-check ordered-chain execution after Slices 23-24 if possible,
   especially GUI status fields during executing, completed, failed, and
   stopped states.
2. Consider extracting final chain status payloads (`active=False`,
   `phase=done/failed`) into `status_adapter.py`.
3. Keep controller execution, stop handling, and trajectory sends in the backend
   until a clearer executor boundary is introduced.
4. Do not remove the legacy nested unwind function yet; keep it available as
   fallback/reference until extracted unwind has more runtime validation.

### 2026-08-10 - Slice 25: ordered terminal status payload adapter

Status: completed and compile/smoke/build-checked. Live execution behavior is
intended to remain unchanged. This slice extracts final ordered-chain terminal
payload construction into `status_adapter.py`.

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/status_adapter.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py
docs/refactoring/refactoring_plan_cpp_validation.md
```

What was done:

- Added `ordered_chain_terminal_status(...)`.
- Backend terminal phases now delegate payload construction through the adapter:
  - `rejected`
  - `failed`
  - `completed`
  - `error`
- Kept the exact existing phase names and result codes.
- Kept controller execution, stop handling, timing, and trajectory sends in the
  backend.

Issues / repo facts found:

- During verification, an initial import placement mistake caused a Python
  `SyntaxError`; it was fixed before build. `ordered_chain_terminal_status` is
  now imported outside the adapter-validation `try` because it is used after
  that block even if batch adapter validation falls back.
- `active=False` still goes through `normalize_ordered_chain_status(...)`, which
  applies compatibility resets for current/preplanned fields after the terminal
  payload is merged. This preserves existing behavior.
- No live launch was run after this slice; only syntax, focused payload smoke,
  and package build were checked.
- The legacy nested unwind implementation remains intentionally in place as
  fallback/reference per operator request.

Verification:

```text
python3 -m py_compile \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/status_adapter.py

PYTHONPATH=eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts:$PYTHONPATH python3 -c "from motion.scheduling.status_adapter import ordered_chain_terminal_status; print(ordered_chain_terminal_status(phase='rejected', result=-10)); print(ordered_chain_terminal_status(phase='completed', result=0)); print(ordered_chain_terminal_status(phase='error', result=-1, error='boom'))"

/bin/bash -lc "env -i HOME=\"$HOME\" TERM=\"${TERM:-xterm}\" PATH=\"/usr/bin:/bin:/usr/sbin:/sbin\" bash --noprofile --norc -lc 'source /opt/ros/rolling/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws/eRob_moveit; colcon build --packages-select erob_moveit_runtime'"
```

Build note:

- The build still prints
  `/opt/ros/rolling/setup.bash: No such file or directory` in this environment,
  then completes `erob_moveit_runtime` successfully.

Next pickup point:

1. Runtime-check ordered-chain execution after Slices 23-25 if possible,
   especially GUI status fields during rejected, executing, completed, failed,
   stopped, and error states.
2. Consider extracting initial ordered-chain status payload construction from
   `execute_ordered_motion_chain(...)` into `status_adapter.py`, but keep it
   narrow because it includes scheduler group and segment metadata.
3. Keep controller execution, stop handling, and trajectory sends in the backend
   until a clearer executor boundary is introduced.
4. Do not remove the legacy nested unwind function yet; keep it available as
   fallback/reference until extracted unwind has more runtime validation.

### 2026-08-10 - Slice 26: ordered starting status payload adapter

Status: completed and compile/smoke/build-checked. Live execution behavior is
intended to remain unchanged. This slice extracts the initial ordered-chain
`phase=starting` compatibility payload from `execute_ordered_motion_chain(...)`
into `status_adapter.py`.

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/status_adapter.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py
docs/refactoring/refactoring_plan_cpp_validation.md
```

What was done:

- Added `ordered_chain_starting_status(...)`.
- Backend starting-state publication now delegates payload construction through
  the status adapter.
- Preserved existing scheduler group metadata, scheduler segment metadata,
  current/preplanned field resets, and `result=None`.
- Kept adapter validation, drive/hardware rejection checks, controller
  execution, stop handling, timing, and trajectory sends in the backend.

Issues / repo facts found:

- The helper accepts already-derived scheduler group/segment state maps. It does
  not create or validate scheduler grouping; that remains in the adapter and
  scheduler setup path.
- No live launch was run after this slice; only syntax, focused payload smoke,
  and package build were checked.
- The legacy nested unwind implementation remains intentionally in place as
  fallback/reference per operator request.

Verification:

```text
python3 -m py_compile \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/status_adapter.py

PYTHONPATH=eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts:$PYTHONPATH python3 -c "from motion.scheduling.status_adapter import ordered_chain_starting_status; print(ordered_chain_starting_status(total_segments=2, scheduler_group_status=({'group_index':0},), scheduler_group_states={0:{'group_index':0,'state':'PENDING'}}, scheduler_segment_states={0:{'segment_index':0,'state':'PENDING'},1:{'segment_index':1,'state':'PENDING'}}))"

/bin/bash -lc "env -i HOME=\"$HOME\" TERM=\"${TERM:-xterm}\" PATH=\"/usr/bin:/bin:/usr/sbin:/sbin\" bash --noprofile --norc -lc 'source /opt/ros/rolling/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws/eRob_moveit; colcon build --packages-select erob_moveit_runtime'"
```

Build note:

- The build still prints
  `/opt/ros/rolling/setup.bash: No such file or directory` in this environment,
  then completes `erob_moveit_runtime` successfully.

Next pickup point:

1. Runtime-check ordered-chain execution after Slices 23-26 if possible,
   especially GUI status fields during starting, rejected, executing,
   completed, failed, stopped, and error states.
2. Consider extracting the MotionBatch adapter validation/status setup around
   `ordered_motion_batch_from_mappings(...)` into a scheduler adapter helper.
3. Keep controller execution, stop handling, and trajectory sends in the backend
   until a clearer executor boundary is introduced.
4. Do not remove the legacy nested unwind function yet; keep it available as
   fallback/reference until extracted unwind has more runtime validation.

### 2026-08-10 - Slice 27: ordered MotionBatch validation helper

Status: completed and compile/smoke/build-checked. Live execution behavior is
intended to remain unchanged. This slice extracts MotionBatch adapter
validation/status setup out of `moveit_robot_backend.py` and into
`motion/scheduling/motion_adapters.py`.

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/motion_adapters.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py
docs/refactoring/refactoring_plan_cpp_validation.md
```

What was done:

- Added frozen dataclass `OrderedMotionBatchValidation`.
- Added `validate_ordered_motion_batch_from_mappings(...)`.
- The helper returns:
  - typed `MotionBatch`
  - scheduler group status
  - scheduler group state map
  - scheduler segment state map
  - segment-count summary string
  - scheduler group summary string
- Backend `execute_ordered_motion_chain(...)` now calls this helper and keeps
  the same logging/timing message shape.
- Adapter-validation fallback behavior remains in the backend: if validation
  raises, it logs the warning and continues with the legacy ordered-chain
  executor path.

Issues / repo facts found:

- The helper still uses the existing grouping heuristic from
  `group_motion_batch(...)`; it does not change grouping behavior.
- The backend still owns log/timing publication around adapter validation.
- No live launch was run after this slice; only syntax, focused helper smoke,
  and package build were checked.
- The legacy nested unwind implementation remains intentionally in place as
  fallback/reference per operator request.

Verification:

```text
python3 -m py_compile \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/motion_adapters.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/status_adapter.py

PYTHONPATH=eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts:$PYTHONPATH python3 -c "from motion.scheduling.motion_adapters import validate_ordered_motion_batch_from_mappings; v=validate_ordered_motion_batch_from_mappings([{'type':'linear','label':'l1','position':[0,0,0,0,0,0],'blendR':0},{'type':'ptp','label':'p1','position':[1,2,3,4,5,6],'blendR':1},{'type':'ptp','label':'p2','position':[2,3,4,5,6,7],'blendR':0}], blocking=True, tool=1, user=0); print(len(v.motion_batch.segments), v.segment_counts_text, v.group_summary); print(v.scheduler_group_status); print(v.scheduler_segment_states)"

/bin/bash -lc "env -i HOME=\"$HOME\" TERM=\"${TERM:-xterm}\" PATH=\"/usr/bin:/bin:/usr/sbin:/sbin\" bash --noprofile --norc -lc 'source /opt/ros/rolling/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws/eRob_moveit; colcon build --packages-select erob_moveit_runtime'"
```

Build note:

- The build still prints
  `/opt/ros/rolling/setup.bash: No such file or directory` in this environment,
  then completes `erob_moveit_runtime` successfully.

Next pickup point:

1. Runtime-check ordered-chain execution after Slices 23-27 if possible,
   especially GUI status fields and adapter-validation timing fields.
2. Consider extracting adapter-validation logging/timing into a backend-local
   wrapper or scheduler bridge method only if it reduces backend complexity
   without hiding runtime observability.
3. Keep controller execution, stop handling, and trajectory sends in the backend
   until a clearer executor boundary is introduced.
4. Do not remove the legacy nested unwind function yet; keep it available as
   fallback/reference until extracted unwind has more runtime validation.

### 2026-08-10 - Slice 28: ordered MotionBatch validation presentation helpers

Status: completed and compile/smoke/build-checked. Live execution behavior is
intended to remain unchanged. This slice keeps adapter-validation logging and
timing publication in the backend, but moves the message/field formatting onto
the validation result object.

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/motion_adapters.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py
docs/refactoring/refactoring_plan_cpp_validation.md
```

What was done:

- Added `OrderedMotionBatchValidation.log_message()`.
- Added `OrderedMotionBatchValidation.timing_fields()`.
- Backend now calls those methods for the existing
  `ordered_motion_batch_adapter_validated` log/timing path.
- Removed the now-unused local `motion_batch` assignment from the backend
  validation block.

Issues / repo facts found:

- Observability remains backend-owned: the helper formats values, but the
  backend still emits the actual log and timing event.
- No live launch was run after this slice; only syntax, focused helper smoke,
  and package build were checked.
- The legacy nested unwind implementation remains intentionally in place as
  fallback/reference per operator request.

Verification:

```text
python3 -m py_compile \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/motion_adapters.py

PYTHONPATH=eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts:$PYTHONPATH python3 -c "from motion.scheduling.motion_adapters import validate_ordered_motion_batch_from_mappings; v=validate_ordered_motion_batch_from_mappings([{'type':'linear','label':'l1','position':[0,0,0,0,0,0],'blendR':0},{'type':'ptp','label':'p1','position':[1,2,3,4,5,6],'blendR':0}], blocking=False, tool=1, user=0); print(v.log_message()); print(v.timing_fields())"

/bin/bash -lc "env -i HOME=\"$HOME\" TERM=\"${TERM:-xterm}\" PATH=\"/usr/bin:/bin:/usr/sbin:/sbin\" bash --noprofile --norc -lc 'source /opt/ros/rolling/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws/eRob_moveit; colcon build --packages-select erob_moveit_runtime'"
```

Build note:

- The build still prints
  `/opt/ros/rolling/setup.bash: No such file or directory` in this environment,
  then completes `erob_moveit_runtime` successfully.

Next pickup point:

1. Runtime-check ordered-chain execution after Slices 23-28 if possible,
   especially GUI status fields and adapter-validation timing fields.
2. Consider extracting the adapter-validation fallback record into a small
   helper only if it stays explicit enough for debugging.
3. Keep controller execution, stop handling, and trajectory sends in the backend
   until a clearer executor boundary is introduced.
4. Do not remove the legacy nested unwind function yet; keep it available as
   fallback/reference until extracted unwind has more runtime validation.

### 2026-08-10 - Slice 29: ordered MotionBatch validation failure record

Status: completed and compile/smoke/build-checked. Live execution behavior is
intended to remain unchanged. This slice mirrors the success validation
presentation helper with an explicit failure record.

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/motion_adapters.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py
docs/refactoring/refactoring_plan_cpp_validation.md
```

What was done:

- Added frozen dataclass `OrderedMotionBatchValidationFailure`.
- Added `OrderedMotionBatchValidationFailure.log_message()`.
- Added `OrderedMotionBatchValidationFailure.timing_fields()`.
- Backend adapter-validation exception path now uses the failure record for the
  existing warning and `ordered_motion_batch_adapter_failed` timing event.
- Fallback behavior remains explicit and unchanged: adapter validation failure
  still clears scheduler group state and continues with the legacy ordered-chain
  executor path.

Issues / repo facts found:

- Backend still owns log/timing publication. The new failure record only formats
  the warning message and timing fields.
- No live launch was run after this slice; only syntax, focused helper smoke,
  and package build were checked.
- The legacy nested unwind implementation remains intentionally in place as
  fallback/reference per operator request.

Verification:

```text
python3 -m py_compile \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/motion_adapters.py

PYTHONPATH=eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts:$PYTHONPATH python3 -c "from motion.scheduling.motion_adapters import OrderedMotionBatchValidationFailure, validate_ordered_motion_batch_from_mappings; v=validate_ordered_motion_batch_from_mappings([{'type':'linear','label':'l1','position':[0,0,0,0,0,0],'blendR':0}], blocking=True); f=OrderedMotionBatchValidationFailure('bad segment'); print(v.log_message()); print(v.timing_fields()); print(f.log_message()); print(f.timing_fields())"

/bin/bash -lc "env -i HOME=\"$HOME\" TERM=\"${TERM:-xterm}\" PATH=\"/usr/bin:/bin:/usr/sbin:/sbin\" bash --noprofile --norc -lc 'source /opt/ros/rolling/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws/eRob_moveit; colcon build --packages-select erob_moveit_runtime'"
```

Build note:

- The build still prints
  `/opt/ros/rolling/setup.bash: No such file or directory` in this environment,
  then completes `erob_moveit_runtime` successfully.

Next pickup point:

1. Runtime-check ordered-chain execution after Slices 23-29 if possible,
   especially GUI status fields and adapter-validation timing fields.
2. Consider whether `execute_ordered_motion_chain(...)` is now thin enough on
   the request/status side, then move attention to execution/controller
   boundary extraction.
3. Keep controller execution, stop handling, and trajectory sends in the backend
   until a clearer executor boundary is introduced.
4. Do not remove the legacy nested unwind function yet; keep it available as
   fallback/reference until extracted unwind has more runtime validation.

### 2026-08-10 - Slice 30: ordered trajectory timeout helper

Status: completed and compile/smoke/build-checked. Live execution behavior is
intended to remain unchanged. This is the first narrow execution-boundary slice:
it extracts only the repeated ordered-chain trajectory timeout calculation.

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/execution/ordered_execution.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py
docs/refactoring/refactoring_plan_cpp_validation.md
```

What was done:

- Added dataclass `OrderedTrajectoryTiming`.
- Added `ordered_trajectory_timing(...)`.
- Backend normal trajectory execution and planned unwind trajectory execution
  now use the helper for:
  - trajectory duration
  - controller goal tolerance
  - wait timeout
- Controller sends, state matching, wait calls, unwind checks, stop handling,
  and timing/log publication remain in the backend.

Issues / repo facts found:

- This helper assumes the legacy precondition that the trajectory has at least
  one point. That matches existing backend behavior because the old code also
  indexed `points[-1]` directly.
- No live launch was run after this slice; only syntax, focused helper smoke,
  and package build were checked.
- The legacy nested unwind implementation remains intentionally in place as
  fallback/reference per operator request.

Verification:

```text
python3 -m py_compile \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/execution/ordered_execution.py

PYTHONPATH=eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts:$PYTHONPATH python3 -c "from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint; from builtin_interfaces.msg import Duration; from motion.execution.ordered_execution import ordered_trajectory_timing; traj=JointTrajectory(); p=JointTrajectoryPoint(); p.time_from_start=Duration(sec=3,nanosec=250000000); traj.points=[p]; print(ordered_trajectory_timing(traj, min_timeout_s=5.0, timeout_multiplier=2.0))"

/bin/bash -lc "env -i HOME=\"$HOME\" TERM=\"${TERM:-xterm}\" PATH=\"/usr/bin:/bin:/usr/sbin:/sbin\" bash --noprofile --norc -lc 'source /opt/ros/rolling/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws/eRob_moveit; colcon build --packages-select erob_moveit_runtime'"
```

Build note:

- The build still prints
  `/opt/ros/rolling/setup.bash: No such file or directory` in this environment,
  then completes `erob_moveit_runtime` successfully.

Next pickup point:

1. Runtime-check ordered-chain execution after Slice 30 if possible, especially
   controller timeout log values for normal trajectories and planned unwind
   trajectories.
2. Continue execution-boundary extraction carefully: the next safe slice could
   extract the ordered trajectory send/wait block into a helper that still
   receives `_wait_ordered_trajectory_point_match`, `_send_trajectory_to_controller`,
   and `_wait_execution_complete` as injected callables.
3. Keep controller execution, stop handling, and trajectory sends in the backend
   until a clearer executor boundary is introduced.
4. Do not remove the legacy nested unwind function yet; keep it available as
   fallback/reference until extracted unwind has more runtime validation.

### 2026-08-10 - Slice 31: ordered timed trajectory execution helper

Status: completed and compile/smoke/build-checked. Live execution behavior is
intended to remain unchanged. This slice extracts the normal ordered-chain
timed trajectory send/wait sequence into an execution helper while keeping ROS
side effects injected by the backend.

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/execution/ordered_execution.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py
docs/refactoring/refactoring_plan_cpp_validation.md
```

What was done:

- Added `execute_ordered_timed_trajectory(...)`.
- Backend normal ordered LIN/PTP/PATH/blended trajectory execution now delegates
  this sequence to the helper:
  - start-state match
  - controller handoff timing event
  - trajectory send
  - wait-execution timing events
  - end-state match after successful controller execution
- The helper receives backend callables for state matching, trajectory send,
  execution wait, timing publication, and logger use.
- No-op segments and planned unwind trajectories remain backend-local in this
  slice.

Issues / repo facts found:

- The helper still assumes legacy planned segment keys exist:
  `label`, `trajectory`, and `plan_elapsed_s`.
- A first smoke-test command failed due to invalid one-line Python class syntax;
  the command was corrected and the helper smoke test passed. The compile check
  had already passed before that.
- No live launch was run after this slice; only syntax, focused helper smoke,
  and package build were checked.
- The legacy nested unwind implementation remains intentionally in place as
  fallback/reference per operator request.

Verification:

```text
python3 -m py_compile \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/execution/ordered_execution.py

PYTHONPATH=eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts:$PYTHONPATH python3 -c "from types import SimpleNamespace; from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint; from builtin_interfaces.msg import Duration; from motion.execution.ordered_execution import ordered_trajectory_timing, execute_ordered_timed_trajectory; traj=JointTrajectory(); p=JointTrajectoryPoint(); p.time_from_start=Duration(sec=1,nanosec=0); traj.points=[p]; planned={'label':'s1','type':'linear','trajectory':traj,'plan_elapsed_s':0.2}; timing=ordered_trajectory_timing(traj,min_timeout_s=5,timeout_multiplier=2); events=[]; logger=SimpleNamespace(info=lambda msg: events.append(('log',msg))); mark=lambda *a, **k: events.append(('mark',a,k)); send=lambda n,t: events.append(('send',len(t.points))); wait=lambda n,t: 0; match=lambda label,t,point,phase: True; print(execute_ordered_timed_trajectory(node=None,index=1,total=1,planned_segment=planned,segment_type='linear',timing=timing,execution_started_s=0.0,motion_error_result=-6,logger=logger,mark_motion_timing=mark,wait_point_match=match,send_trajectory=send,wait_execution_complete=wait)); print([event[0] for event in events]); print(execute_ordered_timed_trajectory(node=None,index=1,total=1,planned_segment=planned,segment_type='linear',timing=timing,execution_started_s=0.0,motion_error_result=-6,logger=logger,mark_motion_timing=mark,wait_point_match=lambda *a: False,send_trajectory=send,wait_execution_complete=wait))"

/bin/bash -lc "env -i HOME=\"$HOME\" TERM=\"${TERM:-xterm}\" PATH=\"/usr/bin:/bin:/usr/sbin:/sbin\" bash --noprofile --norc -lc 'source /opt/ros/rolling/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws/eRob_moveit; colcon build --packages-select erob_moveit_runtime'"
```

Build note:

- The build still prints
  `/opt/ros/rolling/setup.bash: No such file or directory` in this environment,
  then completes `erob_moveit_runtime` successfully.

Next pickup point:

1. Runtime-check ordered-chain execution after Slice 31 if possible, especially
   normal LIN/PTP/PATH/blended controller handoff and wait timing logs.
2. Consider extracting planned unwind trajectory send/wait into a similarly
   injected helper, but keep runtime unwind and final explicit unwind
   verification in the backend until the boundary is clearer.
3. Keep stop handling and high-level trajectory sends in the backend until a
   clearer executor boundary is introduced.
4. Do not remove the legacy nested unwind function yet; keep it available as
   fallback/reference until extracted unwind has more runtime validation.

### 2026-08-10 - Fake-system runtime validation after Slice 31

Status: operator-tested on fake hardware using log capture at:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/zeroerr/docs/temp_logs
```

Result: passed for the covered fake-system workflow.

Runtime evidence:

- Ordered-chain request 1 completed:
  `ordered_chain_done`, `request_id=1`, `result=0`,
  `total_elapsed_s=4.914`.
- Ordered-chain request 2 completed:
  `ordered_chain_done`, `request_id=2`, `result=0`,
  `total_elapsed_s=25.706`.
- Slice 31 normal timed-trajectory helper path is covered by runtime logs:
  - segment 1/4 linear handoff/wait completed with `result=0`
  - segment 2/4 linear handoff/wait completed with `result=0`
  - segment 3/4 blended handoff/wait completed with `result=0`
  - request 2 path segment `paint_contact_1:Workpiece` handoff/wait completed
    with `result=0`
  - request 2 blended dropoff segment handoff/wait completed with `result=0`
- Controller timeout log values are still present and plausible, for example:
  - linear segment: `duration_s=1.239`,
    `controller_goal_tolerance_s=8.000`, `wait_timeout_s=11.239`
  - path segment: `duration_s=13.972`,
    `controller_goal_tolerance_s=27.944`, `wait_timeout_s=43.916`
- Blend-consumed logical segments and final `unwind_joint6` completed with
  `result=0`.

Issues / repo facts found:

- Ruckig still reports implausible output on an early trajectory and falls back
  to seeded TOTG. The workflow continued successfully.
- `TOTG_PATH_DIAG reversal_candidate` warnings remain noisy. Some later path
  diagnostics include large joint-6 reversal angles, but the run still
  completed successfully.
- No Traceback or ordered-chain failure was observed in the checked log.

Next pickup point:

1. Treat Slice 31 as fake-system runtime validated for the logged workflow.
2. Consider extracting planned unwind trajectory send/wait into a similarly
   injected helper, but keep runtime unwind and final explicit unwind
   verification in the backend until the boundary is clearer.
3. Keep stop handling and high-level trajectory sends in the backend until a
   clearer executor boundary is introduced.
4. Do not remove the legacy nested unwind function yet; keep it available as
   fallback/reference until extracted unwind has more runtime validation.

### 2026-08-10 - Slice 32: ordered planned-unwind trajectory execution helper

Status: completed and compile/smoke/build-checked. Live execution behavior is
intended to remain unchanged. This slice extracts only the planned unwind
trajectory send/wait loop into `motion/execution/ordered_execution.py`.

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/execution/ordered_execution.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py
docs/refactoring/refactoring_plan_cpp_validation.md
```

What was done:

- Added `execute_ordered_unwind_trajectories(...)`.
- Backend `unwind_joint6` execution now delegates planned trajectory send/wait
  to the helper.
- The helper receives injected backend callables for trajectory send, execution
  wait, and logging.
- Runtime unwind selection, final explicit-unwind verification, unwind failure
  timestamps, stop handling, and high-level segment status remain in the
  backend.

Issues / repo facts found:

- The helper assumes legacy planned unwind keys exist: `trajectories`,
  optional `trajectory_checks`, and optional `check`.
- A first smoke-test command failed due to invalid one-line Python `def` syntax;
  the command was corrected and the helper smoke test passed. The compile check
  had already passed before that.
- No live launch was run after this slice; only syntax, focused helper smoke,
  and package build were checked.
- The legacy nested unwind implementation remains intentionally in place as
  fallback/reference per operator request.

Verification:

```text
python3 -m py_compile \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/execution/ordered_execution.py

PYTHONPATH=eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts:$PYTHONPATH python3 -c "from types import SimpleNamespace; from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint; from builtin_interfaces.msg import Duration; from motion.execution.ordered_execution import execute_ordered_unwind_trajectories; t1=JointTrajectory(); p1=JointTrajectoryPoint(); p1.time_from_start=Duration(sec=1,nanosec=0); t1.points=[p1]; t2=JointTrajectory(); p2=JointTrajectoryPoint(); p2.time_from_start=Duration(sec=2,nanosec=0); t2.points=[p2]; events=[]; logger=SimpleNamespace(info=lambda msg: events.append(('log',msg))); send=lambda *a, **k: events.append(('send',k.get('preserve_explicit_wrap'),k.get('unwind_check'))); waits=[0,-9]; wait=lambda n,t: waits.pop(0); planned={'trajectories':[t1,t2], 'trajectory_checks':['c1','c2'], 'check':'fallback'}; print(execute_ordered_unwind_trajectories(node=None, planned_segment=planned, min_timeout_s=5, timeout_multiplier=2, logger=logger, send_trajectory=send, wait_execution_complete=wait)); print(events)"

/bin/bash -lc "env -i HOME=\"$HOME\" TERM=\"${TERM:-xterm}\" PATH=\"/usr/bin:/bin:/usr/sbin:/sbin\" bash --noprofile --norc -lc 'source /opt/ros/rolling/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws/eRob_moveit; colcon build --packages-select erob_moveit_runtime'"
```

Build note:

- The build still prints
  `/opt/ros/rolling/setup.bash: No such file or directory` in this environment,
  then completes `erob_moveit_runtime` successfully.

Next pickup point:

1. Runtime-check ordered-chain execution after Slice 32 if possible, especially
   planned unwind segments that actually send one or more unwind trajectories.
2. Keep runtime unwind and final explicit-unwind verification backend-local
   until the executor boundary is clearer.
3. Keep stop handling and high-level trajectory sends in the backend until a
   clearer executor boundary is introduced.
4. Do not remove the legacy nested unwind function yet; keep it available as
   fallback/reference until extracted unwind has more runtime validation.

### 2026-08-10 - Slice 33: ordered unwind finalize helper

Status: completed and compile/smoke/build-checked. Live execution behavior is
intended to remain unchanged. This slice extracts final explicit-unwind
verification and failure bookkeeping into `motion/execution/ordered_execution.py`.

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/execution/ordered_execution.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py
docs/refactoring/refactoring_plan_cpp_validation.md
```

What was done:

- Added `finalize_ordered_unwind_result(...)`.
- Backend `unwind_joint6` execution now delegates:
  - final explicit unwind verification after successful planned execution
  - conversion of failed verification to result `-6`
  - `_last_ordered_unwind_failure_time`
  - `_last_ordered_unwind_failure_result`
- Runtime unwind selection and call to
  `_unwind_joint6_with_rotational_path(...)` remain in the backend.

Issues / repo facts found:

- The helper receives `now_s` and `verify_explicit_unwind_complete` as injected
  values to keep the helper testable and avoid owning ROS/node internals beyond
  the legacy failure attributes.
- No live launch was run after this slice; only syntax, focused helper smoke,
  and package build were checked.
- The legacy nested unwind implementation remains intentionally in place as
  fallback/reference per operator request.

Verification:

```text
python3 -m py_compile \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/execution/ordered_execution.py

PYTHONPATH=eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts:$PYTHONPATH python3 -c "from types import SimpleNamespace; from motion.execution.ordered_execution import finalize_ordered_unwind_result; n=SimpleNamespace(); print(finalize_ordered_unwind_result(node=n, planned_segment={'check':'ok'}, result=0, now_s=12.5, verify_explicit_unwind_complete=lambda c: True)); print(hasattr(n,'_last_ordered_unwind_failure_result')); n=SimpleNamespace(); print(finalize_ordered_unwind_result(node=n, planned_segment={'check':'bad'}, result=0, now_s=13.5, verify_explicit_unwind_complete=lambda c: False)); print(n._last_ordered_unwind_failure_time, n._last_ordered_unwind_failure_result); n=SimpleNamespace(); print(finalize_ordered_unwind_result(node=n, planned_segment={}, result=-9, now_s=14.5, verify_explicit_unwind_complete=lambda c: True)); print(n._last_ordered_unwind_failure_time, n._last_ordered_unwind_failure_result)"

/bin/bash -lc "env -i HOME=\"$HOME\" TERM=\"${TERM:-xterm}\" PATH=\"/usr/bin:/bin:/usr/sbin:/sbin\" bash --noprofile --norc -lc 'source /opt/ros/rolling/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws/eRob_moveit; colcon build --packages-select erob_moveit_runtime'"
```

Build note:

- The build still prints
  `/opt/ros/rolling/setup.bash: No such file or directory` in this environment,
  then completes `erob_moveit_runtime` successfully.

Next pickup point:

1. Runtime-check ordered-chain execution after Slices 32-33 if possible,
   especially non-no-op planned unwind paths and explicit unwind verification.
2. Keep runtime unwind selection/call backend-local until the executor boundary
   is clearer.
3. Keep stop handling and high-level trajectory sends in the backend until a
   clearer executor boundary is introduced.
4. Do not remove the legacy nested unwind function yet; keep it available as
   fallback/reference until extracted unwind has more runtime validation.

### 2026-08-10 - Fake-system runtime validation after Slice 33

Status: operator-tested on fake hardware using log capture at:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/zeroerr/docs/temp_logs
```

Result: passed for the covered fake-system workflow.

Runtime evidence:

- Ordered-chain request 1 completed:
  `ordered_chain_done`, `request_id=1`, `result=0`,
  `total_elapsed_s=4.825`.
- Ordered-chain request 2 completed:
  `ordered_chain_done`, `request_id=2`, `result=0`,
  `total_elapsed_s=25.580`.
- Normal ordered trajectory helper path remained healthy:
  linear, blended, and path controller handoff/wait timing events were present
  and completed with `result=0`.
- Final ordered unwind segment completed:
  `label=prepare_dropoff_unwind`, `segment_type=unwind_joint6`, `result=0`.
- The final unwind was live-planned and skipped safely:
  `[UNWIND_J6] Rotational-path unwind skipped - no unwind needed`.
- No Traceback or ordered-chain failure was observed in the checked log.

Coverage note:

- This run did not show `Sending planned unwind ...` lines. It validates the
  successful live-final-unwind/no-op path after Slice 33, but does not runtime
  cover a non-no-op planned unwind trajectory send through
  `execute_ordered_unwind_trajectories(...)`.

Issues / repo facts found:

- Ruckig still reports implausible output on an early trajectory and falls back
  to seeded TOTG. The workflow continued successfully.
- `TOTG_PATH_DIAG reversal_candidate` warnings remain noisy, including later
  joint-6 reversal diagnostics on the long path. The run still completed
  successfully.

Next pickup point:

1. Treat Slice 33 as fake-system runtime validated for the logged no-op final
   unwind workflow.
2. Still runtime-check a non-no-op planned unwind trajectory path when a
   suitable test exists, because Slice 32's helper was only smoke/build checked.
3. Keep runtime unwind selection/call backend-local until the executor boundary
   is clearer.
4. Keep stop handling and high-level trajectory sends in the backend until a
   clearer executor boundary is introduced.
5. Do not remove the legacy nested unwind function yet; keep it available as
   fallback/reference until extracted unwind has more runtime validation.

### 2026-08-10 - Slice 34: ordered segment execution finish helper

Status: completed and compile/smoke/build-checked. Live execution behavior is
intended to remain unchanged. This slice extracts the common segment-finished
status/timing/scheduler publication block into `motion/execution/ordered_execution.py`.

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/execution/ordered_execution.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py
docs/refactoring/refactoring_plan_cpp_validation.md
```

What was done:

- Added `finish_ordered_segment_execution(...)`.
- Backend `_execute_planned_segment(...)` now delegates final per-segment
  bookkeeping to the helper:
  - ordered-chain segment-finished status publication
  - `[TIMING] ordered_motion_chain_segment` log
  - `ordered_segment_execute_done` timing event
  - scheduler `mark_finished(...)` update publication
- The helper receives status, timing, logger, and scheduler callbacks from the
  backend.

Issues / repo facts found:

- The helper does not own stop handling, trajectory sends, or runtime unwind
  selection. It only publishes the existing completion side effects after a
  result is known.
- No live launch was run after this slice; only syntax, focused helper smoke,
  and package build were checked.
- The legacy nested unwind implementation remains intentionally in place as
  fallback/reference per operator request.

Verification:

```text
python3 -m py_compile \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/execution/ordered_execution.py

PYTHONPATH=eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts:$PYTHONPATH python3 -c "from types import SimpleNamespace; from motion.execution.ordered_execution import finish_ordered_segment_execution; events=[]; logger=SimpleNamespace(info=lambda msg: events.append(('log',msg))); status=lambda **kw: events.append(('status',kw)); status_payload=lambda **kw: {'payload':kw}; mark=lambda *a, **kw: events.append(('mark',a,kw)); scheduler=lambda: {'scheduler':'done'}; publish=lambda updates: events.append(('scheduler',updates)); print(finish_ordered_segment_execution(node='node', index=2, planned_segment={'label':'s2','type':'linear'}, segment_type='linear', result=0, execution_started_s=0.0, logger=logger, set_ordered_motion_chain_status=status, ordered_chain_segment_finished_status=status_payload, mark_motion_timing=mark, mark_scheduler_finished=scheduler, publish_scheduler_updates=publish)); print([event[0] for event in events]); print(events[0]); print(events[-1])"

/bin/bash -lc "env -i HOME=\"$HOME\" TERM=\"${TERM:-xterm}\" PATH=\"/usr/bin:/bin:/usr/sbin:/sbin\" bash --noprofile --norc -lc 'source /opt/ros/rolling/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws/eRob_moveit; colcon build --packages-select erob_moveit_runtime'"
```

Build note:

- The build still prints
  `/opt/ros/rolling/setup.bash: No such file or directory` in this environment,
  then completes `erob_moveit_runtime` successfully.

Next pickup point:

1. Runtime-check ordered-chain execution after Slice 34 if possible, especially
   per-segment finished status/timing and scheduler state updates.
2. Still runtime-check a non-no-op planned unwind trajectory path when a
   suitable test exists, because Slice 32's helper was only smoke/build checked.
3. Keep runtime unwind selection/call backend-local until the executor boundary
   is clearer.
4. Keep stop handling and high-level trajectory sends in the backend until a
   clearer executor boundary is introduced.
5. Do not remove the legacy nested unwind function yet; keep it available as
   fallback/reference until extracted unwind has more runtime validation.

### 2026-08-10 - Slice 35: ordered segment execution start helper

Status: completed and compile/smoke/build-checked. Live execution behavior is
intended to remain unchanged. This slice pairs the Slice 34 finish helper with a
small start helper for the matching per-segment executing side effects.

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/execution/ordered_execution.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py
docs/refactoring/refactoring_plan_cpp_validation.md
```

What was done:

- Added `start_ordered_segment_execution(...)`.
- Backend `_execute_planned_segment(...)` now delegates the start-of-execution
  bookkeeping to the helper:
  - scheduler `mark_executing(...)` update publication
  - `ordered_segment_execute_start` timing event
  - ordered-chain executing status publication
- The helper preserves the previous side-effect order: scheduler update first,
  timing mark second, status publication third.

Issues / repo facts found:

- The helper does not own stop handling, controller sends, trajectory timing,
  or runtime unwind selection.
- No live launch was run after this slice; only syntax, focused helper smoke,
  and package build were checked.
- The legacy nested unwind implementation remains intentionally in place as
  fallback/reference per operator request.

Verification:

```text
python3 -m py_compile \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/execution/ordered_execution.py

PYTHONPATH=eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts:$PYTHONPATH python3 -c "from motion.execution.ordered_execution import start_ordered_segment_execution; events=[]; status=lambda **kw: events.append(('status',kw)); payload=lambda **kw: {'payload':kw}; mark=lambda *a, **kw: events.append(('mark',a,kw)); scheduler=lambda: {'scheduler':'executing'}; publish=lambda updates: events.append(('scheduler',updates)); start_ordered_segment_execution(node='node', index=3, total=5, planned_segment={'label':'s3','type':'ptp'}, preplanned_ready_count=2, set_ordered_motion_chain_status=status, ordered_chain_executing_status=payload, mark_motion_timing=mark, mark_scheduler_executing=scheduler, publish_scheduler_updates=publish); print([event[0] for event in events]); print(events[0]); print(events[-1])"

/bin/bash -lc "env -i HOME=\"$HOME\" TERM=\"${TERM:-xterm}\" PATH=\"/usr/bin:/bin:/usr/sbin:/sbin\" bash --noprofile --norc -lc 'source /opt/ros/rolling/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws/eRob_moveit; colcon build --packages-select erob_moveit_runtime'"
```

Build note:

- The build still prints
  `/opt/ros/rolling/setup.bash: No such file or directory` in this environment,
  then completes `erob_moveit_runtime` successfully.

Next pickup point:

1. Runtime-check ordered-chain execution after Slices 34-35 if possible,
   especially per-segment executing/finished status, timing events, and
   scheduler state updates.
2. Still runtime-check a non-no-op planned unwind trajectory path when a
   suitable test exists, because Slice 32's helper was only smoke/build checked.
3. Keep runtime unwind selection/call backend-local until the executor boundary
   is clearer.
4. Keep stop handling and high-level trajectory sends in the backend until a
   clearer executor boundary is introduced.
5. Do not remove the legacy nested unwind function yet; keep it available as
   fallback/reference until extracted unwind has more runtime validation.

### 2026-08-10 - Fake-system runtime validation after Slice 35

Status: passed for the logged fake-system ordered-chain workflow.

Log inspected:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/zeroerr/docs/temp_logs
```

What the run validated:

- Startup reached MoveIt planning readiness and runtime readiness:
  - `You can start planning now!`
  - `[Init] RobotController ready (0.69s total)`
- Two ordered-chain requests completed successfully:
  - request 1: 4 logical segments, `ordered_chain_done result=0`,
    total `4.809s`
  - request 2: 15 logical segments, `ordered_chain_done result=0`,
    total `25.700s`
- Slice 35 start helper path is runtime-covered:
  - every executed logical segment emitted `ordered_segment_execute_start`
  - the run covered `linear`, `blended`, `blend_consumed`, `path`, and
    `unwind_joint6` segment types
  - `preplanned_ready_count` values were present and changed as expected during
    lookahead/consumption
- Slice 34 finish helper path is runtime-covered:
  - every executed logical segment emitted `[TIMING] ordered_motion_chain_segment`
  - every executed logical segment emitted `ordered_segment_execute_done`
  - all logged segment results were `0`
- The scheduler bridge continued to preserve lookahead behavior:
  - request 1 began executing segment 1 while later segments were still being
    planned/queued
  - request 2 began executing a blended group with `preplanned_ready_count=5`
    and later consumed preplanned blend segments with decreasing ready counts

Issues / repo facts found:

- The run still did not exercise the non-no-op planned unwind send loop:
  - final segment logged `Executing live final unwind label='prepare_dropoff_unwind'`
  - no `Sending planned unwind ...` lines were present
  - keep Slice 32's planned-unwind helper marked as smoke/build checked only
    until a suitable run exercises that branch
- Ruckig helper reported implausible output on two early LIN plans and fell
  back to seeded TOTG:
  - `Ruckig output deemed implausible - falling back to seeded TOTG trajectory`
  - the fallback did not block execution; both ordered chains completed
- Remaining warnings/errors in the inspected run appear environmental or
  diagnostic rather than refactor regressions:
  - ROS localhost deprecation warnings
  - duplicate ROS logger publisher warnings
  - octomap/no 3D sensor plugin messages
  - missing collision geometry warnings for `tcp`
  - TOTG reversal diagnostic warnings

Next pickup point:

1. Treat Slices 34-35 as fake-system runtime validated for the logged workflow.
2. Still runtime-check a non-no-op planned unwind trajectory path when a
   suitable test exists, because Slice 32's helper remains unvalidated in
   runtime logs.
3. Continue with the next execution-boundary slice only if it is narrow and
   preserves behavior. Good candidates:
   - extract ordered controller handoff/state-match dependencies behind a
     small executor dependency object, or
   - start promoting the scheduler bridge toward the generalized `MotionQueue`
     without changing queue semantics.
4. Keep runtime unwind selection/call backend-local until the executor boundary
   is clearer.
5. Keep stop handling and high-level trajectory sends in the backend until a
   clearer executor boundary is introduced.
6. Do not remove the legacy nested unwind function yet; keep it available as
   fallback/reference until extracted unwind has more runtime validation.

### 2026-08-10 - Slice 36: ordered segment execution hook bundle

Status: completed and compile/smoke/build-checked. Live execution behavior is
intended to remain unchanged. This slice reduces the callback surface between
the backend and ordered execution helpers without moving controller behavior.

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/execution/ordered_execution.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py
docs/refactoring/refactoring_plan_cpp_validation.md
```

What was done:

- Added `OrderedSegmentExecutionHooks`.
- `start_ordered_segment_execution(...)` now receives the hook bundle instead
  of separate status/timing/scheduler callbacks.
- `finish_ordered_segment_execution(...)` now receives the same hook bundle.
- Backend `_execute_planned_segment(...)` constructs one
  `segment_execution_hooks` object and passes it to both helpers.
- Existing side-effect ordering was preserved:
  - start: scheduler update, timing mark, executing status
  - finish: finished status, timing log, timing mark, scheduler update

Issues / repo facts found:

- This is structural only. It does not move controller sends, wait-complete,
  state matching, stop handling, or runtime unwind selection.
- No live/fake launch was run after this slice; only syntax, focused helper
  smoke, and package build were checked.
- The planned non-no-op unwind send loop remains unvalidated in runtime logs.
- The legacy nested unwind implementation remains intentionally in place as
  fallback/reference per operator request.

Verification:

```text
python3 -m py_compile \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/execution/ordered_execution.py

PYTHONPATH=eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts:$PYTHONPATH python3 -c "from types import SimpleNamespace; from motion.execution.ordered_execution import OrderedSegmentExecutionHooks, start_ordered_segment_execution, finish_ordered_segment_execution; events=[]; logger=SimpleNamespace(info=lambda msg: events.append(('log',msg))); hooks=OrderedSegmentExecutionHooks(node='node', logger=logger, set_ordered_motion_chain_status=lambda **kw: events.append(('status',kw)), ordered_chain_executing_status=lambda **kw: {'executing':kw}, ordered_chain_segment_finished_status=lambda **kw: {'finished':kw}, mark_motion_timing=lambda *a, **kw: events.append(('mark',a,kw)), publish_scheduler_updates=lambda updates: events.append(('scheduler',updates))); start_ordered_segment_execution(hooks=hooks, index=1, total=2, planned_segment={'label':'s1','type':'ptp'}, preplanned_ready_count=1, mark_scheduler_executing=lambda: {'scheduler':'executing'}); print([event[0] for event in events]); print(events[0]); events.clear(); print(finish_ordered_segment_execution(hooks=hooks, index=1, planned_segment={'label':'s1','type':'ptp'}, segment_type='ptp', result=0, execution_started_s=0.0, mark_scheduler_finished=lambda: {'scheduler':'finished'})); print([event[0] for event in events]); print(events[-1])"

/bin/bash -lc "env -i HOME=\"$HOME\" TERM=\"${TERM:-xterm}\" PATH=\"/usr/bin:/bin:/usr/sbin:/sbin\" bash --noprofile --norc -lc 'source /opt/ros/rolling/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws/eRob_moveit; colcon build --packages-select erob_moveit_runtime'"
```

Build note:

- The build still prints
  `/opt/ros/rolling/setup.bash: No such file or directory` in this environment,
  then completes `erob_moveit_runtime` successfully.

Next pickup point:

1. Runtime-check ordered-chain execution after Slice 36 if possible, especially
   start/finish status/timing event ordering.
2. Still runtime-check a non-no-op planned unwind trajectory path when a
   suitable test exists, because Slice 32's helper remains unvalidated in
   runtime logs.
3. Continue with narrow execution-boundary work. Good next candidates:
   - add an ordered controller execution dependency object for
     `execute_ordered_timed_trajectory(...)` and
     `execute_ordered_unwind_trajectories(...)`, or
   - start promoting the scheduler bridge toward the generalized `MotionQueue`
     without changing queue semantics.
4. Keep runtime unwind selection/call backend-local until the executor boundary
   is clearer.
5. Keep stop handling and high-level trajectory sends in the backend until a
   clearer executor boundary is introduced.
6. Do not remove the legacy nested unwind function yet; keep it available as
   fallback/reference until extracted unwind has more runtime validation.

### 2026-08-10 - Slice 37: ordered controller execution hook bundle

Status: completed and compile/smoke/build-checked. Live execution behavior is
intended to remain unchanged. This slice reduces the callback surface for
controller-facing ordered execution helpers while keeping the backend as the
owner of the actual send/wait/state-match functions.

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/execution/ordered_execution.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py
docs/refactoring/refactoring_plan_cpp_validation.md
```

What was done:

- Added `OrderedControllerExecutionHooks`.
- `execute_ordered_timed_trajectory(...)` now receives the controller hook
  bundle instead of separate node/logger/timing/send/wait/state-match
  callbacks.
- `execute_ordered_unwind_trajectories(...)` now receives the same controller
  hook bundle for planned unwind trajectory sends and waits.
- Backend `_execute_planned_segment(...)` constructs one
  `controller_execution_hooks` object and passes it to both helpers.
- Controller send, wait-complete, and ordered start/end state-match behavior
  remain backend-provided callables.

Issues / repo facts found:

- This is structural only. It does not move stop handling, high-level send
  semantics, runtime unwind selection, or explicit unwind verification.
- No live/fake launch was run after this slice; only syntax, focused helper
  smoke, and package build were checked.
- The focused smoke test covers the planned-unwind helper's send/wait path at
  unit level, but the non-no-op planned unwind path remains unvalidated in
  runtime logs.
- The legacy nested unwind implementation remains intentionally in place as
  fallback/reference per operator request.

Verification:

```text
python3 -m py_compile \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/execution/ordered_execution.py

PYTHONPATH=eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts:$PYTHONPATH python3 -c "from types import SimpleNamespace; from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint; from builtin_interfaces.msg import Duration; from motion.execution.ordered_execution import OrderedControllerExecutionHooks, ordered_trajectory_timing, execute_ordered_timed_trajectory, execute_ordered_unwind_trajectories; events=[]; logger=SimpleNamespace(info=lambda msg: events.append(('log',msg))); hooks=OrderedControllerExecutionHooks(node='node', logger=logger, mark_motion_timing=lambda *a, **k: events.append(('mark',a,k)), wait_point_match=lambda label,t,point,phase: events.append(('match',phase)) or True, send_trajectory=lambda *a, **k: events.append(('send',len(a),k)), wait_execution_complete=lambda n,t: events.append(('wait',round(t,3))) or 0); traj=JointTrajectory(); p=JointTrajectoryPoint(); p.time_from_start=Duration(sec=1,nanosec=0); traj.points=[p]; planned={'label':'s1','type':'linear','trajectory':traj,'plan_elapsed_s':0.2}; timing=ordered_trajectory_timing(traj,min_timeout_s=5,timeout_multiplier=2); print(execute_ordered_timed_trajectory(hooks=hooks,index=1,total=1,planned_segment=planned,segment_type='linear',timing=timing,execution_started_s=0.0,motion_error_result=-6)); print([event[0] for event in events]); events.clear(); print(execute_ordered_unwind_trajectories(hooks=hooks, planned_segment={'trajectories':[traj], 'trajectory_checks':['c1'], 'check':'fallback'}, min_timeout_s=5, timeout_multiplier=2)); print(events)"

/bin/bash -lc "env -i HOME=\"$HOME\" TERM=\"${TERM:-xterm}\" PATH=\"/usr/bin:/bin:/usr/sbin:/sbin\" bash --noprofile --norc -lc 'source /opt/ros/rolling/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws/eRob_moveit; colcon build --packages-select erob_moveit_runtime'"
```

Build note:

- The build still prints
  `/opt/ros/rolling/setup.bash: No such file or directory` in this environment,
  then completes `erob_moveit_runtime` successfully.

Next pickup point:

1. Runtime-check ordered-chain execution after Slices 36-37 if possible,
   especially normal timed trajectory execution and segment start/finish
   telemetry.
2. Still runtime-check a non-no-op planned unwind trajectory path when a
   suitable test exists, because Slice 32's helper remains unvalidated in
   runtime logs.
3. Continue with narrow execution-boundary work. Good next candidates:
   - extract explicit unwind finalization dependencies into a small hook object,
     or
   - start promoting the scheduler bridge toward the generalized `MotionQueue`
     without changing queue semantics.
4. Keep runtime unwind selection/call backend-local until the executor boundary
   is clearer.
5. Keep stop handling and high-level trajectory sends in the backend until a
   clearer executor boundary is introduced.
6. Do not remove the legacy nested unwind function yet; keep it available as
   fallback/reference until extracted unwind has more runtime validation.

### 2026-08-10 - Runtime observation: unwind verification vs visible J6 motion

Status: investigated from the latest fake-system log after the operator noted
that the log reported unwind verification but Joint 6 was not visibly rotating
in RViz.

Log inspected:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/zeroerr/docs/temp_logs
```

What the log shows:

- The ordered-chain final unwind at request 2 segment 15 did not rotate:
  - `ordered_segment_execute_start ... index=15 label=prepare_dropoff_unwind`
  - `Executing live final unwind label='prepare_dropoff_unwind'`
  - `[UNWIND_J6] Rotational-path unwind skipped - no unwind needed`
  - segment then completed with `result=0`
- The skip is intentional in current code when
  `abs(canonical_angle(Joint_6) - current Joint_6) <
  EXECUTOR_POST_UNWIND_MIN_DELTA_RAD`.
  The default minimum delta is `0.5rad`.
- A later explicit rotational unwind did send visible Joint 6 motion:
  - `Executing rotational-path unwind: Joint_6 -3.334 -> 2.949 rad delta=6.283 rad`
  - segment 1 sent controller points ending near `Joint_6=-0.192`
  - segment 2 sent controller points ending near `Joint_6=2.942`
  - final verification logged:
    `Explicit Joint_6 unwind verified: actual=2.9416 target=2.9494`
- Therefore, in this run, "verified" belongs to the later explicit unwind
  path, not to the ordered-chain final unwind segment that was skipped as a
  no-op.

Issue / interpretation:

- A successful unwind segment can be a no-op if Joint 6 is already close enough
  to its canonical target. That will not visibly rotate in RViz.
- The current log wording can be confusing because the later explicit unwind
  verification makes it look like the ordered-chain final unwind rotated.
- If operators need visible motion whenever an unwind segment is requested,
  the skip threshold/config/semantics must be revisited separately as a
  behavioral change, not as part of the structural refactor.

Next pickup point:

1. Do not treat `Explicit Joint_6 unwind verified` as proof that the ordered
   final unwind segment rotated. Check for `Sending planned unwind ...` or
   `Executing rotational-path unwind ...` near the same request/segment.
2. Consider adding more explicit log wording later, for example:
   `Ordered final unwind no-op: current=..., target=..., delta=..., min_delta=...`.
3. Keep the legacy nested unwind implementation in place until planned and
   live unwind behavior is clearer and runtime-validated.

### 2026-08-10 - Slice 38: explicit no-op unwind diagnostics

Status: completed and compile/build-checked. This is a diagnostic-only change
from the runtime observation above; motion behavior is intended to remain
unchanged.

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py
docs/refactoring/refactoring_plan_cpp_validation.md
```

What was done:

- Improved ordered-chain planned no-op unwind logging. The log now includes:
  - joint name
  - current Joint 6 value
  - canonical target
  - delta
  - configured `min_delta`
- Improved live rotational-path no-op unwind logging with the same values.
- This should make the RViz/log interpretation clear when an unwind segment
  succeeds without visible motion because it was below the configured unwind
  threshold.

Issues / repo facts found:

- Explicit queued/direct unwind no-op logs in `trajectory_executor.py` still do
  not include the same numeric details because `_build_post_success_unwind_trajectory(...)`
  currently returns only `None`, not a structured skip reason.
- No runtime/fake launch was run after this diagnostic slice.
- The planned non-no-op unwind send loop remains unvalidated in runtime logs.
- The legacy nested unwind implementation remains intentionally in place as
  fallback/reference per operator request.

Verification:

```text
python3 -m py_compile \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py

/bin/bash -lc "env -i HOME=\"$HOME\" TERM=\"${TERM:-xterm}\" PATH=\"/usr/bin:/bin:/usr/sbin:/sbin\" bash --noprofile --norc -lc 'source /opt/ros/rolling/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws/eRob_moveit; colcon build --packages-select erob_moveit_runtime'"
```

Build note:

- The build still prints
  `/opt/ros/rolling/setup.bash: No such file or directory` in this environment,
  then completes `erob_moveit_runtime` successfully.

Next pickup point:

1. Runtime-check an ordered final unwind no-op and confirm the new log shows
   current/target/delta/min_delta.
2. Still runtime-check a non-no-op planned unwind trajectory path when a
   suitable test exists, because Slice 32's helper remains unvalidated in
   runtime logs.
3. If explicit queued/direct no-op unwind logs remain confusing, consider
   returning a structured skip reason from `_build_post_success_unwind_trajectory(...)`
   in a separate diagnostic-only slice.
4. Do not remove the legacy nested unwind function yet; keep it available as
   fallback/reference until extracted unwind has more runtime validation.

### 2026-08-10 - Runtime validation: explicit J6 unwind fine; jog logger issue found

Status: explicit rotational-path unwind validated from the latest fake-system
log. A separate ROS logger API misuse in jog timeout/error handling was found
and fixed.

Log inspected:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/zeroerr/docs/temp_logs
```

What the run validated:

- Explicit rotational-path unwind sent two segments:
  - `Executing rotational-path unwind: Joint_6 -3.334 -> 2.949 rad delta=6.283 rad`
  - segment 1: `-3.334 -> 2.949 rad`, Cartesian delta `-180deg`
  - segment 2: `-0.193 -> 2.949 rad`, Cartesian delta `-180deg`
- Controller trajectories were sent and accepted for both unwind segments.
- Final unwind verification passed:
  - `Explicit Joint_6 unwind verified: actual=2.9457 target=2.9494 error=-0.0037 rad tol=0.1200`

Issue found:

- The same log also contained:
  - `Jog error: RcutilsLogger.error() takes 2 positional arguments but 3 were given`
- Root cause was ROS `RcutilsLogger.error()` being called with a format string
  plus a separate argument in the jog timeout path. ROS logger methods here
  expect a single formatted message string.
- A similar latent `%s`-style logger call was found in stop handling.

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/execution/motion_coordinator.py
docs/refactoring/refactoring_plan_cpp_validation.md
```

What was done:

- Converted the jog timeout error log to an f-string before passing it to
  `self.node.get_logger().error(...)`.
- Converted the stop failure log in `motion_coordinator.py` to an f-string.
- Searched for remaining `get_logger().error(..., extra_arg)` patterns in
  runtime scripts; no remaining ROS logger error calls with extra formatting
  args were found.

Verification:

```text
python3 -m py_compile \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/execution/motion_coordinator.py

rg -n "get_logger\(\)\.error\([^\n]*,|_node\.get_logger\(\)\.error\([^\n]*," \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts

/bin/bash -lc "env -i HOME=\"$HOME\" TERM=\"${TERM:-xterm}\" PATH=\"/usr/bin:/bin:/usr/sbin:/sbin\" bash --noprofile --norc -lc 'source /opt/ros/rolling/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws/eRob_moveit; colcon build --packages-select erob_moveit_runtime'"
```

Build note:

- The build still prints
  `/opt/ros/rolling/setup.bash: No such file or directory` in this environment,
  then completes `erob_moveit_runtime` successfully.

Next pickup point:

1. Runtime-check rotational jog again and confirm the
   `RcutilsLogger.error() takes 2 positional arguments` error is gone.
2. Runtime-check an ordered final unwind no-op and confirm the new
   current/target/delta/min_delta log appears when the skip branch is taken.
3. Still runtime-check a non-no-op planned unwind trajectory path when a
   suitable test exists, because Slice 32's helper remains unvalidated in
   runtime logs.
4. Do not remove the legacy nested unwind function yet; keep it available as
   fallback/reference until extracted unwind has more runtime validation.

### 2026-08-10 - Slice 39: explicit unwind skip reasons

Status: completed and compile/smoke/build-checked. This is a diagnostic-only
change; motion behavior is intended to remain unchanged.

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/execution/trajectory_executor.py
docs/refactoring/refactoring_plan_cpp_validation.md
```

What was done:

- Added `_set_post_success_unwind_skip_reason(...)` and
  `_format_post_success_unwind_skip_reason(...)`.
- `_build_post_success_unwind_trajectory(...)` now records why it returned
  `None`:
  - disabled
  - joint not configured
  - latest joint state unavailable
  - no unwind needed, including current/target/delta/min_delta
  - target outside allowed range
- Explicit queued/direct unwind skip logs now include that reason instead of
  always saying only `no unwind needed`.

Issues / repo facts found:

- The automatic post-motion unwind path still silently does nothing when the
  builder returns `None`; this slice only improves explicit request feedback.
- No runtime/fake launch was run after this slice.
- The planned non-no-op unwind send loop remains unvalidated in runtime logs.
- The legacy nested unwind implementation remains intentionally in place as
  fallback/reference per operator request.

Verification:

```text
python3 -m py_compile \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/execution/trajectory_executor.py

EROB_CONFIG_PACKAGE=zeroerr \
PYTHONPATH=eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts:$PYTHONPATH \
python3 - <<'PY'
from motion.execution.trajectory_executor import TrajectoryExecutor
executor = TrajectoryExecutor.__new__(TrajectoryExecutor)
executor._last_post_success_unwind_skip_reason = None
executor._set_post_success_unwind_skip_reason(
    'no_unwind_needed',
    joint_name='Joint_6',
    current_value=0.1,
    target_value=0.0,
    delta=-0.1,
    min_delta=0.5,
)
print(executor._format_post_success_unwind_skip_reason())
executor._set_post_success_unwind_skip_reason(
    'latest_joint_state_unavailable',
    joint_name='Joint_6',
)
print(executor._format_post_success_unwind_skip_reason())
executor._set_post_success_unwind_skip_reason(
    'target_out_of_range',
    joint_name='Joint_6',
    target_value=3.5,
    target_range=3.14159,
)
print(executor._format_post_success_unwind_skip_reason())
PY

/bin/bash -lc "env -i HOME=\"$HOME\" TERM=\"${TERM:-xterm}\" PATH=\"/usr/bin:/bin:/usr/sbin:/sbin\" bash --noprofile --norc -lc 'source /opt/ros/rolling/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws/eRob_moveit; colcon build --packages-select erob_moveit_runtime'"
```

Build note:

- The build still prints
  `/opt/ros/rolling/setup.bash: No such file or directory` in this environment,
  then completes `erob_moveit_runtime` successfully.

Next pickup point:

1. Runtime-check explicit unwind skip/no-op and confirm the new reason appears.
2. Runtime-check rotational jog again and confirm the
   `RcutilsLogger.error() takes 2 positional arguments` error is gone.
3. Runtime-check an ordered final unwind no-op and confirm the backend
   current/target/delta/min_delta log appears when that skip branch is taken.
4. Still runtime-check a non-no-op planned unwind trajectory path when a
   suitable test exists, because Slice 32's helper remains unvalidated in
   runtime logs.
5. Do not remove the legacy nested unwind function yet; keep it available as
   fallback/reference until extracted unwind has more runtime validation.

### 2026-08-10 - Slice 40: rotational jog wait timeout

Status: completed and compile/smoke/build-checked. This is a runtime
robustness fix for blocking jog calls; the planned path and controller
trajectory are unchanged.

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py
docs/refactoring/refactoring_plan_cpp_validation.md
```

What was done:

- Added `_jog_blocking_timeout_s(...)` so interpolated rotational jogs get a
  timeout based on angular delta and requested velocity.
- Linear jogs and small rotational jogs still use the existing
  `JOG_BLOCKING_TIMEOUT_S` base timeout.
- The blocking jog timeout log now reports the computed timeout.
- Fixed a second latent jog logger formatting issue in the hardware-not-ready
  reject path by converting the message to an f-string before calling
  `get_logger().error(...)`.

Issues / runtime facts found:

- The fresh fake-system run showed the earlier ROS logger crash was gone, but
  an RZ rotational jog still timed out at the fixed 5.00s wait:
  `[JOG] Timed out waiting for jog motion to complete after 5.00s`.
- The controller accepted the trajectory and reported success shortly after
  the jog wait timed out, so the failure was a premature blocking wait timeout,
  not a controller execution failure.
- The example jog was 180deg at 10% velocity. The new timeout helper returns
  10.20s for that case, while preserving 5.00s for a 4deg rotational jog and
  for linear jogs.
- Ruckig still reports implausible stretched output on some rotational paths
  and falls back to the seeded trajectory. That fallback behaved correctly in
  this run and is separate from the jog wait timeout.

Verification:

```text
python3 -m py_compile \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py

EROB_CONFIG_PACKAGE=zeroerr \
PYTHONPATH=eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts:$PYTHONPATH \
python3 - <<'PY'
from backend.moveit_robot_backend import MoveItRobotBackend
backend = MoveItRobotBackend.__new__(MoveItRobotBackend)
print(f'{backend._jog_blocking_timeout_s(6, [0,0,0,0,0,180.0], 10.0):.2f}')
print(f'{backend._jog_blocking_timeout_s(6, [0,0,0,0,0,4.0], 10.0):.2f}')
print(f'{backend._jog_blocking_timeout_s(1, [50,0,0,0,0,0], 10.0):.2f}')
PY

rg -n "get_logger\(\)\.error\([^\n]*,|_node\.get_logger\(\)\.error\([^\n]*,|JOG_BLOCKING_TIMEOUT_S|_jog_blocking_timeout_s|JOG\] Rejected" \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/execution/motion_coordinator.py

/bin/bash -lc "env -i HOME=\"$HOME\" TERM=\"${TERM:-xterm}\" PATH=\"/usr/bin:/bin:/usr/sbin:/sbin\" bash --noprofile --norc -lc 'source /opt/ros/rolling/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws/eRob_moveit; colcon build --packages-select erob_moveit_runtime'"
```

Smoke output:

```text
10.20
5.00
5.00
```

Build note:

- The build still prints
  `/opt/ros/rolling/setup.bash: No such file or directory` in this environment,
  then completes `erob_moveit_runtime` successfully.

Next pickup point:

1. Runtime-check a 180deg RZ jog again in fake system. Expected result: no
   premature `[JOG] Timed out ...` when the controller succeeds within the
   computed timeout.
2. Confirm no remaining `RcutilsLogger.error()` crash on jog failures or
   hardware-not-ready jog rejection.
3. If very large or very slow rotational jogs still exceed the dynamic wait,
   tune `_jog_blocking_timeout_s(...)` or promote the heuristic to config.
4. Continue non-no-op planned unwind runtime validation when a suitable test
   exists.
5. Do not remove the legacy nested unwind function yet; keep it available as
   fallback/reference until extracted unwind has more runtime validation.

### 2026-08-10 - Slice 41: ordered unwind finalization hook bundle

Status: completed and compile/smoke/build-checked. Live execution behavior is
intended to remain unchanged. This slice reduces another backend callback
surface in the ordered execution helper boundary.

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/execution/ordered_execution.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py
docs/refactoring/refactoring_plan_cpp_validation.md
```

What was done:

- Added `OrderedUnwindFinalizationHooks`.
- `finalize_ordered_unwind_result(...)` now receives the hook bundle instead
  of separate `node` and `verify_explicit_unwind_complete` arguments.
- Backend `_execute_ordered_motion_chain_pipelined(...)` constructs one
  `unwind_finalization_hooks` object beside the existing segment/controller
  hook bundles.
- Existing finalization behavior was preserved:
  - verification success leaves result as `0`
  - verification failure maps to `-6`
  - non-zero controller/send result is preserved
  - unwind failure time/result bookkeeping is still written on the runtime node

Issues / repo facts found:

- This is structural only. It does not move runtime unwind selection, planned
  unwind planning, controller sends, or stop handling.
- No live/fake launch was run after this slice; only syntax, focused helper
  smoke, and package build were checked.
- The non-no-op planned unwind send loop remains unvalidated in runtime logs.
- The legacy nested unwind implementation remains intentionally in place as
  fallback/reference per operator request.

Verification:

```text
python3 -m py_compile \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/execution/ordered_execution.py

PYTHONPATH=eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts:$PYTHONPATH \
python3 -c "from types import SimpleNamespace; from motion.execution.ordered_execution import OrderedUnwindFinalizationHooks, finalize_ordered_unwind_result; n=SimpleNamespace(); hooks=OrderedUnwindFinalizationHooks(node=n, verify_explicit_unwind_complete=lambda c: True); print(finalize_ordered_unwind_result(hooks=hooks, planned_segment={'check':'ok'}, result=0, now_s=12.5)); print(hasattr(n,'_last_ordered_unwind_failure_result')); n=SimpleNamespace(); hooks=OrderedUnwindFinalizationHooks(node=n, verify_explicit_unwind_complete=lambda c: False); print(finalize_ordered_unwind_result(hooks=hooks, planned_segment={'check':'bad'}, result=0, now_s=13.5)); print(n._last_ordered_unwind_failure_time, n._last_ordered_unwind_failure_result); n=SimpleNamespace(); hooks=OrderedUnwindFinalizationHooks(node=n, verify_explicit_unwind_complete=lambda c: True); print(finalize_ordered_unwind_result(hooks=hooks, planned_segment={}, result=-9, now_s=14.5)); print(n._last_ordered_unwind_failure_time, n._last_ordered_unwind_failure_result)"

rg -n "finalize_ordered_unwind_result\(|OrderedUnwindFinalizationHooks" \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts

/bin/bash -lc "env -i HOME=\"$HOME\" TERM=\"${TERM:-xterm}\" PATH=\"/usr/bin:/bin:/usr/sbin:/sbin\" bash --noprofile --norc -lc 'source /opt/ros/rolling/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws/eRob_moveit; colcon build --packages-select erob_moveit_runtime'"
```

Smoke output:

```text
0
False
-6
13.5 -6
-9
14.5 -9
```

Build note:

- The build still prints
  `/opt/ros/rolling/setup.bash: No such file or directory` in this environment,
  then completes `erob_moveit_runtime` successfully.

Next pickup point:

1. Runtime-check ordered-chain execution after Slices 36-37 and 41 if possible,
   especially normal timed trajectory execution, segment start/finish telemetry,
   and unwind finalization bookkeeping.
2. Runtime-check a 180deg RZ jog again after Slice 40. Expected result: no
   premature `[JOG] Timed out ...` when the controller succeeds within the
   computed timeout.
3. Still runtime-check a non-no-op planned unwind trajectory path when a
   suitable test exists, because planned unwind send remains smoke/build
   checked only.
4. Continue with narrow execution-boundary work only; keep runtime unwind
   selection/call backend-local until the executor boundary is clearer.
5. Do not remove the legacy nested unwind function yet; keep it available as
   fallback/reference until extracted unwind has more runtime validation.

### 2026-08-10 - Slice 42: ordered planned-segment execution helper

Status: completed and compile/smoke/build-checked. Live execution behavior is
intended to remain unchanged. This slice moves the ordered planned-segment
dispatch out of the backend into the ordered execution helper module.

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/execution/ordered_execution.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py
docs/refactoring/refactoring_plan_cpp_validation.md
```

What was done:

- Added `execute_ordered_planned_segment(...)`.
- Backend `_execute_planned_segment(...)` now delegates segment execution
  dispatch to that helper.
- The helper owns the existing per-type branch dispatch for:
  - timed controller trajectory segments: `linear`, `ptp`, `path`, `blended`
  - no-op motion segments
  - `blend_consumed`
  - `unwind_joint6`
- The backend still injects all runtime-owned effects:
  - scheduler executing/finished callbacks
  - controller hook bundle
  - unwind finalization hook bundle
  - runtime live-unwind callable
  - config-derived timeouts/defaults/error code
- Existing side-effect ordering is preserved: start bookkeeping, branch
  execution, finish bookkeeping.

Issues / repo facts found:

- This is structural only. It does not move planning, stop handling,
  controller send/wait implementations, or runtime unwind selection.
- Focused smoke made an existing live-final-unwind behavior explicit:
  `runtime_unwind(...)` can return a non-zero result, but the result is then
  overwritten by `execute_ordered_unwind_trajectories(...)`. When the planned
  trajectory list is empty, that helper returns `0`. This was preserved for
  compatibility in this slice, but it should be reviewed separately because it
  may hide a live-final-unwind failure.
- No live/fake launch was run after this slice; only syntax, focused helper
  smoke, and package build were checked.
- The non-no-op planned unwind send loop remains unvalidated in runtime logs.
- The legacy nested unwind implementation remains intentionally in place as
  fallback/reference per operator request.

Verification:

```text
python3 -m py_compile \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/execution/ordered_execution.py

PYTHONPATH=eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts:$PYTHONPATH \
python3 - <<'PY'
from types import SimpleNamespace
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from motion.execution.ordered_execution import (
    OrderedControllerExecutionHooks,
    OrderedSegmentExecutionHooks,
    OrderedUnwindFinalizationHooks,
    execute_ordered_planned_segment,
)

def make_traj():
    traj = JointTrajectory()
    p0 = JointTrajectoryPoint(); p0.time_from_start = Duration(sec=0, nanosec=0)
    p1 = JointTrajectoryPoint(); p1.time_from_start = Duration(sec=1, nanosec=0)
    traj.points = [p0, p1]
    return traj

events = []
logger = SimpleNamespace(info=lambda msg: events.append(('log', msg)))
segment_hooks = OrderedSegmentExecutionHooks(
    node='node',
    logger=logger,
    set_ordered_motion_chain_status=lambda **kw: events.append(('status', kw)),
    ordered_chain_executing_status=lambda **kw: {'executing': kw},
    ordered_chain_segment_finished_status=lambda **kw: {'finished': kw},
    mark_motion_timing=lambda *a, **kw: events.append(('mark', a, kw)),
    publish_scheduler_updates=lambda updates: events.append(('scheduler', updates)),
)
controller_hooks = OrderedControllerExecutionHooks(
    node='node',
    logger=logger,
    mark_motion_timing=lambda *a, **kw: events.append(('mark', a, kw)),
    wait_point_match=lambda label, traj, point, phase: events.append(('match', phase)) or True,
    send_trajectory=lambda *a, **kw: events.append(('send', len(a), kw)),
    wait_execution_complete=lambda node, timeout: events.append(('wait', round(timeout, 3))) or 0,
)
unwind_hooks = OrderedUnwindFinalizationHooks(
    node=SimpleNamespace(),
    verify_explicit_unwind_complete=lambda check: events.append(('verify', check)) or True,
)
base_kwargs = dict(
    segment_hooks=segment_hooks,
    controller_hooks=controller_hooks,
    unwind_finalization_hooks=unwind_hooks,
    total=4,
    preplanned_ready_count=1,
    execution_started_s=0.0,
    mark_scheduler_executing=lambda: {'scheduler': 'executing'},
    mark_scheduler_finished=lambda result: {'scheduler': 'finished', 'result': result},
    min_timeout_s=5.0,
    timeout_multiplier=2.0,
    motion_error_result=-6,
    default_velocity_percent=20.0,
    default_acceleration_percent=20.0,
    runtime_unwind=lambda **kw: events.append(('runtime_unwind', kw)) or -8,
    now_s=lambda: 42.0,
)
print(execute_ordered_planned_segment(index=1, planned_segment={'type':'linear','label':'lin','trajectory':make_traj(),'plan_elapsed_s':0.2}, **base_kwargs))
print([e[0] for e in events])
events.clear()
print(execute_ordered_planned_segment(index=2, planned_segment={'type':'linear','label':'noop','noop':True}, **base_kwargs))
print([e[0] for e in events])
events.clear()
print(execute_ordered_planned_segment(index=3, planned_segment={'type':'blend_consumed','label':'blend'}, **base_kwargs))
print([e[0] for e in events])
events.clear()
print(execute_ordered_planned_segment(index=4, planned_segment={'type':'unwind_joint6','label':'uw','runtime_unwind':True,'plan_elapsed_s':0.1,'trajectories':[], 'trajectory_checks':[], 'check':None}, **base_kwargs))
print([e[0] for e in events])
PY

/bin/bash -lc "env -i HOME=\"$HOME\" TERM=\"${TERM:-xterm}\" PATH=\"/usr/bin:/bin:/usr/sbin:/sbin\" bash --noprofile --norc -lc 'source /opt/ros/rolling/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws/eRob_moveit; colcon build --packages-select erob_moveit_runtime'"
```

Smoke output:

```text
0
['scheduler', 'mark', 'status', 'log', 'match', 'mark', 'send', 'mark', 'wait', 'mark', 'log', 'match', 'status', 'log', 'mark', 'scheduler']
0
['scheduler', 'mark', 'status', 'log', 'status', 'log', 'mark', 'scheduler']
0
['scheduler', 'mark', 'status', 'log', 'status', 'log', 'mark', 'scheduler']
0
['scheduler', 'mark', 'status', 'log', 'runtime_unwind', 'status', 'log', 'mark', 'scheduler']
```

Build note:

- The build still prints
  `/opt/ros/rolling/setup.bash: No such file or directory` in this environment,
  then completes `erob_moveit_runtime` successfully.

Next pickup point:

1. Runtime-check ordered-chain execution after Slice 42 if possible, especially
   per-segment start/finish telemetry and normal timed trajectory execution.
2. Review the preserved live-final-unwind result overwrite as a separate
   behavioral fix candidate. Do not change it silently inside another
   structural extraction.
3. Runtime-check a 180deg RZ jog again after Slice 40. Expected result: no
   premature `[JOG] Timed out ...` when the controller succeeds within the
   computed timeout.
4. Still runtime-check a non-no-op planned unwind trajectory path when a
   suitable test exists, because planned unwind send remains smoke/build
   checked only.
5. Do not remove the legacy nested unwind function yet; keep it available as
   fallback/reference until extracted unwind has more runtime validation.

### 2026-08-10 - Slice 43: preserve live-final-unwind failure result

Status: completed and compile/smoke/build-checked. This is a narrow behavior
fix for an issue found during Slice 42.

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/execution/ordered_execution.py
docs/refactoring/refactoring_plan_cpp_validation.md
```

What was done:

- Changed the ordered `unwind_joint6` execution branch so
  `execute_ordered_unwind_trajectories(...)` runs only when the current result
  is still `0`.
- This preserves a non-zero live-final-unwind result instead of overwriting it
  with `0` from the empty planned-trajectory helper.
- Planned non-runtime unwind behavior is preserved: planned unwind
  trajectories still send, wait, and verify when the incoming result is `0`.

Issues / repo facts found:

- Slice 42 exposed that `runtime_unwind(...)` could fail, but the result was
  immediately overwritten by `execute_ordered_unwind_trajectories(...)`.
  Runtime final unwind segments normally carry an empty trajectory list, so the
  planned-trajectory helper returned `0` and could hide the live unwind
  failure.
- This change is behavioral and intentionally separate from the Slice 42
  structural extraction.
- No live/fake launch was run after this slice; only syntax, focused helper
  smoke, and package build were checked.
- The non-no-op planned unwind send loop remains unvalidated in runtime logs,
  though its focused smoke branch still sends/waits/verifies.
- The legacy nested unwind implementation remains intentionally in place as
  fallback/reference per operator request.

Verification:

```text
python3 -m py_compile \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/execution/ordered_execution.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py

PYTHONPATH=eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts:$PYTHONPATH \
python3 - <<'PY'
from types import SimpleNamespace
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from motion.execution.ordered_execution import (
    OrderedControllerExecutionHooks,
    OrderedSegmentExecutionHooks,
    OrderedUnwindFinalizationHooks,
    execute_ordered_planned_segment,
)

def make_traj():
    traj = JointTrajectory()
    p = JointTrajectoryPoint(); p.time_from_start = Duration(sec=1, nanosec=0)
    traj.points = [p]
    return traj

def make_hooks(events, runtime_result):
    logger = SimpleNamespace(info=lambda msg: events.append(('log', msg)))
    segment_hooks = OrderedSegmentExecutionHooks(
        node='node', logger=logger,
        set_ordered_motion_chain_status=lambda **kw: events.append(('status', kw)),
        ordered_chain_executing_status=lambda **kw: {'executing': kw},
        ordered_chain_segment_finished_status=lambda **kw: {'finished': kw},
        mark_motion_timing=lambda *a, **kw: events.append(('mark', a, kw)),
        publish_scheduler_updates=lambda updates: events.append(('scheduler', updates)),
    )
    controller_hooks = OrderedControllerExecutionHooks(
        node='node', logger=logger,
        mark_motion_timing=lambda *a, **kw: events.append(('mark', a, kw)),
        wait_point_match=lambda label, traj, point, phase: True,
        send_trajectory=lambda *a, **kw: events.append(('send', kw)),
        wait_execution_complete=lambda node, timeout: events.append(('wait', round(timeout, 3))) or 0,
    )
    unwind_hooks = OrderedUnwindFinalizationHooks(
        node=SimpleNamespace(),
        verify_explicit_unwind_complete=lambda check: events.append(('verify', check)) or True,
    )
    return dict(
        segment_hooks=segment_hooks,
        controller_hooks=controller_hooks,
        unwind_finalization_hooks=unwind_hooks,
        total=1,
        preplanned_ready_count=0,
        execution_started_s=0.0,
        mark_scheduler_executing=lambda: {},
        mark_scheduler_finished=lambda result: {'result': result},
        min_timeout_s=5.0,
        timeout_multiplier=2.0,
        motion_error_result=-6,
        default_velocity_percent=20.0,
        default_acceleration_percent=20.0,
        runtime_unwind=lambda **kw: events.append(('runtime_unwind', kw)) or runtime_result,
        now_s=lambda: 42.0,
    )

events = []
result = execute_ordered_planned_segment(
    index=1,
    planned_segment={'type':'unwind_joint6','label':'live_fail','runtime_unwind':True,'plan_elapsed_s':0.1,'trajectories':[], 'trajectory_checks':[], 'check':None},
    **make_hooks(events, -8),
)
print(result)
print([e[0] for e in events])

events = []
result = execute_ordered_planned_segment(
    index=1,
    planned_segment={'type':'unwind_joint6','label':'planned','runtime_unwind':False,'plan_elapsed_s':0.1,'trajectories':[make_traj()], 'trajectory_checks':['c1'], 'check':'final'},
    **make_hooks(events, 0),
)
print(result)
print([e[0] for e in events])
PY

/bin/bash -lc "env -i HOME=\"$HOME\" TERM=\"${TERM:-xterm}\" PATH=\"/usr/bin:/bin:/usr/sbin:/sbin\" bash --noprofile --norc -lc 'source /opt/ros/rolling/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws/eRob_moveit; colcon build --packages-select erob_moveit_runtime'"
```

Smoke output:

```text
-8
['scheduler', 'mark', 'status', 'log', 'runtime_unwind', 'status', 'log', 'mark', 'scheduler']
0
['scheduler', 'mark', 'status', 'log', 'send', 'wait', 'verify', 'status', 'log', 'mark', 'scheduler']
```

Build note:

- The build still prints
  `/opt/ros/rolling/setup.bash: No such file or directory` in this environment,
  then completes `erob_moveit_runtime` successfully.

Next pickup point:

1. Runtime-check ordered-chain final live unwind after Slice 43 if possible.
   Expected result: if live unwind fails, ordered segment/chain should now
   report the non-zero failure instead of silently completing.
2. Runtime-check ordered-chain execution after Slice 42 if possible, especially
   per-segment start/finish telemetry and normal timed trajectory execution.
3. Runtime-check a 180deg RZ jog again after Slice 40. Expected result: no
   premature `[JOG] Timed out ...` when the controller succeeds within the
   computed timeout.
4. Still runtime-check a non-no-op planned unwind trajectory path when a
   suitable test exists, because planned unwind send remains runtime-uncovered.
5. Do not remove the legacy nested unwind function yet; keep it available as
   fallback/reference until extracted unwind has more runtime validation.

### 2026-08-10 - Slice 44: ordered state-match helper extraction

Status: completed and compile/smoke/build-checked. Live execution behavior is
intended to remain unchanged. This slice removes the ordered state-match
implementation from the backend and keeps runtime state/config as injected
dependencies.

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/execution/ordered_execution.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py
docs/refactoring/refactoring_plan_cpp_validation.md
```

What was done:

- Added `OrderedStateMatchHooks`.
- Added `ordered_trajectory_point_match_error(...)`.
- Added `wait_ordered_trajectory_point_match(...)`.
- Removed the nested backend functions:
  - `_ordered_trajectory_point_match_error(...)`
  - `_wait_ordered_trajectory_point_match(...)`
- Backend `_execute_ordered_motion_chain_pipelined(...)` now constructs one
  `state_match_hooks` object and injects a thin `wait_point_match` lambda into
  `OrderedControllerExecutionHooks`.
- Preserved the existing state-match behavior:
  - same enabled flag
  - same tolerance and timeout config
  - same timing event names
  - same info/error log text
  - same polling interval

Issues / repo facts found:

- This is structural only. It does not move planning, controller send/wait,
  stop handling, scheduler loop ownership, or unwind target math.
- The explicit unwind target issue observed in the latest fake-system log was
  not changed per operator request; leave that for later real-robot testing.
- No live/fake launch was run after this slice; only syntax, focused helper
  smoke, and package build were checked.
- The legacy nested unwind implementation remains intentionally in place as
  fallback/reference per operator request.

Verification:

```text
python3 -m py_compile \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/execution/ordered_execution.py

PYTHONPATH=eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts:$PYTHONPATH \
python3 - <<'PY'
from types import SimpleNamespace
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from motion.execution.ordered_execution import OrderedStateMatchHooks, wait_ordered_trajectory_point_match

def state(names, positions):
    return SimpleNamespace(name=names, position=positions)

def traj_point(value):
    traj = JointTrajectory()
    traj.joint_names = ['Joint_1']
    point = JointTrajectoryPoint()
    point.positions = [value]
    return traj, point

def run(enabled, live_state, planned, tolerance=0.02, timeout=0.0):
    events = []
    hooks = OrderedStateMatchHooks(
        node='node',
        logger=SimpleNamespace(
            info=lambda msg: events.append(('info', msg)),
            error=lambda msg: events.append(('error', msg)),
        ),
        get_live_joint_state=lambda: live_state,
        mark_motion_timing=lambda *a, **kw: events.append(('mark', a, kw)),
        enabled=enabled,
        tolerance_rad=tolerance,
        timeout_s=timeout,
    )
    traj, point = traj_point(planned)
    result = wait_ordered_trajectory_point_match(
        hooks=hooks,
        label='seg',
        joint_trajectory=traj,
        point=point,
        phase='start',
    )
    print(result, [e[0] for e in events], events[-1])

run(True, state(['Joint_1'], [1.0]), 1.01, tolerance=0.02, timeout=0.0)
run(False, None, 1.0)
run(True, state(['Joint_1'], [1.2]), 1.0, tolerance=0.02, timeout=0.0)
run(True, None, 1.0, tolerance=0.02, timeout=0.0)
PY

rg -n "_ordered_trajectory_point_match_error|_wait_ordered_trajectory_point_match|OrderedStateMatchHooks|wait_ordered_trajectory_point_match|ordered_trajectory_point_match_error" \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/execution/ordered_execution.py

/bin/bash -lc "env -i HOME=\"$HOME\" TERM=\"${TERM:-xterm}\" PATH=\"/usr/bin:/bin:/usr/sbin:/sbin\" bash --noprofile --norc -lc 'source /opt/ros/rolling/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws/eRob_moveit; colcon build --packages-select erob_moveit_runtime'"
```

Smoke output:

```text
True ['mark', 'info', 'info', 'mark'] ('mark', ('node', 'ordered_state_match_done'), {'label': 'seg', 'phase': 'start', 'matched': True, 'duration_s': 1.4151970390230417e-05, 'max_error_rad': 0.010000000000000009, 'joint': 'Joint_1'})
True ['info', 'mark'] ('mark', ('node', 'ordered_state_match_skipped'), {'label': 'seg', 'phase': 'start'})
False ['mark', 'info', 'error', 'mark'] ('mark', ('node', 'ordered_state_match_done'), {'label': 'seg', 'phase': 'start', 'matched': False, 'duration_s': 9.000010322779417e-06, 'max_error_rad': 0.19999999999999996, 'joint': 'Joint_1'})
False ['mark', 'info', 'error', 'mark'] ('mark', ('node', 'ordered_state_match_done'), {'label': 'seg', 'phase': 'start', 'matched': False, 'duration_s': 1.6579870134592056e-06, 'reason': 'no live joint state'})
```

Build note:

- The build still prints
  `/opt/ros/rolling/setup.bash: No such file or directory` in this environment,
  then completes `erob_moveit_runtime` successfully.

Next pickup point:

1. Runtime-check ordered-chain execution after Slice 44 if possible, especially
   start/end state-match telemetry and normal timed trajectory execution.
2. Continue the planned architecture by extracting the remaining ordered-chain
   orchestration out of `_execute_ordered_motion_chain_pipelined(...)`.
3. Do not change explicit unwind target selection until the operator is ready
   to test on the real robot.
4. Runtime-check a 180deg RZ jog again after Slice 40 when convenient.
5. Do not remove the legacy nested unwind function yet; keep it available as
   fallback/reference until extracted unwind has more runtime validation.

### 2026-08-10 - Runtime validation after Slice 44

Status: fake-system runtime validated for the logged ordered-chain workflow.

Log inspected:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/zeroerr/docs/temp_logs
```

What the run validated:

- Startup reached runtime readiness quickly:
  - `[Init] RobotController ready (0.38s total)`
- `ptp_helper`, `contour_ik_helper`, `ipp_helper`, and `ruckig_helper` all
  reported ready.
- Slice 44's extracted ordered state-match path is runtime-covered:
  - `ordered_state_match_start` and `ordered_state_match_done` were emitted
    for both start and end phases.
  - Start/end matches reported `matched=True`.
  - The largest logged end-state match error was still within tolerance:
    `max_error_rad=0.018 joint=Joint_6` with tolerance `0.020`.
- Ordered-chain execution telemetry remained intact:
  - `ordered_segment_execute_start`
  - `ordered_segment_execute_done`
  - `ordered_motion_chain_total result=0`
  - `ordered_chain_done result=0`
- The 15-segment paint/prep ordered chain completed successfully:
  - `ordered_chain_done ... result=0 total_elapsed_s=25.735`
- The final ordered unwind segment was a live no-op in this run:
  - `Rotational-path unwind skipped - no unwind needed`
  - `Joint_6 current=-0.1993rad target=-0.1993rad delta=0.0000rad`

Issues / repo facts found:

- One early `move_ptp` request happened before `ptp_helper` fully reported
  ready:
  - `[PTP] native PTP planning timed out after 1.100s`
  - `PTP helper ready` appeared about 2.8s later
  - subsequent PTP requests succeeded at about 93ms planning time
  This looks like a remaining startup/readiness race for PTP helper readiness,
  not a Slice 44 regression.
- Ruckig still sometimes reports implausible stretched output and falls back to
  seeded TOTG. The fallback allowed motion to continue and both ordered chains
  completed.
- The explicit unwind target-selection issue observed earlier was not retested
  here and remains intentionally deferred until real-robot testing.
- Environmental warnings remain present:
  - ROS localhost deprecation warnings
  - duplicate ROS logger publisher warnings
  - octomap/no 3D sensor plugin messages
  - missing collision geometry warning for `tcp`

Next pickup point:

1. Treat Slice 44 as fake-system runtime validated for the logged ordered-chain
   workflow.
2. Startup helper service advertisement race was addressed in Slice 45.
3. Continue the planned architecture by extracting the remaining ordered-chain
   orchestration out of `_execute_ordered_motion_chain_pipelined(...)`.
4. Do not change explicit unwind target selection until the operator is ready
   to test on the real robot.
5. Do not remove the legacy nested unwind function yet; keep it available as
   fallback/reference until extracted unwind has more runtime validation.

### 2026-08-10 - Slice 45: helper services advertise only after ready

Status: implemented and build validated.

Issue found from fresh fake-system logs:

- Fake hardware startup made the race easy to see because the runtime and
  controller manager came up quickly.
- The root issue was not fake-only: `ptp_helper` advertised `/compute_ptp` in
  its constructor, before `initialize()` finished loading the MoveIt robot model
  and local PlanningScene.
- Runtime readiness checked service availability, so `/compute_ptp` could look
  ready before the helper logged `PTP helper ready`.
- A client could then submit a first PTP request into the helper warmup window:
  `[PTP] native PTP planning timed out after 1.100s`.

What changed:

- Deferred C++ helper service advertisement until after initialization succeeds:
  - `ptp_helper_node.cpp`: `/compute_ptp`
  - `contour_ik_helper_node.cpp`: `/compute_contour_ik`
  - `ipp_helper_node.cpp`: `/apply_ipp`
  - `ruckig_helper_node.cpp`: `/apply_ruckig`
- Added `SERVICE_APPLY_RUCKIG` to shared defaults and robot runtime config.
- Added a first-class `RobotController.ruckig_client`.
- Expanded motion-stack readiness to check the Ruckig optimizer service as well
  as PTP, Contour IK, IPP/TOTG, MoveIt services and the controller action
  server.
- Updated the Ruckig optimizer call path to use configured
  `SERVICE_APPLY_RUCKIG` instead of hard-coded `/apply_ruckig`.

Startup/readiness rule:

- A helper service must not be advertised until the helper is ready to process a
  real request.
- Every service, action server, helper, background publisher/subscriber input or
  runtime dependency used by a motion path must be represented in
  `RobotController.get_motion_stack_fault_reason()` /
  `RobotController.is_motion_stack_ready()`.
- GUI/client ready state must come from motion-stack readiness, not merely from
  Python node construction or REST/WebSocket availability.

Verification:

```text
python3 -m py_compile \
  erob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/config.py \
  erob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/robot_controller.py \
  erob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/execution/trajectory_optimization.py

colcon build --packages-select erob_moveit_runtime
```

Build result:

- `erob_moveit_runtime` built successfully.
- Existing/non-blocking output remains:
  - `/opt/ros/rolling/setup.bash` missing in the local build shell
  - colcon underlay override warning
  - Jazzy `tl_expected/expected.hpp` deprecation notes

Next pickup point:

1. Runtime-check fake startup and confirm `/health` stays
   `motion_stack_warming` until `PTP helper ready`, `Contour IK helper ready`,
   `IPP service '/apply_ipp' is now ready`, `Ruckig service '/apply_ruckig' is
   now ready`, and the controller action server are all available.
2. Confirm the first GUI move after ready no longer hits the PTP helper warmup
   timeout.
3. Continue the planned architecture by extracting the remaining ordered-chain
   orchestration out of `_execute_ordered_motion_chain_pipelined(...)`.
4. Leave explicit unwind target selection unchanged until real-robot testing.
5. Keep the nested legacy unwind function in place as fallback/reference until
   the extracted unwind path has enough real-robot validation.

### 2026-08-10 - Runtime validation after Slice 45

Status: fake-system runtime validated from fresh operator logs.

Log inspected:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/zeroerr/docs/temp_logs
```

What the run validated:

- Helper service advertisement now happens after helper initialization:
  - `ipp_helper`: `Service '/apply_ipp' created` immediately followed by
    `IPP service '/apply_ipp' is now ready`.
  - `ruckig_helper`: `Service '/apply_ruckig' created` immediately followed by
    `Ruckig service '/apply_ruckig' is now ready`.
  - `contour_ik_helper`: `Service '/compute_contour_ik' created` immediately
    followed by `Contour IK helper ready with local PlanningScene`.
  - `ptp_helper`: `Service '/compute_ptp' created` immediately followed by
    `PTP helper ready`.
- No `native PTP planning timed out`, `motion stack is not ready`, JSON decode
  failure, or early PTP rejection appeared in this run.
- Initial PTP moves succeeded:
  - `backend_move_ptp submitted blocking=True result=0`.
  - Controller reported `Goal reached, success!`.
- Ordered-chain execution still succeeds after the readiness lifecycle change:
  - request 1 completed with `ordered_chain_done ... result=0
    total_elapsed_s=4.856`.
  - request 2 completed with `ordered_chain_done ... result=0
    total_elapsed_s=25.604`.
- Extracted ordered state-match logic remained runtime-covered:
  - `ordered_state_match_start` / `ordered_state_match_done` logs present.
  - Live end-state checks matched before advancing.
- Final ordered unwind remained a live no-op in this run:
  - `Rotational-path unwind skipped - no unwind needed`.

Open observations:

- Backend-only review did not include GUI/client `/health` payload lines, but
  the later GUI log append below validates the exact client-visible transition.
- Ruckig still reports implausible stretched output and falls back to seeded
  TOTG. This is noisy but fallback preserved successful execution.
- Environmental warnings remain unchanged:
  ROS localhost deprecation, duplicate rosout publishers, missing `tcp`
  collision geometry, and no octomap sensor plugin.

Next pickup point:

1. Treat Slice 45 as fake-system runtime validated for helper lifecycle and
   motion execution.
2. If GUI health logs are available later, confirm client-visible `/health`
   stays unready until all helper services are advertised.
3. Continue the planned architecture by extracting the remaining ordered-chain
   orchestration out of `_execute_ordered_motion_chain_pipelined(...)`.
4. Leave explicit unwind target selection unchanged until real-robot testing.
5. Keep the nested legacy unwind function in place as fallback/reference until
   the extracted unwind path has enough real-robot validation.

### 2026-08-10 - GUI health validation after Slice 45

Status: GUI/client startup readiness validated from the same fake-system run.

Log inspected:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/zeroerr/docs/temp_logs
```

What the GUI saw:

- Before the ROS2 backend HTTP server was up, `/health` failed with expected
  connection-refused messages.
- Once HTTP was available, health returned JSON instead of parse failures:
  - `status='http_ready'`
  - `phase='http_ready'`
  - `ready=False`
  - `ros2_active=False`
- After the runtime object existed but motion dependencies were still warming,
  health stayed unready:
  - first fault: `current Cartesian pose not available yet`
  - then repeated fault: `PTP helper service not available`
  - `motion_stack_ready=False`
  - `ready=False`
  - `status='motion_stack_warming'`
- The first GUI-visible ready health payload appeared only after the PTP helper
  service was advertised and ready:
  - `message='Robot runtime is ready'`
  - `motion_stack_ready=True`
  - `ready=True`
  - `phase='ready'`
  - `status='ok'`

Correlation with backend helper lifecycle:

- Backend log: `Service '/compute_ptp' created` and `PTP helper ready` at ROS
  time `1786356021.998...`.
- GUI log: first `ready=True/status='ok'` at wall time `13:00:22`.
- This confirms the GUI did not see a ready robot while PTP was still missing.

Motion result from GUI side:

- The first logged `move_ptp` after readiness returned:
  - `http=200`
  - `raw={'queued': False, 'result': 0, 'success': True, 'task_id': 1}`
  - `move_ptp_total success=True`
- Later PTP and ordered-chain calls also returned HTTP 200 with `result=0`.
- No GUI-side `JSONDecodeError`, `Expecting value`, `motion stack is not ready`,
  or PTP warmup timeout appeared in this log.

Remaining notes:

- Multiple client adapters appear to be polling/reconnecting simultaneously,
  causing duplicate health/WebSocket connection logs. This is noisy but did not
  cause incorrect readiness.
- Connection-refused logs before backend startup are expected if the GUI starts
  before ROS2. They are not a readiness regression.

Next pickup point:

1. Treat Slice 45 as validated from both backend and GUI logs.
2. Continue the planned architecture by extracting the remaining ordered-chain
   orchestration out of `_execute_ordered_motion_chain_pipelined(...)`.
3. Leave explicit unwind target selection unchanged until real-robot testing.
4. Keep the nested legacy unwind function in place as fallback/reference until
   the extracted unwind path has enough real-robot validation.

### 2026-08-10 - Slice 46: ordered planned-sequence consumer extraction

Status: implemented and smoke validated.

What changed:

- Added `OrderedPlannedSequenceHooks` to
  `motion/execution/ordered_execution.py`.
- Added `execute_ordered_planned_sequence(...)` to consume preplanned ordered
  segments from `OrderedSchedulerBridge`, update ready/preplanned status, handle
  ordered-chain stop requests, toggle `_suppress_post_success_unwind`, execute
  each planned segment in order, wait for planner completion, and restore
  suppression state in `finally`.
- Updated `MoveItRobotBackend._execute_ordered_motion_chain_pipelined(...)` to
  delegate the planned-segment consumer loop to the extracted helper.

Behavior preserved:

- Existing timing events remain:
  - `ordered_plan_wait_start`
  - `ordered_motion_chain_plan_ready`
  - `ordered_plan_ready`
- Stop behavior still returns `-14` and publishes stopped status.
- Planned segment execution still goes through the existing
  `_execute_planned_segment(...)` backend closure, so controller execution,
  state matching, unwind handling and status publication stay unchanged.
- Executor shutdown remains owned by the backend.

Verification:

```text
python3 -m py_compile \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/execution/ordered_execution.py

PYTHONPATH=eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts:$PYTHONPATH \
python3 -c "<smoke test for execute_ordered_planned_sequence>"
```

Smoke result:

```text
0 True True ['mark', 'log', 'mark', 'status', 'exec']
```

Backend size:

```text
moveit_robot_backend.py: 2475 lines
```

Next pickup point:

1. Runtime-check ordered-chain execution after Slice 46 with fake system when
   convenient. Expected result: same `ordered_plan_wait_start`,
   `ordered_plan_ready`, `ordered_segment_execute_start`,
   `ordered_segment_execute_done`, and `ordered_chain_done result=0` telemetry.
2. Continue extracting the planning worker or blend-group planning block toward
   `motion/scheduling/motion_scheduler.py`.
3. Leave explicit unwind target selection unchanged until real-robot testing.
4. Keep the nested legacy unwind function in place as fallback/reference until
   the extracted unwind path has enough real-robot validation.

### 2026-08-10 - Slice 46 runtime validation

Status: passed for the logged fake-system ordered-chain workflow.

Log inspected:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/zeroerr/docs/temp_logs
```

What the run covered:

- The extracted planned-sequence consumer emitted the expected telemetry:
  - `ordered_plan_wait_start`
  - `ordered_motion_chain_plan_ready`
  - `ordered_plan_ready`
  - `ordered_segment_execute_start`
  - `ordered_segment_execute_done`
  - `ordered_motion_chain_total`
  - `ordered_chain_done`
- Two ordered-chain requests completed successfully:
  - request 1: `ordered_chain_done ... result=0 total_elapsed_s=4.878`
  - request 2: `ordered_chain_done ... result=0 total_elapsed_s=25.616`
- The planning worker completed without errors:
  - request 1: `ordered_planning_worker_done ... duration_s=1.296`
  - request 2: `ordered_planning_worker_done ... duration_s=6.975`
- Normal PTP requests in the same run submitted with `result=0`.
- The ordered final unwind segment was a no-op in this run:
  - `prepare_dropoff_unwind ... segment_type=unwind_joint6`
  - `Rotational-path unwind skipped - no unwind needed`
  - `ordered_segment_execute_done ... result=0`

Issues / notes found:

- No `ordered_planning_worker_error` was present.
- Ruckig still occasionally reports implausible output and falls back to seeded
  TOTG. The fallback allowed execution to continue; keep this as a separate
  optimizer-quality/noise issue.
- Remaining warnings are known startup/environment noise: ROS localhost
  deprecation, duplicate rosout publishers, missing `tcp` collision geometry,
  no octomap sensor plugin and initial TF lookup before transforms are live.
- Stop/cancel behavior was not exercised by this run.
- Explicit unwind target selection remains deferred until real-robot testing.

Next pickup point:

1. Treat Slice 46 as fake-system runtime validated.
2. Extract the ordered planning worker or blend-group planning block toward
   `motion/scheduling/motion_scheduler.py`.
3. Keep the nested legacy unwind function in place as fallback/reference until
   the extracted unwind path has enough real-robot validation.

### 2026-08-10 - Slice 47: ordered planning worker extraction

Status: completed and compile/build checked. Runtime behavior is intended to
remain unchanged.

Files added:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/ordered_planning_worker.py
```

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/__init__.py
```

What was done:

- Added `OrderedPlanningWorkerHooks` and
  `execute_ordered_planning_worker(...)`.
- Moved the ordered planning worker loop out of
  `_execute_ordered_motion_chain_pipelined(...)`.
- Moved contiguous LIN/PTP blend-group planning and consumed-segment
  publication into the scheduling helper.
- Kept backend-owned dependencies injected:
  - `mark_motion_timing`
  - planned-segment publisher
  - `_plan_ordered_segment(...)`
  - extracted `BlendBuilder`
  - optimizer callback
  - final-state conversion callback
- Backend still owns thread creation, `OrderedSchedulerBridge`, stop event,
  planned-sequence execution and executor shutdown.

Issues / notes found:

- `moveit_robot_backend.py` is still large, but this slice reduced it from
  2475 lines to 2178 lines.
- The planning helper lazily imports `RobotTrajectory` only for blend groups so
  simple non-ROS smoke checks can import and exercise the non-blend path.
- Stop/cancel behavior still needs runtime validation after this extraction.
- Explicit unwind target behavior was left unchanged by request.
- Keep the nested legacy unwind function in place as fallback/reference until
  real-robot validation.

Verification:

```text
python3 -m py_compile \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/ordered_planning_worker.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/__init__.py

PYTHONPATH=eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts:$PYTHONPATH \
python3 -c "<smoke test for execute_ordered_planning_worker non-blend path>"

/bin/bash -lc "env -i HOME=\"$HOME\" TERM=\"${TERM:-xterm}\" PATH=\"/usr/bin:/bin:/usr/sbin:/sbin\" bash --noprofile --norc -lc 'source /opt/ros/rolling/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws/eRob_moveit; colcon build --packages-select erob_moveit_runtime'"
```

Verification notes:

- `py_compile` passed.
- Smoke result: planned entries were published in order and
  `ordered_planning_worker_done` was emitted.
- `colcon build --packages-select erob_moveit_runtime` passed.
- The build still prints the existing
  `/opt/ros/rolling/setup.bash: No such file or directory` warning before
  sourcing the installed workspace; it does not fail the build.

Next pickup point:

1. Runtime-check ordered-chain execution after Slice 47 with fake system.
   Expected result: same ordered planning and execution telemetry as Slice 46,
   with no `ordered_planning_worker_error`.
2. Continue reducing `_plan_ordered_segment(...)`, especially the remaining
   ordered unwind planning branch, but keep the nested legacy unwind fallback
   until real-robot validation.
3. Start moving backend hook construction into a small ordered-chain context
   object once the planning worker extraction has runtime coverage.

### 2026-08-10 - Slice 48: ordered unwind planner extraction

Status: completed and compile/build checked. Explicit unwind target behavior is
unchanged.

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/planning/ordered_unwind_planner.py
```

What was done:

- Added `plan_ordered_unwind_segment(...)` to
  `motion/planning/ordered_unwind_planner.py`.
- Moved the ordered-chain `unwind_joint6` planning branch out of
  `_plan_ordered_segment(...)`.
- Added shared planner helpers:
  - `joint_positions_by_name(...)`
  - `force_ordered_unwind_joint_branch(...)`
- Backend still owns the MoveIt/direct-IK-specific trajectory planning callback
  because it needs local workobject waypoint generation, tool transform and
  optimizer context.
- Standalone/nested legacy unwind execution remains in place as
  fallback/reference by request.

Issues / notes found:

- `moveit_robot_backend.py` is now 1868 lines, down from 2178 after Slice 47.
- The no-op/live-final unwind smoke paths can be exercised without MoveIt.
- The planned direct-IK unwind path still needs runtime coverage; the user asked
  to defer real unwind behavior decisions until real-robot testing.

Verification:

```text
python3 -m py_compile \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/planning/ordered_unwind_planner.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/ordered_planning_worker.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/__init__.py

PYTHONPATH=eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts:$PYTHONPATH \
python3 -c "<smoke test for plan_ordered_unwind_segment live-final and no-op paths>"

/bin/bash -lc "env -i HOME=\"$HOME\" TERM=\"${TERM:-xterm}\" PATH=\"/usr/bin:/bin:/usr/sbin:/sbin\" bash --noprofile --norc -lc 'source /opt/ros/rolling/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws/eRob_moveit; colcon build --packages-select erob_moveit_runtime'"
```

Verification notes:

- `py_compile` passed.
- Smoke result:
  - live-final path returned `runtime_unwind=True`, unchanged target pose,
    `blendR=0.0`
  - no-op planned path returned `check=None`, unchanged target pose,
    `blendR=0.0`
- `colcon build --packages-select erob_moveit_runtime` passed.
- The build still prints the existing
  `/opt/ros/rolling/setup.bash: No such file or directory` warning before
  sourcing the installed workspace; it does not fail the build.

Next pickup point:

1. Runtime-check ordered-chain execution after Slices 47-48 with fake system.
   Expected result: same ordered planning/execution telemetry as Slice 46 and
   no `ordered_planning_worker_error`.
2. If runtime is clean, extract ordered-chain setup/hook construction from
   `_execute_ordered_motion_chain_pipelined(...)` into a context/builder helper.
3. Keep explicit unwind target decisions and nested legacy unwind removal
   deferred until real-robot validation.

### 2026-08-10 - Slice 49: ordered execution hook bundle

Status: completed and compile/build checked. Runtime behavior is intended to
remain unchanged.

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/execution/ordered_execution.py
```

What was done:

- Added `OrderedExecutionHookBundle` to
  `motion/execution/ordered_execution.py`.
- Added `build_ordered_execution_hook_bundle(...)` to centralize construction
  of:
  - `OrderedSegmentExecutionHooks`
  - `OrderedStateMatchHooks`
  - `OrderedControllerExecutionHooks`
  - `OrderedUnwindFinalizationHooks`
- Replaced inline hook construction in
  `_execute_ordered_motion_chain_pipelined(...)` with one call to the new
  builder.
- Backend still injects all side-effecting callbacks:
  - ordered-chain status setter
  - timing marker
  - scheduler status publisher
  - controller trajectory sender
  - controller wait function
  - explicit unwind verifier

Issues / notes found:

- `moveit_robot_backend.py` is now 1835 lines, down from 1868 after Slice 48.
- This slice intentionally moved construction only; controller execution,
  state matching and unwind finalization behavior stayed in the existing helper
  functions.
- Runtime validation after Slices 47-49 is still needed.

Verification:

```text
python3 -m py_compile \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/execution/ordered_execution.py

PYTHONPATH=eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts:$PYTHONPATH \
python3 -c "<smoke test for build_ordered_execution_hook_bundle>"

/bin/bash -lc "env -i HOME=\"$HOME\" TERM=\"${TERM:-xterm}\" PATH=\"/usr/bin:/bin:/usr/sbin:/sbin\" bash --noprofile --norc -lc 'source /opt/ros/rolling/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws/eRob_moveit; colcon build --packages-select erob_moveit_runtime'"
```

Verification notes:

- `py_compile` passed.
- Smoke result: `True 0.03 0.4 True`.
- `colcon build --packages-select erob_moveit_runtime` passed.
- The build still prints the existing
  `/opt/ros/rolling/setup.bash: No such file or directory` warning before
  sourcing the installed workspace; it does not fail the build.

Next pickup point:

1. Runtime-check ordered-chain execution after Slices 47-49 with fake system.
   Expected result: same ordered planning/execution telemetry as Slice 46 and
   no `ordered_planning_worker_error`.
2. Continue setup extraction by moving ordered initial-state creation and
   scheduler state defaults out of `_execute_ordered_motion_chain_pipelined(...)`.
3. Keep explicit unwind target decisions and nested legacy unwind removal
   deferred until real-robot validation.

### 2026-08-10 - Slices 47-49 runtime validation

Status: passed for the logged fake-system ordered-chain workflow.

Log inspected:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/zeroerr/docs/temp_logs
```

What the run covered:

- Ordered planning worker extraction is runtime-covered:
  - request 1: `ordered_planning_worker_done ... duration_s=1.317`
  - request 2: `ordered_planning_worker_done ... duration_s=7.126`
  - no `ordered_planning_worker_error` was present.
- Ordered execution hook bundle path is runtime-covered:
  - `ordered_state_match_start` / `ordered_state_match_done`
  - `ordered_controller_handoff_start`
  - `ordered_wait_execution_done`
  - `ordered_segment_execute_done`
- Two ordered-chain requests completed successfully:
  - request 1: `ordered_chain_done ... result=0 total_elapsed_s=4.872`
  - request 2: `ordered_chain_done ... result=0 total_elapsed_s=25.771`
- Normal PTP calls in the same run submitted with `result=0`.
- Ordered final unwind remained a live no-op:
  - `Ordered final unwind will be planned live during execution`
  - `Executing live final unwind label='prepare_dropoff_unwind'`
  - `Rotational-path unwind skipped - no unwind needed`
  - `ordered_segment_execute_done ... segment_type=unwind_joint6 result=0`

Issues / notes found:

- No JSON decode errors, controller-manager rejection or motion-stack-not-ready
  rejection appeared in the inspected run.
- Ruckig still emitted implausible-output warnings and fell back to seeded TOTG.
  The fallback did not block execution.
- Remaining warnings are known environment/startup noise: no octomap sensor
  plugin and an early TF lookup before transforms are connected.
- Stop/cancel behavior was not exercised by this run.
- Explicit unwind target behavior remains deferred until real-robot testing.

Next pickup point:

1. Treat Slices 47-49 as fake-system runtime validated.
2. Continue setup extraction by moving ordered initial-state creation and
   scheduler state defaults out of `_execute_ordered_motion_chain_pipelined(...)`.
3. Keep explicit unwind target decisions and nested legacy unwind removal
   deferred until real-robot validation.

### 2026-08-10 - Slice 50: ordered initial-state/default scheduler extraction

Status: completed and compile/build checked. Runtime behavior is intended to
remain unchanged.

Files added:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/planning/ordered_initial_state.py
```

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/__init__.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/status_adapter.py
```

What was done:

- Added `OrderedInitialPlanningState` and
  `build_ordered_initial_planning_state(...)`.
- Moved ordered-chain starting Cartesian capture, clean `RobotState` creation,
  selected optimizer calculation and previous unwind-suppression capture out of
  `_execute_ordered_motion_chain_pipelined(...)`.
- Added `ordered_chain_initial_segment_states_from_mappings(...)` for legacy
  ordered-chain dictionaries.
- Replaced the backend's inline scheduler segment-state default builder with
  the new status helper.

Issues / notes found:

- `moveit_robot_backend.py` is now 1818 lines, down from 1835 after Slice 49.
- `ordered_initial_state.py` lazily imports `moveit_msgs.msg.RobotState`, so
  pure status-adapter smoke checks remain possible outside a launched runtime.
- Runtime validation after Slice 50 is still needed.
- Explicit unwind target decisions and nested legacy unwind removal remain
  deferred until real-robot validation.

Verification:

```text
python3 -m py_compile \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/planning/ordered_initial_state.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/status_adapter.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/__init__.py

PYTHONPATH=eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts:$PYTHONPATH \
python3 -c "<smoke test for ordered_chain_initial_segment_states_from_mappings>"

/bin/bash -lc "env -i HOME=\"$HOME\" TERM=\"${TERM:-xterm}\" PATH=\"/usr/bin:/bin:/usr/sbin:/sbin\" bash --noprofile --norc -lc 'source /opt/ros/rolling/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws/eRob_moveit; colcon build --packages-select erob_moveit_runtime'"
```

Verification notes:

- `py_compile` passed.
- Smoke result returned two `PENDING` scheduler segment-state dictionaries, with
  default label `segment_2` for an unlabeled segment.
- `colcon build --packages-select erob_moveit_runtime` passed.
- The build still prints the existing
  `/opt/ros/rolling/setup.bash: No such file or directory` warning before
  sourcing the installed workspace; it does not fail the build.

Next pickup point:

1. Runtime-check ordered-chain execution after Slice 50 with fake system.
   Expected result: same ordered initial-state, planning-worker and execution
   telemetry as the Slices 47-49 validation.
2. Continue extracting `_execute_ordered_motion_chain_pipelined(...)` by moving
   ordered blend-builder creation or planning callback construction out of the
   backend.
3. Keep explicit unwind target decisions and nested legacy unwind removal
   deferred until real-robot validation.

### 2026-08-10 - Slice 51: ordered blend-builder setup extraction

Status: completed and compile/build checked. Runtime behavior is intended to
remain unchanged.

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/blending/__init__.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/blending/blend_builder.py
```

What was done:

- Added `wait_moveit_state_validity(...)` to `motion/blending/blend_builder.py`.
- Added `build_ordered_blend_builder(...)` to centralize ordered blend-builder
  construction from runtime config.
- Moved the MoveIt state-validity service request used by ordered blend sample
  validation out of `_execute_ordered_motion_chain_pipelined(...)`.
- Replaced the backend's inline `BlendBuilderConfig` mapping and local
  `_wait_state_validity(...)` function with
  `build_ordered_blend_builder(planning_node, config)`.
- Exported the blend helpers from `motion/blending/__init__.py`.

Issues / notes found:

- `moveit_robot_backend.py` is now 1773 lines, down from 1818 after Slice 50.
- The state-validity helper still lazily imports MoveIt service message types,
  so simple builder-construction smoke checks can run outside a launched
  runtime.
- Runtime validation after Slice 51 is still needed.

Verification:

```text
python3 -m py_compile \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/blending/blend_builder.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/blending/__init__.py

PYTHONPATH=eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts:$PYTHONPATH \
python3 -c "<smoke test for build_ordered_blend_builder config mapping>"

/bin/bash -lc "env -i HOME=\"$HOME\" TERM=\"${TERM:-xterm}\" PATH=\"/usr/bin:/bin:/usr/sbin:/sbin\" bash --noprofile --norc -lc 'source /opt/ros/rolling/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws/eRob_moveit; colcon build --packages-select erob_moveit_runtime'"
```

Verification notes:

- `py_compile` passed.
- Smoke result: `True 6.0 0.04 0.7 9`.
- `colcon build --packages-select erob_moveit_runtime` passed.
- The build still prints the existing
  `/opt/ros/rolling/setup.bash: No such file or directory` warning before
  sourcing the installed workspace; it does not fail the build.

Next pickup point:

1. Runtime-check ordered-chain execution after Slices 50-51 with fake system.
   Expected result: same ordered initial-state, blend validation,
   planning-worker and execution telemetry as the Slices 47-49 validation.
2. Continue extracting `_execute_ordered_motion_chain_pipelined(...)` by moving
   ordered planning callback construction or the planner thread submit/wait
   wrapper out of the backend.
3. Keep explicit unwind target decisions and nested legacy unwind removal
   deferred until real-robot validation.

### 2026-08-10 - Slice 52: ordered segment planner dispatcher extraction

Status: completed and compile/build checked. Runtime behavior is intended to
remain unchanged.

Files added:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/planning/ordered_segment_planner.py
```

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py
```

What was done:

- Added `OrderedSegmentPlannerHooks` and `plan_ordered_segment(...)`.
- Moved the ordered segment type dispatcher out of
  `_execute_ordered_motion_chain_pipelined(...)`.
- The extracted dispatcher now owns the LIN/PTP/PATH/unwind branch selection and
  calls the already-extracted ordered segment planners.
- Backend still injects all runtime dependencies:
  - planning node and tool transform
  - workobject transform callback
  - timing marker
  - segment planner/optimizer callbacks
  - follow-path builder
  - final-state converter
  - unwind clamp/canonical-angle/direct-IK callbacks

Issues / notes found:

- `moveit_robot_backend.py` is now 1667 lines, down from 1773 after Slice 51.
- The direct-IK ordered unwind trajectory callback remains in the backend
  because it still depends on backend rotational waypoint generation and local
  tool/workobject context.
- Runtime validation after Slice 52 is still needed.

Verification:

```text
python3 -m py_compile \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/planning/ordered_segment_planner.py

PYTHONPATH=eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts:$PYTHONPATH \
python3 -c "<smoke test for plan_ordered_segment unsupported-type path>"

/bin/bash -lc "env -i HOME=\"$HOME\" TERM=\"${TERM:-xterm}\" PATH=\"/usr/bin:/bin:/usr/sbin:/sbin\" bash --noprofile --norc -lc 'source /opt/ros/rolling/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws/eRob_moveit; colcon build --packages-select erob_moveit_runtime'"
```

Verification notes:

- `py_compile` passed.
- Smoke result: `Unsupported ordered-chain segment type: 'bad' bad`.
- `colcon build --packages-select erob_moveit_runtime` passed.
- The build still prints the existing
  `/opt/ros/rolling/setup.bash: No such file or directory` warning before
  sourcing the installed workspace; it does not fail the build.

Next pickup point:

1. Runtime-check ordered-chain execution after Slices 50-52 with fake system.
   Expected result: same ordered segment planning, blend validation,
   planning-worker and execution telemetry as the Slices 47-49 validation.
2. Continue extracting `_execute_ordered_motion_chain_pipelined(...)` by moving
   planner thread submit/wait orchestration or ordered planned-segment execution
   callback construction out of the backend.
3. Keep explicit unwind target decisions and nested legacy unwind removal
   deferred until real-robot validation.

### 2026-08-10 - Slice 53: ordered planned-segment executor callback extraction

Status: completed and compile/smoke/build checked. Runtime behavior is intended
to remain unchanged.

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/execution/ordered_execution.py
```

What was done:

- Added `OrderedPlannedSegmentExecutorConfig`.
- Added `build_ordered_planned_segment_executor(...)` in
  `motion/execution/ordered_execution.py`.
- Moved the backend's nested `_execute_planned_segment(...)` callback
  construction into the execution helper module.
- Backend now only supplies:
  - the ordered execution hook bundle
  - the scheduler bridge
  - timeout/default velocity/default acceleration config values
  - the live runtime unwind callback
  - the clock callback

Issues / notes found:

- `moveit_robot_backend.py` is now 1653 lines, down from 1667 after Slice 52.
- The extracted callback still executes the existing dictionary-shaped planned
  segments. Typed `PlannedTrajectory` integration remains a later architecture
  step.
- Runtime validation after Slices 50-53 is still needed.
- The planned-segment execution helper depends on the scheduler bridge being
  constructed first; keep this ordering if the remaining pipeline wrapper is
  extracted.

Verification:

```text
python3 -m py_compile \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/execution/ordered_execution.py

PYTHONPATH=eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts:$PYTHONPATH \
python3 - <<'PY'
<smoke test: build_ordered_planned_segment_executor executes a blend_consumed
segment and marks scheduler executing/finished>
PY

/bin/bash -lc "env -i HOME=\"$HOME\" TERM=\"${TERM:-xterm}\" PATH=\"/usr/bin:/bin:/usr/sbin:/sbin\" bash --noprofile --norc -lc 'source /opt/ros/rolling/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws/eRob_moveit; colcon build --packages-select erob_moveit_runtime'"
```

Verification notes:

- `py_compile` passed.
- Smoke result:
  `result 0`, scheduler events
  `[('executing', 0, 'blend_consumed'), ('finished', 0, 0)]`.
- `colcon build --packages-select erob_moveit_runtime` passed.
- The build still prints the existing
  `/opt/ros/rolling/setup.bash: No such file or directory` warning before
  sourcing the installed workspace; it does not fail the build.

Next pickup point:

1. Runtime-check ordered-chain execution after Slices 50-53 with fake system.
   Expected result: same ordered initial-state, blend validation,
   planning-worker, planned-segment execution and controller telemetry as the
   Slices 47-49 validation.
2. Continue extracting `_execute_ordered_motion_chain_pipelined(...)` by moving
   planner thread submit/wait orchestration out of the backend, or extract the
   direct-IK ordered unwind callback once the dependency boundary is clear.
3. Keep explicit unwind target decisions and nested legacy unwind removal
   deferred until real-robot validation.

### 2026-08-10 - Runtime validation after Slice 53

Status: completed from fake-system log review.

Log reviewed:

```text
/home/ilv/ros2_ws/eRob_moveit/src/eRob_ROS2_MoveIt/zeroerr/docs/temp_logs
mtime: 2026-08-10 13:43:22 +0300
lines: 1060
```

What was validated:

- Startup reached MoveIt readiness and helper readiness:
  - `You can start planning now!`
  - `IPP Helper fully initialized and ready`
  - `Ruckig Helper fully initialized and ready`
  - `Contour IK helper ready with local PlanningScene`
  - `PTP helper ready`
- Request 1 completed successfully:
  - `ordered_planning_worker_done request_id=1`
  - linear/blended/blend-consumed segment execution covered
  - `ordered_motion_chain_total result=0 elapsed_s=4.915`
  - `ordered_chain_done request_id=1 ... result=0`
- Request 2 completed successfully:
  - `ordered_planning_worker_done request_id=2`
  - blended, path, blend-consumed, and final live unwind/no-op execution covered
  - `ordered_motion_chain_total result=0 elapsed_s=25.684`
  - `ordered_chain_done request_id=2 ... result=0`
- Slice 53's extracted planned-segment executor callback is runtime-covered:
  - `ordered_segment_execute_start`
  - `ordered_controller_handoff_start`
  - `ordered_wait_execution_done`
  - `ordered_state_match_done`
  - `ordered_segment_execute_done`

Issues / notes found:

- No `ordered_planning_worker_error`, no rejected motion requests, no
  Traceback, and no ordered-chain failures were present in this log.
- Known non-blocking environment warnings remain:
  - early TF lookup before all frames are available
  - no octomap 3D sensor plugin configured
  - duplicate rosout publisher warnings from same-name MoveIt/helper nodes
- Ruckig again reported an implausible stretched output for one segment and
  fell back to the seeded trajectory. Execution still completed successfully.
  Treat this as the existing Ruckig fallback behavior, not a Slice 53
  regression.
- Final ordered live unwind was covered only as the no-op path:
  `Rotational-path unwind skipped - no unwind needed`. Keep explicit non-no-op
  unwind behavior deferred for real-robot validation as requested.

Next pickup point:

1. Treat Slices 50-53 as fake-system runtime validated.
2. Continue reducing `_execute_ordered_motion_chain_pipelined(...)` by moving
   planner thread submit/wait orchestration into a scheduling helper.
3. Keep direct-IK ordered unwind callback extraction for later unless its
   backend waypoint/tool/workobject dependencies are isolated cleanly.
4. Keep explicit unwind target behavior and nested legacy unwind removal
   deferred until real-robot testing.

### 2026-08-10 - Slice 54: ordered pipeline runner extraction

Status: completed and compile/smoke/build checked. Runtime behavior is intended
to remain unchanged.

Files added:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/ordered_pipeline_runner.py
```

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/__init__.py
```

What was done:

- Added `OrderedPipelineRunnerConfig`.
- Added `run_ordered_planning_and_execution(...)`.
- Moved the ordered-chain `ThreadPoolExecutor`, planning stop event,
  planner-worker submission, planned-sequence consumption, and executor
  shutdown wrapper out of `_execute_ordered_motion_chain_pipelined(...)`.
- Backend now provides a planning-worker factory and sequence hooks, while the
  scheduling helper owns the planning thread lifetime.

Issues / notes found:

- `moveit_robot_backend.py` is now 1652 lines, down from 1653 after Slice 53.
- The line-count drop is small because this slice moved orchestration ownership
  rather than large planning logic. The important architectural change is that
  backend no longer directly imports or manages `ThreadPoolExecutor`/`Event`
  for ordered-chain execution.
- Runtime validation after Slice 54 is still needed.

Verification:

```text
python3 -m py_compile \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/ordered_pipeline_runner.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/__init__.py

PYTHONPATH=eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts:$PYTHONPATH \
python3 - <<'PY'
<smoke test: run_ordered_planning_and_execution submits a fake planning worker,
consumes one planned segment, executes it, and returns result 0>
PY

/bin/bash -lc "env -i HOME=\"$HOME\" TERM=\"${TERM:-xterm}\" PATH=\"/usr/bin:/bin:/usr/sbin:/sbin\" bash --noprofile --norc -lc 'source /opt/ros/rolling/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws/eRob_moveit; colcon build --packages-select erob_moveit_runtime'"
```

Verification notes:

- `py_compile` passed.
- Smoke result:
  `result 0`, with timing events for `ordered_planner_submit_start`,
  `ordered_planner_submit_done`, `ordered_plan_wait_start`,
  `ordered_plan_ready`, and execution of `runner_smoke`.
- `colcon build --packages-select erob_moveit_runtime` passed.
- The build still prints the existing
  `/opt/ros/rolling/setup.bash: No such file or directory` warning before
  sourcing the installed workspace; it does not fail the build.

Next pickup point:

1. Runtime-check ordered-chain execution after Slice 54 with fake system.
   Expected result: same ordered planner submit/wait, planned segment execution,
   controller handoff and ordered-chain completion telemetry as the Slice 53
   runtime validation.
2. Continue reducing `_execute_ordered_motion_chain_pipelined(...)` by
   extracting direct-IK ordered unwind planning dependencies, or by introducing
   a single setup object for ordered planner/execution hooks if that boundary is
   cleaner.
3. Keep explicit unwind target behavior and nested legacy unwind removal
   deferred until real-robot testing.

### 2026-08-10 - Slice 55: ordered planning worker factory extraction

Status: completed and compile/smoke/build checked. Runtime behavior is intended
to remain unchanged.

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/__init__.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/ordered_planning_worker.py
```

What was done:

- Added `build_ordered_planning_worker_factory(...)`.
- Moved the backend's nested stop-event-aware planning worker factory into the
  ordered planning worker module.
- Backend now builds `OrderedPlanningWorkerHooks` and passes the resulting
  factory to `run_ordered_planning_and_execution(...)`.

Issues / notes found:

- This continues the scheduling ownership split started in Slice 54.
- Runtime validation after Slices 54-55 is still needed.

Verification:

```text
python3 -m py_compile \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/ordered_planning_worker.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/__init__.py

PYTHONPATH=eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts:$PYTHONPATH \
python3 - <<'PY'
<smoke test: build_ordered_planning_worker_factory builds a worker that plans
one fake segment, publishes done, and records worker start/queued/done timing>
PY
```

Verification notes:

- `py_compile` passed.
- Smoke result: `done True`, events included
  `ordered_planning_worker_start`, `ordered_segment_queued`,
  `ordered_planning_worker_done`.

### 2026-08-10 - Slice 56: ordered segment planner callback extraction

Status: completed and compile/smoke/build checked. Runtime behavior is intended
to remain unchanged.

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/planning/ordered_segment_planner.py
```

What was done:

- Added `build_ordered_segment_planner_callback(...)`.
- Moved the backend's nested `_plan_ordered_segment(...)` wrapper into the
  ordered segment planner module.
- Backend now creates `OrderedSegmentPlannerHooks` and receives a callback in
  the legacy worker shape.

Issues / notes found:

- `moveit_robot_backend.py` is now 1639 lines, down from 1652 after Slice 54.
- The direct-IK ordered unwind planning callback still remains in the backend
  because it depends on backend rotational waypoint generation and local
  tool/workobject context.
- Runtime validation after Slices 54-56 is still needed.

Verification:

```text
python3 -m py_compile \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/planning/ordered_segment_planner.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/ordered_planning_worker.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/__init__.py

PYTHONPATH=eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts:$PYTHONPATH \
python3 - <<'PY'
<smoke test: build_ordered_segment_planner_callback forwards index and
defer_optimization to plan_ordered_segment>
PY

/bin/bash -lc "env -i HOME=\"$HOME\" TERM=\"${TERM:-xterm}\" PATH=\"/usr/bin:/bin:/usr/sbin:/sbin\" bash --noprofile --norc -lc 'source /opt/ros/rolling/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws/eRob_moveit; colcon build --packages-select erob_moveit_runtime'"
```

Verification notes:

- `py_compile` passed.
- Segment planner callback smoke result:
  `result {'ok': True, 'index': 3}`, `forwarded 3 True`.
- Planning worker factory smoke result from Slice 55 remained passing.
- `colcon build --packages-select erob_moveit_runtime` passed.
- The build still prints the existing
  `/opt/ros/rolling/setup.bash: No such file or directory` warning before
  sourcing the installed workspace; it does not fail the build.

Next pickup point:

1. Runtime-check ordered-chain execution after Slices 54-56 with fake system.
   Expected result: same planner submit/wait, planning worker, planned segment
   execution, controller handoff, and ordered-chain completion telemetry as the
   Slice 53 runtime validation.
2. Continue reducing `_execute_ordered_motion_chain_pipelined(...)` by
   extracting direct-IK ordered unwind planning dependencies, or by introducing
   a single setup object for ordered pipeline dependencies if that boundary is
   cleaner.
3. Keep explicit unwind target behavior and nested legacy unwind removal
   deferred until real-robot testing.

### 2026-08-10 - Slice 57: ordered unwind direct-IK planner callback extraction

Status: completed and compile/smoke/build checked. Runtime behavior is intended
to remain unchanged.

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/planning/ordered_unwind_planner.py
```

What was done:

- Added `build_ordered_unwind_direct_ik_planner(...)`.
- Moved the ordered-chain direct-IK unwind planning callback body out of the
  backend and into `ordered_unwind_planner.py`.
- Backend now injects only:
  - planning node
  - config object
  - active tool transform
  - backend rotational waypoint generator
  - optimizer callback
- Backend no longer imports direct contour IK or pose-list helpers in the
  ordered-chain pipeline section.

Issues / notes found:

- The backend rotational waypoint generator remains backend-owned because it is
  also used by standalone unwind/jog paths. This slice isolates it as an
  injected dependency instead of changing behavior.
- Runtime validation after Slice 57 is still needed.

Verification:

```text
python3 -m py_compile \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/planning/ordered_unwind_planner.py

PYTHONPATH=eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts:$PYTHONPATH \
python3 - <<'PY'
<smoke test: build_ordered_unwind_direct_ik_planner uses fake waypoint,
pose-list, IK, logging, optimization dependencies and returns the forced
Joint_6 branch>
PY
```

Verification notes:

- `py_compile` passed.
- Smoke result: `points 2`, `end 1.0`, `logged 1`.

### 2026-08-10 - Slice 58: ordered scheduler runtime setup extraction

Status: completed and compile/smoke/build checked. Runtime behavior is intended
to remain unchanged.

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/__init__.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/ordered_scheduler_bridge.py
```

What was done:

- Added `OrderedSchedulerRuntime`.
- Added `build_ordered_scheduler_runtime(...)`.
- Moved ordered scheduler bridge creation, default segment-state creation,
  scheduler update publication, and ready-planned publication callback setup
  out of the backend.
- Backend now uses `scheduler_runtime.bridge`,
  `scheduler_runtime.publish_scheduler_updates`, and
  `scheduler_runtime.publish_planned`.

Issues / notes found:

- `moveit_robot_backend.py` is now 1577 lines, down from 1639 after Slice 56.
- Runtime validation after Slices 54-58 is still needed before starting C++
  work.
- Explicit unwind target behavior and nested legacy unwind removal remain
  deferred until real-robot testing as requested.

Verification:

```text
python3 -m py_compile \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/planning/ordered_unwind_planner.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/ordered_scheduler_bridge.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/__init__.py

PYTHONPATH=eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts:$PYTHONPATH \
python3 - <<'PY'
<smoke test: build_ordered_scheduler_runtime creates a bridge and publishing a
planned segment emits scheduler_segment_states with state READY>
PY

/bin/bash -lc "env -i HOME=\"$HOME\" TERM=\"${TERM:-xterm}\" PATH=\"/usr/bin:/bin:/usr/sbin:/sbin\" bash --noprofile --norc -lc 'source /opt/ros/rolling/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws/eRob_moveit; colcon build --packages-select erob_moveit_runtime'"
```

Verification notes:

- `py_compile` passed.
- Scheduler runtime smoke result:
  `bridge OrderedSchedulerBridge`, `updates 1 True`, `state0 READY`.
- `colcon build --packages-select erob_moveit_runtime` passed.
- The build still prints the existing
  `/opt/ros/rolling/setup.bash: No such file or directory` warning before
  sourcing the installed workspace; it does not fail the build.

Next pickup point:

1. Runtime-check ordered-chain execution after Slices 54-58 with fake system.
   Expected result: same startup readiness, scheduler state updates, planner
   submit/wait, planning worker, planned segment execution, controller handoff,
   state match, final no-op unwind, and ordered-chain completion telemetry as
   the Slice 53 runtime validation.
2. If runtime validation passes, the Python ordered-chain refactor is ready to
   pause for the first C++ preparation slice.
3. First C++ preparation slice should inspect existing CMake/package/service
   layout and add the shared C++ trajectory validation boundary without routing
   production motion through it yet.
4. Keep explicit unwind target behavior and nested legacy unwind removal
   deferred until real-robot testing.

### 2026-08-10 - Runtime validation after Slices 54-58

Status: completed from fake-system log review.

Log reviewed:

```text
/home/ilv/ros2_ws/eRob_moveit/src/eRob_ROS2_MoveIt/zeroerr/docs/temp_logs
mtime: 2026-08-10 14:01:43 +0300
lines: 1072
```

What was validated:

- Startup reached MoveIt readiness and helper readiness:
  - `You can start planning now!`
  - `IPP Helper fully initialized and ready`
  - `Ruckig Helper fully initialized and ready`
  - `Contour IK helper ready with local PlanningScene`
  - `PTP helper ready`
- Slice 54 pipeline runner path is runtime-covered:
  - `ordered_planner_submit_start`
  - `ordered_planner_submit_done`
  - `ordered_plan_ready`
- Slice 55 planning worker factory path is runtime-covered:
  - `ordered_planning_worker_done request_id=1`
  - `ordered_planning_worker_done request_id=2`
- Slice 56 planner callback path is runtime-covered by normal ordered segment
  planning and execution across LIN, blended, path, blend-consumed and unwind
  segment types.
- Slice 57 ordered unwind direct-IK callback extraction is covered for final
  live-unwind planning selection; the actual final unwind execution was no-op.
- Slice 58 scheduler runtime setup is runtime-covered by ready/preplanned
  segment status publication and ordered execution status transitions.
- Request 1 completed successfully:
  - `ordered_motion_chain_total result=0 elapsed_s=4.920`
  - `ordered_chain_done request_id=1 ... result=0`
- Request 2 completed successfully:
  - `ordered_motion_chain_total result=0 elapsed_s=25.661`
  - `ordered_chain_done request_id=2 ... result=0`

Issues / notes found:

- No `ordered_planning_worker_error`, no rejected motion requests, no
  Traceback, and no ordered-chain failures were present in this log.
- Known non-blocking environment warnings remain:
  - no octomap 3D sensor plugin configured
  - duplicate rosout publisher warnings from same-name MoveIt/helper nodes
- Ruckig again reported an implausible stretched output for one segment and
  fell back to the seeded trajectory. Execution still completed successfully.
  Treat this as existing Ruckig fallback behavior, not a Slice 54-58
  regression.
- Final ordered live unwind was covered only as the no-op path:
  `Rotational-path unwind skipped - no unwind needed`. Keep explicit non-no-op
  unwind behavior deferred for real-robot validation as requested.
- `moveit_robot_backend.py` remains 1577 lines. The largest remaining
  responsibilities are public robot API wrappers, standalone unwind/jog/control
  methods, current state accessors, and hardware/service guard glue. The
  ordered-chain scheduler/planner/execution internals have been split out.

Next pickup point:

1. Python ordered-chain refactor is ready to pause for the first C++
   preparation slice.
2. Before C++ routing, inspect existing `CMakeLists.txt`, package manifests,
   generated service layout, helper node patterns, and C++ validation utilities.
3. Add the shared C++ trajectory validation boundary first without routing
   production motion through it yet.
4. Keep explicit unwind target behavior and nested legacy unwind removal
   deferred until real-robot testing.

### 2026-08-10 - Slice 59: shared C++ trajectory validation boundary

Status: completed and build/smoke checked. Production motion routing is
intentionally unchanged.

Files added:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/include/erob_moveit_runtime/trajectory_validation.hpp
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/src_cpp/trajectory_validation.cpp
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/src_cpp/trajectory_validation_smoke.cpp
```

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/CMakeLists.txt
```

What was done:

- Inspected the existing `erob_moveit_runtime` CMake/package/service layout.
- Added a reusable C++ `trajectory_validation` library target.
- Added public validation API:
  - `TrajectoryValidationOptions`
  - `TrajectoryValidationResult`
  - `durationToSeconds(...)`
  - `validateJointTrajectory(...)`
- Added structural trajectory checks for:
  - non-empty joint names
  - non-empty point list unless explicitly allowed
  - point `positions` size matching joint count
  - optional `velocities` and `accelerations` size matching joint count
  - finite position/velocity/acceleration/effort values
  - valid and strictly increasing `time_from_start`
- Added `trajectory_validation_smoke` executable to exercise a valid trajectory
  and a deliberately invalid timestamp sequence.
- Exported the include directory and library target for future C++ helper use.

Issues / notes found:

- Existing C++ helpers are service-backed nodes under `src_cpp/`; services are
  generated in the same package through `rosidl_generate_interfaces(...)`.
- Existing helper nodes already maintain local PlanningScene state for collision
  checking, but the first validation boundary is intentionally shape/time/value
  only. PlanningScene collision validation can be layered on top in a later
  helper/service slice.
- This slice does not add a ROS service and does not route any production
  trajectory through C++ validation yet.

Verification:

```text
/bin/bash -lc "env -i HOME=\"$HOME\" TERM=\"${TERM:-xterm}\" PATH=\"/usr/bin:/bin:/usr/sbin:/sbin\" bash --noprofile --norc -lc 'source /opt/ros/rolling/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws/eRob_moveit; colcon build --packages-select erob_moveit_runtime'"

cd /home/ilv/ros2_ws/eRob_moveit
./build/erob_moveit_runtime/trajectory_validation_smoke
```

Verification notes:

- `colcon build --packages-select erob_moveit_runtime` passed.
- The build still prints the existing
  `/opt/ros/rolling/setup.bash: No such file or directory` warning before
  sourcing the installed workspace; it does not fail the build.
- CMake emitted an existing ROS Jazzy `tl_expected` deprecation warning through
  MoveIt dependency discovery; it does not fail the build.
- Smoke result:
  `trajectory_validation_smoke ok: point 1 time_from_start is not strictly increasing`.

Next pickup point:

1. Add the service contract for linked LIN only after confirming the desired
   request/response fields.
2. Candidate next service: `ComputeLinkedLin.srv` carrying ordered LIN poses,
   seed state, group/link names, tool/workobject context, velocity/acceleration
   scales, and validation limits.
3. The linked-LIN helper should use the new `trajectory_validation` library
   before returning any trajectory to Python.
4. Keep production routing unchanged until the helper builds and has a smoke
   client path.

### 2026-08-10 - Slice 60: linked-LIN service contract and helper boundary

Status: completed and build/interface/startup checked. Production motion
routing is intentionally unchanged.

Files added:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/srv/ComputeLinkedLin.srv
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/src_cpp/linked_lin_helper_node.cpp
```

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/CMakeLists.txt
```

What was done:

- Added `ComputeLinkedLin.srv`.
- Request fields cover:
  - seed joint state
  - linked Cartesian pose list
  - MoveIt group/link/reference-frame names
  - tool and workobject transforms
  - velocity/acceleration scales
  - Cartesian step/jump/collision settings
  - FK/joint-span/joint-step validation limits
  - full-turn joint policy limits
- Response fields cover:
  - success/error/message
  - `moveit_msgs/RobotTrajectory`
  - requested/solved pose counts and failed index
  - validation maxima
  - planning/validation/total timings
- Added `linked_lin_helper` C++ node skeleton.
- The helper advertises `/compute_linked_lin` only after `initialize()`.
- Current helper response is intentionally controlled failure:
  `ERROR_NOT_IMPLEMENTED=-100`.
- Helper links the shared `trajectory_validation` library but does not yet
  perform linked-LIN planning.

Issues / notes found:

- Adding a service regenerated package interfaces and made the build take
  longer than previous C++ slices.
- Running the installed helper directly without sourcing the fresh overlay
  loaded stale generated typesupport from `/home/ilv/ros2_ws/install/...` and
  produced a missing `ComputeLinkedLin` symbol. Running through the sourced
  `/home/ilv/ros2_ws/eRob_moveit/install/setup.bash` overlay resolved that.
- The tool sandbox cannot open normal ROS log/network resources freely:
  - first direct run failed writing `/home/ilv/.ros/log`
  - rerun with `ROS_LOG_DIR=/tmp` started successfully
  - FastDDS emitted sandbox socket/getifaddrs permission errors, but the helper
    still reached service creation and ready logs
- No production launch files were changed. The helper is build/install
  available only.

Verification:

```text
/bin/bash -lc "env -i HOME=\"$HOME\" TERM=\"${TERM:-xterm}\" PATH=\"/usr/bin:/bin:/usr/sbin:/sbin\" bash --noprofile --norc -lc 'source /opt/ros/rolling/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws/eRob_moveit; colcon build --packages-select erob_moveit_runtime'"

cd /home/ilv/ros2_ws/eRob_moveit
./build/erob_moveit_runtime/trajectory_validation_smoke

/bin/bash -lc "source /opt/ros/jazzy/setup.bash; source /home/ilv/ros2_ws/eRob_moveit/install/setup.bash; ros2 interface show erob_moveit_runtime/srv/ComputeLinkedLin"

/bin/bash -lc "source /opt/ros/jazzy/setup.bash; source /home/ilv/ros2_ws/eRob_moveit/install/setup.bash; cd /home/ilv/ros2_ws/eRob_moveit; ROS_LOG_DIR=/tmp timeout 3s ros2 run erob_moveit_runtime linked_lin_helper --ros-args -r __node:=linked_lin_helper_smoke"
```

Verification notes:

- `colcon build --packages-select erob_moveit_runtime` passed.
- `trajectory_validation_smoke` passed.
- `ros2 interface show erob_moveit_runtime/srv/ComputeLinkedLin` passed and
  displayed the generated service contract.
- `linked_lin_helper` startup smoke reached:
  - `Linked LIN helper starting...`
  - `Service '/compute_linked_lin' created`
  - `Linked LIN helper ready`
- The startup smoke exits with timeout code `124`, which is expected because
  the node stays alive until interrupted.

Next pickup point:

1. Add the actual linked-LIN helper implementation behind `/compute_linked_lin`.
2. Reuse existing C++ helper patterns:
   local RobotModel/PlanningSceneMonitor initialization, service advertisement
   only after ready, and structured timing/diagnostics in the response.
3. Use `trajectory_validation` before returning any successful trajectory.
4. Keep production Python routing unchanged until the helper has a real
   successful smoke/client path.

### 2026-08-10 - Slice 61: first real linked-LIN C++ helper implementation

Status: completed and build checked. Production motion routing is still
intentionally unchanged.

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/CMakeLists.txt
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/src_cpp/linked_lin_helper_node.cpp
```

What was done:

- Replaced the Slice 60 controlled-failure skeleton with a first real
  `/compute_linked_lin` implementation.
- The helper now loads `RobotModelLoader`, creates a local
  `PlanningSceneMonitor`, starts scene/world monitors, requests
  `/get_planning_scene`, and advertises the service only after initialization.
- Implemented request validation for non-empty poses, MoveIt group name, link
  name, and seed joint state coverage.
- Implemented tool/workobject transform handling:
  request poses are treated as active TCP targets; the helper converts them to
  the requested IK link pose using `target * inverse(tool_transform)`.
- Implemented sequential IK solving with nearest-equivalent joint unwrapping
  against the previous point to preserve joint continuity.
- Implemented optional joint-step, joint-span, endpoint-delta, FK-position,
  FK-orientation, and collision checks.
- Successful responses now return `moveit_msgs/RobotTrajectory`, solved pose
  counts, validation maxima, and planning/validation/total timings.
- Successful trajectories are passed through the shared
  `trajectory_validation` library before returning to Python.
- Updated `linked_lin_helper` CMake dependencies from skeleton-only ROS
  message dependencies to the same MoveIt planning dependencies used by the
  existing PTP/contour helpers.

Issues / limitations found:

- The helper is not yet wired into Python, launch files, startup readiness, or
  production ordered-chain planning. This slice is only the C++ service-side
  implementation.
- Standalone `ros2 run erob_moveit_runtime linked_lin_helper` now requires the
  usual launch-provided MoveIt parameters, especially `robot_description`.
  Without the full launch context, the process can start and then wait/fail
  during model initialization instead of reaching ready immediately.
- `reference_frame`, `jump_threshold`, velocity/acceleration scaling, and
  explicit full-turn policy fields are present in the service contract but are
  not fully enforced by this first implementation.
- Timing in the returned trajectory is intentionally simple and monotonic.
  It is suitable as a functional boundary check, not final execution-quality
  parameterization.
- No changes were made to the explicit Joint_6 unwind path. Keep the current
  fallback/reference behavior in place until real-robot testing resumes.

Verification:

```text
/bin/bash -lc "env -i HOME=\"$HOME\" TERM=\"${TERM:-xterm}\" PATH=\"/usr/bin:/bin:/usr/sbin:/sbin\" bash --noprofile --norc -lc 'source /opt/ros/rolling/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws/eRob_moveit; colcon build --packages-select erob_moveit_runtime'"

/home/ilv/ros2_ws/eRob_moveit/build/erob_moveit_runtime/trajectory_validation_smoke

/bin/bash -lc "source /opt/ros/jazzy/setup.bash; source /home/ilv/ros2_ws/eRob_moveit/install/setup.bash; cd /home/ilv/ros2_ws/eRob_moveit; ROS_LOG_DIR=/tmp timeout 5s ros2 run erob_moveit_runtime linked_lin_helper --ros-args -r __node:=linked_lin_helper_smoke"
```

Verification notes:

- `colcon build --packages-select erob_moveit_runtime` passed.
- Build stderr only contained the existing ROS Jazzy `tl_expected` deprecation
  warning.
- `trajectory_validation_smoke` passed:
  `trajectory_validation_smoke ok: point 1 time_from_start is not strictly increasing`.
- Standalone helper smoke reached `Linked LIN helper starting...` but timed out
  after 5s while waiting for normal launch-provided MoveIt model context. That
  is expected for the real implementation outside the ZeroErr launch stack.
- FastDDS still prints sandbox network permission noise during local tool-run
  smoke checks; this is unrelated to package compilation.

Next pickup point:

1. Add Python client plumbing for `/compute_linked_lin` without routing
   production traffic yet.
2. Add the linked-LIN helper to launch/startup readiness so the GUI cannot see
   ready before this service is usable once production routing depends on it.
3. Add a full-stack fake-system smoke where the helper is launched with
   `robot_description` and a small service request is issued.
4. After that passes, route one narrow contiguous LIN group through the helper
   behind a config flag and compare logs against the current Python planner.

### 2026-08-10 - Slice 62: linked-LIN Python client and readiness wiring

Status: completed and build/import checked. Production motion routing is still
unchanged.

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/config.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/planning/planner_support_service.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/planning/planner_context.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/robot_controller.py
eRob_moveit/src/eRob_ROS2_MoveIt/zeroerr/launch/full_stack.launch.py
eRob_moveit/src/eRob_ROS2_MoveIt/zeroerr/launch/ethercat_only.launch.py
eRob_moveit/src/eRob_ROS2_MoveIt/zeroerr/scripts/EtherCatStart.sh
eRob_moveit/src/eRob_ROS2_MoveIt/fairino5_v6_moveit2_config/launch/full_stack.launch.py
```

What was done:

- Added `SERVICE_LINKED_LIN=/compute_linked_lin` to runtime config defaults.
- Added lazy `ComputeLinkedLin` client creation in `PlannerSupportService`.
- Exposed `get_linked_lin_client()` through `PlannerContext` and
  `RobotController`.
- Added linked-LIN service availability to `RobotController`
  `get_motion_stack_fault_reason()`.
- Added `linked_lin_helper` launch nodes to:
  - ZeroErr full stack launch
  - ZeroErr EtherCAT-only launch
  - Fairino MoveIt config full stack launch
- Added `ptp_helper` and `linked_lin_helper` to the ZeroErr non-RT process
  pinning list in `EtherCatStart.sh`.

Important readiness note:

- This follows the rule that every new runtime service/helper must be included
  in state/readiness logic before any production code depends on it.
- From this slice onward, ZeroErr readiness will remain false until
  `/compute_linked_lin` is available. That is intentional because the helper is
  now launched with the stack.
- If a launch path starts `zeroerr_runtime` but does not launch
  `linked_lin_helper`, the GUI will stay in startup/warming state with
  `Linked LIN helper service not available`.

Verification:

```text
python3 -m py_compile \
  erob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/config.py \
  erob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/planning/planner_support_service.py \
  erob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/planning/planner_context.py \
  erob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/robot_controller.py \
  erob_moveit/src/eRob_ROS2_MoveIt/zeroerr/launch/full_stack.launch.py \
  erob_moveit/src/eRob_ROS2_MoveIt/zeroerr/launch/ethercat_only.launch.py \
  erob_moveit/src/eRob_ROS2_MoveIt/fairino5_v6_moveit2_config/launch/full_stack.launch.py

bash -n eRob_moveit/src/eRob_ROS2_MoveIt/zeroerr/scripts/EtherCatStart.sh

/bin/bash -lc "env -i HOME=\"$HOME\" TERM=\"${TERM:-xterm}\" PATH=\"/usr/bin:/bin:/usr/sbin:/sbin\" bash --noprofile --norc -lc 'source /opt/ros/rolling/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws/eRob_moveit; colcon build --packages-select erob_moveit_runtime zeroerr'"

/bin/bash -lc "source /opt/ros/jazzy/setup.bash; source /home/ilv/ros2_ws/eRob_moveit/install/setup.bash; python3 - <<'PY'
from erob_moveit_runtime.srv import ComputeLinkedLin
req = ComputeLinkedLin.Request()
print(type(req).__name__, hasattr(req, 'poses'), hasattr(req, 'max_joint_step_rad'))
PY"
```

Verification notes:

- Python compile checks passed.
- `bash -n` for `EtherCatStart.sh` passed.
- `colcon build --packages-select erob_moveit_runtime zeroerr` passed.
- Installed overlay import check printed:
  `ComputeLinkedLin_Request True True`.

Next pickup point:

1. Run a full fake-system launch and confirm the logs contain:
   - `linked_lin_helper`: `Service '/compute_linked_lin' created`
   - `linked_lin_helper`: `Linked LIN helper ready`
   - `zeroerr_runtime` health reaches ready after all helper services are up
2. Add a small explicit service-request smoke under the launch stack.
3. Only after that, add a config-flagged Python planner client function that
   can call `/compute_linked_lin` for one compatible linked-LIN group while
   falling back to current Python planning.

### 2026-08-10 - Slice 63: linked-LIN Python request helper boundary

Status: completed and build/import checked. Production motion routing is still
unchanged.

Fresh user runtime confirmation:

```text
[linked_lin_helper]: Service '/compute_linked_lin' created
[linked_lin_helper]: Linked LIN helper ready
```

The `linked_lin_helper.moveit.ros.occupancy_map_monitor` messages about missing
octomap 3D sensor plugins match the existing MoveIt helper warning pattern and
are not treated as a startup failure.

Files added:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/planning/linked_lin_client.py
```

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/config.py
```

What was done:

- Added a self-contained Python boundary module for `/compute_linked_lin`.
- Added:
  - `LinkedLinReport`
  - `LinkedLinPlanningResult`
  - `request_linked_lin_trajectory(...)`
- The helper function:
  - is disabled by default behind `LINKED_LIN_HELPER_ENABLED`
  - returns `None` only when disabled
  - returns explicit failed results for import, availability, or helper
    rejection problems once enabled
  - builds `ComputeLinkedLin.Request`
  - maps current/seed joint state into configured joint order
  - fills group/link/reference-frame, tool/workobject transforms, scaling,
    collision, FK, joint-continuity and full-turn policy fields
  - converts the service response into a structured report
  - logs accepted/rejected linked-LIN results
- Added conservative linked-LIN defaults to `config.py`:
  - `LINKED_LIN_HELPER_ENABLED: False`
  - service timeout
  - Cartesian step/jump settings
  - FK tolerances
  - joint-step/span/endpoint limits
  - full-turn joint names and limits

Issues / notes found:

- Importing installed runtime modules outside launch requires
  `EROB_CONFIG_PACKAGE` because `config.py` now intentionally requires a
  robot-specific runtime config package.
- This slice does not route ordered-chain LIN groups through the helper. It
  only creates the safe Python call boundary for the next slice.
- User direction: do not add linked-LIN fallback routing once this path is
  enabled, because fallback would mask regressions. A disabled feature flag may
  preserve the current planner, but enabled linked-LIN failures must be visible.
- The new module duplicates a few tiny joint-state/future-wait utilities rather
  than importing private functions from `direct_contour_ik.py`; this keeps the
  linked-LIN boundary independent for later extraction.

Verification:

```text
python3 -m py_compile \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/planning/linked_lin_client.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/config.py

/bin/bash -lc "env -i HOME=\"$HOME\" TERM=\"${TERM:-xterm}\" PATH=\"/usr/bin:/bin:/usr/sbin:/sbin\" bash --noprofile --norc -lc 'source /opt/ros/rolling/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws/eRob_moveit; colcon build --packages-select erob_moveit_runtime zeroerr'"

/bin/bash -lc "source /opt/ros/jazzy/setup.bash; source /home/ilv/ros2_ws/eRob_moveit/install/setup.bash; EROB_CONFIG_PACKAGE=zeroerr python3 - <<'PY'
import sys
sys.path.insert(0, '/home/ilv/ros2_ws/eRob_moveit/install/erob_moveit_runtime/lib/erob_moveit_runtime')
from motion.planning.linked_lin_client import LinkedLinReport, request_linked_lin_trajectory
from erob_moveit_runtime.srv import ComputeLinkedLin
print(LinkedLinReport().ok, callable(request_linked_lin_trajectory), ComputeLinkedLin.Request.__name__)
PY"
```

Verification notes:

- Python compile passed.
- `colcon build --packages-select erob_moveit_runtime zeroerr` passed.
- Installed import check passed with:
  `True True ComputeLinkedLin_Request`.

Next pickup point:

1. Add a narrow launch-stack service smoke that calls
   `request_linked_lin_trajectory(...)` with `LINKED_LIN_HELPER_ENABLED=True`
   and a tiny safe pose list.
2. Compare the C++ helper response trajectory shape and timing against the
   current Python LIN planner for one known fake-system motion.
3. Route one compatible linked-LIN group behind a config flag, preserving
   the current Python planner only when the linked-LIN feature flag is disabled.
   When the flag is enabled, unavailable/rejected helper responses must fail
   loudly instead of falling back.

### 2026-08-10 - Slice 64: route ordered LIN blend groups through linked-LIN

Status: completed and build checked. Ready for fake-system validation.

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/execution/ordered_execution.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/planning/linked_lin_client.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/ordered_planning_worker.py
eRob_moveit/src/eRob_ROS2_MoveIt/zeroerr/config/runtime.yaml
```

What was done:

- Wired continuous ordered LIN blend groups through `/compute_linked_lin` when
  `LINKED_LIN_HELPER_ENABLED=True`.
- Enabled `LINKED_LIN_HELPER_ENABLED: true` in ZeroErr runtime config so the
  next ZeroErr fake-system test exercises the C++ linked-LIN helper.
- Left the shared runtime default disabled so other robot configs only opt in
  deliberately.
- Extended ordered planning worker hooks with `apply_workobject`,
  `tool_transform`, and `user`, so linked-LIN request construction does not
  reach back into backend internals.
- Added TCP-target pose conversion for linked-LIN. This intentionally preserves
  the TCP pose and passes the tool transform to C++ instead of using the older
  Python helper that removes TCP offset before MoveIt Cartesian planning.
- Added `linked_lin` as a timed ordered-execution segment type.
- Linked-LIN helper outputs are still optimized through the configured
  trajectory optimizer before controller handoff.

No-fallback rule:

- When `LINKED_LIN_HELPER_ENABLED=False`, the existing Python ordered LIN/blend
  path is used.
- When `LINKED_LIN_HELPER_ENABLED=True`, eligible LIN blend groups must use
  linked-LIN.
- If the helper is unavailable, rejects the group, returns an empty trajectory,
  or optimization fails, planning fails visibly through
  `ordered_planning_worker_error`. There is no legacy planner fallback in the
  enabled path.

Current routing scope:

- Routed now: continuous ordered-chain LIN blend groups.
- Not routed yet: hard-stop LIN sequences, single `move_liner`, generic
  `execute_path`, and mixed LIN/PTP blend groups.
- This avoids changing hard-stop semantics while testing the linked-LIN helper
  on the scheduler group that was already reported as `linked_lin`.

Expected validation log markers:

```text
[LINKED_LIN] accepted ...
[ORDERED_CHAIN_TIMING] ordered_linked_lin_group_planned ...
[OrderedChain] Sending planned segment ... type=linked_lin ...
```

Verification:

```text
python3 -m py_compile \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/ordered_planning_worker.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/planning/linked_lin_client.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/backend/moveit_robot_backend.py

/bin/bash -lc "env -i HOME=\"$HOME\" TERM=\"${TERM:-xterm}\" PATH=\"/usr/bin:/bin:/usr/sbin:/sbin\" bash --noprofile --norc -lc 'source /opt/ros/rolling/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws/eRob_moveit; colcon build --packages-select erob_moveit_runtime zeroerr'"
```

Verification notes:

- Python compile passed.
- `colcon build --packages-select erob_moveit_runtime zeroerr` passed.

Next pickup point:

1. Run the fake-system ordered-chain test.
2. Inspect logs for the linked-LIN markers above and any
   `ordered_planning_worker_error`.
3. If linked-LIN fails, fix the helper/request semantics directly. Do not add
   fallback routing.
4. If linked-LIN passes, compare first-motion latency and group planning time
   against the previous Python blend-group baseline.

### 2026-08-10 - Slice 65: densify linked-LIN group requests after fake-system failure

Status: completed and build checked. Ready for another fake-system validation.

Fresh user log reviewed:

```text
[linked_lin_helper]: Service '/compute_linked_lin' created
[linked_lin_helper]: Linked LIN helper ready
[OrderedChain] MotionBatch adapter validated ... groups=1:linked_lin:6:hard,...
[linked_lin_helper]: Linked LIN rejected: joint step 1.49083 rad exceeds limit 0.08 at linked LIN pose 0
[ORDERED_CHAIN_TIMING] ordered_planning_worker_error ... linked-LIN helper rejected group 1-6 ...
```

Finding:

- Startup and service readiness were correct.
- The no-fallback behavior worked correctly: the ordered chain failed visibly
  when the helper rejected the linked-LIN group.
- The request was under-sampled. Python sent only the six LIN segment endpoints
  to `/compute_linked_lin`, so the first endpoint required a 1.49 rad joint
  jump from the seed state and correctly violated `max_joint_step_rad=0.08`.
- This was a Python request-construction issue, not a C++ helper readiness
  issue.

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/config.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/ordered_planning_worker.py
```

What was done:

- Added linked-LIN TCP waypoint densification before calling the helper.
- Each ordered LIN segment in the group is now interpolated from the current
  group pose to the segment target pose.
- Densification limits are explicit config defaults:
  - `LINKED_LIN_DENSIFY_MAX_TRANSLATION_MM: 8.0`
  - `LINKED_LIN_DENSIFY_MAX_ORIENTATION_DEG: 2.0`
- Interpolation preserves TCP target semantics:
  - translation is linearly interpolated
  - orientation uses SciPy `Slerp`
  - tool transform is still passed separately to C++
- Added `input_poses` timing metadata to
  `ordered_linked_lin_group_planned`.

Verification:

```text
python3 -m py_compile \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/ordered_planning_worker.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/config.py

/bin/bash -lc "cd /home/ilv/ros2_ws/eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts; EROB_CONFIG_PACKAGE=zeroerr PYTHONPATH=/home/ilv/ros2_ws/eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts:/home/ilv/ros2_ws/eRob_moveit/install/erob_moveit_runtime/lib/python3.12/site-packages:/opt/ros/jazzy/lib/python3.12/site-packages python3 - <<'PY'
from motion.scheduling.ordered_planning_worker import _tcp_poses_between
poses = _tcp_poses_between([0, 0, 0, 180, 0, 0], [100, 0, 0, 180, 0, 0])
print(len(poses), round(poses[-1].position.x, 4), round(poses[-1].orientation.w, 4))
PY"

/bin/bash -lc "env -i HOME=\"$HOME\" TERM=\"${TERM:-xterm}\" PATH=\"/usr/bin:/bin:/usr/sbin:/sbin\" bash --noprofile --norc -lc 'source /opt/ros/rolling/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws/eRob_moveit; colcon build --packages-select erob_moveit_runtime zeroerr'"
```

Verification notes:

- Python compile passed.
- Densifier smoke returned `13 0.1 0.0` for a 100 mm move, confirming
  roughly 8 mm spacing and correct target position in metres.
- `colcon build --packages-select erob_moveit_runtime zeroerr` passed.

Next pickup point:

1. Rerun fake-system ordered chain.
2. Confirm linked-LIN no longer rejects pose 0 for an endpoint-sized joint jump.
3. Inspect `ordered_linked_lin_group_planned input_poses=...` and helper timing.
4. If it still rejects for joint-step, tune densification or C++ IK continuity,
   not fallback routing.

### 2026-08-10 - Slice 66: remove exact linked-LIN local joint reversal before optimization

Status: completed, compile-checked and build-checked. Ready for another
fake-system validation.

Fresh user log reviewed:

```text
[linked_lin_helper]: Linked LIN success: poses=187 points=187 total=1.909s planning=1.909s validation=0.016s
[zeroerr_runtime]: [LINKED_LIN] accepted poses=187 solved=187 points=187 fk_max_mm=0.0000 ori_max_deg=0.0000 max_joint_step=0.0347 total_s=1.909
[TOTG_PATH_DIAG] detected 1 near-180deg joint-space reversal(s); worst_middle=56 worst_cos=-1.000000000
[ORDERED_CHAIN_TIMING] ordered_planning_worker_error ... error=Trajectory optimizer failed
```

Finding:

- Startup/readiness and linked-LIN service wiring were healthy.
- Slice 65 fixed the endpoint-sized jump rejection: the helper accepted all
  densified poses.
- The remaining failure is downstream of the helper. The linked-LIN trajectory
  contained an exact local out-and-back triplet where point `i - 1` and point
  `i + 1` are effectively identical, while point `i` steps away and returns.
  TOTG rejects that as a 180-degree joint-space reversal.
- This is not handled with fallback routing. Enabled linked-LIN failures remain
  visible.

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/config.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/ordered_planning_worker.py
```

What was done:

- Added strict linked-LIN reversal cleanup before trajectory optimization.
- The cleanup removes only a middle point when:
  - the previous and next points are within
    `LINKED_LIN_EXACT_REVERSAL_NEIGHBOR_TOL_RAD`
  - the middle point forms a near-perfect reversal using
    `LINKED_LIN_EXACT_REVERSAL_COS_THRESHOLD`
- Added explicit config defaults:
  - `LINKED_LIN_REMOVE_EXACT_REVERSALS_ENABLED: True`
  - `LINKED_LIN_EXACT_REVERSAL_NEIGHBOR_TOL_RAD: 1e-5`
  - `LINKED_LIN_EXACT_REVERSAL_COS_THRESHOLD: -0.999999`
- Added `linked_lin_reversal_repairs` metadata to the planned linked-LIN group
  and `reversal_repairs` timing metadata to
  `ordered_linked_lin_group_planned`.

Verification:

```text
python3 -m py_compile \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/ordered_planning_worker.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/config.py
```

Verification notes:

- Python compile passed.
- `colcon build --packages-select erob_moveit_runtime zeroerr` passed.
- The installed `ordered_planning_worker.py` contains
  `_remove_exact_joint_reversals(...)`.

Next pickup point:

1. Rerun the fake-system ordered chain.
2. Look for `[LINKED_LIN] Removed 1 exact local joint-space reversal point(s)
   before optimization`.
3. Confirm the linked-LIN group reaches `ordered_linked_lin_group_planned` and
   the full ordered chain either succeeds or exposes the next non-fallback
   failure.

### 2026-08-10 - Slice 67: remove linked-LIN duplicate points exposed by reversal repair

Status: completed, compile-checked and build-checked. Ready for another
fake-system validation.

Fresh user log reviewed:

```text
[LINKED_LIN] accepted poses=187 solved=187 points=187 fk_max_mm=0.0000 ori_max_deg=0.0000 max_joint_step=0.0347 total_s=1.911
[LINKED_LIN] Removed 1 exact local joint-space reversal point(s) before optimization
[TOTG_PATH_DIAG] points=186 joints=6 near_duplicate_segments=1
[TOTG_PATH_DIAG] near-duplicate segment indexes=55 epsilon=1.0e-08rad
[ipp_helper.moveit.core.time_optimal_trajectory_generation]: The path requires a 180 deg. turn, which is not supported by the current implementation.
[ORDERED_CHAIN_TIMING] ordered_planning_worker_error ... error=Trajectory optimizer failed
```

Finding:

- Slice 66 did run and removed the exact reversal middle point.
- Removing that middle point exposed a consecutive near-duplicate joint segment.
- TOTG still rejected the path, now with `near_duplicate_segments=1`.
- This remains a deterministic trajectory-quality issue, not a service wiring
  issue and not a fallback-routing case.

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/config.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/ordered_planning_worker.py
```

What was done:

- Added strict removal of consecutive near-duplicate linked-LIN joint points
  before optimization.
- Cleanup order is now:
  - remove existing near-duplicate joint points
  - remove exact local joint-space reversal triplets
  - remove near-duplicate joint points exposed by the reversal repair
- Added explicit config defaults:
  - `LINKED_LIN_REMOVE_DUPLICATE_POINTS_ENABLED: True`
  - `LINKED_LIN_DUPLICATE_POINT_TOL_RAD: 1e-7`
- Added `linked_lin_duplicate_repairs` metadata to the planned linked-LIN group
  and `duplicate_repairs` timing metadata to
  `ordered_linked_lin_group_planned`.
- No fallback path was added.

Verification:

```text
python3 -m py_compile \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/ordered_planning_worker.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/config.py

/bin/bash -lc "env -i HOME=\"$HOME\" TERM=\"${TERM:-xterm}\" PATH=\"/usr/bin:/bin:/usr/sbin:/sbin\" bash --noprofile --norc -lc 'source /opt/ros/rolling/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws/eRob_moveit; colcon build --packages-select erob_moveit_runtime zeroerr'"
```

Verification notes:

- Python compile passed.
- `colcon build --packages-select erob_moveit_runtime zeroerr` passed.
- The build command still prints the existing
  `/opt/ros/rolling/setup.bash: No such file or directory` warning, then builds
  successfully from the available environment.

Next pickup point:

1. Rerun fake-system ordered chain.
2. Look for:
   - `[LINKED_LIN] Removed 1 exact local joint-space reversal point(s)`
   - `[LINKED_LIN] Removed 1 near-duplicate joint point(s)`
3. Confirm the next `TOTG_PATH_DIAG` for the linked-LIN group reports
   `near_duplicate_segments=0`.
4. Confirm the linked-LIN group reaches `ordered_linked_lin_group_planned`, or
   capture the next explicit optimizer/helper failure without adding fallback.

### 2026-08-10 - Slice 68: iterate linked-LIN joint cleanup to fixed point

Status: completed, compile-checked and build-checked. Ready for another
fake-system validation.

Fresh user log reviewed:

```text
[LINKED_LIN] Removed 1 exact local joint-space reversal point(s) before optimization
[LINKED_LIN] Removed 1 near-duplicate joint point(s) before optimization
[TOTG_PATH_DIAG] points=185 joints=6 near_duplicate_segments=0
[TOTG_PATH_DIAG] detected 1 near-180deg joint-space reversal(s); worst_middle=55 worst_cos=-1.000000000
[ORDERED_CHAIN_TIMING] ordered_planning_worker_error ... error=Trajectory optimizer failed
```

Finding:

- Slice 67 removed the duplicate point and cleared `near_duplicate_segments`.
- Removing the duplicate exposed another exact local out-and-back triplet at
  the same path region.
- The cleanup needs to run to a fixed point rather than performing one
  duplicate pass and one reversal pass.
- This is still a linked-LIN trajectory-quality cleanup, not fallback routing.

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/config.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/ordered_planning_worker.py
```

What was done:

- Added `_cleanup_linked_lin_joint_path(...)` as the single pre-optimizer
  cleanup entrypoint.
- The cleanup alternates duplicate-point removal and exact-reversal removal
  until a pass makes no changes.
- Added a bounded convergence guard:
  - `LINKED_LIN_CLEANUP_MAX_PASSES: 8`
- If cleanup does not converge, planning fails visibly with a runtime error.
- Added `linked_lin_cleanup_passes` metadata to planned linked-LIN groups and
  `cleanup_passes` timing metadata to `ordered_linked_lin_group_planned`.
- No fallback path was added.

Verification:

```text
python3 -m py_compile \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/ordered_planning_worker.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/config.py

/bin/bash -lc "env -i HOME=\"$HOME\" TERM=\"${TERM:-xterm}\" PATH=\"/usr/bin:/bin:/usr/sbin:/sbin\" bash --noprofile --norc -lc 'source /opt/ros/rolling/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws/eRob_moveit; colcon build --packages-select erob_moveit_runtime zeroerr'"
```

Verification notes:

- Python compile passed.
- `colcon build --packages-select erob_moveit_runtime zeroerr` passed.
- The build command still prints the existing
  `/opt/ros/rolling/setup.bash: No such file or directory` warning, then builds
  successfully from the available environment.

Next pickup point:

1. Rerun fake-system ordered chain.
2. For the current repro path, expect cleanup to remove at least:
   - 2 exact local joint-space reversal points
   - 1 near-duplicate joint point
3. Confirm the linked-LIN `TOTG_PATH_DIAG` has:
   - `near_duplicate_segments=0`
   - no `detected ... near-180deg joint-space reversal(s)` error
4. Confirm linked-LIN reaches `ordered_linked_lin_group_planned`, or capture
   the next explicit non-fallback failure.

### 2026-08-10 - Slice 69: replace linked-LIN per-pose IK with C++ Cartesian path

Status: completed, compile-checked and build-checked. This supersedes the
temporary Python linked-LIN cleanup from Slices 66-68.

User correction:

- The intended linked-LIN design was to port the proven single-LIN
  `compute_cartesian_path` behavior into C++ and run it once for the whole
  compatible LIN batch.
- The previous helper implementation used custom per-pose IK and created
  duplicate/cusp/180-degree joint artifacts that the old single-LIN path did
  not produce.
- Do not continue patching those artifacts in Python.

Files changed:

```text
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/src_cpp/linked_lin_helper_node.cpp
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/ordered_planning_worker.py
eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/config.py
```

What was done:

- Replaced linked-LIN helper internals:
  - removed the effective per-waypoint `RobotState::setFromIK(...)` planning
    loop as the active path
  - added one full-group call to
    `moveit::core::CartesianInterpolator::computeCartesianPath(...)`
  - builds the link waypoints from TCP request poses using the existing
    workobject and tool transforms
  - seeds the interpolator from the supplied request `seed_state`
  - validates the generated path for bounds, configured max joint step/span and
    optional collision checking
  - returns the resulting `RobotState` path as one `RobotTrajectory`
- Removed the temporary Python linked-LIN duplicate/reversal cleanup and its
  config knobs:
  - `LINKED_LIN_REMOVE_EXACT_REVERSALS_ENABLED`
  - `LINKED_LIN_REMOVE_DUPLICATE_POINTS_ENABLED`
  - `LINKED_LIN_DUPLICATE_POINT_TOL_RAD`
  - `LINKED_LIN_EXACT_REVERSAL_NEIGHBOR_TOL_RAD`
  - `LINKED_LIN_EXACT_REVERSAL_COS_THRESHOLD`
  - `LINKED_LIN_CLEANUP_MAX_PASSES`
- Kept no-fallback behavior: if the C++ Cartesian path fails or validates
  badly, linked-LIN fails visibly.

Verification:

```text
python3 -m py_compile \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/motion/scheduling/ordered_planning_worker.py \
  eRob_moveit/src/eRob_ROS2_MoveIt/erob_moveit_runtime/scripts/config.py

/bin/bash -lc "env -i HOME=\"$HOME\" TERM=\"${TERM:-xterm}\" PATH=\"/usr/bin:/bin:/usr/sbin:/sbin\" bash --noprofile --norc -lc 'source /opt/ros/rolling/setup.bash; source /home/ilv/ros2_ws/install/setup.bash; cd /home/ilv/ros2_ws/eRob_moveit; colcon build --packages-select erob_moveit_runtime zeroerr'"
```

Verification notes:

- Python compile passed.
- `colcon build --packages-select erob_moveit_runtime zeroerr` passed.
- The build command still prints the existing
  `/opt/ros/rolling/setup.bash: No such file or directory` warning, then builds
  successfully from the available environment.

Next pickup point:

1. Rerun fake-system ordered chain.
2. Confirm linked-LIN logs still show helper success, but no Python
   duplicate/reversal cleanup logs.
3. Inspect the linked-LIN `TOTG_PATH_DIAG`; it should now look like a normal
   Cartesian-path result rather than a custom IK artifact path.
4. If the helper rejects, fix the C++ Cartesian request/validation semantics
   directly. Do not add Python fallback or cleanup masking.

## Goal

Refactor the motion backend so responsibilities are clearly separated,
latency-critical motion computation can live in C++, and Python remains
responsible for high-level motion orchestration.

The backend should converge on **one generalized motion pipeline** rather
than maintaining separate execution systems for normal moves and ordered
motion chains.

The current `moveit_robot_backend.py` has accumulated too many
responsibilities:

- request handling
- LIN/PTP/PATH planning
- ordered/batched motion scheduling
- lookahead planning
- trajectory blending
- trajectory optimization
- controller execution
- state matching
- cancellation / controlled stopping
- collision validation
- timing/logging
- special handling for short or rotation-dominant segments

Process-level pause/resume semantics should **not** live in the ROS2
backend. The platform owns process state and decides what to submit after a
controlled stop.

---

## Core Architecture Principle

There should be one internal motion pipeline:

```text
single PTP/LIN/PATH request
            │
            │
            ├──────────────────────────────┐
            │                              │
            ▼                              ▼
      MotionSegment                 MotionBatch
                                      segments=[...]
                                            │
                                            ▼
                                   MotionScheduler
                                   - sees full batch
                                   - partitions groups
                                   - plans ahead
                                   - preserves order
                                            │
                                            ▼
                                      MotionQueue
                                 PENDING / PLANNING /
                                 READY / EXECUTING
                                            │
                                            ▼
                                  TrajectoryExecutor
                                            │
                                            ▼
                                        Controller
```

A single move is simply a `MotionBatch` containing one segment.

The old "ordered chain" concept should become **batch submission + scheduler
lookahead**, not a separate execution subsystem.

Startup/readiness rule:

- Every service, action server, C++ helper or background runtime dependency
  required by any accepted motion path must be represented in
  `RobotController.is_motion_stack_ready()` and
  `RobotController.get_motion_stack_fault_reason()` when it is introduced.
- The API and GUI-visible state contract must use that same readiness source:
  `/startup/status`, `/health`, `/status`, state WebSocket and execution
  WebSocket must not report `runtime_ready=true` until those dependencies are
  available.
- Motion endpoints must fail early with HTTP 503 and a specific
  `motion_stack_fault` while such dependencies are still warming up.
- Status, state inspection, tool, safety and drive diagnostics may remain
  available during warmup, but they must not imply motion readiness.

---

## Target Architecture

```text
scripts/
├── backend/
│   ├── moveit_robot_backend.py
│   │   └── thin public API / facade only
│   └── i_robot_backend.py
│       └── public compatibility contract
│
└── motion/
    ├── planning/
    │   ├── lin_planner.py
    │   ├── linked_lin_planner.py
    │   ├── ptp_planner.py
    │   ├── path_planner.py
    │   ├── planning_types.py
    │   └── existing planner modules reused during migration
    │
    ├── blending/
    │   ├── blend_builder.py
    │   ├── blend_geometry.py
    │   └── blend_validation.py
    │
    ├── scheduling/
    │   ├── motion_scheduler.py
    │   ├── motion_batch.py
    │   ├── motion_group.py
    │   └── scheduling_types.py
    │
    ├── execution/
    │   ├── motion_queue.py
    │   ├── trajectory_executor.py
    │   ├── controller_handoff.py
    │   ├── state_matcher.py
    │   ├── trajectory_optimizer.py
    │   ├── trajectory_optimization.py
    │   └── existing execution modules reused during migration
    │
    └── helpers/
        ├── trajectory_utils.py
        └── timing.py
```

Do not create a second parallel planning/execution tree under `backend/`.
The current package already has `scripts/motion/planning`,
`scripts/motion/execution`, `MotionQueue`, `TrajectoryExecutor`, PTP helper
integration, contour IK integration, IPP/TOTG integration, and Ruckig
integration. The refactor should extend and clean up those modules while
shrinking `backend/moveit_robot_backend.py`.

`MoveItRobotBackend` should become primarily a facade:

```python
class MoveItRobotBackend:
    def move_ptp(...):
        return self.scheduler.submit(
            MotionBatch([PtpSegment(...)])
        )

    def move_liner(...):
        return self.scheduler.submit(
            MotionBatch([LinearSegment(...)])
        )

    def execute_path(...):
        return self.scheduler.submit(
            MotionBatch([PathSegment(...)])
        )

    def execute_motion_batch(...):
        return self.scheduler.submit(
            MotionBatch(segments)
        )
```

The public REST/API compatibility name
`execute_ordered_motion_chain()` can remain temporarily, but internally it
should translate directly to `MotionBatch` submission.

The current public method names include `move_liner(...)`, `move_ptp(...)` and
`execute_path(...)`; keep those names in `IRobotBackend`, REST handlers and
compatibility adapters. Internal aliases such as `move_lin` are optional, but
they must not replace the current public API during the refactor.

`get_ordered_motion_chain_status()` and the ordered-chain websocket/status
payload should also remain as compatibility wrappers while external clients are
migrated to generic batch execution status.

---

## 1. Extract Blending

Move `_build_blended_group()` and supporting functions out of
`moveit_robot_backend.py`.

Desired API:

```python
blend_result = self.blend_builder.build(
    segments,
    trajectories,
)
```

Use a typed result:

```python
@dataclass
class BlendedTrajectory:
    trajectory: JointTrajectory
    source_segments: list[int]
    effective_radii: list[float]
    boundary_indices: list[int]
```

The blending module should own:

- blend radius calculations
- entry/exit selection
- short-segment densification
- rotation-dominant handling
- overlap handling
- blend geometry
- blend validation

Blending should remain in Python initially. Move it to C++ only if later
profiling proves that it is a meaningful bottleneck.

---

## 2. Introduce Typed Motion Models

Replace loosely structured dictionaries with typed internal objects before
moving planning code. This makes the extraction of `_plan_ordered_segment()`
less likely to preserve the current dictionary coupling.

```python
@dataclass
class MotionSegment:
    label: str
    velocity: float
    acceleration: float
    blend_radius: float = 0.0

@dataclass
class LinearSegment(MotionSegment):
    target: list[float]

@dataclass
class PtpSegment(MotionSegment):
    target: list[float]

@dataclass
class PathSegment(MotionSegment):
    waypoints: list[list[float]]

@dataclass
class UnwindSegment(MotionSegment):
    joint_name: str = "Joint_6"
```

This reduces mistakes involving missing keys, naming differences and
segment-specific behavior.

---

## 3. Extract Individual Segment Planning

Move `_plan_ordered_segment()` into `motion/planning`.

Provide separate planners for LIN, PTP and PATH. Each planner should return
the same internal result type:

```python
@dataclass
class PlannedTrajectory:
    trajectory: JointTrajectory
    start_state: RobotState
    end_state: RobotState
    planning_time: float
    metadata: dict
```

The planning layer should not know how controller goals are sent or waited
on.

During migration, the planning layer may wrap existing modules such as
`segment_planning.py`, `trajectory_planner.py`, `ptp_target.py`,
`single_target.py` and `direct_contour_ik.py`. Prefer adapters around proven
code before replacing behavior.

---

## 4. Introduce MotionBatch

The scheduler must receive the whole known sequence as a batch in order to
perform lookahead.

```python
@dataclass
class MotionBatch:
    segments: list[MotionSegment]
    blocking: bool = True
```

Example:

```text
PTP
LIN
LIN
LIN
PTP
PTP
LIN
LIN
```

The scheduler sees all eight segments immediately and can partition them
into compatible planning groups.

The batch is the high-level unit of submission.

---

## 5. Add a General Motion Scheduler

Replace the special ordered-chain planning/execution path with a generalized
`MotionScheduler`.

The scheduler owns:

```text
parse MotionBatch
→ preserve original order
→ identify hard-stop boundaries
→ identify compatible contiguous runs
→ choose the best planner for each run
→ plan only enough to start execution
→ continue planning future groups while current group executes
→ queue ready trajectories
→ invalidate future work when execution is cancelled/stopped
```

### Example grouping

Input:

```text
1  PTP
2  LIN
3  LIN
4  LIN
5  PTP
6  PTP
7  LIN
8  LIN
```

Possible planning groups:

```text
Group A:
    PTP

Group B:
    LIN
    LIN
    LIN
    → LinkedLinPlanner

Group C:
    PTP
    PTP

Group D:
    LIN
    LIN
    → LinkedLinPlanner
```

Grouping is based on **contiguous compatibility**, not on whether the entire
batch has the same motion type.

---

## 6. Define Planning Boundaries

`blendR = 0` is a natural hard execution/planning boundary after the segment
that owns that `blendR` value.

Example:

```text
1 LIN blendR=20
2 LIN blendR=0
  ---------------- hard boundary after segment 2
3 LIN blendR=20
4 LIN blendR=20
```

Even though all four segments are LIN, they should not automatically become
one linked-LIN group.

Other group boundaries can include:

- explicit hard stop
- incompatible planner type
- PATH boundaries where independent planning is required
- unwind/special motion operations
- safety-required state verification
- any segment that cannot legally participate in the same blended trajectory

These rules belong in the scheduler/grouping layer.

---

## 7. Generalize the Motion Queue

Reuse the existing `motion/execution/motion_queue.py` concept, but promote it
into a queue that can represent planning state as well as execution state.

Suggested states:

```text
PENDING
PLANNING
READY
EXECUTING
DONE
FAILED
CANCELLED
```

The queue should hold **planning groups / planned trajectories**, not just
raw controller goals.

Example:

```text
MotionQueue

Group A [EXECUTING]
    blended pickup group

Group B [READY]
    paint path

Group C [PLANNING]
    dropoff route

Group D [PENDING]
    final unwind
```

The queue should not contain process-level `PAUSED` / `RESUMING` states.

---

## 8. Preserve Pipelined Lookahead

Continue planning future groups while the current group executes.

```text
GROUP 1
pickup approach
pickup exact stop
→ execute as soon as GROUP 1 is ready

while GROUP 1 executes:

GROUP 2
lift + align
safe travel
staging
→ plan in parallel

while GROUP 2 executes:

GROUP 3
paint path
→ already planning / ready

while paint executes:

GROUP 4
dropoff route
→ already planning / ready
```

Only the first executable group should need to be ready before robot motion
starts.

---

## 9. Separate Planning From Execution

Controller interaction should move into an execution layer.

Execution responsibilities:

- send trajectory
- controller goal acceptance
- cancellation
- controlled stop behavior
- completion tracking
- result mapping
- state matching at trajectory boundaries

The executor should execute a prepared trajectory and report its result. It
should not understand process workflows or batch semantics.

---

## 10. Remove Pause/Resume Semantics From ROS2 Backend

ROS2 should not own application-level pause/resume behavior.

### Platform owns

- process state
- `RUNNING / PAUSED / RESUMING / ERROR`
- user pause/resume commands
- deciding which logical operation should be resumed
- deciding whether to restart, skip, abort or move to a safe point
- retaining the original workpiece/process context
- constructing the new `MotionBatch` after resume

### ROS2 backend owns

- execute
- cancel
- controlled stop
- report actual robot state
- report completed segment/group progress
- report the segment/group that was active when execution stopped

Backend API should look more like:

```python
submit_batch(...)
cancel_active(...)
stop_controlled(...)
get_execution_state(...)
get_robot_state(...)
```

and **not**:

```python
pause_process(...)
resume_process(...)
resume_ordered_chain(...)
```

When the platform requests pause:

```text
platform
    ↓
controlled stop
    ↓
ROS stops active controller trajectory
    ↓
ROS reports actual state + execution progress
    ↓
all preplanned future trajectories are invalidated
    ↓
platform decides what should happen on resume
```

Resume should be a **new MotionBatch submission from the actual robot state**.

---

## 11. Add a C++ Linked-LIN Helper

This is the next major performance improvement after the structural
refactor.

Current behavior performs multiple sequential `computeCartesianPath` calls.
Recent profiling showed roughly 1.25 seconds across six sequential Cartesian
path calls before the first blended group could execute.

The scheduler should identify any compatible contiguous LIN run, for example:

```text
PTP
LIN
LIN
LIN
PTP
PTP
LIN
LIN
```

and route both LIN runs through the linked-LIN helper independently.

Desired flow:

```text
Python MotionScheduler
→ identify contiguous compatible LIN run
→ ONE request to C++ linked-LIN helper
→ continuous Cartesian sampling
→ continuous seeded IK
→ joint-limit / branch-continuity validation
→ collision checking against the current PlanningScene
→ state-validity / constraint verification
→ optional FK endpoint verification
→ whole-path validation
→ raw validated joint trajectory + segment boundaries
→ Python blending / TOTG
```

The helper should return:

```text
joint_trajectory
segment_boundary_indices
per-segment planning metadata
validation_result
invalid_waypoint_indices
validation_timing
```

The returned joint trajectory should already be validated in C++ before it crosses back into Python.

This preserves original LIN boundaries and different `blendR` values.

The helper must not require that the whole MotionBatch consists only of LIN
segments.

### Linked-LIN service and build integration

Add the helper as a normal `erob_moveit_runtime` service-backed C++ helper,
consistent with the existing PTP, contour IK, IPP/TOTG and Ruckig helpers.

Required package changes:

```text
srv/ComputeLinkedLin.srv
src_cpp/linked_lin_helper_node.cpp
CMakeLists.txt rosidl_generate_interfaces(...)
CMakeLists.txt add_executable(linked_lin_helper ...)
CMakeLists.txt install(TARGETS linked_lin_helper ...)
motion/planning/linked_lin_planner.py
motion/planning/planner_support_service.py client creation
```

The service request should carry enough information for C++ to validate in
process without calling back into Python:

```text
sensor_msgs/JointState seed_state
geometry_msgs/Pose[] target_poses
string[] labels
float64[] velocities
float64[] accelerations
float64[] blend_radii
string group_name
string link_name
float64 timeout_s
float64 cartesian_step_mm
float64 ik_timeout_s
uint32 ik_attempts
float64 max_joint_step_rad
float64 max_joint_span_rad
float64 max_endpoint_delta_rad
string[] full_turn_joint_names
float64 full_turn_max_joint_span_rad
float64 full_turn_max_endpoint_delta_rad
float64 fk_position_tolerance_mm
float64 fk_orientation_tolerance_deg
bool check_collision
bool check_state_validity
bool check_fk
---
bool success
int32 error_code
string message
moveit_msgs/RobotTrajectory trajectory
uint32[] segment_boundary_indices
uint32[] invalid_waypoint_indices
float64 max_joint_step_rad
float64 max_joint_span_rad
float64 max_endpoint_delta_rad
float64 solve_time_s
float64 validation_time_s
float64 total_time_s
```

The exact field list can evolve, but it must preserve original segment
boundaries and return enough diagnostics to explain failed validation without
re-running per-waypoint checks in Python.

### C++ validation responsibilities

The linked-LIN helper should own the complete numerical validity pipeline for
the trajectory it generates. Validation should happen in the same C++ process
that owns the generated `RobotState` objects, rather than bouncing states back
through Python or ROS2 validity services.

For every generated state, validate at least:

- joint bounds
- branch continuity
- configured maximum joint step
- configured joint span / endpoint delta policy
- collision status against the current `PlanningScene`
- MoveIt state validity / feasibility
- configured constraints
- optional FK verification where required

After all samples are generated, perform a whole-path validation pass before
returning the result.

The helper must validate against the **current MoveIt PlanningScene**, including:

- world collision objects
- attached collision objects
- active tool geometry
- allowed-collision matrix
- current robot model and joint limits

A fast helper using a stale or incomplete PlanningScene is not acceptable.

### Shared C++ trajectory validator

Avoid duplicating the same continuity and validity logic independently in PTP,
contour IK and linked-LIN helpers.

Long term, introduce a reusable C++ validator used by all numerical planners:

```cpp
struct TrajectoryValidationOptions
{
    double max_joint_step;
    double max_joint_span;
    double max_endpoint_delta;

    std::vector<std::string> full_turn_joints;
    double full_turn_max_span;
    double full_turn_max_endpoint_delta;

    bool check_bounds = true;
    bool check_collision = true;
    bool check_state_validity = true;
    bool check_fk = false;
};

struct TrajectoryValidationResult
{
    bool valid;
    int invalid_index;
    std::string reason;

    double max_joint_step;
    double max_joint_span;
    double max_endpoint_delta;

    double bounds_check_time;
    double collision_check_time;
    double state_validity_time;
    double fk_validation_time;
};

TrajectoryValidationResult validateTrajectory(
    const std::vector<moveit::core::RobotState>& states,
    const planning_scene::PlanningSceneConstPtr& planning_scene,
    const moveit::core::JointModelGroup* joint_model_group,
    const moveit_msgs::msg::Constraints& path_constraints,
    const TrajectoryValidationOptions& options);
```

The exact API can evolve, but the implementation must receive the live
PlanningScene, joint model group and constraints directly. A validator that
only sees joint states and scalar options cannot satisfy collision,
allowed-collision, attached-object or constraint requirements.

The architectural goal is one consistent validation implementation shared by:

```text
PTP helper
Contour IK helper
Linked-LIN helper
```

Python should receive a validated trajectory plus diagnostics; it should not
repeat per-waypoint collision/state checks.

---

## 12. Define the Python/C++ Boundary

General rule:

> Python decides what motion should happen and how motion requests are grouped.
> C++ computes how numerically intensive motion requests become valid joint
> trajectories.

### Keep in Python

- process-independent motion scheduling
- MotionBatch parsing
- compatible-run grouping
- blend/hard-stop boundary decisions
- lookahead orchestration
- configuration
- REST/API integration
- high-level logging

### Keep in the platform, outside ROS2 backend

- workpiece workflow
- production process state
- pause/resume semantics
- pickup/dropoff policy
- safe-point selection
- retry/skip/abort decisions

### Prefer C++ for

- linked LIN interpolation
- repeated IK solving
- PTP IK
- contour IK
- collision checking
- PlanningScene state-validity / feasibility checks
- joint bounds validation
- joint branch selection
- joint continuity validation
- joint span / endpoint-delta validation
- FK verification where required
- whole-path validity checking
- shared trajectory-validation utilities used by all numerical planners
- numerical trajectory operations
- TOTG / trajectory time parameterization
- potentially blend generation later if profiling justifies it

---

## 13. Motion-Specific Cleanup After Refactor

Do behavioral simplification only after the structural refactor is stable.

### Pickup

Pickup contact must remain an exact stop:

```text
approach
→ pickup       blendR = 0
→ lift
```

### Combine Lift + Paint-Axis Alignment

Prefer:

```text
pickup
→ vertical lift + orientation alignment
→ safe travel
```

rather than a separate tiny mostly-rotational
`Aligning workpiece to paint axis` segment.

### Dropoff

Safe travel should end above/near the actual dropoff before final descent.

Prefer:

```text
safe travel
→ pre-dropoff pose above target
→ controlled final approach
→ exact dropoff
```

Consider LIN for final descent if Cartesian approach is safer than direct
PTP interpolation.

### J6 Cable Relief

Keep explicit post-dropoff J6 unwind as a fallback.

Distributed cable relief during safe travel can reduce or eliminate the
fallback motion, but long term prefer explicit J6 branch control over relying
only on Cartesian RZ because equivalent ±360° orientations collapse when
represented as quaternions.

---

## 14. Timing and Diagnostics

Retain detailed timing instrumentation throughout the refactor:

- batch request received
- scheduler grouping time
- initial state acquisition
- per-planner compute time
- linked-LIN helper time
- IK time
- joint-continuity / bounds-validation time
- collision-validation time
- state-validity / feasibility time
- whole-path-validation time
- FK-verification time where enabled
- blend construction
- optimization/TOTG
- queue-ready time
- controller handoff
- first active execution
- state matching
- total segment/group execution
- total batch execution

### Current performance baseline

```text
batch request → first motion             ~1.58 s
first blended trajectory                 ~4.15 s
paint planning                           ~2.0 s (hidden by execution)
paint motion                             ~5.88 s
dropoff motion                           ~3.17 s
whole ROS motion chain                   ~15.81 s
```

Structural refactoring should preserve these numbers within small numerical
variation.

---

## Refactoring Order

Do not perform a big-bang rewrite.

1. Introduce typed `MotionSegment`, `MotionBatch`, `MotionGroup` and
   `PlannedTrajectory` models in `motion/planning` / `motion/scheduling`.
2. Add adapters that convert existing REST/API ordered-chain dictionaries into
   typed models without changing behavior.
3. Extract `_build_blended_group()` and related blend helpers into
   `motion/blending`.
4. Extract `_plan_ordered_segment()` into `motion/planning`, initially wrapping
   existing proven planner functions.
5. Extract/generalize the current ordered-chain worker into
   `motion/scheduling/motion_scheduler.py`.
6. Promote the existing `motion/execution/motion_queue.py` into the generalized
   planning/execution queue.
7. Separate controller execution and state matching from scheduling/planning.
8. Preserve `execute_ordered_motion_chain()` and
   `get_ordered_motion_chain_status()` as temporary compatibility wrappers
   around MotionBatch submission and batch status.
9. Remove backend process pause/resume semantics; keep generic controlled
   stop/cancel.
10. Route single `move_ptp`, `move_liner` and `execute_path` commands through
    the same scheduler pipeline.
11. Extract/commonize C++ trajectory validation used by existing numerical
    helpers.
12. Add `ComputeLinkedLin.srv`, `linked_lin_helper_node.cpp`, CMake entries and
    Python client plumbing.
13. Add the C++ linked-LIN helper with in-process PlanningScene validation.
14. Route contiguous compatible LIN runs through the linked-LIN helper.
15. Benchmark request-to-first-motion latency and validation cost.
16. Optimize non-paint process motion only after the backend refactor is stable.
17. Evaluate moving blend generation to C++ only if profiling shows meaningful
    benefit.
18. Remove obsolete duplicate/compatibility paths.

---

## Refactoring Principle

Every extraction should preserve current robot behavior and be independently
testable.

Avoid combining structural refactoring and behavioral changes in the same
commit where possible.

Suggested commit sequence:

```text
refactor: add typed motion segment models
refactor: introduce motion batch model
refactor: adapt ordered-chain requests to motion batch
refactor: extract blend builder without behavior change
refactor: extract ordered segment planner
refactor: generalize ordered chain worker into motion scheduler
refactor: generalize execution queue
refactor: separate trajectory execution and state matching
refactor: preserve ordered-chain API as batch compatibility wrapper
refactor: remove backend process pause/resume semantics
refactor: route single moves through common scheduler
refactor: extract shared C++ trajectory validator
feat: add linked LIN service contract and helper target
feat: add linked LIN C++ helper with PlanningScene validation
feat: use linked LIN planning for contiguous compatible LIN runs
perf: benchmark batch planning, validation, and first-motion latency
cleanup: remove legacy ordered-chain and Cartesian planning paths
```

---

## Architecture Summary

```text
PLATFORM
- process workflow
- pause/resume
- production state
- pickup/dropoff policy
- safe-point selection
           │
           │ full known sequence
           ▼
     MotionBatch
           │
           ▼
ROS2 MotionScheduler
- inspect entire batch
- preserve order
- partition compatible runs
- select planners
- look ahead
           │
           ▼
      MotionQueue
           │
           ▼
   TrajectoryExecutor
- execute
- cancel
- controlled stop
- report progress/state
           │
           ▼
      ros2_control
```

This provides one generalized backend path for single moves and batched
motion, preserves full-batch lookahead, makes linked-LIN optimization natural,
keeps process-level pause/resume out of ROS2, and keeps latency-critical IK,
collision checking, state validity, and trajectory verification inside C++.
