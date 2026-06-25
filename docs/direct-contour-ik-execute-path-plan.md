# Direct Contour IK For `execute_path`

## Implementation Status

Status as of this pass:

- Done: disabled-by-default runtime config keys added to `scripts/config.py`.
- Done: disabled-by-default runtime config keys added to ZeroErr and Fairino runtime YAMLs.
- Done: first `direct_contour_ik.py` module added under `motion/planning/`.
- Done: direct contour IK selection is wired into `trajectory.py` after the existing far-from-first-waypoint approach check.
- Done: full dense IK solve prototype added using MoveIt `/compute_ik`.
- Done: FK accuracy validation added for every solved point.
- Done: joint continuity, finite-value, and optional sampled state-validity validation added.
- Done: joint span and endpoint-delta validation added. The policy is joint-specific: most joints stay limited to about `pi`, while configured full-turn joints such as Joint 6 may span about one revolution for shaft/paint contours.
- Done: successful direct IK trajectories are passed into the existing Ruckig/TOTG and controller execution pipeline.
- Not done: reduced-anchor `compute_cartesian_path` validation is not wired yet, so `CONTOUR_VALIDATE_REDUCED_MOVEIT_ENABLED` defaults to `false`.
- Done: batched in-process MoveIt IK helper service `/compute_contour_ik` added in C++.
- Done: Python direct contour planner now prefers `/compute_contour_ik` and falls back to the per-point `/compute_ik` prototype if the batch helper is unavailable.
- Done: FK-checked joint smoothing added inside the batched IK helper to reduce joint-space wiggle without accepting TCP drift beyond the configured smoothing tolerances.
- Not done: offline recorded-contour regression tests are not added yet.

## Findings Log

- The first implementation keeps `kinematics.yaml` untouched. The direct contour path is controlled only by `CONTOUR_*` runtime keys.
- The first implementation uses MoveIt `/compute_ik` per contour point. This is the lowest-risk correctness prototype but may still be slower than the final target because it pays ROS service overhead for every point.
- Direct IK currently starts only when the robot is already near the first waypoint. If the existing `PATH_APPROACH_THRESHOLD_MM` logic decides an approach is needed, the current MoveIt plan-then-approach path remains responsible.
- Reduced-anchor MoveIt validation was intentionally left disabled by default because the code path is not implemented yet. Full dense FK and joint-continuity validation are implemented before execution.
- Joint continuity validation checks the first solved IK point against the live joint seed as well as every consecutive contour point. This is intended to catch wrong-branch first IK solutions before execution.
- Failed direct IK attempts fall back to the existing MoveIt Cartesian path instead of returning an error immediately.
- While adding runtime keys, `zeroerr/config/runtime.yaml` was found to contain a stray `x` immediately after `WRIST_LINK: Link_6`; that was removed so the YAML remains parseable.
- Test log `zeroerr/config/paint/temp_logs.txt` showed the service-based direct IK prototype accepted a 732-point path with excellent FK accuracy (`0.0137 mm`, `0.0005 deg`) but took `17.264 s` before optimization. This is slower than the earlier MoveIt Cartesian planning time and confirms that per-point `/compute_ik` service calls are not the final performance solution.
- The same log showed a Joint 6 contour span of about `6.29 rad` with small local steps (`0.0174 rad`). Initially this was treated as an excessive branch drift. After clarifying that the workflow includes painting a workpiece around a shaft, the policy was changed: configured joints such as Joint 6 may intentionally make a near-360-degree sweep, while other joints remain constrained by the stricter span limit.
- The direct contour trajectory itself completed, but the subsequent explicit Joint 6 unwind failed controller tolerance with final Joint 6 error about `4.595 rad`. This shows the unwind path still needs monitoring, but a full-turn Joint 6 contour is not automatically invalid for shaft painting.
- A new custom service `ComputeContourIK.srv` was added because MoveIt's existing `/compute_ik` service accepts only one pose per request.
- The new `contour_ik_helper` C++ node loads the MoveIt robot model once, solves the full pose list in-process with the previous solution as the next seed, validates FK error and joint step/span, and returns an untimed `RobotTrajectory`.
- Build finding: this ROS/MoveIt version does not support `RobotState::setVariablePositions(sensor_msgs::msg::JointState)` directly, so the helper sets seed positions with explicit `name` and `position` vectors.
- `erob_moveit_runtime` builds successfully with the new service and executable.
- Full updated log `zeroerr/config/paint/temp_logs.txt` confirmed the 691-point fallback trajectory executed successfully after direct batch IK rejected the Joint 6 full-turn branch under the initial strict policy. MoveIt planning took `14.208 s`, Ruckig/TOTG fallback timing took `0.080 s`, controller execution succeeded, and `backend_execute_path` completed in `34.970 s`.
- The same run confirmed the explicit Joint 6 unwind from about `-6.466 rad` to `-0.183 rad` succeeded.
- Policy update: because shaft painting can legitimately require a 360-degree wrist/workpiece sweep, the batch IK service now accepts `full_turn_joint_names` and relaxed full-turn span/endpoint-delta limits. Runtime defaults allow Joint 6 full-turn motion while keeping other joints on the stricter span guard.
- Smoothing update: the batch IK helper now runs a small local joint-space smoothing pass after the dense IK solve. Each candidate point is accepted only if FK against the original Cartesian target remains inside the smoothing tolerance and local joint steps remain inside the configured step limit. The final trajectory is revalidated against the main FK and continuity limits before it is returned.
- Build finding: `erob_moveit_runtime` builds successfully after the smoothing service/API changes.
- Paint smoothness finding: the latest paint log still felt steppy even though direct IK solved `635` poses and FK-checked smoothing accepted `1145` tiny updates. The likely issue is not only post-optimizer point count, because MoveIt Cartesian paths also resample yet felt smooth. The stronger signal is direct IK joint-space curvature/branch selection: the path shows repeated small velocity reversals while Joint 6 wraps.
- Fix update: the batch IK helper now seeds each pose from a constant-velocity prediction when two previous points are available, with fallback to the previous-solution seed. This keeps FK validation unchanged but biases TRAC-IK `Distance` toward a lower-curvature joint path before time parameterization.
- Latest run finding: predictive seeding alone did not reduce the logged joint curvature (`curvature=0.03711->0.03711`) on the 459-point simplified contour. Post-solve smoothing still made only tiny FK-valid edits and did not change the worst curvature.
- Fix update: the batch IK helper now tries multiple local IK seeds per pose and chooses the valid FK-checked solution with the lowest local curvature score before final validation. This targets branch/joint wiggle directly instead of only smoothing after the branch has already been chosen.
- App finding: the platform client defaulted `execute_path` to `trajectory_optimizer="RUCKIG"`, overriding the ROS paint runtime setting `PATH_TRAJECTORY_OPTIMIZER: TOTG`. The client now omits the optimizer unless explicitly requested so paint runtime config can choose the optimizer.
- Latest app timing finding: duplicate projection is fixed. In `paint_timing_pickup_and_paint_20260625_125421.csv`, `project_execution_path` took about `0.139 s`, `build_diagnostics` about `0.0004 s`, and robot execution dominated at about `22.834 s`.
- Trace finding: `execution_motion_trace_execute_workpiece_xz_y_ry_20260625_125415.json` reports large rotation/phase errors, but the comparison uses Euler angles through a wrapped/singular rotation path (`RY` command reaches about `-342 deg` while actual Euler samples wrap around `-90..+90 deg`). Those rotation-error diagnostics are not reliable until the trace comparison unwraps or uses quaternion/geodesic orientation error.

## Goal

Reduce `execute_path` planning latency for dense contour paths while preserving accurate TCP motion along the requested contour.

Current observed timing for a 698-point path:

- `compute_cartesian_path`: about 14.35 s
- Ruckig: about 0.09 s
- controller execution: about 18.81 s

The bottleneck is MoveIt Cartesian planning over hundreds of Cartesian anchors. Ruckig and controller execution are not the planning bottleneck.

## Non-Goals

- Do not change the global MoveIt/TRAC-IK settings in `kinematics.yaml`.
- Do not weaken single-target or sub-mm move behavior.
- Do not remove the existing `compute_cartesian_path` path until the direct contour path has enough validation and fallback coverage.
- Do not rely only on reduced-anchor validation for final execution safety.

## Design Summary

Add a separate execution path for dense contours:

```text
execute_path dense contour
  -> transform workobject/tool exactly as today
  -> solve full Cartesian contour into a full joint trajectory using fast tracking IK
  -> validate every solved point with FK error checks
  -> validate joint continuity and limits
  -> optionally validate reduced anchors with MoveIt compute_cartesian_path
  -> optionally sample state validity/collision checks
  -> time-parameterize the full joint trajectory with Ruckig/TOTG
  -> execute through the existing FollowJointTrajectory pipeline
```

Keep existing behavior for:

```text
single target / sub-mm move
  -> existing MoveIt Cartesian path / Jacobian fallback behavior
```

## Configuration

Add runtime config keys under `runtime.yaml`, not `kinematics.yaml`:

```yaml
CONTOUR_DIRECT_IK_ENABLED: false
CONTOUR_DIRECT_IK_MIN_POINTS: 50
CONTOUR_DIRECT_IK_MIN_TOTAL_LENGTH_MM: 20.0

CONTOUR_IK_TIMEOUT_S: 0.003
CONTOUR_IK_ATTEMPTS: 1
CONTOUR_IK_RETRY_TIMEOUT_S: 0.02
CONTOUR_IK_RETRY_ATTEMPTS: 3

CONTOUR_IK_FK_POSITION_TOL_MM: 0.15
CONTOUR_IK_FK_ORIENTATION_TOL_DEG: 0.25
CONTOUR_IK_MAX_JOINT_STEP_RAD: 0.08
CONTOUR_IK_MAX_JOINT_VELOCITY_SCALE: 1.0

CONTOUR_IK_SMOOTHING_ENABLED: true
CONTOUR_IK_SMOOTHING_ITERATIONS: 2
CONTOUR_IK_SMOOTHING_ALPHA: 0.35
CONTOUR_IK_SMOOTHING_FK_POSITION_TOL_MM: 0.05
CONTOUR_IK_SMOOTHING_FK_ORIENTATION_TOL_DEG: 0.10

CONTOUR_VALIDATE_REDUCED_MOVEIT_ENABLED: true
CONTOUR_VALIDATE_REDUCED_POSITION_TOL_MM: 0.35
CONTOUR_VALIDATE_REDUCED_ORIENTATION_TOL_DEG: 0.35
CONTOUR_VALIDATE_REDUCED_MAX_TRANSLATION_MM: 10.0

CONTOUR_STATE_VALIDITY_ENABLED: false
CONTOUR_STATE_VALIDITY_STRIDE: 10
```

Initial default should be disabled. Enable only after testing against recorded contours.

## Solver Strategy

### Phase 1: Service-Based Prototype

Implement the first version using MoveIt `/compute_ik` calls if that is fastest to integrate.

Pros:

- Reuses current TRAC-IK plugin and robot model.
- Lower implementation risk.
- Easier to compare against current MoveIt behavior.

Cons:

- 698 ROS service calls may still be slow.
- Timeout and attempts may still follow the loaded MoveIt kinematics plugin behavior unless per-request timeout is honored.

This phase is useful for proving correctness and validation logic, not necessarily final performance.

### Phase 2: Batched IK Helper

Add a helper node/service that accepts the full contour and returns a full joint trajectory in one request.

Preferred behavior:

- Keep robot model and IK solver loaded in-process.
- Seed point `i` from solution `i - 1`.
- Use a short fast timeout first.
- Retry only failed/difficult points with the slower timeout.
- Return rich diagnostics for the first failed point.

Expected performance should be much closer to solver time than ROS service overhead.

## Path Selection Rules

Use direct contour IK only when all are true:

- `CONTOUR_DIRECT_IK_ENABLED` is true.
- Request has at least `CONTOUR_DIRECT_IK_MIN_POINTS`.
- Total path length exceeds `CONTOUR_DIRECT_IK_MIN_TOTAL_LENGTH_MM`.
- Motion is a path request, not a single-target request.
- Current joint state is available.
- The path has a stable orientation mode that the direct solver supports:
  - `constant`
  - later: `per_waypoint`

Fallback to existing MoveIt path when:

- IK helper unavailable.
- direct IK fails.
- validation fails.
- path is short enough that existing MoveIt latency is acceptable.
- robot state is stale or missing.

## Implementation Steps

### 1. Add Config

Update `scripts/config.py` defaults and relevant `runtime.yaml` files with disabled-by-default contour IK settings.

### 2. Add Data Structures

Create a small module such as:

```text
erob_moveit_runtime/scripts/motion/planning/direct_contour_ik.py
```

Core types:

- `ContourIkRequest`
- `ContourIkResult`
- `ContourIkFailure`
- `ContourValidationReport`

Keep this module independent from REST and backend code.

### 3. Convert Cartesian Contour To IK Targets

Reuse the same workobject and tool transform flow used by current `execute_path`.

Requirements:

- Do not double-apply tool transforms.
- Use the same base frame, planning group, and EE link as `_build_cartesian_request`.
- Preserve `orientation_mode`.
- Record original waypoint index for every generated IK target.

### 4. Seed And Solve Full Contour

Algorithm:

```text
seed = current_joint_state in planning joint order
for each target pose:
    solve IK with seed and fast timeout
    if fail:
        retry with slower timeout
    normalize equivalent joint branches against seed
    append solution
    seed = solution
```

Branch handling:

- Normalize continuous/equivalent revolute joints against the previous point.
- Pay special attention to Joint 6 / wrist wrap.
- Reject sudden branch flips even if IK technically succeeds.

### 5. FK Accuracy Validation

For every solved joint point:

- Run FK.
- Compare FK TCP pose to requested TCP pose.
- Reject if position error exceeds `CONTOUR_IK_FK_POSITION_TOL_MM`.
- Reject if orientation error exceeds `CONTOUR_IK_FK_ORIENTATION_TOL_DEG`.

Report:

- max position error
- max orientation error
- first failed index
- corresponding requested pose
- corresponding solved joints

This is mandatory. Reduced MoveIt validation is not enough for contour accuracy.

### 6. Joint Continuity Validation

For every consecutive joint pair:

- Check absolute joint delta.
- Reject deltas above `CONTOUR_IK_MAX_JOINT_STEP_RAD`, unless a joint is explicitly allowed to wrap and has been normalized.
- Check joint limits.
- Detect NaN/inf.
- Detect zero-length or duplicate trajectory segments.

Also compute summary stats:

- max joint step per joint
- total joint travel per joint
- Joint 6 span and max step

### 7. Reduced MoveIt Validation

Use the existing bounded simplifier to produce validation anchors.

### 7a. FK-Checked Joint Smoothing

After the dense IK solve, optionally smooth internal joint points before time parameterization:

- preserve the first and last points exactly
- compute each candidate from the neighboring joint points
- reject candidates that exceed `CONTOUR_IK_MAX_JOINT_STEP_RAD`
- run FK for every accepted candidate against its original Cartesian target
- reject candidates that exceed `CONTOUR_IK_SMOOTHING_FK_POSITION_TOL_MM` or `CONTOUR_IK_SMOOTHING_FK_ORIENTATION_TOL_DEG`
- re-run final FK and joint-continuity validation on the complete trajectory before returning it

This is intended to reduce velocity sign changes and small joint wiggles that can make the controller feel steppy. It must not simplify, decimate, or move the requested Cartesian contour.

Run `compute_cartesian_path` on the reduced anchor set using the existing MoveIt config.

Purpose:

- Catch major reachability/continuity issues.
- Keep a conservative comparison against the current production path.

Important limitation:

- This does not prove that every dense point is valid.
- Full dense validation still depends on IK + FK + joint continuity checks.

### 8. State Validity / Collision Sampling

If collision checking is enabled:

- Call `/check_state_validity` on sampled joint states.
- Use `CONTOUR_STATE_VALIDITY_STRIDE`.
- Always check first and last point.
- Always check points around high curvature or high joint delta if available.

If collision checking is disabled for paint/weld contact:

- Log that collision validation was intentionally skipped.
- Keep workspace, FK, joint-limit, and continuity checks active.

### 9. Build Full JointTrajectory

Create a `moveit_msgs/RobotTrajectory` containing all dense joint points.

Rules:

- Joint names must match controller order.
- Initial point should be close to current joint state.
- Do not include velocities/accelerations before time parameterization unless the optimizer expects them.
- Use the same trajectory optimizer interface already used by `_apply_time_param`.

### 10. Execute Through Existing Pipeline

After validation, pass the full raw joint trajectory to the existing time parameterization and execution path.

Do not add a separate controller submission path unless unavoidable.

## Safety Requirements

### Must Pass Before Execution

- Hardware readiness check.
- Current joint state freshness check.
- Current Cartesian state freshness check where needed.
- Workspace bounds for all requested Cartesian points.
- IK success for all requested contour points.
- FK error within configured tolerance for all points.
- Joint limit validation for all points.
- Joint continuity validation for all consecutive points.
- Time parameterization success.
- Existing controller goal timeout logic.

### Should Pass When Enabled

- Reduced MoveIt Cartesian path validation.
- Sampled `/check_state_validity` validation.
- Collision validation against safety walls.

### Must Fallback Instead Of Executing

- Any NaN/inf joint value.
- Missing joint name or joint order mismatch.
- First IK solution is far from current joint state unexpectedly.
- Any FK error exceeds tolerance.
- Any joint step exceeds limit.
- Any state validity check fails.
- Any reduced MoveIt validation fraction is below threshold.
- Time parameterization fails.

## Corner Cases

### Sub-mm Moves

Do not route single-target or short sub-mm corrections through direct contour IK.

Reason:

- Existing config and Jacobian fallback are tuned for these.
- Direct dense-contour assumptions do not apply.

### Sparse Paths

Sparse paths may not benefit from direct IK and may be better served by existing MoveIt behavior.

Use point count and total length thresholds.

### Duplicate Points

Handle duplicate Cartesian points before IK:

- Drop exact duplicates if pose is identical.
- Preserve intentional dwell only if the request format later supports dwell time explicitly.

### Very Sharp Corners

Sharp corners can be geometrically accurate but dynamically difficult.

Validation should:

- Keep the corner point.
- Let Ruckig/TOTG slow down as needed.
- Reject if joint deltas around the corner exceed the configured limit.

Future enhancement:

- Add corner tagging and automatically densify around high curvature.

### Orientation Discontinuities

Per-waypoint orientations can wrap at `180/-180` degrees.

Validation should:

- unwrap orientation inputs before interpolation/comparison where appropriate.
- reject sudden orientation jumps unless explicitly requested.

### Joint Wrap And Wrist Flips

TRAC-IK may return an equivalent orientation on a different wrist branch.

Mitigation:

- seed from previous solution
- use `Distance`
- normalize equivalent angles to previous joint values
- reject large Joint 6 jumps
- log branch changes with index and pose

### Singularity Or Near-Singularity

Near singularities may produce large joint changes for tiny Cartesian moves.

Detection:

- high joint delta between adjacent contour points
- retry failures clustered around same path segment
- FK accuracy OK but continuity bad

Policy:

- reject and fallback to existing MoveIt path
- report index and pose

### Start Not At First Waypoint

Current `execute_path` has a plan-then-approach mode when the robot is far from the first waypoint.

Direct contour IK should initially only handle the contour portion when robot is already near the first waypoint.

If start distance exceeds `PATH_APPROACH_THRESHOLD_MM`:

- use existing plan-then-approach path first
- or fallback entirely to current MoveIt behavior

Combining direct contour IK with automatic approach can be a second phase.

### Workobject And Tool Changes

The contour solver must use the transformed base-frame pose after workobject/tool handling.

Reject if:

- active tool changes during queued execution
- workobject changes while a queued direct-IK trajectory is pending

Best practice:

- snapshot tool and workobject values at planning time
- store them with the queued task metadata

### Stale Robot State

Reject direct IK if the joint state used as seed is stale.

Add or reuse a freshness timestamp if not already available.

### Queueing And Preemption

Direct IK must respect existing motion generation/preemption logic.

If a newer command preempts planning:

- discard the result
- do not execute a trajectory computed from stale state

### Time Parameterization Failure

If Ruckig/TOTG fails:

- do not execute raw positions
- return existing time-parameterization error code
- include validation stats in logs for diagnosis

## Logging And Metrics

Add one timing line per phase:

```text
[TIMING] contour_ik prepare elapsed_s=...
[TIMING] contour_ik solve points=... fast_failures=... retries=... elapsed_s=...
[TIMING] contour_ik validate fk_max_mm=... ori_max_deg=... max_joint_step=... elapsed_s=...
[TIMING] contour_ik reduced_moveit anchors=... fraction=... elapsed_s=...
[TIMING] contour_ik total elapsed_s=...
```

Also log fallback reason:

```text
[CONTOUR_IK] fallback reason=fk_error index=...
```

## Test Plan

### Unit Tests

- waypoint duplicate removal
- angle unwrap / branch normalization
- FK error tolerance comparison
- joint continuity rejection
- validation report formatting

### Offline Regression Tests

Use recorded contours:

- smooth 698-point contour
- contour with sharp corner
- path near wrist wrap
- path near singularity
- path with duplicate points
- path with per-waypoint orientation, if supported

For each contour compare:

- planning time
- max FK position error
- max FK orientation error
- max joint step
- execution acceptance

### Hardware Dry Run

Before cutting or painting:

- run at low velocity
- monitor `/cartesian_position`
- compare measured TCP trace against requested contour
- verify stop/preemption still works

## Rollout Plan

1. [x] Add disabled config and no-op selection path.
2. [x] Add direct IK module and validation reports.
3. [x] Add service-based prototype behind `CONTOUR_DIRECT_IK_ENABLED`.
4. [ ] Add reduced MoveIt validation.
5. [ ] Run offline contour tests.
6. [ ] Run low-speed hardware test.
7. [x] Add batched helper node if service-based prototype is still too slow.
8. [x] Add FK-checked joint smoothing for the batched helper.
9. [ ] Enable only for the active contour workflow after measured accuracy is acceptable.

## Implementation Notes

- Batch contour IK helper is implemented as an in-process MoveIt node and integrated into the Python planner.
- ZeroErr runtime has direct contour IK enabled for testing; Fairino config remains disabled by default.
- A 360-degree painting rotation around Joint 6 is expected for shaft/workpiece painting and must not be rejected solely because Joint 6 spans about `2*pi`.
- Direct contour IK now has relaxed full-turn span/endpoint limits only for configured full-turn joints (`Joint_6` on ZeroErr, `j6` on Fairino). Other joints keep the stricter span checks.
- Explicit `/unwind/joint6` is the cable/vacuum/camera hose relief move after a wrapped paint path. It preserves the continuous Joint 6 target, splits large unwinds into smaller controller waypoints, and verifies the final measured continuous Joint 6 angle after controller success.
- Verification intentionally does not accept modulo-equivalent angles. If Joint 6 remains near `-2*pi` when the target is near `0`, the unwind is considered failed because the cable is still wrapped.
- Queued explicit unwind requests compute the target from the latest Joint 6 state when the queue reaches the unwind task. This avoids using a stale pre-paint joint state if the UI calls `/unwind/joint6` while a paint trajectory is still executing.
- Direct contour IK smoothing is conservative: it only adjusts internal joint points, preserves endpoints, FK-checks every accepted adjustment against the original pose, and then validates the whole final trajectory again before returning it to Python.
- Expected logs after restart include `Contour IK smoothing accepted ...` and `curvature=...->... smoothing_updates=... smoothing_s=...`. After the candidate-scored IK update, the key success signal is lower initial curvature before smoothing and no forced `optimizer=RUCKIG` unless explicitly requested.
- Paint contour wiggle investigation found that post-projection cusp removal conflicts with the desired accuracy model: the projected path should be a faithful consequence of the source contour. The projection-stage cusp function is now diagnostic-only and no longer mutates robot command points.
- Resampling wiggle handling moved upstream into the paint contour interpolation path after the 1 mm execution resample and before pivot projection. It removes only tiny local source-contour samples when the bridge stays within a spacing-derived tolerance capped at `0.10 mm`, and logs the count/max bridge error when it acts.
- Path-preparation debug plot generation was moved off the synchronous execution-plan path. Canonicalization, contour reorder, contour pipeline, and interpolated path plots are now queued to a single daemon background worker with a bounded queue; if plotting falls behind, only the debug plot is dropped and motion planning continues.
- Path-preparation clarity/performance cleanup: `default_workpiece_path_preparation_service.py` now uses `execution_path`/`total_execution_points` internally instead of the misleading `execution_spline` local names, while preserving the public `total_spline_pts` field for compatibility. Homography preview calibration objects are resolved once per plan instead of once per segment, and sampled preview storage now copies from the final execution path instead of rebuilding another equivalent copy.
- A later single-target staging move failed at the drive with `0x8400 velocity error exceeds the limit value` on the final joint. The timed trajectory moved Joint 6 by about `1.54 rad` in `0.785 s`, with local intervals above the drive's practical tracking limit. Single-target trajectories now get a post-TOTG Joint 6 rate guard: timestamps are stretched and velocity/acceleration fields are scaled when configured joint interval rates exceed `SINGLE_TARGET_JOINT_RATE_LIMITS_RAD_S`.
- Paint executor performance: the pickup-stage planner already computes the first job's projected pivot path, including carried source rotation. `_execute_pivot_paths` now reuses that cached projection when executing the same source path instead of projecting the same contour again. `_pivot_source_path` also avoids copying already-list-backed paths. Expected effect is removing one full `project_execution_path` pass from the common one-job pickup-and-paint cycle.

## Acceptance Criteria

For the 698-point contour:

- direct contour planning time is significantly below current `compute_cartesian_path` time
- max FK position error is within configured tolerance
- no joint continuity violations
- no unexpected wrist branch flips
- time parameterization remains below about 0.2 s
- controller accepts and executes the timed trajectory
- fallback to existing MoveIt path works cleanly when direct IK is rejected
