# Single-Robot Regression Suite (zeroerr)

A self-contained regression suite proving that the single-robot runtime stack
still works after the prepared/synchronized-execution refactor on the
`twin_robots` branch. It builds the runtime through the public construction
path, moves real (or fake) hardware, and reports results with explicit exit
codes.

This suite **only adds new test files** under `zeroerr/scripts/regression/`. It
does not modify any production code.

## Layout

| File | Purpose |
|------|---------|
| `common.py` | Shared runtime construction (`SingleRobotRuntime`), result-code map, CLI parser, env printing, helpers |
| `test_01_readiness.py` | Stack readiness: `wait_until_ready`, `startup_status`, `runtime_state_snapshot`, `state_snapshot`, ordered joint positions |
| `test_02_move_ptp.py` | `gateway.move_ptp` (safe-direction retry, live anchor) |
| `test_03_move_lin.py` | `gateway.move_linear` |
| `test_04_execute_path.py` | Public `execute_path` with 3D waypoints + `orientation_mode="constant"` |
| `test_05_execute_sequence.py` | `execute_sequence` with blend radius (known issue: move_group `GetMotionSequence` hangs on real requests) |
| `test_06_stop_motion.py` | `stop_motion` mid-active motion, queue overflow (`-5`), queue cleared, stack reusable |
| `test_07_ordered_motion_chain.py` | `execute_ordered_motion_chain` (ptp + linear), terminal chain status |
| `test_08_prepared_execution.py` | `prepare_path`/`execute_prepared`: live-anchor, `require_exact` mismatch (`-15`), `require_exact` success |
| `test_09_prepared_noop.py` | No-op prepared trajectory: must return `noop=True`, returns 0 without dispatching |
| `test_10_status_snapshots.py` | Read-only status/snapshot surface (`startup_status`, kinematics, drive, interlock, walls) |
| `test_11_static_profiles.py` | Pure-config validation of every profile (paint, welding, twin_robots) — no runtime; validates `PRIMARY_ROBOT` |
| `test_12_restart.py` | Sequential runtime close/reopen: tear down, fresh runtime, re-verify readiness + move |
| `run_all.sh` | Sequential runner for the full suite |

## Exit codes

| Code | Meaning |
|------|---------|
| `0` | PASS |
| `1` | FAIL |
| `2` | SETUP / environment problem |
| `3` | USAGE error |

## Environment

This machine runs **ROS 2 Jazzy** (not Rolling). Source the Jazzy setup and the
workspace install, then point the runtime at the zeroerr config package:

```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
source /opt/ros/jazzy/setup.bash
source /home/ventsi/fairino_ros2/install/setup.bash
export EROB_CONFIG_PACKAGE=zeroerr
export ZEROERR_USE_FAKE_HARDWARE=1
```

`ROS_DOMAIN_ID` and `ROS_LOCALHOST_ONLY` must match the values the launched
stack uses (see `eRob_moveit/zeroerr_launch.conf`).

`common.py` refuses to run motion tests unless `ZEROERR_USE_FAKE_HARDWARE` is in
`{1,true,yes,on}`, or `--allow-real-hardware` is passed. With fake hardware the
velocity/acceleration are clamped to 15%.

## Building and launching the stack

Build order is enforced by the workspace scripts. The regression suite uses the
installed (`--symlink-install`) Python sources, so no rebuild is needed for
Python-only changes:

```bash
# Build the overlay (zeroerr + erob_moveit_runtime)
./build_zeroerr.sh

# Launch the ZeroErr stack with fake hardware (reads zeroerr_launch.conf)
./launch_zeroerr.sh --fake
```

Real hardware launch requires the EtherCAT master prepared:

```bash
./launch_zeroerr.sh --real        # or plain ./launch_zeroerr.sh
```

Startup takes ~40 s while collision geometry is processed. Wait until
`/robot_status` or the motion stack reports ready before running the suite.

## Active profile (twin_robots vs paint/welding)

The loaded profile is selected by `ACTIVE_PROFILE` in
`zeroerr/config/runtime.yaml`:

- `twin_robots`: robots `robot1`/`robot2`, scoped joints and topics (every
  link/group/action carries a `robot1_`/`robot2_` prefix). Use
  `--robot robot1` or `--robot robot2`.
- `paint` / `welding`: single-robot profiles with unscoped joints
  (`Joint_1..Joint_6`), links and topics.

Both kinds of profile construct through the same
`RobotRuntimeContext.from_config()` path. The base `config/runtime.yaml`
defines a default logical robot identity for single-robot profiles:

```yaml
PRIMARY_ROBOT: robot
ROBOTS:
  robot:
    joint_names: [Joint_1, ..., Joint_6]
    planning_group: manipulator
    base_link: base_link
    ee_link: ee_link
    wrist_link: Link_6
    cartesian_source_link: ee_link
    collision_tip_link: tool0
    action_follow_trajectory: /manipulator_controller/follow_joint_trajectory
```

The name `robot` is purely logical — it never alters link/group/controller
names. Multi-robot profiles override `PRIMARY_ROBOT` and `ROBOTS` with their
own scoped definitions. (The earlier `e8f0b9c` regression — no resolvable
identity for paint/welding — is resolved by this default identity block.)

The suite wires topics and the active-TCP frame automatically: scoped when the
robot name prefixes its own planning-group/base-link (twin), unscoped
otherwise (paint/welding).

## Running

Single test:

```bash
cd eRob_moveit/src/eRob_ROS2_MoveIt/zeroerr/scripts/regression
python3 test_02_move_ptp.py --move-mm 5 --settle-s 1.0
```

Whole suite (static profile check runs first, then the motion tests):

```bash
./run_all.sh
```

`run_all.sh` resolves the default `--robot` from the active profile's
`PRIMARY_ROBOT` (`robot` under paint/welding, `robot1` under twin_robots) —
pass `--robot` explicitly to override. It aborts on a setup error (`2`),
continues after PASS/FAIL, and returns `0` (all pass) or `1` (any fail).

### CLI flags (all tests)

| Flag | Default | Meaning |
|------|---------|---------|
| `--robot NAME` | `PRIMARY_ROBOT` | Robot name (`robot` under paint/welding, `robot1`/`robot2` under twin_robots) |
| `--vel PCT` | per profile | Velocity percent (clamped to 15 under fake hardware) |
| `--acc PCT` | per profile | Acceleration percent |
| `--ready-timeout S` | `30` | Max seconds to wait for readiness |
| `--settle-s S` | `0.25` | Post-idle settle pause before joint-delta checks |
| `--move-mm MM` | `20` | Test displacement (tests 02–05, 08, 09) |
| `--move-deg DEG` | `5` | Test rotation (currently unused) |
| `--allow-real-hardware` | off | Bypass the fake-hardware guard |
| `--keep-going` | off | (reserved) |

## Safety notes

- Under fake hardware no robot can move; under real hardware the tests use
  small displacements with a safe-direction retry and a live-anchor start
  policy (`EXECUTOR_PREPARED_START_TOL_RAD` respected).
- The twin-mode `/joint_states` carries 12 joints; the suite filters positions
  by the robot's own joint names before comparing against prepared
  trajectories.
- `test_12_restart.py` runs two runtimes sequentially (never concurrently) to
  avoid rclpy re-initialisation issues on ROS 2 Jazzy.

## Known issues

- **test_05 (`execute_sequence`)** — move_group `/plan_sequence_path`
  (`GetMotionSequence`) hangs on any non-empty request. Empty requests
  succeed. `/compute_cartesian_path` and pilz both work fine. This is a
  move_group-side issue, not a harness defect; the test cannot pass until it
  is resolved upstream. Pending investigation.
