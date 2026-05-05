# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS2-based robotic manipulation system supporting two robot arms:
- **Fairino5 v6** — 6-DOF collaborative arm, connected via libfairino.so.2
- **eRobo3 (ZeroErr)** — EtherCAT-driven arm

Both share a common Python runtime package (`erob_moveit_runtime`) and use MoveIt2 + Pilz for motion planning, plus a REST API bridge.

## Workspace Layout

Two separate colcon workspaces, stacked as underlay/overlay:

| Workspace | Path | Purpose |
|-----------|------|---------|
| Base | `/home/ilv/ros2_ws/` | Hardware/vendor packages (`fairino_msgs`, `fairino_hardware`, `fairino_description`) |
| Overlay | `/home/ilv/ros2_ws/eRob_moveit/` | Shared runtime + per-robot MoveIt configs (`erob_moveit_runtime`, `fairino5_v6_moveit2_config`, `zeroerr`) |

## Commands

### Build

```bash
# Build everything (3-step: fairino_msgs → base packages → overlay)
./quick_build.sh

# Build only overlay packages (erob_moveit_runtime, zeroerr)
./build_zeroerr.sh

# Build a specific package (script routes to correct workspace automatically)
./quick_build.sh erob_moveit_runtime
./quick_build.sh fairino_hardware

# After build, source only the overlay (it chains to base underlay)
source eRob_moveit/install/setup.bash
```

Build order is enforced by `quick_build.sh`: `fairino_msgs` → base workspace → overlay packages. The overlay workspace sources the base `local_setup.bash` before building.

**Important:** If you encounter `rosidl_default_generators` CMake errors, install the missing package:
```bash
sudo apt install -y ros-rolling-rosidl-default-generators
```

### Launch

```bash
# Launch the robot stack (reads eRob_moveit/robot_launch.conf)
./launch_robot.sh
```

**`eRob_moveit/robot_launch.conf`** controls which robot is launched — edit `ROBOT_TYPE=fairino|zeroerr` and `ZEROERR_PROFILE=ethercat_only|full`. Set `BUILD_WORKSPACE=1` to auto-build before launching.

Startup takes ~40 seconds due to collision geometry processing.

### Environment

```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
export DISPLAY=:1
source /opt/ros/rolling/setup.bash
source eRob_moveit/install/setup.bash   # overlay (includes base underlay)
```

Clean FastDDS shared memory between sessions: `rm -rf /dev/shm/fastdds_* /dev/shm/sem.fastdds_*`

## Architecture

### Packages

| Package | Workspace | Language | Purpose |
|---------|-----------|----------|---------|
| `fairino_msgs` | base | IDL | Custom `ApplyIPP` service definition |
| `fairino_hardware` | base | C++ | ros2_control hardware interface; RPC to robot via libfairino.so.2 |
| `fairino_description` | base | URDF/Meshes | Fairino5 v6 robot model; meshes used for workspace boundary extraction |
| `erob_moveit_runtime` | overlay | Python/C++ | **Shared runtime**: all Python scripts, C++ helper nodes, `ApplyIPP.srv` |
| `fairino5_v6_moveit2_config` | overlay | Launch/YAML | Fairino-specific MoveIt config, URDF, `config/runtime.yaml` |
| `zeroerr` | overlay | Launch/YAML | eRobo3-specific MoveIt config, URDF, EtherCAT scripts, `config/runtime.yaml` |

### Python Scripts (`erob_moveit_runtime/scripts/`)

All runtime Python lives in the shared `erob_moveit_runtime` package, not in the robot config packages.

**Core Node:**
- `robot_controller.py` — Main `rclpy.Node` (`RobotController`). Owns motion queue, tool/workobject state, and delegates to all submodules.
- `fairino_ros2_robot.py` — Drop-in SDK-compatible wrapper implementing `IRobotBackend`. Used by REST layer.
- `zeroerr_ros2_robot.py` — Thin alias; imports `MoveItRobotBackend` for the ZeroErr robot.

**Backend abstraction (`backend/`):**
- `i_robot_backend.py` — `IRobotBackend` ABC defining the common robot interface.
- `moveit_robot_backend.py` — Concrete MoveIt implementation used by both robots.
- `backend_factory.py` — Selects backend based on `ROBOT_BACKEND` in runtime config.

**Motion subsystem (`motion/`):**
- `motion/planning/trajectory_planner.py` — Calls `/compute_cartesian_path`. Time parameterization controlled by `TRAJECTORY_OPTIMIZER` in `runtime.yaml`.
- `motion/planning/jacobian_move.py` — Incremental Cartesian jogging via Jacobian IK.
- `motion/execution/trajectory_optimization.py` — Calls `/apply_ipp` (ipp_helper for TOTG, ruckig_helper for S-curves).
- `motion/execution/trajectory_executor.py` — Sends to `/fairino5_controller/follow_joint_trajectory` action.
- `motion/execution/motion_queue.py` — Sequential queue (max `MOTION_QUEUE_MAX_SIZE`); prevents competing motions.
- `motion/strategies.py` — Routes single-target vs path moves to appropriate planning module.

**Safety subsystem (`safety/`):**
- `safety_wall_manager.py` — Validates waypoints against workspace boundaries (from URDF mesh). Publishes planning-scene walls.
- `collision_detection/dynamics_collision_detector.py` — **Currently disabled.** Compares measured vs expected torques.
- `collision_detection/inverse_dynamics_model.py` — KDL RNEA for expected torque. Disabled due to 6.6× gravity model error.

**Status subsystem (`status/`):**
- `robot_monitor.py` — Subscribes to C++ topics at 50 Hz; provides cached state.
- `robot_status_publisher.py` — Publishes `/robot_status` at 10 Hz.

**Utilities:**
- `utils/transformation_utils.py` — Euler↔quaternion, pose↔transform matrix, TCP offset math, TF2 helpers.
- `utils/work_object.py` — User coordinate frame management and pose transforms.

### C++ Nodes (in `erob_moveit_runtime/src_cpp/`)

- **`robot_state_publisher.cpp`** — Publishes `/cartesian_position`, `/cartesian_velocity`, `/cartesian_acceleration`, `/joint_velocity`, `/joint_acceleration` at 50 Hz using KDL FK.
- **`ipp_helper_node.cpp`** / **`ruckig_helper_node.cpp`** — Trajectory optimization service nodes serving `/apply_ipp`.
- **`fairino_hardware_interface.cpp`** (base ws) — ros2_control SystemInterface; joint position commands, state readback, `/set_do` digital output.

### Configuration

All runtime constants live in **`config/runtime.yaml`** inside each robot config package (`fairino5_v6_moveit2_config` or `zeroerr`). The shared `erob_moveit_runtime/scripts/config.py` module loads the correct file at startup based on the `EROB_CONFIG_PACKAGE` env var (set in launch files).

**To tune any parameter** — velocity defaults, safety margins, DH parameters, topic names, collision thresholds, trajectory optimizer — edit the robot's `config/runtime.yaml`. Do not hardcode values in scripts.

Key `runtime.yaml` groups:
- `TRAJECTORY_OPTIMIZER`: `TOTG` (default) or `RUCKIG`
- `DEFAULT_VEL_SCALING`, `DEFAULT_ACC_SCALING`, `DEFAULT_VEL_PERCENT`
- `SAFETY_MARGIN_M`, `WALL_XY_OFFSET_M`, `SAFETY_WALL_NAMES`
- `DH_D1`, `DH_A2`, `DH_A3`, `DH_D4`, `DH_D5` — Fairino5 v6 kinematics
- All topic/service/action names
- Collision detection thresholds and filter parameters

### Motion Command Data Flow

```
REST POST /move/cartesian
  → rest_server.py
  → FairinoRos2Robot (IRobotBackend impl)
      applies workobject + tool transforms
  → RobotController.send_cartesian_goal()
  → motion/strategies.py → motion/planning/trajectory_planner.py
      → safety_wall_manager.check_position_safety()
      → /compute_cartesian_path (MoveIt)
      → motion/execution/trajectory_optimization.py → /apply_ipp (ipp_helper)
      → motion/execution/trajectory_executor.py
          → /fairino5_controller/follow_joint_trajectory (action)
  → hardware executes → publishes /joint_states
```

### Key ROS2 Interfaces

**Topics (C++ layer, 50 Hz):** `/cartesian_position`, `/cartesian_velocity`, `/cartesian_acceleration`, `/joint_velocity`, `/joint_acceleration`

**Topics:** `/joint_states` (100 Hz, hardware), `/robot_status` (10 Hz, Python)

**Services:** `/compute_cartesian_path`, `/compute_fk`, `/compute_ik`, `/check_state_validity`, `/apply_ipp` (custom `fairino_msgs/ApplyIPP`)

**Actions:** `/fairino5_controller/follow_joint_trajectory`

### MoveIt Configuration

- **Planner:** Pilz only (CHOMP/OMPL/STOMP disabled for faster startup)
- **Trajectory parameterization:** TOTG by default; Ruckig for smoother S-curve profiles
- **Adaptive step sizing:** 0.8–1.5 mm based on path complexity

## Error Codes

See [`docs/error_codes.md`](docs/error_codes.md) for the full table of internal motion error codes (`0`, `-1` through `-10`, `> 0` for queued), HTTP mappings, and source locations. Also: `erob_moveit_runtime/docs/REST_API.md` and `erob_moveit_runtime/docs/MOTION_AND_SAFETY.md`.

## Known Issues / Important Notes

- **Collision detection is disabled** — URDF gravity model is 6.6× off vs. SDK compensation; causes false positives.
- **Build order is strict**: `fairino_msgs` → base ws → overlay ws.
- **libfairino.so.2** must exist at `install/fairino_hardware/lib/libfairino.so.2` or launch fails.
- Python changes take effect immediately (`--symlink-install`); C++ changes require rebuild.
- `ws_moveit2` workspace paths are explicitly filtered from `PYTHONPATH`/`AMENT_PREFIX_PATH` to avoid conflicts.
- ZeroErr launch waits for EtherCAT slaves to reach OP state before proceeding (`WaitForSlavesOp.sh`).