# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS2-based robotic manipulation system for the **Fairino5 v6 6-DOF collaborative robot arm**, integrating MoveIt2 for motion planning. Provides both a REST API bridge and a direct ROS2 Python interface that mirrors the original Fairino SDK.

## Commands

### Build

```bash
# Build all packages (two-step: messages first, then rest)
./quick_build.sh

# Build a specific package
./quick_build.sh fairino_hardware

# After build, source the workspace
source install/setup.bash
```

The two-step build order matters: `fairino_msgs` must be built before other packages that depend on it.

### Launch

```bash
# Launch the full robot stack (MoveIt2 + GUI + RViz)
./launch_robot.sh

# Start the REST API bridge server (separate terminal, after launch_robot.sh)
python3 fairino_bridge_server.py
```

Startup takes ~40 seconds due to collision geometry processing.

### Environment

```bash
# Required environment variables (set by launch_robot.sh)
export ROS_DOMAIN_ID=42        # DDS isolation
export ROS_LOCALHOST_ONLY=1    # Local only
export DISPLAY=:1              # X11 for RViz/GUI
source /opt/ros/rolling/setup.bash
source install/setup.bash
```

Always unset stale env vars before sourcing (see `launch_robot.sh` for the pattern). Clean FastDDS shared memory between sessions: `rm -rf /dev/shm/fastdds_* /dev/shm/sem.fastdds_*`

## Architecture

### Packages

| Package | Language | Purpose |
|---------|----------|---------|
| `fairino_msgs` | IDL | Custom service/message definitions (ApplyIPP) |
| `fairino_hardware` | C++ | ros2_control hardware interface; connects to robot via libfairino.so.2 |
| `fairino_description` | URDF/Meshes | Robot model; meshes used for workspace boundary extraction |
| `fairino5_v6_moveit2_config` | Python/Launch | MoveIt config, all Python scripts, launch files |

### Python Scripts (all under `eRob_moveit/src/eRob_ROS2_MoveIt/fairino5_v6_moveit2_config/scripts/`)

**Core Node:**
- `robot_controller.py` — Main `rclpy.Node` (`RobotController`). Owns motion queue, tool/workobject state, and delegates to all submodules.
- `fairino_ros2_robot.py` — Drop-in SDK-compatible wrapper around `RobotController`. Used by bridge server and external scripts.

**Motion subsystem (`motion/`):**
- `trajectory_planner.py` — Calls MoveIt's `/compute_cartesian_path` service. Configurable time parameterization via `TIME_PARAMETERIZATION = "TOTG" | "RUCKIG"` constant at top of file.
- `trajectory_optimization.py` — Calls C++ helper nodes (`ipp_helper` for TOTG, `ruckig_helper` for Ruckig S-curves) via `/apply_ipp` service.
- `trajectory_executor.py` — Sends trajectories to `/fairino5_controller/follow_joint_trajectory` action server.
- `jog_controller.py` — Incremental Cartesian jogging.
- `motion_queue.py` — Sequential execution queue (max 10 tasks); prevents competing motions.

**Safety subsystem (`safety/`):**
- `safety_wall_manager.py` — Validates waypoints against workspace boundaries (extracted from URDF mesh). Publishes visualization markers.
- `collision_detection/inverse_dynamics_model.py` — KDL-based RNEA for expected torque computation. **Currently disabled** (false positives due to 6.6× gravity model error in URDF).
- `collision_detection/dynamics_collision_detector.py` — Compares measured vs expected torques. Enable via `robot_controller.enable_collision_detection()`.

**Status subsystem (`status/`):**
- `robot_monitor.py` — Subscribes to C++ publisher topics (`/cartesian_position`, `/cartesian_velocity`, `/cartesian_acceleration`, `/joint_velocity`, `/joint_acceleration`) at 50 Hz. Provides cached state queries.
- `robot_status_publisher.py` — Publishes `/robot_status` at 10 Hz.

**Utilities:**
- `utils/transformation_utils.py` — Euler↔quaternion, pose↔transform matrix, TCP offset math, TF2 helpers.
- `utils/work_object.py` — User coordinate frame management and pose transforms.
- `tools/tool_manager.py` — Tool frame registry (`TOOL_0` = base, `TOOL_1` = [0.081, -7.250, 0, 0, 0, 0] mm offset).

### C++ Nodes

- **`fairino_hardware_interface.cpp`** — `ros2_control` SystemInterface. Command interface: joint positions. State interfaces: position, velocity, acceleration, effort. Connects to physical robot via RPC. Subscribes to `/set_do` for digital output control.
- **`robot_state_publisher`** (compiled separately) — Publishes `/cartesian_position`, `/cartesian_velocity`, `/cartesian_acceleration`, `/joint_velocity`, `/joint_acceleration` at 50 Hz using KDL FK.
- **`ipp_helper`** / **`ruckig_helper`** — Trajectory optimization service nodes, launched by `demo.launch.py`.

### Motion Command Data Flow

```
REST POST /move/cartesian
  → fairino_bridge_server.py
  → FairinoRos2Robot.move_cartesian()   (applies workobject + tool transforms)
  → RobotController.send_cartesian_goal()
  → trajectory_planner.send_cartesian_goal()
      → safety_wall_manager.check_position_safety()
      → /compute_cartesian_path (MoveIt service)
      → trajectory_optimization.apply_ipp_totg()  → /apply_ipp (ipp_helper)
      → trajectory_executor._send_trajectory_to_controller()
          → /fairino5_controller/follow_joint_trajectory (action)
  → hardware executes → publishes /joint_states
```

### Key ROS2 Interfaces

**Topics published by C++ layer:**
- `/joint_states` — 100 Hz, from hardware interface
- `/cartesian_position`, `/cartesian_velocity`, `/cartesian_acceleration` — 50 Hz
- `/joint_velocity`, `/joint_acceleration` — 50 Hz

**Services:**
- `/compute_cartesian_path` (MoveIt `GetCartesianPath`)
- `/compute_fk` (MoveIt `GetPositionFK`, used for FK diagnostics)
- `/apply_ipp` (custom `fairino_msgs/ApplyIPP`, served by `ipp_helper`)

**Actions:**
- `/fairino5_controller/follow_joint_trajectory` — Hardware controller action server

### MoveIt Configuration

- **Planner:** Pilz only (CHOMP/OMPL/STOMP disabled for faster startup)
- **Trajectory parameterization:** TOTG by default; switch to Ruckig for smoother S-curve profiles
- **Adaptive step sizing:** 0.8–1.5 mm based on path complexity

## Error Codes

See [`docs/error_codes.md`](docs/error_codes.md) for the full table of internal motion error codes (`0`, `-1` through `-10`, `> 0` for queued), their meanings, HTTP status mappings, and which source file generates each code.

## Known Issues / Important Notes

- **Collision detection is disabled** (`dynamics_collision_detector.py`). The URDF gravity model is 6.6× incorrect vs. what the SDK compensates for, causing false positives.
- **Build order is strict**: `fairino_msgs` → everything else.
- **libfairino.so.2** must exist at `install/fairino_hardware/lib/libfairino.so.2` or launch fails.
- When modifying Python scripts, `--symlink-install` means changes take effect without rebuilding. C++ changes require a rebuild.
- The `ws_moveit2` workspace paths are explicitly filtered from `PYTHONPATH`/`AMENT_PREFIX_PATH` to avoid conflicts.