# Full Stack Launch Refactor Plan

## Goal

Make `zeroerr/launch/full_stack.launch.py` easier to maintain without changing launch behavior.

The current launch file mixes:

- runtime/profile YAML parsing
- path resolution
- CPU affinity policy
- fake hardware startup sequencing
- MoveIt configuration
- node construction
- process ordering

The refactor should separate these concerns while keeping `full_stack.launch.py` as the production orchestrator.

## Target Structure

Create a Python helper package inside the installed `launch/` directory:

```text
zeroerr/
  launch/
    full_stack.launch.py
    ethercat_only.launch.py
    move_group.launch.py
    moveit_rviz.launch.py
    ros2_controllers.launch.py
    rsp.launch.py
    spawn_controllers.launch.py

    zeroerr_launch/
      __init__.py
      runtime_config.py
      cpu_policy.py
      moveit_config.py
      fake_hardware.py
      nodes.py
```

`zeroerr/CMakeLists.txt` already installs the full `launch/` directory:

```cmake
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
```

So modules under `launch/zeroerr_launch/` will be available from installed launch files.

## Design Rules

- Keep `full_stack.launch.py` as the top-level production launcher.
- Do not split into many launch files just for neatness.
- Split launch files only when the piece is useful to run independently.
- Keep `zeroerr_rt_policy.env` as the single source of truth for CPU policy.
- Launch Python should consume exported CPU policy values; it should not rediscover policy from `/sys` or `/proc/cmdline`.
- Keep endpoint/runtime behavior unchanged.
- Keep fake hardware behavior unchanged.
- Keep launch argument names unchanged.
- Keep node names unchanged unless there is a separate migration plan.

## Phase 1: Extract Runtime Config Helpers

Create:

```text
launch/zeroerr_launch/runtime_config.py
```

Move these helpers out of launch files:

```python
load_runtime_config(package_path: str) -> dict
resolve_config_path(config_yaml: str, value: str) -> str
urdf_path_from_runtime(package_path: str) -> str
srdf_path_from_runtime(package_path: str) -> str | None
runtime_value(package_path: str, key: str, default)
load_state_publisher_params(package_path: str) -> dict
```

Also include profile-aware config merging:

```text
config/runtime.yaml
config/<profile>/runtime.yaml
config/contour_ik_config.yaml
config/ptp_config.yaml
config/<profile>/contour_ik_config.yaml
config/<profile>/ptp_config.yaml
config/erob_state_publisher_config.yaml
config/<profile>/erob_state_publisher_config.yaml
```

Then update these launch files to import the shared helpers:

```text
full_stack.launch.py
ethercat_only.launch.py
move_group.launch.py
ros2_controllers.launch.py
rsp.launch.py
```

Acceptance criteria:

- No duplicate `_load_runtime_config()` remains in ZeroErr launch files.
- No duplicate `_resolve_config_path()` remains in ZeroErr launch files.
- `full_stack.launch.py` still resolves profile URDF/SRDF paths the same way.
- `python3 -m py_compile` passes for all affected launch files.

## Phase 2: Extract CPU Policy Helpers

Create:

```text
launch/zeroerr_launch/cpu_policy.py
```

The launch files should only consume exported policy values:

```text
ZEROERR_NON_RT_CORES
ZEROERR_PLANNER_CORES
ZEROERR_LOW_PRIORITY_CORES
ZEROERR_CONTROL_CORES
ZEROERR_SERVO_LOW_CPU
ZEROERR_SERVO_REALTIME
ZEROERR_SERVO_PERIOD
```

Do not read these from launch Python anymore:

```text
/sys/devices/system/cpu/online
/sys/devices/system/cpu/isolated
/proc/cmdline
```

Those lookups belong in:

```text
zeroerr_rt_policy.env
launch_robot.sh
EtherCatStart.sh
```

Suggested API:

```python
from dataclasses import dataclass

@dataclass(frozen=True)
class CpuPolicy:
    non_rt_cores: str
    planner_cores: str
    low_priority_cores: str
    control_cores: str
    non_rt_prefix: str
    planner_prefix: str
    low_priority_prefix: str
    control_prefix: str

def load_cpu_policy() -> CpuPolicy:
    ...
```

Conservative fallback behavior:

```text
ZEROERR_NON_RT_CORES        -> "0"
ZEROERR_PLANNER_CORES       -> ZEROERR_NON_RT_CORES
ZEROERR_LOW_PRIORITY_CORES  -> ZEROERR_NON_RT_CORES
ZEROERR_CONTROL_CORES       -> "0"
```

Acceptance criteria:

- No `/sys/devices/system/cpu/*` reads remain in ZeroErr launch Python files.
- No `/proc/cmdline` reads remain in ZeroErr launch Python files.
- Launch files still use the same exported env variable names.
- `zeroerr_rt_policy.env` remains the single source of truth.

## Phase 3: Extract MoveIt Config Builder

Create:

```text
launch/zeroerr_launch/moveit_config.py
```

Suggested API:

```python
def build_moveit_config(
    package_name: str,
    package_path: str,
    *,
    use_fake_hardware: str | bool | None = None,
):
    ...
```

This helper should centralize:

- `MoveItConfigsBuilder("eRobo3", package_name="zeroerr")`
- runtime URDF resolution
- runtime SRDF resolution
- `use_fake_hardware` xacro mapping when needed
- planning pipeline list

Acceptance criteria:

- `full_stack.launch.py`, `move_group.launch.py`, `rsp.launch.py`, and `ros2_controllers.launch.py` no longer duplicate MoveIt config construction logic.
- Fake hardware xacro mapping remains available for `full_stack.launch.py`.
- Real hardware behavior remains unchanged.

## Phase 4: Extract Fake Hardware Startup Sequence

Create:

```text
launch/zeroerr_launch/fake_hardware.py
```

Move this behavior out of `full_stack.launch.py`:

- wait for one `/cartesian_position` sample in fake hardware mode
- start `zeroerr_runtime` only after fake Cartesian state is available
- keep real hardware runtime startup timing unchanged

Suggested API:

```python
def add_runtime_startup_actions(
    ld,
    *,
    use_fake_hardware,
    runtime_node,
    state_publisher_node,
):
    ...
```

Acceptance criteria:

- Fake hardware still waits for `/cartesian_position --once` before starting runtime.
- Real hardware still starts runtime on the existing timer.
- `zeroerr_state_publisher` startup timing remains unchanged.
- No fake-specific runtime startup orchestration remains inline in `full_stack.launch.py`.

## Phase 5: Extract Node Factory Helpers

Create:

```text
launch/zeroerr_launch/nodes.py
```

Move repeated node construction into functions.

Suggested APIs:

```python
def make_robot_state_publisher(moveit_config): ...
def make_move_group(moveit_config, cpu_policy): ...
def make_servo_node(package_path, moveit_config, cpu_policy): ...
def make_rviz(package_path, moveit_config, cpu_policy): ...
def make_ros2_control_node(package_path, use_fake_hardware, cpu_policy): ...
def make_controller_spawners(use_fake_hardware): ...
def make_runtime_node(package_path, cpu_policy): ...
def make_state_publisher(package_path, cpu_policy): ...
def make_motion_helper_nodes(moveit_config, cpu_policy): ...
def make_zeroerr_diagnostics_nodes(package_path, use_fake_hardware, cpu_policy): ...
```

Keep timers and event ordering in `full_stack.launch.py` unless moving them clearly improves readability.

Acceptance criteria:

- Node names, packages, executables, parameters, remappings, prefixes, and conditions are unchanged.
- `full_stack.launch.py` reads as orchestration rather than construction detail.
- Launch behavior remains unchanged.

## Phase 6: Optional Launch File Splitting

Only split additional launch files when they are useful independently.

Possible future files:

```text
description.launch.py     # robot_state_publisher + virtual joints
planning.launch.py        # move_group + planning/helper nodes
control.launch.py         # ros2_control + controller spawners
runtime.launch.py         # zeroerr_runtime + zeroerr_state_publisher
diagnostics.launch.py     # SDO server + error monitor + drive diagnostics
```

Do not do this before helper extraction. Otherwise the duplicated helper logic spreads further.

Acceptance criteria:

- Each split launch file has a clear standalone use.
- `full_stack.launch.py` includes or composes them without changing startup order.
- Operators can still use `ros2 launch zeroerr full_stack.launch.py`.

## Suggested Final `full_stack.launch.py` Shape

```python
def generate_launch_description():
    ctx = build_launch_context()

    ld = LaunchDescription()
    add_launch_arguments(ld, ctx)
    add_environment(ld, ctx)
    add_description_actions(ld, ctx)
    add_moveit_actions(ld, ctx)
    add_control_actions(ld, ctx)
    add_runtime_actions(ld, ctx)
    add_helper_actions(ld, ctx)
    add_diagnostics_actions(ld, ctx)
    return ld
```

The final file should make startup order obvious.

## Verification

Syntax-check launch files:

```bash
python3 -m py_compile \
  zeroerr/launch/full_stack.launch.py \
  zeroerr/launch/ethercat_only.launch.py \
  zeroerr/launch/move_group.launch.py \
  zeroerr/launch/moveit_rviz.launch.py \
  zeroerr/launch/ros2_controllers.launch.py \
  zeroerr/launch/rsp.launch.py \
  zeroerr/launch/spawn_controllers.launch.py \
  zeroerr/launch/zeroerr_launch/*.py
```

Search for duplicated config helpers:

```bash
rg "_load_runtime_config|_resolve_config_path|_runtime_urdf_path|_urdf_path_from_runtime" zeroerr/launch
```

Search for launch-side CPU discovery:

```bash
rg "/sys/devices/system/cpu|/proc/cmdline|_kernel_isolated_cores|_default_non_rt_cores|_default_control_cores" zeroerr/launch
```

Build:

```bash
source /opt/ros/jazzy/setup.bash
source /home/ilv/ros2_ws/install/local_setup.bash
cd /home/ilv/ros2_ws/eRob_moveit
colcon build --packages-select zeroerr --allow-overriding zeroerr
```

Optional launch dry checks:

```bash
source /opt/ros/jazzy/setup.bash
source /home/ilv/ros2_ws/eRob_moveit/install/local_setup.bash
ros2 launch zeroerr full_stack.launch.py --show-args
ros2 launch zeroerr full_stack.launch.py use_fake_hardware:=true --show-args
```

## Risk Notes

- CPU policy changes are operationally sensitive. Keep env variable names stable.
- Fake hardware startup sequencing is behavior-sensitive. Move it mechanically before changing it.
- `ros2_control_node` should not be wrapped in whole-process FIFO scheduling unless separately justified; controller manager owns its real-time thread.
- Do not remove `full_stack.launch.py`; it is the production launcher.
