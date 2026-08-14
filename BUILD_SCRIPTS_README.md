# ROS2 Workspace Build Scripts

This directory contains helper scripts for building and launching your ROS2 workspace.

## Scripts Overview

### 1. `clean_build.sh` - Complete Clean Build
Performs a full clean and rebuild of the entire workspace.

**Usage:**
```bash
./clean_build.sh
```

**What it does:**
- Removes `build/`, `install/`, and `log/` directories
- Sources ROS2 Rolling base installation
- Builds all packages from scratch
- Verifies critical packages and libraries
- Checks library paths

**When to use:**
- After pulling major changes from git
- When experiencing strange build errors
- After modifying CMakeLists.txt or package.xml files
- First time setup

---

### 2. `quick_build.sh` - Incremental Build
Rebuilds packages without cleaning (faster for development).

**Usage:**
```bash
# Build all packages
./quick_build.sh

# Build specific packages only
./quick_build.sh fairino5_v6_moveit2_config
./quick_build.sh fairino_hardware fairino_msgs
```

**What it does:**
- Sources ROS2 Rolling
- Builds specified packages (or all if none specified)
- Uses existing build artifacts for faster compilation

**When to use:**
- During iterative development
- After modifying source code
- When you only changed specific packages

---

### 3. `build_zeroerr.sh` - Build Overlay Workspace
Builds only the overlay workspace packages (`erob_moveit_runtime` and `zeroerr`).

**Usage:**
```bash
# Build all overlay packages
./build_zeroerr.sh

# Build specific overlay packages
./build_zeroerr.sh erob_moveit_runtime
```

**What it does:**
- Sources ROS2 Rolling base installation
- Sources base workspace (`install/local_setup.bash`) for dependencies
- Builds overlay packages in `eRob_moveit/` workspace
- Uses `--symlink-install` for Python changes to take effect immediately

**When to use:**
- When working only on overlay packages (`erob_moveit_runtime`, `zeroerr`)
- Faster than full rebuild when base workspace hasn't changed
- After modifying Python scripts in `erob_moveit_runtime`

**Note:** This script requires the base workspace to be built first.

---

### 4. `launch_robot.sh` - Launch with Environment Check
Launches the robot system with proper environment setup.

**Usage:**
```bash
# Launch default full_stack.launch.py
./launch_robot.sh

# Launch specific launch file
./launch_robot.sh full_stack.launch.py

# Launch with additional arguments
./launch_robot.sh full_stack.launch.py use_sim_time:=true
```

**What it does:**
- Sources ROS2 and workspace environment
- Verifies critical libraries exist
- Launches the specified launch file

**When to use:**
- Instead of manually sourcing and running ros2 launch
- To ensure environment is properly set up

---

## Typical Workflow

### Initial Setup
```bash
cd /home/ilv/ros2_ws
./clean_build.sh
```

### Daily Development (Base Workspace)
```bash
# Make code changes...

# Quick rebuild (all packages)
./quick_build.sh

# Or rebuild specific package
./quick_build.sh fairino_hardware
./quick_build.sh erob_moveit_runtime

# Launch robot
./launch_robot.sh
```

### Daily Development (Overlay/ZeroErr Only)
```bash
# After modifying only overlay packages (erob_moveit_runtime, zeroerr)
./build_zeroerr.sh

# Launch ZeroErr robot (edit eRob_moveit/robot_launch.conf first)
./launch_robot.sh
```

### After Git Pull
```bash
./clean_build.sh
./launch_robot.sh
```

### Rebuild Single Package
```bash
# After editing Python scripts in erob_moveit_runtime
./quick_build.sh erob_moveit_runtime

# After editing fairino5_v6_moveit2_config
./quick_build.sh fairino5_v6_moveit2_config
```

---

## Manual Commands (Alternative)

If you prefer manual control:

### Complete Clean Build
```bash
cd /home/ilv/ros2_ws
rm -rf build install log
source /opt/ros/rolling/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Quick Rebuild
```bash
cd /home/ilv/ros2_ws
source /opt/ros/rolling/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Launch
```bash
source /home/ilv/ros2_ws/install/setup.bash
ros2 launch fairino5_v6_moveit2_config full_stack.launch.py
```

---

## Troubleshooting

### Missing `rosidl_default_generators` Error
If you see this CMake error during build:
```
By not providing "Findrosidl_default_generators.cmake" in CMAKE_MODULE_PATH
this project has asked CMake to find a package configuration file provided
by "rosidl_default_generators"...
```

Install the missing package:
```bash
sudo apt update && sudo apt install -y ros-rolling-rosidl-default-generators
```

Then rebuild:
```bash
./build_zeroerr.sh
```

### "libfairino.so.2 not found" Error
This library must exist at `install/fairino_hardware/lib/libfairino.so.2`. Run a clean build:
```bash
./clean_build.sh
```

### Build Fails on Specific Package
Clean build just that package:
```bash
rm -rf build/PACKAGE_NAME install/PACKAGE_NAME
./quick_build.sh PACKAGE_NAME
```

### Overlay/Underlay Workspace Conflict Warning
If you see warnings about packages being built in multiple workspaces:
```
WARNING: Some selected packages are already built in one or more underlay workspaces
```

This is normal for `erob_moveit_runtime` which exists in both workspaces. To suppress, add `--allow-overriding erob_moveit_runtime` to the colcon command (already handled by `quick_build.sh`).

### Environment Not Set Up
Always source the **overlay** workspace (it chains to the base underlay):
```bash
source /home/ilv/ros2_ws/eRob_moveit/install/setup.bash
```

For convenience, add to your `~/.bashrc`:
```bash
echo "source /home/ilv/ros2_ws/eRob_moveit/install/setup.bash" >> ~/.bashrc
```

### FastDDS Shared Memory Issues
If ROS2 nodes fail to communicate, clean shared memory:
```bash
rm -rf /dev/shm/fastdds_* /dev/shm/sem.fastdds_*
```

---

## Workspace Structure

The project uses a **stacked workspace** layout with an underlay (base) and overlay:

| Workspace | Path | Packages |
|-----------|------|----------|
| **Base (underlay)** | `/home/ilv/ros2_ws/` | `fairino_msgs`, `fairino_hardware`, `fairino_description` |
| **Overlay** | `/home/ilv/ros2_ws/eRob_moveit/` | `erob_moveit_runtime`, `fairino5_v6_moveit2_config`, `zeroerr` |

The overlay automatically sources the base workspace, so you only need to source the overlay:
```bash
source eRob_moveit/install/setup.bash
```

## Package Dependencies

The workspace includes these key packages:
- **fairino_msgs** - Custom ROS2 messages/services (`ApplyIPP.srv`)
- **fairino_hardware** - ros2_control hardware interface for Fairino robots (C++)
- **fairino_description** - URDF/Meshes for Fairino5 v6 robot model
- **erob_moveit_runtime** - Shared runtime (Python/C++): motion planning, execution, safety, REST API
- **fairino5_v6_moveit2_config** - MoveIt2 config for Fairino robot
- **zeroerr** - MoveIt2 config for eRobo3 (ZeroErr) EtherCAT-driven arm

### Build Order
The build order is critical and handled automatically by the build scripts:
1. `fairino_msgs` (message definitions)
2. Base workspace packages
3. Overlay workspace packages

**Important:** The overlay (`eRob_moveit/`) must be built after the base workspace.

---

## Tips

1. **Parallel builds**: The scripts use all CPU cores by default. To limit:
   ```bash
   colcon build --parallel-workers 4
   ```

2. **Only rebuild changed packages**:
   ```bash
   colcon build --packages-up-to PACKAGE_NAME
   ```

3. **See detailed output**:
   ```bash
   colcon build --event-handlers console_direct+
   ```

4. **Build in debug mode** (for development):
   Edit the scripts and change:
   ```bash
   -DCMAKE_BUILD_TYPE=Release
   ```
   to:
   ```bash
   -DCMAKE_BUILD_TYPE=Debug
   ```