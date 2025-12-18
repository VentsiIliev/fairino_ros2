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

### 3. `launch_robot.sh` - Launch with Environment Check
Launches the robot system with proper environment setup.

**Usage:**
```bash
# Launch default demo.launch.py
./launch_robot.sh

# Launch specific launch file
./launch_robot.sh demo.launch.py

# Launch with additional arguments
./launch_robot.sh demo.launch.py use_sim_time:=true
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

### Daily Development
```bash
# Make code changes...

# Quick rebuild
./quick_build.sh

# Launch robot
./launch_robot.sh
```

### After Git Pull
```bash
./clean_build.sh
./launch_robot.sh
```

### Rebuild Single Package
```bash
# After editing velocity_monitor.py
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
ros2 launch fairino5_v6_moveit2_config demo.launch.py
```

---

## Troubleshooting

### "libfairino.so.2 not found" Error
Run a clean build:
```bash
./clean_build.sh
```

### Build Fails on Specific Package
Clean build just that package:
```bash
rm -rf build/PACKAGE_NAME install/PACKAGE_NAME
./quick_build.sh PACKAGE_NAME
```

### Environment Not Set Up
Always source the workspace:
```bash
source /home/ilv/ros2_ws/install/setup.bash
```

Or add to your `~/.bashrc`:
```bash
echo "source /home/ilv/ros2_ws/install/setup.bash" >> ~/.bashrc
```

---

## Package Dependencies

The workspace includes these key packages:
- **fairino_hardware** - Hardware interface for Fairino robots
- **fairino_msgs** - Custom ROS2 messages
- **fairino5_v6_moveit2_config** - MoveIt2 configuration and launch files

Build order is automatically handled by colcon based on dependencies.

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