# eRob MoveIt Overlay Workspace

This is the overlay workspace for the ROS2 robotic manipulation system. It depends on the base workspace at `/home/ilv/ros2_ws/`.

## Workspace Structure

This overlay contains:
- **erob_moveit_runtime** - Shared runtime package (Python/C++) with motion planning, execution, safety, and REST API
- **fairino5_v6_moveit2_config** - MoveIt2 configuration for Fairino5 v6 robot
- **zeroerr** - MoveIt2 configuration for eRobo3 (ZeroErr) EtherCAT-driven arm

## Building

### Build Overlay Packages Only
```bash
cd /home/ilv/ros2_ws
./build_zeroerr.sh
```

This script:
1. Sources ROS2 Rolling (`/opt/ros/rolling/setup.bash`)
2. Sources base workspace (`/home/ilv/ros2_ws/install/local_setup.bash`)
3. Builds overlay packages using colcon

### Build Specific Package
```bash
./build_zeroerr.sh erob_moveit_runtime
./build_zeroerr.sh zeroerr
```

### Full Build (Base + Overlay)
```bash
./quick_build.sh
```

## Environment Setup

After building, source the overlay workspace (it automatically chains to the base underlay):
```bash
source /home/ilv/ros2_ws/eRob_moveit/install/setup.bash
```

## Launching

Use the launch script from the base workspace:
```bash
cd /home/ilv/ros2_ws
./launch_robot.sh
```

The launch configuration is controlled by `robot_launch.conf` in this directory.

## Dependencies

The overlay requires these packages from the base workspace:
- `fairino_msgs` - Custom message definitions
- `fairino_hardware` - Hardware interface
- `fairino_description` - Robot URDF/meshes

## Development Notes

- Python changes take effect immediately (built with `--symlink-install`)
- C++ changes require rebuilding
- The overlay workspace is at `/home/ilv/ros2_ws/eRob_moveit/`
- Base workspace is at `/home/ilv/ros2_ws/`
