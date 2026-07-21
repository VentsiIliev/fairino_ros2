# Workspace Requirements

This file records the environment needed to run this ROS 2 workspace on another PC. It is not a Python-only `requirements.txt`; the workspace depends on Ubuntu packages, ROS 2 Rolling packages, a PREEMPT_RT kernel, a custom EtherLab/IgH EtherCAT master install, and the source packages in this repository.

Observed on this PC on 2026-07-20.

## Target machine

- Ubuntu: 24.04.3 LTS (`noble`)
- Kernel: `6.14.0-rt3 #1 SMP PREEMPT_RT Tue Oct 28 19:47:32 EET 2025 x86_64`
- ROS 2 distro: Rolling (`/opt/ros/rolling`)
- ROS 2 core packages: `rclcpp` 30.1.4, `rclpy` 10.0.4, `rmw_fastrtps_cpp` 9.4.4
- MoveIt: 2.14.1
- ros2_control: 6.3.2
- ros2_controllers: 6.2.0
- Ruckig: 0.9.2
- EtherCAT master: IgH EtherCAT master 1.6.8 installed in `/usr/local/etherlab`
- Main workspace path: scripts/configs now derive this from the clone location.

The exact observed versions are locked in `requirements_versions.lock`. Use that file when you need to match this PC closely, and use `requirements_apt.txt` as the practical install list for apt.

ROS 2 Rolling is a rolling distro stream, not a fixed release version. For reproducibility, match the individual `ros-rolling-*` apt versions in `requirements_versions.lock`.

The workspace can be cloned outside `/home/ilv/ros2_ws`; the build and launch scripts resolve paths relative to their own location. If a custom `ROBOT_LAUNCH_CONFIG` points to an external config file, that file must still use valid paths or the `SCRIPT_DIR` / `ROOT_WS_DIR` variables provided by `eRob_moveit/launch_robot.sh`.

## Apt packages

Install the ROS 2 Rolling apt repository first, then install:

```bash
sudo apt update
sudo apt install -y $(grep -vE '^\s*(#|$)' requirements_apt.txt)
```

The exact package list is in `requirements_apt.txt`. Key installed versions on this PC include:

- `ros-rolling-moveit-core`: `2.14.1-1noble.20260113.122818`
- `ros-rolling-moveit-ros-move-group`: `2.14.1-1noble.20260113.133238`
- `ros-rolling-ros2-control`: `6.3.2-1noble.20260113.124524`
- `ros-rolling-ros2-controllers`: `6.2.0-1noble.20260113.134429`
- `ros-rolling-rviz2`: `15.1.15-1noble.20260113.134321`
- `ros-rolling-ruckig`: `0.9.2-4noble.20251204.191513`
- `python3-numpy`: `1.26.4`
- `python3-scipy`: `1.11.4`

For a closer version match, install apt packages using the `package=version` values in `requirements_versions.lock`. Be aware that ROS Rolling package versions move over time; if the apt repository no longer carries these exact builds, use an apt snapshot/mirror or install from matching `.deb` artifacts.

## Python bridge dependencies

The existing Python bridge file is still separate:

```bash
python3 -m pip install -r requirements_bridge.txt
```

Prefer apt packages for ROS-launched Python modules where possible (`python3-numpy`, `python3-scipy`, `python3-pyqt6`, `python3-flask`, `python3-requests`). Use pip only if the apt package is unavailable or the bridge specifically needs the pinned versions in `requirements_bridge.txt`.

## EtherCAT / realtime requirements

This workspace expects the IgH/EtherLab master outside apt:

- `/usr/local/etherlab/bin/ethercat`
- `/usr/local/etherlab/include/ecrt.h`
- `/usr/local/etherlab/lib/libethercat.so`
- `/usr/local/etherlab/etc/init.d/ethercat`
- `/usr/bin/ethercat -> /usr/local/etherlab/bin/ethercat`
- `/etc/init.d/ethercat -> /usr/local/etherlab/etc/init.d/ethercat`

Check the installed master:

```bash
ethercat version
# Expected here: IgH EtherCAT master 1.6.8 1.6.8
```

The `ethercat_interface` and `ethercat_manager` packages hard-code `ETHERLAB_DIR` to `/usr/local/etherlab`, so the new PC must install EtherLab there or the CMake files must be changed.

The running kernel is not installed as a normal Ubuntu `linux-image-*` dpkg package on this PC. It appears to be a custom PREEMPT_RT kernel:

- `/boot/vmlinuz-6.14.0-rt3`
- `/boot/config-6.14.0-rt3`
- `/lib/modules/6.14.0-rt3`
- `/lib/modules/6.14.0-rt3/build -> /home/ilv/Downloads/linux-6.14`

Important enabled kernel config options:

- `CONFIG_PREEMPT_RT=y`
- `CONFIG_PREEMPT=y`
- `CONFIG_NO_HZ=y`
- `CONFIG_HIGH_RES_TIMERS=y`
- `CONFIG_CPU_ISOLATION=y`

For the ZeroErr robot, the launch config expects isolated realtime cores:

- EtherCAT IRQ/control core: `14`
- ROS control core: `15`
- Non-RT cores: `0-13`
- Isolated mask: `0xC000`

See `docs/rt_cpu_isolation.md` before running the real robot on new hardware. CPU numbering may need changes on a PC with a different core count.

## Source packages expected in this workspace

Base workspace packages:

- `fairino_description`
- `fairino_hardware`
- `fairino_msgs`
- `ethercat_driver`
- `ethercat_driver_ros2`
- `ethercat_generic_cia402_drive`
- `ethercat_generic_slave`
- `ethercat_interface`
- `ethercat_manager`
- `ethercat_msgs`

Overlay/runtime packages:

- `erob_moveit_runtime`
- `fairino5_v6_moveit2_config`
- `zeroerr`

Some `package.xml` dependencies are local or robot-vendor packages, not apt packages:

- `fairino_msgs`, `fairino_hardware`, `fairino_description`: built from this repo.
- `ethercat_manager`, `ethercat_msgs`: built from this repo.
- `erob_moveit_runtime`: built from `eRob_moveit/src`.
- `fairino5_v6_robot_description`, `erob_arm`, `cubeeye_camera`: referenced by MoveIt config package XML; make sure the corresponding source/vendor packages are present if those launch paths require them.
- `moveit_ros_perception` and `warehouse_ros_mongo`: referenced by local MoveIt config package XML files, but not present as installed ROS packages here. `ros-rolling-moveit-ros-warehouse` is installed and available; no `ros-rolling-warehouse-ros-mongo` or `ros-rolling-moveit-ros-perception` apt package was found in the configured Rolling repository.
- Fairino SDK library: `src/fairino_hardware/libfairino/lib/libfairino.so`, installed into `install/fairino_hardware/lib/libfairino.so.2` by the build.

## Build order

From a fresh clone at `/home/ilv/ros2_ws`:

```bash
cd /home/ilv/ros2_ws
source /opt/ros/rolling/setup.bash
rosdep update
rosdep install --from-paths src eRob_moveit/src --ignore-src -r -y
./quick_build.sh
```

If only the overlay packages changed:

```bash
cd /home/ilv/ros2_ws
./build_zeroerr.sh
```

Manual equivalent:

```bash
cd /home/ilv/ros2_ws
source /opt/ros/rolling/setup.bash
colcon build --symlink-install --packages-select fairino_msgs --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon build --symlink-install --packages-skip fairino_msgs --cmake-args -DCMAKE_BUILD_TYPE=Release
cd eRob_moveit
source /home/ilv/ros2_ws/install/local_setup.bash
colcon build --symlink-install --packages-select erob_moveit_runtime fairino5_v6_moveit2_config zeroerr --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Runtime checks

Before launching on another PC:

```bash
source /opt/ros/rolling/setup.bash
source /home/ilv/ros2_ws/install/setup.bash
source /home/ilv/ros2_ws/eRob_moveit/install/setup.bash
ros2 pkg list | grep -E 'fairino|zeroerr|erob|ethercat'
ethercat version
```

For Fairino:

```bash
cd /home/ilv/ros2_ws
./launch_fairino.sh
```

For ZeroErr:

```bash
cd /home/ilv/ros2_ws
./launch_zeroerr.sh
```

Do not run the real robot until EtherCAT master configuration, NIC selection, realtime kernel boot options, CPU isolation settings, and emergency-stop/safety wiring are verified on the new PC.
