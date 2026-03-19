# Fix: pl25488 (eRobo3) URDF Cross-Contamination via DDS

## Problem

On launch, RViz flooded with errors:

```
[rviz2-3] [ERROR]: Could not load resource [package://pl25488/meshes/Link_1.stl]: Unknown exception
[rviz2-3] [ERROR]: Could not load resource [package://pl25488/meshes/Link_2.stl]: Unknown exception
...
```

And `move_group` crashed shortly after startup:

```
Variable 'Joint_1' is not known to model 'fairino5_v6_robot'
terminate called after throwing an instance of 'moveit::Exception'
```

The system was trying to load meshes and joint names from `pl25488` (the eRobo3 robot), which is a completely different robot than our `fairino5_v6_robot`.

## Root Cause

Another machine on the local network (user `zewy`, workspace `/home/zewy/ros2_ws/`) was running the eRobo3/pl25488 robot system. Both systems used the default DDS configuration:

- `ROS_DOMAIN_ID=0` (default)
- `ROS_LOCALHOST_ONLY` unset (DDS multicast enabled)

FastDDS discovers all ROS 2 nodes on the same network and domain via multicast. The eRobo3 system's `robot_state_publisher` was broadcasting its URDF on the `/robot_description` topic with **Transient Local** durability (QoS). This meant:

1. RViz received the eRobo3 URDF from the remote machine's `/robot_description` topic
2. RViz's MotionPlanning plugin tried to load `package://pl25488/meshes/*.stl` (which don't exist locally)
3. The MotionPlanning plugin called the local `move_group` FK service with eRobo3 joint names (`Joint_1`..`Joint_6`)
4. `move_group` only knows about `fairino5_v6_robot` joints (`j1`..`j6`), so it crashed with an unhandled exception

## Fix Applied

**File changed:** `launch_robot.sh`

Two environment variables added to isolate DDS discovery:

```bash
# Isolate DDS domain to prevent cross-talk with other robots on the network
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
```

Additionally, a cleanup step was added before launch to clear stale FastDDS shared memory:

```bash
# Clean up stale FastDDS shared memory from previous sessions
rm -rf /dev/shm/fastdds_* /dev/shm/sem.fastdds_* 2>/dev/null
```

### What each setting does

| Setting | Value | Effect |
|---------|-------|--------|
| `ROS_LOCALHOST_ONLY` | `1` | Restricts DDS discovery to localhost only. No multicast, no network cross-talk. |
| `ROS_DOMAIN_ID` | `42` | Uses a separate DDS domain (port range) as additional isolation. |
| FastDDS shm cleanup | `rm -rf /dev/shm/fastdds_*` | Removes stale shared memory segments that can persist across restarts. |

## Configuration Location

All changes are in a single file:

```
/home/ilv/ros2_ws/launch_robot.sh    (lines 42-43, 85)
```

No changes were made to URDF, SRDF, MoveIt config, or any ROS package files.