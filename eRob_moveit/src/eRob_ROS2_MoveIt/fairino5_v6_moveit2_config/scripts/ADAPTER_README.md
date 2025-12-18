# MoveIt Robot Adapter

A ROS2 MoveIt-based implementation of the `IRobot` interface, providing a drop-in replacement for the `FairinoRobot` class.

## Overview

The `MoveItRobotAdapter` allows you to control your robot through ROS2 MoveIt instead of direct RPC communication. This provides:

- **Unified Interface**: Same API as `FairinoRobot`
- **Motion Planning**: Uses MoveIt for collision-free trajectories
- **Smooth Motion**: Leverages MoveIt's trajectory generation
- **Easy Integration**: Drop-in replacement for existing code

## Architecture

```
┌─────────────────────────────────────┐
│      Your Application Code          │
│  (Uses IRobot interface methods)    │
└──────────────┬──────────────────────┘
               │
               │ IRobot Interface
               │ (move_cartesian, jog, etc.)
               │
       ┌───────┴────────┐
       │                │
┌──────▼──────┐   ┌────▼────────────────┐
│FairinoRobot │   │MoveItRobotAdapter   │
│ (RPC/SDK)   │   │  (ROS2 MoveIt)      │
└──────┬──────┘   └────┬────────────────┘
       │               │
       │               │
┌──────▼──────┐   ┌────▼────────────────┐
│   Robot     │   │  MoveIt + ROS2      │
│ Controller  │   │  Control Stack      │
└─────────────┘   └─────────────────────┘
```

## Files

- **`moveit_robot_adapter.py`**: Main adapter implementation
- **`adapter_usage_example.py`**: Usage examples and demos
- **`ADAPTER_README.md`**: This documentation

## Installation

1. Ensure ROS2 and MoveIt2 are installed
2. Build your workspace:
   ```bash
   cd /home/ilv/ros2_ws
   colcon build --packages-select fairino5_v6_moveit2_config
   source install/setup.bash
   ```

3. Make scripts executable (if needed):
   ```bash
   chmod +x scripts/moveit_robot_adapter.py
   chmod +x scripts/adapter_usage_example.py
   ```

## Quick Start

### Launch MoveIt

First, start MoveIt:
```bash
ros2 launch fairino5_v6_moveit2_config demo.launch.py
```

### Use the Adapter

In your Python code:

```python
import rclpy
from moveit_robot_adapter import MoveItRobotAdapter
from threading import Thread

# Initialize ROS2
rclpy.init()

# Create adapter (replaces FairinoRobot)
robot = MoveItRobotAdapter()

# Spin in background thread
spin_thread = Thread(target=rclpy.spin, args=(robot,), daemon=True)
spin_thread.start()

# Enable robot
robot.enable()

# Move to position (same API as FairinoRobot!)
position = [300.0, 0.0, 400.0, 180.0, 0.0, 0.0]  # [x, y, z, rx, ry, rz]
result = robot.move_cartesian(position, vel=30, acc=30)

# Get current position
current = robot.get_current_position()
print(f"Position: {current}")

# Cleanup
robot.destroy_node()
rclpy.shutdown()
```

## API Reference

The adapter implements all `IRobot` interface methods:

### Motion Control

#### `move_cartesian(position, tool=0, user=0, vel=30, acc=30, blendR=0)`
Move to a Cartesian position.

**Parameters:**
- `position` (list): `[x, y, z, rx, ry, rz]` in mm and degrees
- `vel` (float): Velocity percentage (0-100)
- `acc` (float): Acceleration percentage (0-100)

**Returns:** `[error_code, message]`

**Example:**
```python
target = [350.0, 50.0, 450.0, 180.0, 0.0, 0.0]
result = robot.move_cartesian(target, vel=30, acc=30)
```

#### `move_liner(position, tool=0, user=0, vel=30, acc=30, blendR=0)`
Execute linear motion (same as `move_cartesian` for single points).

**Example:**
```python
result = robot.move_liner(target, vel=30, acc=30)
```

#### `start_jog(axis, direction, step, vel, acc)`
Jog in a specific axis and direction.

**Parameters:**
- `axis` (Axis): `Axis.X`, `Axis.Y`, `Axis.Z`, `Axis.RX`, `Axis.RY`, `Axis.RZ`
- `direction` (Direction): `Direction.PLUS` or `Direction.MINUS`
- `step` (float): Distance to move in mm
- `vel` (float): Velocity percentage
- `acc` (float): Acceleration percentage

**Example:**
```python
from moveit_robot_adapter import Axis, Direction

# Jog 10mm in +X direction
robot.start_jog(Axis.X, Direction.PLUS, step=10.0, vel=20, acc=20)
```

### State Query

#### `get_current_position()`
Get current TCP position.

**Returns:** `[x, y, z, rx, ry, rz]` or `None`

**Example:**
```python
pos = robot.get_current_position()
if pos:
    print(f"X={pos[0]:.1f}, Y={pos[1]:.1f}, Z={pos[2]:.1f}")
```

#### `get_current_velocity()`
Get current joint velocities.

**Returns:** List of 6 joint velocities in rad/s, or `None`

#### `get_current_acceleration()`
Get current joint accelerations.

**Returns:** `None` (not available from joint_states)

### Robot Control

#### `enable()`
Enable robot motion.

```python
robot.enable()
```

#### `disable()`
Disable robot motion.

```python
robot.disable()
```

#### `stop_motion()`
Emergency stop - cancels current motion.

```python
robot.stop_motion()
```

### I/O Control

#### `setDigitalOutput(portId, value)`
Set digital output (not implemented in MoveIt adapter).

**Note:** Digital I/O requires separate ROS2 topic/service integration.

#### `resetAllErrors()`
Reset errors (not applicable in MoveIt adapter).

## Migrating from FairinoRobot

### Before (using FairinoRobot):
```python
from fairino_robot import FairinoRobot

# Direct RPC connection
robot = FairinoRobot(ip="192.168.1.100")
robot.enable()

# Move robot
position = [300.0, 0.0, 400.0, 180.0, 0.0, 0.0]
robot.move_cartesian(position, vel=30, acc=30)

# Get position
current = robot.get_current_position()
```

### After (using MoveItRobotAdapter):
```python
import rclpy
from moveit_robot_adapter import MoveItRobotAdapter
from threading import Thread

# Initialize ROS2
rclpy.init()

# Create adapter
robot = MoveItRobotAdapter()

# Spin in background
spin_thread = Thread(target=rclpy.spin, args=(robot,), daemon=True)
spin_thread.start()

# Enable robot
robot.enable()

# Move robot (SAME CODE!)
position = [300.0, 0.0, 400.0, 180.0, 0.0, 0.0]
robot.move_cartesian(position, vel=30, acc=30)

# Get position (SAME CODE!)
current = robot.get_current_position()

# Cleanup
robot.destroy_node()
rclpy.shutdown()
```

**Key Changes:**
1. Add ROS2 initialization
2. Spin node in background thread
3. Everything else stays the same!

## Configuration

The adapter can be configured during initialization:

```python
robot = MoveItRobotAdapter(
    node_name='my_robot_adapter',           # ROS2 node name
    group_name='fairino5_v6_group',         # MoveIt planning group
    end_effector_link='wrist3_link',        # End effector link
    base_frame='base_link'                  # Base reference frame
)
```

## Running the Demo

Run the included demo to test the adapter:

```bash
# Terminal 1: Launch MoveIt
ros2 launch fairino5_v6_moveit2_config demo.launch.py

# Terminal 2: Run demo
cd /home/ilv/ros2_ws/eRob_moveit/src/eRob_ROS2_MoveIt/fairino5_v6_moveit2_config/scripts
python3 adapter_usage_example.py
```

The demo will:
1. Enable the robot
2. Get current position
3. Move 50mm up in Z
4. Jog 10mm in +X
5. Return to original position
6. Display velocities

## Advanced Usage

### Custom Motion Planning

You can access the underlying MoveIt action client for advanced planning:

```python
# Access the action client
goal = MoveGroup.Goal()
# ... configure goal ...
future = robot.move_group_client.send_goal_async(goal)
```

### Thread Safety

The adapter is thread-safe using locks and reentrant callback groups:

```python
# Safe to call from multiple threads
from concurrent.futures import ThreadPoolExecutor

with ThreadPoolExecutor(max_workers=3) as executor:
    executor.submit(robot.move_cartesian, pos1)
    executor.submit(robot.move_cartesian, pos2)
```

### Blocking vs Non-blocking

By default, motion commands are blocking (wait for completion). This matches `FairinoRobot` behavior.

## Limitations

1. **Digital I/O**: Not implemented - requires separate ROS2 integration
2. **Tool/User Frames**: Not used - MoveIt uses its own frame definitions
3. **Blend Radius**: Not implemented for single-point moves
4. **Acceleration Query**: Not available from standard joint_states
5. **Emergency Stop**: Cancels goals but not a true hardware E-stop

## Troubleshooting

### "MoveGroup action server not available"
**Solution:** Make sure MoveIt is running:
```bash
ros2 launch fairino5_v6_moveit2_config demo.launch.py
```

### "Current position not available for jogging"
**Solution:** Wait for joint_states to be published:
```python
import time
time.sleep(2.0)  # Wait for joint states
```

### "Robot is disabled"
**Solution:** Call `robot.enable()` before motion:
```python
robot.enable()
robot.move_cartesian(position)
```

### Motion is jerky
**Solution:** Check smooth motion configuration:
- See `pilz_cartesian_limits.yaml`
- See `joint_limits.yaml`
- See `ros2_controllers.yaml`

## See Also

- **velocity_monitor.py**: Real-time robot monitoring GUI
- **MoveIt Documentation**: https://moveit.ros.org
- **ROS2 Actions**: https://docs.ros.org/en/rolling/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html

## Contributing

To extend the adapter:
1. Inherit from `MoveItRobotAdapter`
2. Override methods as needed
3. Maintain `IRobot` interface compatibility

## License

See the main package LICENSE file.