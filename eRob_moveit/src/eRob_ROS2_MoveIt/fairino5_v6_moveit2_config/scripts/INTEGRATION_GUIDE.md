# MoveIt Adapter Integration Guide

Quick guide to integrate the MoveIt adapter into your existing `FairinoRobot` class.

## Option 1: Simple Factory Pattern (Recommended)

Use the robot factory to switch between robot types:

```python
from robot_factory import create_robot, RobotType, shutdown_robot

# Use MoveIt adapter
robot = create_robot(RobotType.MOVEIT)

# Or use real robot
robot = create_robot(RobotType.REAL, ip="192.168.1.100")

# Or use test robot
robot = create_robot(RobotType.TEST)

# Your existing code works unchanged!
robot.enable()
robot.move_cartesian([300, 0, 400, 180, 0, 0], vel=30, acc=30)
position = robot.get_current_position()

# Cleanup
shutdown_robot(robot)
```

## Option 2: Modify Your FairinoRobot Class

Add `use_moveit` parameter to your `__init__`:

```python
class FairinoRobot(IRobot):
    def __init__(self, ip=None, use_moveit=False):
        if use_moveit:
            # Create MoveIt adapter
            import rclpy
            from moveit_robot_adapter import MoveItRobotAdapter
            from threading import Thread

            if not rclpy.ok():
                rclpy.init()

            self.robot = MoveItRobotAdapter()

            # Spin in background
            Thread(target=rclpy.spin, args=(self.robot,), daemon=True).start()

        else:
            # Create real robot
            from libs.fairino.linux.fairino import Robot
            self.robot = Robot.RPC(ip)

        # Rest of your __init__ code...

    # All other methods stay the same!
    # Just add conditional logic for MoveIt vs Real robot methods
```

## Option 3: Direct Import (Quick Test)

For quick testing, directly import and use:

```python
import rclpy
from moveit_robot_adapter import MoveItRobotAdapter
from threading import Thread

# Initialize
rclpy.init()
robot = MoveItRobotAdapter()
Thread(target=rclpy.spin, args=(robot,), daemon=True).start()

# Use robot
robot.enable()
robot.move_cartesian([300, 0, 400, 180, 0, 0], vel=30, acc=30)

# Cleanup
robot.destroy_node()
rclpy.shutdown()
```

## Key Differences in Your Code

### Before (Real Robot):
```python
class FairinoRobot:
    def __init__(self, ip):
        self.robot = Robot.RPC(self.ip)

    def move_cartesian(self, position, tool=0, user=0, vel=30, acc=30, blendR=0):
        result = self.robot.MoveCart(position, tool, user, vel=vel, acc=acc)
        return result
```

### After (With MoveIt Support):
```python
class FairinoRobot:
    def __init__(self, ip=None, use_moveit=False):
        if use_moveit:
            self.robot = MoveItRobotAdapter()
            # Start ROS2 spin thread
        else:
            self.robot = Robot.RPC(self.ip)

    def move_cartesian(self, position, tool=0, user=0, vel=30, acc=30, blendR=0):
        if self.use_moveit:
            # MoveIt adapter already has move_cartesian with same signature!
            result = self.robot.move_cartesian(position, tool, user, vel, acc, blendR)
        else:
            result = self.robot.MoveCart(position, tool, user, vel=vel, acc=acc)
        return result
```

## Method Mapping

The adapter implements the same interface, but some method names differ:

| Your Code | Real Robot (RPC) | MoveIt Adapter |
|-----------|------------------|----------------|
| `move_cartesian()` | `robot.MoveCart()` | `robot.move_cartesian()` |
| `move_liner()` | `robot.MoveL()` | `robot.move_liner()` |
| `get_current_position()` | `robot.GetActualTCPPose()` | `robot.get_current_position()` |
| `enable()` | `robot.RobotEnable(1)` | `robot.enable()` |
| `disable()` | `robot.RobotEnable(0)` | `robot.disable()` |
| `start_jog()` | `robot.StartJOG()` | `robot.start_jog()` |
| `stop_motion()` | `robot.StopMotion()` | `robot.stop_motion()` |
| `resetAllErrors()` | `robot.ResetAllError()` | `robot.resetAllErrors()` |

## Example Integration in Your Existing Code

In the code you showed me, change line 144:

```python
# OLD (line 144):
self.robot = Robot.RPC(self.ip)  # Real robot - use in production
self.robot = MoveItRobotAdapter()  # This line overwrites the previous!

# NEW (recommended):
if use_moveit:
    import rclpy
    from moveit_robot_adapter import MoveItRobotAdapter
    from threading import Thread

    if not hasattr(rclpy, '_initialized') or not rclpy._initialized:
        rclpy.init()

    self.robot = MoveItRobotAdapter(
        node_name='fairino_robot_adapter',
        group_name='fairino5_v6_group',
        end_effector_link='wrist3_link',
        base_frame='base_link'
    )

    # Spin ROS2 in background
    self._ros2_thread = Thread(target=rclpy.spin, args=(self.robot,), daemon=True)
    self._ros2_thread.start()

    import time
    time.sleep(1.0)  # Wait for joint states
else:
    self.robot = Robot.RPC(self.ip)  # Real robot
```

Then in your methods, add conditionals:

```python
def move_cartesian(self, position, tool=0, user=0, vel=30, acc=30, blendR=0):
    if self.use_moveit:
        result = self.robot.move_cartesian(position, tool, user, vel, acc, blendR)
    else:
        result = self.robot.MoveCart(position, tool, user, vel=vel, acc=acc)

    log_debug_message(self.logger_context,
                      f"MoveCart to {position} -> result: {result}")
    return result
```

## Testing

### 1. Test with Factory:
```bash
cd /home/ilv/ros2_ws/eRob_moveit/src/eRob_ROS2_MoveIt/fairino5_v6_moveit2_config/scripts
python3 robot_factory.py
```

### 2. Test MoveIt Adapter:
```bash
# Terminal 1: Launch MoveIt
ros2 launch fairino5_v6_moveit2_config demo.launch.py

# Terminal 2: Test adapter
python3 moveit_robot_adapter.py
```

### 3. Test Integrated Class:
```bash
python3 fairino_robot_integrated.py
```

## Files Created

1. **`moveit_robot_adapter.py`** - Main adapter class
2. **`robot_factory.py`** - Factory for creating robots
3. **`fairino_robot_integrated.py`** - Example integrated class
4. **`adapter_usage_example.py`** - Usage examples
5. **`ADAPTER_README.md`** - Full adapter documentation
6. **`INTEGRATION_GUIDE.md`** - This file

## Benefits

✅ **Same Interface** - No changes to application code
✅ **Easy Switching** - Toggle between real/simulation with one parameter
✅ **Motion Planning** - MoveIt provides collision-free trajectories
✅ **Smooth Motion** - Optimized trajectory generation
✅ **Testing** - Test code without hardware

## Next Steps

1. Choose integration approach (Factory recommended)
2. Modify your `FairinoRobot.__init__()` to add `use_moveit` parameter
3. Add conditional logic in methods that call robot directly
4. Test with MoveIt first, then switch to real robot

See `fairino_robot_integrated.py` for a complete example!