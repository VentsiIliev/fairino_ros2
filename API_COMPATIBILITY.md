# FairinoRos2Client API Compatibility Guide

This document shows the compatibility between `FairinoRobot` (original) and `FairinoRos2Client` (ROS2 bridge).

## ✅ Fully Compatible Methods

All these methods work exactly the same way:

| Method | Signature | Return Type | Status |
|--------|-----------|-------------|--------|
| `move_cartesian` | `(position, tool=0, user=0, vel=30, acc=30, blendR=0)` | `int` | ✅ Full |
| `move_liner` | `(position, tool=0, user=0, vel=30, acc=30, blendR=0)` | `int` | ✅ Full |
| `get_current_position` | `()` | `list or None` | ✅ Full |
| `start_jog` | `(axis, direction, step, vel, acc)` | `int` | ✅ Full |
| `stop_motion` | `()` | `int` | ✅ Full |
| `resetAllErrors` | `()` | `int` | ✅ Full |
| `enable` | `()` | `int` | ✅ Full |
| `disable` | `()` | `int` | ✅ Full |
| `printSdkVersion` | `()` | `str` | ✅ Full |

## ✅ SDK Method Aliases

Original Fairino SDK methods are also supported:

| Original SDK Method | Equivalent | Status |
|---------------------|------------|--------|
| `MoveCart(...)` | `move_cartesian(...)` | ✅ Full |
| `MoveL(...)` | `move_liner(...)` | ✅ Full |
| `StartJOG(...)` | `start_jog(...)` | ✅ Full |
| `StopMotion()` | `stop_motion()` | ✅ Full |
| `GetActualTCPPose()` | `get_current_position()` | ✅ Full |
| `GetSDKVersion()` | `printSdkVersion()` | ✅ Full |
| `RobotEnable(state)` | `enable()` / `disable()` | ✅ Full |
| `ResetAllError()` | `resetAllErrors()` | ✅ Full |
| `SetDO(port, value)` | `setDigitalOutput(port, value)` | ⚠️ Not implemented |

## ⚠️ Partially Compatible Methods

| Method | Notes | Workaround |
|--------|-------|------------|
| `setDigitalOutput` | Not implemented (requires hardware interface) | Returns -1, prints warning |
| `get_current_acceleration` | Not implemented in base bridge | Returns None |

## 🆕 ROS2-Specific Methods

Additional methods available only in `FairinoRos2Client`:

| Method | Description |
|--------|-------------|
| `execute_path(path, rx, ry, rz, vel, acc, blocking)` | Execute multi-waypoint Cartesian path using MoveIt |
| `set_workobject(origin, user_id)` | Set work object coordinate frame |
| `health_check()` | Check bridge server connection status |

## Migration Examples

### Example 1: Simple Replacement

**Before (FairinoRobot):**
```python
from your_module import FairinoRobot

robot = FairinoRobot(ip="192.168.1.100")
robot.enable()

# Move to position
robot.move_cartesian([100, 200, 300, 180, 0, 0], vel=30, acc=30)

# Get position
pos = robot.get_current_position()
print(f"Position: {pos}")
```

**After (FairinoRos2Client):**
```python
from fairino_ros2_client import FairinoRos2Client

robot = FairinoRos2Client(server_url="http://localhost:5000")
robot.enable()

# Move to position (IDENTICAL)
robot.move_cartesian([100, 200, 300, 180, 0, 0], vel=30, acc=30)

# Get position (IDENTICAL)
pos = robot.get_current_position()
print(f"Position: {pos}")
```

**Even Easier:**
```python
from fairino_ros2_client import FairinoRobot  # Same class name!

robot = FairinoRobot(ip="localhost:5000")  # Now ip points to bridge
# Rest of the code stays EXACTLY the same
```

### Example 2: Using SDK Methods

**Before:**
```python
robot = FairinoRobot(ip="192.168.1.100")

# SDK methods
robot.MoveCart([100, 200, 300, 180, 0, 0], 0, 0, 30, 30)
robot.MoveL([150, 200, 300, 180, 0, 0], 0, 0, 30, 30, 0)

status, pose = robot.GetActualTCPPose()
print(f"Status: {status}, Pose: {pose}")
```

**After (NO CHANGES NEEDED):**
```python
robot = FairinoRos2Client(server_url="http://localhost:5000")

# SDK methods (IDENTICAL)
robot.MoveCart([100, 200, 300, 180, 0, 0], 0, 0, 30, 30)
robot.MoveL([150, 200, 300, 180, 0, 0], 0, 0, 30, 30, 0)

status, pose = robot.GetActualTCPPose()
print(f"Status: {status}, Pose: {pose}")
```

### Example 3: Jog Operations

**Before:**
```python
from core.model.robot.enums.axis import Axis, Direction

robot = FairinoRobot(ip="192.168.1.100")

# Jog in X axis
robot.start_jog(Axis.X, Direction.PLUS, step=10, vel=20, acc=20)
```

**After (IDENTICAL):**
```python
from core.model.robot.enums.axis import Axis, Direction

robot = FairinoRos2Client(server_url="http://localhost:5000")

# Jog in X axis (IDENTICAL - handles enums automatically)
robot.start_jog(Axis.X, Direction.PLUS, step=10, vel=20, acc=20)
```

### Example 4: IRobot Interface

**Your IRobot implementation:**
```python
class YourRobotController:
    def __init__(self, robot: IRobot):
        self.robot = robot

    def move_to_position(self, pos):
        result = self.robot.move_cartesian(pos, vel=30, acc=30)
        if result != 0:
            print("Move failed")
```

**Works with both implementations:**
```python
# Option 1: Real robot
from your_module import FairinoRobot
robot = FairinoRobot(ip="192.168.1.100")
controller = YourRobotController(robot)

# Option 2: ROS2 robot (NO CODE CHANGES)
from fairino_ros2_client import FairinoRos2Client
robot = FairinoRos2Client(server_url="http://localhost:5000")
controller = YourRobotController(robot)  # Same interface!
```

## Return Value Compatibility

### move_cartesian / move_liner
- **FairinoRobot**: Returns result from SDK (0 = success, other = error)
- **FairinoRos2Client**: Returns `0` on success, `-1` on error
- ✅ **Compatible**: Both use `0` for success

### get_current_position
- **FairinoRobot**: Returns `currentPose[1]` or `None`
- **FairinoRos2Client**: Returns `[x, y, z, rx, ry, rz]` or `None`
- ✅ **Compatible**: Both return list or None

### GetActualTCPPose
- **FairinoRobot**: Returns `(status, pose)`
- **FairinoRos2Client**: Returns `(0, [x, y, z, rx, ry, rz])` or `(-1, [0,0,0,0,0,0])`
- ✅ **Compatible**: Same tuple format

### start_jog
- **FairinoRobot**: Accepts `Axis` and `Direction` enums (extracts `.value`)
- **FairinoRos2Client**: Accepts enums or raw int values (extracts `.value` if present)
- ✅ **Compatible**: Handles both enums and ints

## Configuration Differences

| Aspect | FairinoRobot | FairinoRos2Client |
|--------|--------------|-------------------|
| Connection | Direct TCP/IP to robot | HTTP to ROS2 bridge |
| Initialization | `ip="192.168.1.100"` | `server_url="http://localhost:5000"` |
| Dependencies | Fairino SDK | `requests` only (no ROS2) |
| Motion Planning | Robot controller | MoveIt2 |
| Safety | Robot limits | MoveIt + workspace boundaries |

## Testing Compatibility

```python
def test_robot_interface(robot):
    """Test that robot implements IRobot correctly"""

    # Test motion
    assert robot.move_cartesian([100, 200, 300, 180, 0, 0]) == 0
    assert robot.move_liner([150, 200, 300, 180, 0, 0]) == 0

    # Test state queries
    pos = robot.get_current_position()
    assert pos is None or len(pos) == 6

    # Test control
    assert robot.enable() == 0 or robot.enable() is None
    assert robot.stop_motion() == 0

    print("✓ All interface tests passed")


# Test both implementations
from your_module import FairinoRobot
from fairino_ros2_client import FairinoRos2Client

test_robot_interface(FairinoRobot(ip="192.168.1.100"))
test_robot_interface(FairinoRos2Client(server_url="http://localhost:5000"))
```

## Summary

✅ **100% API compatible** for all motion and control methods
✅ **Drop-in replacement** - just change the import and initialization
✅ **Same return values** - works with existing error handling
✅ **Enum support** - handles `Axis` and `Direction` enums automatically
⚠️ **I/O methods** not implemented (setDigitalOutput)
🆕 **Additional features** - path execution, workobjects, health checks

## Next Steps

1. **Try it**: Replace `FairinoRobot` with `FairinoRos2Client` in one function
2. **Test**: Verify behavior matches your expectations
3. **Migrate**: Gradually move more code to use the ROS2 client
4. **Enjoy**: Benefit from MoveIt's collision avoidance and motion planning!