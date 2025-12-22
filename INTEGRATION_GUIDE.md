# Integrating FairinoRos2Robot into Non-ROS2 Projects

This guide explains how to use `FairinoRos2Robot` in projects that don't use ROS2 or MoveIt.

## Overview

The `FairinoRos2Robot` class (robot_controller.py:871) is tightly integrated with:
- **ROS2** (`rclpy`) - ROS2 Python client library
- **MoveIt2** - Motion planning framework
- **Action clients** - For trajectory execution
- **TF2** - Coordinate frame transformations

## Integration Options

### ✅ Option 1: HTTP Bridge (Recommended)

**Best for:** Most use cases, especially when you want clean separation

Run ROS2 as a background service and communicate via REST API.

#### Setup:

1. **Start the bridge server** (in ROS2 environment):
```bash
# Terminal 1: Launch ROS2 + MoveIt
cd ~/ros2_ws
source install/setup.bash
ros2 launch fairino5_v6_moveit2_config demo_with_monitor.launch.py

# Terminal 2: Start bridge server
python3 fairino_bridge_server.py
```

2. **Use the client** (in any Python project - no ROS2 needed):
```python
from fairino_client import FairinoClient

# Connect to bridge
robot = FairinoClient("http://localhost:5000")

# Check connection
print(robot.health_check())

# Move robot
robot.move_cartesian([100, 200, 300, 180, 0, 0], vel=30, acc=30)

# Execute path
path = [[100, 100, 300], [200, 100, 300], [200, 200, 300]]
robot.execute_path(path, rx=180, ry=0, rz=0, vel=0.5)

# Get current position
pos = robot.get_current_position()
print(f"Position: {pos}")
```

**Pros:**
- ✅ No ROS2 dependencies in your main project
- ✅ Works with any language (Python, C++, JavaScript, etc.)
- ✅ Can run ROS2 on a different machine
- ✅ Clean separation of concerns

**Cons:**
- ⚠️ Network latency (minimal if localhost)
- ⚠️ Requires running two processes

---

### Option 2: ZeroMQ/Socket Communication

**Best for:** Lower latency, same-machine communication

Similar to Option 1 but uses ZeroMQ instead of HTTP for faster communication.

```python
# server_zmq.py (simplified example)
import zmq
import json

context = zmq.Context()
socket = context.socket(zmq.REP)
socket.bind("tcp://*:5555")

while True:
    message = socket.recv_json()
    command = message['command']

    if command == 'move':
        robot.move_cartesian(message['position'], **message.get('params', {}))
        socket.send_json({"status": "ok"})
```

```python
# client_zmq.py
import zmq
import json

context = zmq.Context()
socket = context.socket(zmq.REQ)
socket.connect("tcp://localhost:5555")

# Send command
socket.send_json({
    "command": "move",
    "position": [100, 200, 300, 180, 0, 0],
    "params": {"vel": 30, "acc": 30}
})
result = socket.recv_json()
```

---

### Option 3: Subprocess Control

**Best for:** Simple integration, same machine only

Launch ROS2 as a subprocess and communicate via stdin/stdout.

```python
import subprocess
import json

class FairinoSubprocess:
    def __init__(self):
        # Start ROS2 bridge as subprocess
        self.process = subprocess.Popen(
            ['python3', 'fairino_bridge_server.py'],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )

    def send_command(self, command):
        self.process.stdin.write(json.dumps(command) + '\n')
        self.process.stdin.flush()
        response = self.process.stdout.readline()
        return json.loads(response)
```

---

### Option 4: Extract Core Logic (Advanced)

**Best for:** Complete independence from ROS2 (requires significant work)

Create a standalone version that uses the robot's native SDK directly.

**Steps:**
1. Identify the robot's native communication protocol (TCP/IP, Modbus, etc.)
2. Extract the core control logic from `FairinoRos2Robot`
3. Reimplement using the robot's SDK

```python
# pseudo_standalone.py
from fairino_robot_sdk import FairinoSDK  # Native SDK

class StandaloneFairinoRobot:
    def __init__(self, ip):
        self.sdk = FairinoSDK(ip)

    def move_cartesian(self, position, vel=30, acc=30):
        # Direct SDK call - no ROS2
        return self.sdk.MoveCart(position, vel, acc)

    def compute_fk(self, joint_positions):
        # Reuse FK from robot_controller.py:176
        # (copy RobotMonitor.compute_fk without ROS2 dependencies)
        pass
```

**Required changes:**
- Remove all `rclpy` imports
- Remove MoveIt action clients
- Replace TF2 with numpy transforms
- Implement direct robot communication

---

## Recommended Approach

For most users, **Option 1 (HTTP Bridge)** is the best choice:

1. **Deploy the bridge server** on the robot controller or a dedicated machine
2. **Use the client library** in your main application
3. **Optionally:** Make the bridge server start automatically at boot

### Production Deployment

```bash
# Install as systemd service
sudo nano /etc/systemd/system/fairino-bridge.service
```

```ini
[Unit]
Description=Fairino ROS2 Bridge Server
After=network.target

[Service]
Type=simple
User=ilv
WorkingDirectory=/home/ilv/ros2_ws
Environment="ROS_DOMAIN_ID=0"
ExecStart=/bin/bash -c "source /opt/ros/humble/setup.bash && source install/setup.bash && python3 fairino_bridge_server.py"
Restart=on-failure

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl daemon-reload
sudo systemctl enable fairino-bridge
sudo systemctl start fairino-bridge
```

---

## API Reference

### Available Endpoints (HTTP Bridge)

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/health` | GET | Check server status |
| `/move/cartesian` | POST | Point-to-point move |
| `/move/linear` | POST | Linear interpolation |
| `/execute/path` | POST | Multi-waypoint path |
| `/position/current` | GET | Get TCP position |
| `/velocity/current` | GET | Get TCP velocity |
| `/stop` | POST | Emergency stop |
| `/workobject/set` | POST | Set coordinate frame |
| `/jog` | POST | Jog in axis |

### Example Payloads

**Move Cartesian:**
```json
{
  "position": [100, 200, 300, 180, 0, 0],
  "tool": 0,
  "user": 0,
  "vel": 30,
  "acc": 30
}
```

**Execute Path:**
```json
{
  "path": [
    [100, 100, 300],
    [200, 100, 300],
    [200, 200, 300]
  ],
  "rx": 180,
  "ry": 0,
  "rz": 0,
  "vel": 0.6,
  "acc": 0.4,
  "blocking": false
}
```

---

## Testing

```bash
# Test the bridge
curl http://localhost:5000/health

# Test movement (from Python)
python3 -c "
from fairino_client import FairinoClient
robot = FairinoClient()
print(robot.get_current_position())
"
```

---

## Troubleshooting

### Bridge server won't start
- Ensure ROS2 is sourced: `source install/setup.bash`
- Check if MoveIt is running: `ros2 node list`
- Verify TCP transform loaded: Check for "TCP transform loaded" in logs

### Connection refused
- Check if server is running: `curl http://localhost:5000/health`
- Verify port 5000 is not blocked by firewall
- Try different port: Update both server and client

### Movement commands fail
- Check workspace boundaries (SAFETY_WORKSPACE in robot_controller.py:37)
- Verify position is reachable: Use `get_current_position()` first
- Check MoveIt planning: Look for planning errors in ROS2 logs

---

## Next Steps

1. **Try the example:** Run `python3 fairino_client.py`
2. **Customize the API:** Add endpoints for your specific needs
3. **Deploy to production:** Use systemd service for reliability
4. **Add authentication:** Secure the bridge with API keys if needed

For more details, see:
- `fairino_bridge_server.py` - Server implementation
- `fairino_client.py` - Client library with examples
- `robot_controller.py:871` - Original FairinoRos2Robot class