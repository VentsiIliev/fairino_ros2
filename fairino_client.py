#!/usr/bin/env python3
"""
Fairino Bridge Client - Use this in your non-ROS2 project
No ROS2 dependencies required!
"""
import requests
import json

class FairinoClient:
    """
    Simple HTTP client for controlling Fairino robot without ROS2.
    Connect to the ROS2 bridge server running in background.
    """

    def __init__(self, server_url="http://localhost:5000"):
        """
        Args:
            server_url: URL of the bridge server
        """
        self.server_url = server_url.rstrip('/')

    def health_check(self):
        """Check if bridge server is running"""
        try:
            response = requests.get(f"{self.server_url}/health", timeout=2)
            return response.json()
        except Exception as e:
            return {"status": "error", "message": str(e)}

    def move_cartesian(self, position, tool=0, user=0, vel=30, acc=30):
        """
        Move robot to Cartesian position (point-to-point)

        Args:
            position: [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg]
            tool: Tool frame ID
            user: User frame ID (workobject)
            vel: Velocity percentage (0-100)
            acc: Acceleration percentage (0-100)

        Returns:
            dict with 'success' and 'result'
        """
        payload = {
            "position": position,
            "tool": tool,
            "user": user,
            "vel": vel,
            "acc": acc
        }
        response = requests.post(f"{self.server_url}/move/cartesian", json=payload)
        return response.json()

    def move_linear(self, position, tool=0, user=0, vel=30, acc=30):
        """
        Move robot in linear path

        Args:
            position: [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg]
            tool: Tool frame ID
            user: User frame ID
            vel: Velocity percentage (0-100)
            acc: Acceleration percentage (0-100)

        Returns:
            dict with 'success' and 'result'
        """
        payload = {
            "position": position,
            "tool": tool,
            "user": user,
            "vel": vel,
            "acc": acc
        }
        response = requests.post(f"{self.server_url}/move/linear", json=payload)
        return response.json()

    def execute_path(self, path, rx=None, ry=None, rz=None, vel=0.6, acc=0.4, blocking=False):
        """
        Execute multi-waypoint Cartesian path

        Args:
            path: List of waypoints [[x,y,z], ...] or [[x,y,z,rx,ry,rz], ...]
            rx, ry, rz: Orientation (if not specified in waypoints)
            vel: Velocity scaling (0.0-1.0)
            acc: Acceleration scaling (0.0-1.0)
            blocking: Wait for completion

        Returns:
            dict with 'success' and 'result'
        """
        payload = {
            "path": path,
            "rx": rx,
            "ry": ry,
            "rz": rz,
            "vel": vel,
            "acc": acc,
            "blocking": blocking
        }
        response = requests.post(f"{self.server_url}/execute/path", json=payload)
        return response.json()

    def get_current_position(self):
        """
        Get current robot TCP position

        Returns:
            [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg] or None
        """
        response = requests.get(f"{self.server_url}/position/current")
        data = response.json()
        return data.get("position")

    def get_current_velocity(self):
        """
        Get current Cartesian velocity

        Returns:
            (vx, vy, vz) in mm/s or None
        """
        response = requests.get(f"{self.server_url}/velocity/current")
        data = response.json()
        return data.get("velocity")

    def stop_motion(self):
        """
        Stop all robot motion immediately

        Returns:
            dict with 'success'
        """
        response = requests.post(f"{self.server_url}/stop")
        return response.json()

    def set_workobject(self, origin, user_id=0):
        """
        Set work object coordinate frame

        Args:
            origin: [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg]
            user_id: User frame ID (default 0)

        Returns:
            dict with 'success'
        """
        payload = {
            "origin": origin,
            "user_id": user_id
        }
        response = requests.post(f"{self.server_url}/workobject/set", json=payload)
        return response.json()

    def jog(self, axis, direction, step, vel=10.0, acc=10.0):
        """
        Jog robot in specified axis

        Args:
            axis: 0=X, 1=Y, 2=Z
            direction: 1=positive, -1=negative
            step: Distance in mm
            vel: Velocity percentage (0-100)
            acc: Acceleration percentage (0-100)

        Returns:
            dict with 'success'
        """
        payload = {
            "axis": axis,
            "direction": direction,
            "step": step,
            "vel": vel,
            "acc": acc
        }
        response = requests.post(f"{self.server_url}/jog", json=payload)
        return response.json()


# ============ Example Usage ============
if __name__ == "__main__":
    # Create a client (no ROS2 needed!)
    robot = FairinoClient("http://localhost:5000")

    # Check connection
    print("Health check:", robot.health_check())

    # Get the current position
    pos = robot.get_current_position()
    print(f"Current position: {pos}")

    # Move to position (PTP)
    result = robot.move_cartesian([100, 200, 300, 180, 0, 0], vel=30, acc=30)
    print(f"Move result: {result}")

    # Execute path
    path = [
        [100, 100, 300],
        [200, 100, 300],
        [200, 200, 300],
        [100, 200, 300]
    ]
    result = robot.execute_path(path, rx=180, ry=0, rz=0, vel=0.5)
    print(f"Path result: {result}")

    # Emergency stop
    # robot.stop_motion()