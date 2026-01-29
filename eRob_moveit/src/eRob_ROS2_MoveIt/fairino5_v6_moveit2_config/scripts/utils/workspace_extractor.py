SAFETY_WORKSPACE_FALLBACK = {
    'x_min': -0.37,
    'x_max': 0.5,
    'y_min': -0.8,
    'y_max': 0.6,
    'z_min': 0.0,
    'z_max': 1.1,
}

def _extract_workspace_from_urdf(robot_controller, max_retries=3, retry_delay=0.1):
    """Extract workspace boundaries from mounting_surface collision mesh in URDF."""
    import xml.etree.ElementTree as ET
    import os
    from ament_index_python.packages import get_package_share_directory
    import time
    from std_msgs.msg import String

    for attempt in range(max_retries):
        try:
            robot_description = None

            # Subscribe to /robot_description topic (the standard ROS2 way)
            try:
                from rclpy.qos import QoSProfile, DurabilityPolicy
                qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

                future_msg = None

                def desc_callback(msg):
                    nonlocal future_msg
                    future_msg = msg.data

                sub = robot_controller.create_subscription(String, '/robot_description', desc_callback, qos)

                # Wait for message with timeout
                wait_start = time.time()
                while future_msg is None and (time.time() - wait_start) < retry_delay:
                    import rclpy
                    rclpy.spin_once(robot_controller, timeout_sec=0.05)

                robot_controller.destroy_subscription(sub)
                robot_description = future_msg

            except Exception as e:
                robot_controller.get_logger().debug(f"Could not subscribe to /robot_description: {e}")

            if not robot_description:
                robot_controller.get_logger().info(
                    f"Retry {attempt + 1}/{max_retries}: robot_description not available yet...")
                if attempt < max_retries - 1:
                    time.sleep(retry_delay)
                    continue
                else:
                    robot_controller.get_logger().info("Using fallback workspace (robot_description not available yet)")
                    return SAFETY_WORKSPACE_FALLBACK.copy()

            root = ET.fromstring(robot_description)

            for link in root.findall('.//link[@name="mounting_surface"]'):
                collision = link.find('collision')
                if collision is not None:
                    mesh_elem = collision.find('.//mesh')
                    if mesh_elem is not None:
                        mesh_filename = mesh_elem.get('filename', '')

                        if mesh_filename.startswith('package://'):
                            package_path = mesh_filename.replace('package://', '')
                            parts = package_path.split('/', 1)
                            if len(parts) == 2:
                                package_name, relative_path = parts
                                try:
                                    package_dir = get_package_share_directory(package_name)
                                    mesh_path = os.path.join(package_dir, relative_path)

                                    # Quick file existence check before loading
                                    if not os.path.exists(mesh_path):
                                        robot_controller.get_logger().info(f"Mesh file not found, using fallback workspace")
                                        return SAFETY_WORKSPACE_FALLBACK.copy()

                                    # Load mesh (this can be slow)
                                    import trimesh
                                    mesh = trimesh.load(mesh_path)
                                    bounds = mesh.bounds

                                    workspace = {
                                        'x_min': float(bounds[0][0]),
                                        'x_max': float(bounds[1][0]),
                                        'y_min': float(bounds[0][1]),
                                        'y_max': float(bounds[1][1]),
                                        'z_min': float(bounds[0][2]),
                                        'z_max': SAFETY_WORKSPACE_FALLBACK['z_max'],
                                    }

                                    robot_controller.get_logger().info(f"✓ Extracted workspace from mesh")
                                    return workspace
                                except Exception as e:
                                    robot_controller.get_logger().debug(f"Mesh load failed: {e}")

            robot_controller.get_logger().info("Using fallback workspace (mounting_surface not in URDF)")
            return SAFETY_WORKSPACE_FALLBACK.copy()

        except Exception as e:
            if attempt < max_retries - 1:
                time.sleep(retry_delay)
                continue
            else:
                robot_controller.get_logger().info(f"Using fallback workspace (URDF parsing failed)")

    return SAFETY_WORKSPACE_FALLBACK.copy()
