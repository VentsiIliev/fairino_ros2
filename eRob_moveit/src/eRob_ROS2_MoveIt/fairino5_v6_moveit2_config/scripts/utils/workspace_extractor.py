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

    for attempt in range(max_retries):
        try:
            robot_description = None

            try:
                from rclpy.parameter import Parameter
                params = robot_controller.get_parameters_by_prefix('')
                if 'robot_description' in params:
                    robot_description = params['robot_description']
            except:
                pass

            if not robot_description:
                robot_controller.get_logger().info(
                    f"Retry {attempt + 1}/{max_retries}: robot_description not available yet...")
                try:
                    from rcl_interfaces.srv import GetParameters
                    client = robot_controller.create_client(GetParameters, '/robot_state_publisher/get_parameters')
                    if client.wait_for_service(timeout_sec=0.3):
                        request = GetParameters.Request()
                        request.names = ['robot_description']
                        future = client.call_async(request)
                        import rclpy
                        rclpy.spin_until_future_complete(robot_controller, future, timeout_sec=0.3)
                        if future.done():
                            response = future.result()
                            if response and len(response.values) > 0:
                                robot_description = response.values[0].string_value
                except Exception as e:
                    robot_controller.get_logger().debug(f"Could not fetch from robot_state_publisher: {e}")

            if not robot_description:
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
