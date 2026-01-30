SAFETY_WORKSPACE_FALLBACK = {
    'x_min': -0.37,
    'x_max': 0.5,
    'y_min': -0.8,
    'y_max': 0.6,
    'z_min': 0.0,
    'z_max': 1.1,
}


def _parse_origin(origin_elem):
    """Parse origin element from URDF, returns (xyz, rpy) tuples."""
    if origin_elem is None:
        return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)

    xyz_str = origin_elem.get('xyz', '0 0 0')
    rpy_str = origin_elem.get('rpy', '0 0 0')

    xyz = tuple(float(v) for v in xyz_str.split())
    rpy = tuple(float(v) for v in rpy_str.split())

    return xyz, rpy


def _build_transform_to_base_link(root, link_name):
    """
    Build the transform chain from a link to base_link by traversing joints.
    Returns cumulative (x, y, z) offset.
    """
    import numpy as np

    # Build parent-child joint map
    joints = {}
    for joint in root.findall('.//joint'):
        child_link = joint.find('child')
        if child_link is not None:
            child_name = child_link.get('link')
            joints[child_name] = joint

    # Traverse from link_name up to base_link or world
    cumulative_offset = np.array([0.0, 0.0, 0.0])
    current_link = link_name
    visited = set()

    while current_link and current_link not in ('base_link', 'world'):
        if current_link in visited:
            break  # Avoid infinite loops
        visited.add(current_link)

        if current_link not in joints:
            break

        joint = joints[current_link]
        origin = joint.find('origin')
        xyz, rpy = _parse_origin(origin)

        # For simplicity, only handle translation (ignore rotation for now)
        # Full implementation would use rotation matrices
        cumulative_offset += np.array(xyz)

        parent_link = joint.find('parent')
        if parent_link is not None:
            current_link = parent_link.get('link')
        else:
            break

    return cumulative_offset


def _extract_workspace_from_urdf(robot_controller, max_retries=3, retry_delay=0.1):
    """Extract workspace boundaries from mounting_surface collision mesh in URDF."""
    import xml.etree.ElementTree as ET
    import os
    from ament_index_python.packages import get_package_share_directory
    import time
    from std_msgs.msg import String
    import numpy as np

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
                    # Get collision origin transform (mesh position relative to link)
                    collision_origin = collision.find('origin')
                    collision_xyz, collision_rpy = _parse_origin(collision_origin)

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
                                    bounds = mesh.bounds  # [[x_min, y_min, z_min], [x_max, y_max, z_max]]

                                    # Get transform from mounting_surface link to base_link frame
                                    link_to_base_offset = _build_transform_to_base_link(root, 'mounting_surface')

                                    # Total offset = collision origin + link-to-base transform
                                    total_offset = np.array(collision_xyz) + link_to_base_offset

                                    robot_controller.get_logger().info(
                                        f"Mesh bounds (local): min={bounds[0]}, max={bounds[1]}"
                                    )
                                    robot_controller.get_logger().info(
                                        f"Collision origin: {collision_xyz}, Link offset: {link_to_base_offset}"
                                    )
                                    robot_controller.get_logger().info(
                                        f"Total offset applied: {total_offset}"
                                    )

                                    # Apply offset to mesh bounds
                                    # the bottom wall goes below the surface so need to account for the surface thickness
                                    # for now will set thickness to 50mm or 0.05m
                                    surface_thickness = 0.05
                                    workspace = {
                                        'x_min': float(bounds[0][0] + total_offset[0]),
                                        'x_max': float(bounds[1][0] + total_offset[0]),
                                        'y_min': float(bounds[0][1] + total_offset[1]),
                                        'y_max': float(bounds[1][1] + total_offset[1]),
                                        'z_min': float(bounds[0][2] + total_offset[2])+surface_thickness,
                                        'z_max': SAFETY_WORKSPACE_FALLBACK['z_max'],
                                    }

                                    robot_controller.get_logger().info(
                                        f"Extracted workspace (base_link frame): "
                                        f"X[{workspace['x_min']:.3f}, {workspace['x_max']:.3f}] "
                                        f"Y[{workspace['y_min']:.3f}, {workspace['y_max']:.3f}] "
                                        f"Z[{workspace['z_min']:.3f}, {workspace['z_max']:.3f}]"
                                    )
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