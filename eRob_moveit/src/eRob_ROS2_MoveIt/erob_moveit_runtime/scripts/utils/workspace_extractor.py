import numpy as np

SAFETY_WORKSPACE_FALLBACK = {
    'x_min': -0.37,
    'x_max': 0.5,
    'y_min': -0.8,
    'y_max': 0.6,
    'z_min': 0.0,
    'z_max': 1.1,
}


def _parse_origin(origin_elem):
    if origin_elem is None:
        return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
    xyz = tuple(float(v) for v in origin_elem.get('xyz', '0 0 0').split())
    rpy = tuple(float(v) for v in origin_elem.get('rpy', '0 0 0').split())
    return xyz, rpy


def _build_transform_to_base_link(root, link_name):
    joints = {}
    for joint in root.findall('.//joint'):
        child_link = joint.find('child')
        if child_link is not None:
            joints[child_link.get('link')] = joint

    cumulative_offset = np.array([0.0, 0.0, 0.0])
    current_link = link_name
    visited = set()

    while current_link and current_link not in ('base_link', 'world'):
        if current_link in visited:
            break
        visited.add(current_link)
        if current_link not in joints:
            break
        joint = joints[current_link]
        xyz, _ = _parse_origin(joint.find('origin'))
        cumulative_offset += np.array(xyz)
        parent = joint.find('parent')
        current_link = parent.get('link') if parent is not None else None

    return cumulative_offset


def _bounds_from_boxes(collision_elements, link_offset):
    """Compute combined AABB from all <box> collision elements.
    Also returns plate_top_z: the z_max of the lowest box (= top of the base plate).
    """
    corners = []
    lowest_z_min = float('inf')
    plate_top_z = None

    for coll in collision_elements:
        box_elem = coll.find('.//box')
        if box_elem is None:
            continue
        size_str = box_elem.get('size', '')
        if not size_str:
            continue
        sx, sy, sz = [float(v) for v in size_str.split()]
        ox, oy, oz = _parse_origin(coll.find('origin'))[0]
        ox += link_offset[0]
        oy += link_offset[1]
        oz += link_offset[2]

        box_z_min = oz - sz / 2
        box_z_max = oz + sz / 2
        if box_z_min < lowest_z_min:
            lowest_z_min = box_z_min
            plate_top_z = box_z_max

        for dx in (-sx / 2, sx / 2):
            for dy in (-sy / 2, sy / 2):
                for dz in (-sz / 2, sz / 2):
                    corners.append([ox + dx, oy + dy, oz + dz])

    if not corners:
        return None

    pts = np.array(corners)
    return {
        'x_min': float(pts[:, 0].min()), 'x_max': float(pts[:, 0].max()),
        'y_min': float(pts[:, 1].min()), 'y_max': float(pts[:, 1].max()),
        'z_min': float(pts[:, 2].min()), 'z_max': float(pts[:, 2].max()),
        'plate_top_z': plate_top_z,
    }


def _extract_workspace_from_urdf(robot_controller, max_retries=3, retry_delay=0.1):
    """Extract workspace boundaries from mounting_surface collision geometry in URDF.

    Prefer the runtime's configured URDF file. This keeps construction independent
    of ROS executor state and is especially important when multiple RobotController
    instances are being assembled before their shared executor starts spinning.
    The legacy /robot_description topic path remains as a compatibility fallback.
    """
    import xml.etree.ElementTree as ET
    import os
    from ament_index_python.packages import get_package_share_directory
    import time
    from std_msgs.msg import String

    surface_thickness = 0.05

    configured_robot_description = None
    try:
        import config

        configured_urdf_path = str(getattr(config, 'URDF_PATH', '') or '').strip()
        if configured_urdf_path and os.path.isfile(configured_urdf_path):
            with open(configured_urdf_path, 'r', encoding='utf-8') as urdf_file:
                configured_robot_description = urdf_file.read()
            robot_controller.get_logger().info(
                f"Workspace extraction using configured URDF: {configured_urdf_path}"
            )
    except Exception as exc:
        robot_controller.get_logger().debug(
            f"Could not read configured URDF for workspace extraction: {exc}"
        )

    for attempt in range(max_retries):
        try:
            robot_description = configured_robot_description

            if not robot_description:
                try:
                    from rclpy.qos import QoSProfile, DurabilityPolicy
                    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
                    future_msg = None

                    def desc_callback(msg):
                        nonlocal future_msg
                        future_msg = msg.data

                    sub = robot_controller.create_subscription(String, '/robot_description', desc_callback, qos)
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
                robot_controller.get_logger().info("Using fallback workspace (robot_description not available)")
                return SAFETY_WORKSPACE_FALLBACK.copy()

            root = ET.fromstring(robot_description)
            link_to_base_offset = _build_transform_to_base_link(root, 'mounting_surface')

            for link in root.findall('.//link[@name="mounting_surface"]'):
                all_collisions = link.findall('collision')

                for coll in all_collisions:
                    mesh_elem = coll.find('.//mesh')
                    if mesh_elem is None:
                        continue
                    mesh_filename = mesh_elem.get('filename', '')
                    if not mesh_filename.startswith('package://'):
                        continue
                    parts = mesh_filename.replace('package://', '').split('/', 1)
                    if len(parts) != 2:
                        continue
                    try:
                        mesh_path = os.path.join(
                            get_package_share_directory(parts[0]), parts[1])
                        if not os.path.exists(mesh_path):
                            continue
                        import trimesh
                        mesh = trimesh.load(mesh_path)
                        bounds = mesh.bounds
                        coll_xyz, _ = _parse_origin(coll.find('origin'))
                        total_offset = np.array(coll_xyz) + link_to_base_offset
                        workspace = {
                            'x_min': float(bounds[0][0] + total_offset[0]),
                            'x_max': float(bounds[1][0] + total_offset[0]),
                            'y_min': float(bounds[0][1] + total_offset[1]),
                            'y_max': float(bounds[1][1] + total_offset[1]),
                            'z_min': float(bounds[0][2] + total_offset[2]) + surface_thickness,
                            'z_max': SAFETY_WORKSPACE_FALLBACK['z_max'],
                        }
                        robot_controller.get_logger().info(
                            f"Workspace from mesh: "
                            f"X[{workspace['x_min']:.3f}, {workspace['x_max']:.3f}] "
                            f"Y[{workspace['y_min']:.3f}, {workspace['y_max']:.3f}] "
                            f"Z[{workspace['z_min']:.3f}, {workspace['z_max']:.3f}]")
                        return workspace
                    except Exception as e:
                        robot_controller.get_logger().debug(f"Mesh load failed: {e}")

                box_bounds = _bounds_from_boxes(all_collisions, link_to_base_offset)
                if box_bounds is not None:
                    plate_top = box_bounds['plate_top_z']
                    workspace = {
                        'x_min': box_bounds['x_min'],
                        'x_max': box_bounds['x_max'],
                        'y_min': box_bounds['y_min'],
                        'y_max': box_bounds['y_max'],
                        'z_min': plate_top + 0.005,
                        'z_max': SAFETY_WORKSPACE_FALLBACK['z_max'],
                    }
                    robot_controller.get_logger().info(
                        f"Workspace from box primitives (plate_top={plate_top:.4f}m): "
                        f"X[{workspace['x_min']:.3f}, {workspace['x_max']:.3f}] "
                        f"Y[{workspace['y_min']:.3f}, {workspace['y_max']:.3f}] "
                        f"Z[{workspace['z_min']:.3f}, {workspace['z_max']:.3f}]")
                    return workspace

            robot_controller.get_logger().info("Using fallback workspace (mounting_surface not in URDF)")
            return SAFETY_WORKSPACE_FALLBACK.copy()

        except Exception as e:
            if attempt < max_retries - 1:
                time.sleep(retry_delay)
                continue
            robot_controller.get_logger().info(
                f"Using fallback workspace (URDF parsing failed: {e})"
            )

    return SAFETY_WORKSPACE_FALLBACK.copy()
