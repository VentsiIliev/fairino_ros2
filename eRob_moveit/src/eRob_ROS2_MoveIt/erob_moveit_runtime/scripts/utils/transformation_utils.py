import numpy as np
from scipy.spatial.transform import Rotation


class TransformationUtils:
    """Centralized transformation utilities for robot coordinate conversions.

    Provides conversions between Euler angles, rotation matrices, quaternions,
    and homogeneous transforms. All angles use XYZ Euler convention.
    """

    # ============ Coordinate Conversions ============

    @staticmethod
    def euler_to_matrix(euler, degrees=True):
        """Convert Euler angles to 3x3 rotation matrix.

        Args:
            euler: Array-like [rx, ry, rz] in degrees (default) or radians
            degrees: True if input is in degrees, False if radians

        Returns:
            3x3 numpy rotation matrix
        """
        return Rotation.from_euler('xyz', euler, degrees=degrees).as_matrix()

    @staticmethod
    def matrix_to_euler(matrix, degrees=True):
        """Convert 3x3 rotation matrix to Euler angles.

        Args:
            matrix: 3x3 rotation matrix
            degrees: True to return degrees, False for radians

        Returns:
            Array [rx, ry, rz] in degrees (default) or radians
        """
        return Rotation.from_matrix(matrix).as_euler('xyz', degrees=degrees)

    @staticmethod
    def quaternion_to_matrix(quat):
        """Convert quaternion to 3x3 rotation matrix.

        Args:
            quat: Quaternion as [x, y, z, w]

        Returns:
            3x3 numpy rotation matrix
        """
        return Rotation.from_quat(quat).as_matrix()

    @staticmethod
    def matrix_to_quaternion(matrix):
        """Convert 3x3 rotation matrix to quaternion.

        Args:
            matrix: 3x3 rotation matrix

        Returns:
            Quaternion as [x, y, z, w]
        """
        return Rotation.from_matrix(matrix).as_quat(canonical=False)

    # ============ Homogeneous Transform Construction ============

    @staticmethod
    def pose_to_transform(pose, mm_to_m=True):
        """Convert 6D pose to 4x4 homogeneous transform.

        Args:
            pose: [x, y, z, rx, ry, rz] where position is in mm (default) and orientation in degrees
            mm_to_m: True to convert position from mm to meters

        Returns:
            4x4 homogeneous transformation matrix
        """
        T = np.eye(4)
        T[:3, :3] = TransformationUtils.euler_to_matrix(pose[3:])
        position = np.array(pose[:3])
        if mm_to_m:
            position = position / 1000.0  # Convert mm to meters
        T[:3, 3] = position
        return T

    @staticmethod
    def transform_to_pose(T, m_to_mm=True):
        """Convert 4x4 homogeneous transform to 6D pose.

        Args:
            T: 4x4 homogeneous transformation matrix
            m_to_mm: True to convert position from meters to mm

        Returns:
            List [x, y, z, rx, ry, rz] where position is in mm (default) and orientation in degrees
        """
        position = T[:3, 3]
        if m_to_mm:
            position = position * 1000.0  # Convert meters to mm
        euler = TransformationUtils.matrix_to_euler(T[:3, :3])
        return [position[0], position[1], position[2], euler[0], euler[1], euler[2]]

    @staticmethod
    def make_transform(position, rotation_matrix):
        """Create 4x4 homogeneous transform from position and rotation.

        Args:
            position: 3D position vector [x, y, z]
            rotation_matrix: 3x3 rotation matrix

        Returns:
            4x4 homogeneous transformation matrix
        """
        T = np.eye(4)
        T[:3, :3] = rotation_matrix
        T[:3, 3] = position
        return T

    # ============ Transform Operations ============

    @staticmethod
    def compose_transforms(*transforms):
        """Compose multiple transforms using matrix multiplication.

        Args:
            *transforms: Variable number of 4x4 transformation matrices

        Returns:
            Composed 4x4 transformation matrix (T1 @ T2 @ T3 @ ...)
        """
        result = transforms[0]
        for T in transforms[1:]:
            result = result @ T
        return result

    @staticmethod
    def invert_transform(T):
        """Compute inverse of 4x4 homogeneous transform.

        Args:
            T: 4x4 transformation matrix

        Returns:
            Inverse 4x4 transformation matrix
        """
        return np.linalg.inv(T)

    @staticmethod
    def transform_pose(pose, T_frame, inverse=False):
        """Transform a pose between coordinate frames.

        Args:
            pose: [x, y, z, rx, ry, rz] in mm and degrees
            T_frame: 4x4 frame transformation matrix
            inverse: If True, apply inverse transform (base → frame), otherwise (frame → base)

        Returns:
            Transformed pose [x, y, z, rx, ry, rz] in mm and degrees
        """
        # Convert pose to transform
        T_pose = TransformationUtils.pose_to_transform(pose)

        # Apply frame transformation
        if inverse:
            T_result = TransformationUtils.invert_transform(T_frame) @ T_pose
        else:
            T_result = T_frame @ T_pose

        # Convert back to pose
        return TransformationUtils.transform_to_pose(T_result)

    # ============ TCP/Tool Operations ============

    @staticmethod
    def remove_tcp_offset(T_tcp_pose, T_tool):
        """Remove TCP tool offset to get end-effector link pose.

        Given: TCP_pose = ee_link_pose @ T_tool
        Returns: ee_link_pose = TCP_pose @ inv(T_tool)

        Args:
            T_tcp_pose: 4x4 desired TCP pose
            T_tool: 4x4 tool offset (ee_link → TCP)

        Returns:
            4x4 end-effector link pose
        """
        return T_tcp_pose @ TransformationUtils.invert_transform(T_tool)

    # ============ FK Helper Matrices ============

    @staticmethod
    def rot_z(angle):
        """Create 4x4 rotation matrix around Z-axis.

        Args:
            angle: Rotation angle in radians

        Returns:
            4x4 homogeneous rotation matrix
        """
        c, s = np.cos(angle), np.sin(angle)
        return np.array([[c, -s, 0, 0],
                         [s, c, 0, 0],
                         [0, 0, 1, 0],
                         [0, 0, 0, 1]])

    @staticmethod
    def rot_x(angle):
        """Create 4x4 rotation matrix around X-axis.

        Args:
            angle: Rotation angle in radians

        Returns:
            4x4 homogeneous rotation matrix
        """
        c, s = np.cos(angle), np.sin(angle)
        return np.array([[1, 0, 0, 0],
                         [0, c, -s, 0],
                         [0, s, c, 0],
                         [0, 0, 0, 1]])

    @staticmethod
    def rot_y(angle):
        """Create 4x4 rotation matrix around Y-axis.

        Args:
            angle: Rotation angle in radians

        Returns:
            4x4 homogeneous rotation matrix
        """
        c, s = np.cos(angle), np.sin(angle)
        return np.array([[c, 0, s, 0],
                         [0, 1, 0, 0],
                         [-s, 0, c, 0],
                         [0, 0, 0, 1]])

    @staticmethod
    def trans(x, y, z):
        """Create 4x4 translation matrix.

        Args:
            x, y, z: Translation distances

        Returns:
            4x4 homogeneous translation matrix
        """
        return np.array([[1, 0, 0, x],
                         [0, 1, 0, y],
                         [0, 0, 1, z],
                         [0, 0, 0, 1]])

    # ============ TF2 Conversions ============

    @staticmethod
    def tf2_to_transform(transform_stamped):
        """Convert ROS2 TransformStamped to 4x4 homogeneous matrix.

        Args:
            transform_stamped: geometry_msgs.msg.TransformStamped

        Returns:
            4x4 numpy transformation matrix
        """
        t = transform_stamped.transform.translation
        q = transform_stamped.transform.rotation
        T = np.eye(4)
        T[:3, 3] = [t.x, t.y, t.z]
        T[:3, :3] = TransformationUtils.quaternion_to_matrix([q.x, q.y, q.z, q.w])
        return T
