import numpy as np

from .transformation_utils import TransformationUtils


class WorkObject:
    """
    Represents a work object (coordinate frame) relative to the robot base.
    """
    def __init__(self, x=0, y=0, z=0, rx=0, ry=0, rz=0):
        """Initialize a work object with position (mm) and orientation (degrees)."""
        # Store individual attributes for external compatibility
        self.x = x
        self.y = y
        self.z = z
        self.rx = rx
        self.ry = ry
        self.rz = rz
        # Also store as arrays for internal use
        self.position = np.array([x, y, z], dtype=float)
        self.orientation = np.array([rx, ry, rz], dtype=float)
        # Precompute 4x4 homogeneous transform from base to work object
        pose = [x, y, z, rx, ry, rz]
        self.transform = TransformationUtils.pose_to_transform(pose)

    def apply(self, pose, inverse=False):
        """
        Apply work object transform to a Cartesian pose.

        Args:
            pose: [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg]
            inverse: If True, transform from base frame to workobject frame

        Returns:
            Transformed pose [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg]
        """
        return TransformationUtils.transform_pose(pose, self.transform, inverse=inverse)
