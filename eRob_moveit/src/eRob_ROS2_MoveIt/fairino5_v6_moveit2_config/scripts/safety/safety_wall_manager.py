#!/usr/bin/env python3
"""
SafetyWallManager - Manages safety workspace boundaries for robot motion control.

This module provides centralized management of safety boundaries including:
- Workspace boundary definitions
- MoveIt collision object creation
- RViz visualization markers
- Position validation (pre-checks)
- Safety enable/disable functionality
"""

import rclpy
import time
from rclpy.node import Node
from moveit_msgs.msg import PlanningScene, CollisionObject, AllowedCollisionMatrix, AllowedCollisionEntry
from shape_msgs.msg import SolidPrimitive
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose
import config


class SafetyWallManager:
    """
    Manages safety workspace boundaries for robot motion control.

    Responsibilities:
    - Define and configure workspace boundaries
    - Create MoveIt collision objects for safety walls
    - Create RViz visualization markers
    - Validate positions before motion planning (pre-checks)
    - Enable/disable safety features
    - Publish collision objects and markers
    """

    def __init__(self, node: Node, workspace: dict = None, margin: float = config.SAFETY_MARGIN_M,
                 enabled: bool = True, marker_publish_interval: float = config.MARKER_PUBLISH_INTERVAL_S,
                 acm_bypass_links: frozenset = config.WALL_BYPASS_LINKS):
        """
        Initialize safety wall manager.

        Args:
            node: ROS2 Node instance for publishing and logging
            workspace: Dict with keys x_min/max, y_min/max, z_min/max (meters)
                      If None, must be set later via set_workspace_bounds()
            margin: Safety margin in meters (default: 0.01m = 10mm)
            enabled: Enable safety checks by default
            marker_publish_interval: Interval for marker publishing (seconds)
        """
        self.node = node
        self.safety_workspace = workspace.copy() if workspace is not None else {}
        self.safety_margin = margin
        self.safety_enabled = enabled
        self.walls_marker_array = None
        self.acm_bypass_links = set(acm_bypass_links)

        # Create ROS2 publishers
        self.planning_scene_pub = self.node.create_publisher(
            PlanningScene, config.TOPIC_PLANNING_SCENE, 10
        )
        self.safety_walls_pub = self.node.create_publisher(
            MarkerArray, config.TOPIC_SAFETY_WALLS, 10
        )

        # Create a timer for periodic marker publishing
        self.marker_timer = self.node.create_timer(
            marker_publish_interval,
            self._publish_markers_callback
        )

        # Don't publish immediately - wait for explicit force_update() call
        # This speeds up initialization
        if self.safety_workspace:
            self.node.get_logger().info('[SafetyWallManager] Initialized (deferred publishing)')
        else:
            self.node.get_logger().info('[SafetyWallManager] Initialized without workspace')

    # ========== Public API Methods ==========

    def check_position_safety(self, x_m: float, y_m: float, z_m: float) -> tuple:
        """
        Validate if a position is within safety boundaries.

        Args:
            x_m, y_m, z_m: Position in meters (base_link frame)

        Returns:
            tuple: (is_safe: bool, message: str)
                   - is_safe: True if position is safe, False if violates boundaries
                   - message: Description of safety status or violation
        """
        if not self.safety_enabled:
            return True, "Safety checks disabled"

        if not self.safety_workspace:
            return True, "No workspace boundaries configured"

        ws = self.safety_workspace
        margin = self.safety_margin

        violations = []

        # Check hard boundary violations
        if x_m < ws['x_min']:
            violations.append(f"X={x_m:.3f}m < X_MIN={ws['x_min']:.3f}m")
        if x_m > ws['x_max']:
            violations.append(f"X={x_m:.3f}m > X_MAX={ws['x_max']:.3f}m")
        if y_m < ws['y_min']:
            violations.append(f"Y={y_m:.3f}m < Y_MIN={ws['y_min']:.3f}m")
        if y_m > ws['y_max']:
            violations.append(f"Y={y_m:.3f}m > Y_MAX={ws['y_max']:.3f}m")
        if z_m < ws['z_min']:
            violations.append(f"Z={z_m:.3f}m < Z_MIN={ws['z_min']:.3f}m")
        if z_m > ws['z_max']:
            violations.append(f"Z={z_m:.3f}m > Z_MAX={ws['z_max']:.3f}m")

        if violations:
            return False, f"Position violates workspace: {', '.join(violations)}"

        # Check warning margin (soft boundaries)
        warnings = []
        if x_m < ws['x_min'] + margin:
            warnings.append(f"Near X_MIN boundary")
        if x_m > ws['x_max'] - margin:
            warnings.append(f"Near X_MAX boundary")
        if y_m < ws['y_min'] + margin:
            warnings.append(f"Near Y_MIN boundary")
        if y_m > ws['y_max'] - margin:
            warnings.append(f"Near Y_MAX boundary")
        if z_m < ws['z_min'] + margin:
            warnings.append(f"Near Z_MIN boundary")
        if z_m > ws['z_max'] - margin:
            warnings.append(f"Near Z_MAX boundary")

        if warnings:
            return True, f"Warning: {', '.join(warnings)}"

        return True, "Position is safe"

    def add_acm_bypass_link(self, link_name: str):
        """
        Allow a link to contact any safety wall without aborting MoveIt planning.

        Call this to suppress wall-collision errors for tool meshes, end-effector
        geometry, or any other link that legitimately touches the workspace boundary.
        Takes effect on the next publish_to_planning_scene() / force_update() call.

        Example:
            safety_manager.add_acm_bypass_link('disk_link')
            safety_manager.force_update()
        """
        self.acm_bypass_links.add(link_name)
        self.node.get_logger().info(
            f'[SafetyWallManager] ACM bypass added for: {link_name}')

    def publish_to_planning_scene(self):
        """Publish safety walls as collision objects + ACM to MoveIt planning scene."""
        if not self.safety_workspace:
            self.node.get_logger().warning(
                '[SafetyWallManager] Cannot publish to planning scene: no workspace configured'
            )
            return

        ps = PlanningScene()
        ps.is_diff = True
        ps.world.collision_objects.extend(self._create_collision_objects())
        ps.allowed_collision_matrix = self._create_wall_acm()

        self.planning_scene_pub.publish(ps)
        self.node.get_logger().info(
            f'[SafetyWallManager] Published safety walls + ACM '
            f'(bypass links: {sorted(self.acm_bypass_links)})')

    def force_update(self):
        """
        Force immediate update of planning scene (for critical operations).

        Publishes collision objects and waits briefly to ensure MoveIt processes them.
        """
        self.publish_to_planning_scene()
        # Give MoveIt time to process the planning scene update
        # The node is spinning in a background thread, so we just need to wait
        # for the message to propagate through ROS2 and be processed by move_group
        time.sleep(0.05)  # 50ms should be enough for local communication

    def get_workspace_bounds(self) -> dict:
        """
        Get current workspace boundaries.

        Returns:
            dict: Copy of workspace boundaries (x_min/max, y_min/max, z_min/max)
        """
        return self.safety_workspace.copy()

    def set_workspace_bounds(self, workspace: dict):
        """
        Update workspace boundaries at runtime.

        Args:
            workspace: Dict with keys x_min/max, y_min/max, z_min/max (meters)

        Note:
            - Invalidates cached markers (will be recreated)
            - Republishes collision objects to planning scene
        """
        self.safety_workspace = workspace.copy()
        self.walls_marker_array = None  # Invalidate marker cache
        self.publish_to_planning_scene()
        self.node.get_logger().info('[SafetyWallManager] Workspace boundaries updated')

    def enable_safety(self):
        """Enable safety boundary checks."""
        self.safety_enabled = True
        self.node.get_logger().info('[SafetyWallManager] Safety checks ENABLED')

    def disable_safety(self):
        """Disable safety boundary checks (use with caution!)."""
        self.safety_enabled = False
        self.node.get_logger().warning('[SafetyWallManager] Safety checks DISABLED - use with caution!')

    def is_enabled(self) -> bool:
        """
        Check if safety is enabled.

        Returns:
            bool: True if safety checks are enabled
        """
        return self.safety_enabled

    # ========== Private Methods ==========

    def _create_wall_acm(self) -> AllowedCollisionMatrix:
        acm = AllowedCollisionMatrix()
        # default_entry_names only fills gaps — does NOT override SRDF disable_collisions entries
        phantom = sorted(config.SAFETY_WALL_NAMES)
        acm.default_entry_names = phantom
        acm.default_entry_values = [True] * len(phantom)
        return acm

    def _wall_geometries(self) -> list:
        """Return list of (name, cx, cy, cz, sx, sy, sz) for all 6 boundary walls."""
        ws = self.safety_workspace
        xc = (ws['x_min'] + ws['x_max']) / 2
        yc = (ws['y_min'] + ws['y_max']) / 2
        zc = (ws['z_min'] + ws['z_max']) / 2
        dx = ws['x_max'] - ws['x_min']
        dy = ws['y_max'] - ws['y_min']
        dz = ws['z_max'] - ws['z_min']
        t  = config.WALL_THICKNESS_M
        return [
            ('wall_x_min', ws['x_min'], yc, zc, t,  dy, dz),
            ('wall_x_max', ws['x_max'], yc, zc, t,  dy, dz),
            ('wall_y_min', xc, ws['y_min'], zc, dx, t,  dz),
            ('wall_y_max', xc, ws['y_max'], zc, dx, t,  dz),
            ('wall_z_min', xc, yc, ws['z_min'], dx, dy, t),
            ('wall_z_max', xc, yc, ws['z_max'], dx, dy, t),
        ]

    def _create_collision_objects(self) -> list:
        """
        Create MoveIt collision objects corresponding to safety walls.

        Returns:
            list: List of 6 CollisionObject instances (one per boundary wall)
        """
        collision_objects = []
        walls = self._wall_geometries()

        for name, cx, cy, cz, sx, sy, sz in walls:
            co = CollisionObject()
            co.id = name
            co.header.frame_id = config.BASE_LINK

            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = [sx, sy, sz]

            pose = Pose()
            pose.position.x = cx
            pose.position.y = cy
            pose.position.z = cz
            pose.orientation.w = 1.0

            co.primitives.append(box)
            co.primitive_poses.append(pose)
            co.operation = CollisionObject.ADD

            collision_objects.append(co)

        return collision_objects


    def _create_markers(self) -> MarkerArray:
        """
        Create RViz visualization markers for safety walls.

        Returns:
            MarkerArray: Array of 6 CUBE markers (cached after first creation)
        """
        if self.walls_marker_array is not None:
            return self.walls_marker_array

        if not self.safety_workspace:
            self.node.get_logger().warning(
                '[SafetyWallManager] Cannot create markers: no workspace configured'
            )
            return MarkerArray()

        self.node.get_logger().info('[SafetyWallManager] Creating workspace boundary wall markers...')

        walls = self._wall_geometries()
        marker_array = MarkerArray()

        for i, (wall_name, cx, cy, cz, sx, sy, sz) in enumerate(walls):
            marker = Marker()
            marker.header.frame_id = config.BASE_LINK
            marker.ns = 'safety_walls'
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            # Position
            marker.pose.position.x = cx
            marker.pose.position.y = cy
            marker.pose.position.z = cz
            marker.pose.orientation.w = 1.0

            # Scale (size of the box)
            marker.scale.x = sx
            marker.scale.y = sy
            marker.scale.z = sz

            # Color (red semi-transparent walls)
            marker.color.r = 1.0
            marker.color.g = 0.2
            marker.color.b = 0.2
            marker.color.a = 0.3  # 30% opacity

            # Lifetime (0 = forever)
            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 0

            marker_array.markers.append(marker)
            self.node.get_logger().info(
                f'[SafetyWallManager] Created {wall_name} at ({cx:.2f}, {cy:.2f}, {cz:.2f})'
            )

        ws = self.safety_workspace
        self.node.get_logger().info('[SafetyWallManager] Workspace boundary markers created!')
        self.node.get_logger().info(
            f'[SafetyWallManager] Workspace: '
            f'X[{ws["x_min"]:.2f},{ws["x_max"]:.2f}] '
            f'Y[{ws["y_min"]:.2f},{ws["y_max"]:.2f}] '
            f'Z[{ws["z_min"]:.2f},{ws["z_max"]:.2f}]'
        )

        self.walls_marker_array = marker_array
        return marker_array

    def _publish_markers_callback(self):
        """Timer callback for periodic marker publishing to keep them visible in RViz."""
        if not self.safety_workspace:
            return

        if self.walls_marker_array is None:
            self.walls_marker_array = self._create_markers()

        current_time = self.node.get_clock().now().to_msg()
        for marker in self.walls_marker_array.markers:
            marker.header.stamp = current_time

        self.safety_walls_pub.publish(self.walls_marker_array)
