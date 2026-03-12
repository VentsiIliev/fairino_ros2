#!/usr/bin/env python3
"""
Centralized configuration for the Fairino5 v6 ROS2 stack.

All tunable values live here. Do NOT hardcode these in individual scripts.
To change a parameter (e.g. velocity default, safety margin, DH params),
edit this file — the change propagates everywhere automatically.
"""

# ── Robot Identity ─────────────────────────────────────────────
NUM_JOINTS     = 6
JOINT_NAMES    = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']
PLANNING_GROUP = 'fairino5_v6_group'
BASE_LINK      = 'base_link'
EE_LINK        = 'ee_link'
COLLISION_TIP_LINK = 'wrist3_link'   # used for dynamics collision FK

# ── ROS2 Topics ────────────────────────────────────────────────
TOPIC_ROBOT_STATUS           = '/robot_status'
TOPIC_CARTESIAN_POSITION     = '/cartesian_position'
TOPIC_CARTESIAN_VELOCITY     = '/cartesian_velocity'
TOPIC_CARTESIAN_ACCELERATION = '/cartesian_acceleration'
TOPIC_JOINT_VELOCITY         = '/joint_velocity'
TOPIC_JOINT_ACCELERATION     = '/joint_acceleration'
TOPIC_PLANNING_SCENE         = '/planning_scene'
TOPIC_SAFETY_WALLS           = '/safety_walls'

# ── ROS2 Services / Actions ────────────────────────────────────
SERVICE_CARTESIAN_PATH   = '/compute_cartesian_path'
SERVICE_FK               = '/compute_fk'
SERVICE_APPLY_IPP        = '/apply_ipp'
SERVICE_STATE_VALIDITY   = '/check_state_validity'
ACTION_FOLLOW_TRAJECTORY = '/fairino5_controller/follow_joint_trajectory'

# ── Safety / Workspace ─────────────────────────────────────────
SAFETY_MARGIN_M  = 0.01   # soft boundary inset (10mm)
WALL_XY_OFFSET_M = 0.13  # expand XY walls outward; positive = outward (0.08=80mm)
WALL_THICKNESS_M = 0.01   # MoveIt collision object thickness

# Links allowed to contact safety walls without aborting planning.
# Applied in TWO places:
#   1. MoveIt ACM (compute_cartesian_path) — via SafetyWallManager._create_wall_acm()
#   2. /check_state_validity filter — via _is_ee_wall_contact_only() in trajectory_planner.py
# TCP position is pre-validated by check_position_safety(); these links legitimately
# reach the boundary when the TCP is at the workspace edge.
# To suppress wall collisions for a tool mesh: add its link name here.
# ee_link collision IS the disk.stl mesh (scale 0.001) — already covered below.
# mounting_surface has real collision geometry (Cobot - Plot.stl, fixed to world) — bypass it
# so the static base plate never blocks arm path planning near the floor.
WALL_BYPASS_LINKS = frozenset({

})
SAFETY_WALL_NAMES = frozenset({
    'wall_x_min', 'wall_x_max', 'wall_y_min', 'wall_y_max', 'wall_z_min', 'wall_z_max'
})

# ── Tool Registry ──────────────────────────────────────────────
# Format: [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg]
TOOL_REGISTRY = {
    "TOOL_0": [0, 0, 0, 0, 0, 0],
    "TOOL_1": [0.081, -7.250, 0, 0, 0, 0],
}
TOOL_ID_MAP = {0: "TOOL_0", 1: "TOOL_1"}

# ── Motion Defaults ────────────────────────────────────────────
DEFAULT_VEL_PERCENT  = 30      # % used as REST API default for move_cartesian/linear
DEFAULT_ACC_PERCENT  = 30
DEFAULT_VEL_SCALING  = 0.6     # 0–1 scale fed to TOTG/Ruckig
DEFAULT_ACC_SCALING  = 0.4
DEFAULT_JERK_SCALING = 0.5     # Ruckig only
DEFAULT_ORIENTATION  = [180.0, 0.0, 0.0]   # fallback TCP rx/ry/rz (deg)

# ── Motion Planning ────────────────────────────────────────────
CARTESIAN_MIN_FRACTION  = 0.9     # MoveIt min success fraction
JACOBIAN_FALLBACK_MM    = 0.1     # delta threshold to use Jacobian bypass
JACOBIAN_MAX_JOINT_STEP = 0.05    # rad, clamp per joint
JACOBIAN_MIN_DURATION_S = 0.05    # min trajectory duration
JACOBIAN_DAMPING        = 1e-5    # pseudoinverse damping
JACOBIAN_NUM_DIFF_EPS   = 1e-7    # numerical Jacobian epsilon

# ── Motion Queue ───────────────────────────────────────────────
MOTION_QUEUE_MAX_SIZE = 10

# ── Trajectory Execution ───────────────────────────────────────
EXECUTOR_GOAL_POS_TOL_RAD = 0.01
EXECUTOR_TIME_MULTIPLIER  = 2.0
EXECUTOR_TIME_MIN_S       = 5.0

# ── Trajectory Optimization ────────────────────────────────────
RUCKIG_SAMPLE_DT_S    = 0.008   # 8ms = 125 Hz
OPT_SERVICE_TIMEOUT_S = 5.0

# ── Blocking Move ──────────────────────────────────────────────
BLOCKING_POS_THRESHOLD_MM  = 0.2
BLOCKING_MOVE_TIMEOUT_S    = 60.0
BLOCKING_CHECK_INTERVAL_S  = 0.01

# ── Status / Monitoring ────────────────────────────────────────
STATUS_PUBLISH_RATE_HZ      = 10.0
MONITOR_UPDATE_RATE_HZ      = 50.0
MONITOR_VELOCITY_WINDOW     = 5
MONITOR_ACCELERATION_WINDOW = 5
MARKER_PUBLISH_INTERVAL_S   = 2.0

# ── Collision Detection ────────────────────────────────────────
COLLISION_RATE_THRESHOLDS      = [15.0, 15.0, 12.0, 8.0, 6.0, 4.0]   # N·m/sample per joint
COLLISION_SUSTAINED_THRESHOLDS = [12.0, 12.0, 10.0, 8.0, 6.0, 5.0]   # N·m per joint
COLLISION_CONFIRMATION_SAMPLES = 1
COLLISION_RECOVERY_TIME_S      = 1.0
COLLISION_FILTER_ALPHA         = 0.7
COLLISION_HISTORY_BUFFER       = 100

# ── DH / Kinematics (Fairino5 v6) ─────────────────────────────
# Used in trajectory_planner.py Jacobian FK AND robot_monitor.py FK.
# Single source of truth — changing robot requires updating here only.
DH_D1 = 0.152     # m  (Z-offset base→joint1)
DH_A2 = -0.425    # m  (X-offset joint1→joint2)
DH_A3 = -0.39501  # m  (X-offset joint2→joint3)
DH_D4 = 0.1021    # m  (Z-offset joint3→joint4)
DH_D5 = 0.102     # m  (Z-offset joint4→joint5)

# ── REST Server ────────────────────────────────────────────────
REST_HOST = '0.0.0.0'
REST_PORT = 5000
REST_LOG  = '/tmp/fairino_rest_server.log'

# ── Initialization ─────────────────────────────────────────────
WS_EXTRACT_MAX_RETRIES = 60
WS_EXTRACT_RETRY_DELAY = 2.0   # s
MONITOR_WAIT_TIMEOUT_S = 10.0
