try:
    from .safety_wall_manager import SafetyWallManager
    from .collision_detection import (
        DynamicsCollisionDetector,
        CollisionState,
        create_dynamics_collision_detector,
        InverseDynamicsModel,
        KDLInverseDynamicsModel,
        ExternalTorqueEstimator,
        CollisionDetectionStrategy,
        RateThresholdStrategy,
        SustainedTorqueStrategy,
    )
except RuntimeError:
    # EROB_CONFIG_PACKAGE not set — package is being imported as a library
    # (e.g. from zeroerr_collision_monitor). Submodules are still importable.
    pass
