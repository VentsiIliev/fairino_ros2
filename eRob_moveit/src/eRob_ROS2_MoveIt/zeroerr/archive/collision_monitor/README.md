# Archived ZeroErr Collision Monitor

This folder contains the experimental ZeroErr collision monitor tooling that was
removed from the active launch/runtime path.

Archived artifacts:

- `scripts/zeroerr_collision_monitor.py`
- `scripts/zeroerr_collision_monitor_gui.py`
- `scripts/zeroerr_torque_model_fit.py`
- `scripts/zeroerr_torque_threshold_fit.py`
- `config/collision_monitor_config.json`
- `config/torque_sensor_model.json`
- `docs/COLLISION_MONITOR.md`
- `docs/PLOTJUGGLER_COLLISION_MONITOR.md`

The production ZeroErr stack no longer launches these nodes, installs these
executables, or subscribes to `/zeroerr/collision_monitor/*` topics.
