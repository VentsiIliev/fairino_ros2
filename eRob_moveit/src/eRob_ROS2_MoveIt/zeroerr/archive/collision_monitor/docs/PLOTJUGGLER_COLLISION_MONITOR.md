# PlotJuggler With ZeroErr Collision Monitor

The ZeroErr collision monitor now publishes PlotJuggler-friendly ROS topics in addition to:
- `/zeroerr/collision_monitor/table`
- `/zeroerr/collision_monitor/json`
- `/zeroerr/collision_monitor/state`

The GUI table is still useful for:
- threshold editing
- config persistence
- quick latch/contact status checks

PlotJuggler is better for:
- choosing only the signals you care about
- comparing joints over time
- overlaying measured torque, expected torque, torque difference, and external torque

## Published Plot Topics

The monitor publishes one `sensor_msgs/JointState` topic per metric:

- `/zeroerr/collision_monitor/plot/position`
- `/zeroerr/collision_monitor/plot/velocity`
- `/zeroerr/collision_monitor/plot/following_error_actual`
- `/zeroerr/collision_monitor/plot/motor_current_a`
- `/zeroerr/collision_monitor/plot/drive_output_torque`
- `/zeroerr/collision_monitor/plot/current_based_output_torque`
- `/zeroerr/collision_monitor/plot/friction_torque`
- `/zeroerr/collision_monitor/plot/measured_torque`
- `/zeroerr/collision_monitor/plot/expected_torque`
- `/zeroerr/collision_monitor/plot/torque_difference`
- `/zeroerr/collision_monitor/plot/external_torque`
- `/zeroerr/collision_monitor/plot/contact_active`
- `/zeroerr/collision_monitor/plot/contact_latched`
- `/zeroerr/collision_monitor/plot/dynamics_active`
- `/zeroerr/collision_monitor/plot/dynamics_latched`

## Encoding

Each topic is a `sensor_msgs/JointState` message with:
- `name = [Joint_1, ..., Joint_6]`
- `position = metric values`

The topic name identifies the metric.

This means:
- PlotJuggler can use the topic as a time series source
- joint ordering is stable and matches the monitor’s `JOINT_NAMES`

## Typical Signals To Compare

For collision tuning, the most useful overlays are:
- `measured_torque`
- `expected_torque`
- `torque_difference`
- `external_torque`
- `following_error_actual`

For actuator/current correlation:
- `motor_current_a`
- `drive_output_torque`
- `current_based_output_torque`
- `friction_torque`

For event state inspection:
- `contact_active`
- `contact_latched`
- `dynamics_active`
- `dynamics_latched`

## Running PlotJuggler

If PlotJuggler is installed in your ROS environment, the common command is:

```bash
ros2 run plotjuggler plotjuggler
```

Then subscribe to the `/zeroerr/collision_monitor/plot/*` topics and select only the series you want to see.

### Optional auto-launch with ZeroErr full stack

You can also start PlotJuggler automatically with the ZeroErr full stack launch:

```bash
export ZEROERR_PLOTJUGGLER=1
./launch_zeroerr.sh
```

If the variable is unset or `0`, PlotJuggler is not launched.

## Notes

- This change does not remove the existing GUI table.
- The table remains the control/config panel.
- PlotJuggler becomes the preferred time-series viewer.
