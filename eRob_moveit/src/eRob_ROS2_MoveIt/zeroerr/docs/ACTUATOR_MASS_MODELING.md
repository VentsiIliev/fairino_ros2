# ZeroErr Actuator Mass Modeling

This note documents how actuator family masses are folded into the ZeroErr arm URDF without replacing the original link inertials from scratch.

Related files:
- [erob_arm_family_motor_masses.urdf](/home/ilv/ros2_ws/eRob_moveit/src/eRob_ROS2_MoveIt/zeroerr/config/urdfs/erob_arm_family_motor_masses.urdf)
- [MOTOR_MASS_ASSUMPTIONS.md](/home/ilv/ros2_ws/eRob_moveit/src/eRob_ROS2_MoveIt/zeroerr/docs/MOTOR_MASS_ASSUMPTIONS.md)

## Current Actuator Configuration

Current arm mapping:
- `J1` = `eRob80H100T-BHM-18ET`
- `J2` = `eRob80H100T-BHM-18ET`
- `J3` = `eRob80H100T-BHM-18ET`
- `J4` = `eRob70H100T-BHM-18ET`
- `J5` = `eRob70H100T-BHM-18ET`
- `J6` = `eRob70H100T-BHM-18ET`

## Source Models And URLs

Robot family mapping source:
- ZeroErr 3kg robot arm recommended configuration:
- https://en.zeroerr.cn/arms/3kg-robot-arm

Actuator family source pages:
- `eRob70T`:
- https://en.zeroerr.cn/rotary_actuators/erob70t
- `eRob80T`:
- https://en.zeroerr.cn/rotary_actuators/erob80t

Referenced family specs used for mass modeling:
- `eRob70HXXT`: mass `1.24 kg`, envelope `73 x 99 mm`
- `eRob80HXXT`: mass `1.94 kg`, envelope `85 x 111.8 mm`

These are family-level published specs. They are the best public values currently available for:
- `eRob70H100T-BHM-18ET`
- `eRob80H100T-BHM-18ET`

## Published Family Specs

### `eRob80HXXT`

- Strain wave gear ratio: `17-50`, `17-80`, `17-100`, `17-120`
- Peak torque for start and stop: `44`, `56`, `70`, `70 Nm`
- Permissible maximum average load torque: `34`, `35`, `51`, `51 Nm`
- Rated torque: `21`, `29`, `31`, `31 Nm`
- Permissible maximum momentary torque: `91`, `113`, `143`, `112 Nm`
- Max output rotational speed: `60`, `37.5`, `30`, `25 RPM`
- Motor power: `146 W`
- Output encoder resolution: `19 Bit`
- Repeatability / Accuracy: `±10 / 45 arcsec`
- Communication bus: `EtherCAT / CANopen`
- `OD x L x ID`: `85 x 111.8 x 18 mm`
- Weight: `1.94 kg`
- Brake: friction brake
- IP grade: `IP65`
- EtherCAT optional interfaces: `RS485`, `Pulse/DIR`, `I/O`, `±10V Analog`, `STO`
- CANopen optional interfaces: `RS485`, `Pulse/DIR`, `I/O`, `±10V Analog`, `STO`

### `eRob70HXXT`

- Strain wave gear ratio: `14-50`, `14-80`, `14-100`, `14-120`
- Peak torque for start and stop: `23`, `30`, `36`, `36 Nm`
- Permissible maximum average load torque: `9`, `14`, `14`, `14 Nm`
- Rated torque: `7`, `10`, `10`, `10 Nm`
- Permissible maximum momentary torque: `46`, `61`, `70`, `70 Nm`
- Max output rotational speed: `60`, `37.5`, `30`, `25 RPM`
- Motor power: `100 W`
- Output encoder resolution: `19 Bit`
- Repeatability / Accuracy: `±10 / 45 arcsec`
- Communication bus: `EtherCAT / CANopen`
- `OD x L x ID`: `73 x 99 x 18 mm`
- Weight: `1.24 kg`
- Brake: friction brake
- IP grade: `IP65`
- EtherCAT optional interfaces: `RS485`, `Pulse/DIR`, `I/O`, `±10V Analog`, `STO`
- CANopen optional interfaces: `RS485`, `Pulse/DIR`, `I/O`, `±10V Analog`, `STO`

## Model Decoding

### `eRob70H100T-BHM-18ET`

- Actuator Series: `eRob`
- Outer Diameter: `70`
- Length and Load: `H` = high torque strain wave gear
- Gear Ratio: `100`
- Model: `T`
- Brakes: `B` = with brakes
- Encoder: `HM` = high precision calibrated multiturn encoder
- Inner Diameter: `18`
- Communication Protocol: `E` = EtherCAT
- Virtual Torque Sensor: `T`

### `eRob80H100T-BHM-18ET`

- Actuator Series: `eRob`
- Outer Diameter: `80`
- Length and Load: `H` = high torque strain wave gear
- Gear Ratio: `100`
- Model: `T`
- Brakes: `B` = with brakes
- Encoder: `HM` = high precision calibrated multiturn encoder
- Inner Diameter: `18`
- Communication Protocol: `E` = EtherCAT
- Virtual Torque Sensor: `T`

## Inertial Update Method

The intent is:
- keep the original link inertial from the URDF
- add the actuator mass as an extra rigid body
- compute the new combined mass, COM, and inertia

This is preferable to discarding the original URDF inertial and inventing a brand new one.

### Variables

- `m_link`: original link mass
- `c_old`: original link COM
- `I_link`: original link inertia tensor about `c_old`
- `m_motor`: actuator mass
- `c_motor`: assumed actuator COM in the same link frame
- `I_motor`: approximated actuator inertia tensor about `c_motor`

### New combined mass

```text
M = m_link + m_motor
```

### New combined COM

```text
c_new = (m_link * c_old + m_motor * c_motor) / M
```

### COM shift deltas

```text
d_link  = c_old - c_new
d_motor = c_motor - c_new
```

### Corrected inertia tensor

Using the parallel-axis theorem:

```text
I_new =
  I_link + m_link * (||d_link||^2 * I3 - d_link d_link^T)
+ I_motor + m_motor * (||d_motor||^2 * I3 - d_motor d_motor^T)
```

Where:
- `I3` is the `3 x 3` identity matrix
- `d d^T` is the outer product of the offset vector with itself

## Actuator COM Assumption

The current first-pass model places the added actuator COM:
- approximately halfway along the outgoing segment from the link origin toward the next joint/tool frame

This is only an engineering estimate. It is useful for improving gravity, COM, and inertia compared to omitting the actuator mass entirely, but it is not equivalent to factory CAD inertials.

## Actuator Inertia Assumption

The current first-pass model approximates each actuator as a solid box:

- `80T`: `0.085 x 0.085 x 0.1118 m`
- `70T`: `0.073 x 0.073 x 0.099 m`

This produces `I_motor`, which is then shifted to the combined COM using the parallel-axis theorem.

## Practical Notes

- The mass values are much more certain than the exact actuator COM and inertia orientation.
- If vendor CAD, STEP, or factory inertial data becomes available, only `c_motor` and `I_motor` need to be improved.
- The update method itself should remain the same.

## Current Status

The copied URDF variant:
- does not replace the active runtime model automatically
- exists so the mass-adjusted dynamics can be tested separately from the current ZeroErr configuration

If this model is adopted, the next step should be:
- switch launch/runtime to the copied URDF behind an explicit config choice
- compare gravity behavior, dynamics estimates, and collision/damping behavior against the current model
