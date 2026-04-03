ZeroErr actuator family inertial assumptions used in `erob_arm_family_motor_masses.urdf`.

Sources:
- `eRob70T` family page: `1.24 kg`, `73 x 99 mm`
- `eRob80T` family page: `1.94 kg`, `85 x 111.8 mm`
- The published values are family-level specs, not per-serial CAD inertias.

Assignment used in this first-pass model:
- `Link_1`, `Link_2`, `Link_3`: add one `eRob80H100T` actuator each
- `Link_4`, `Link_5`, `Link_6`: add one `eRob70H100T` actuator each

COM assumption:
- Each added actuator COM is placed at half of the outgoing segment from the link origin toward the next joint/tool frame.

Inertia assumption:
- Each actuator is approximated as a solid box with dimensions:
- `80T`: `0.085 x 0.085 x 0.1118 m`
- `70T`: `0.073 x 0.073 x 0.099 m`

Combination rule:
- The new link mass is `m_link + m_actuator`
- The new COM is the mass-weighted average
- The new inertia uses the parallel-axis theorem from the original link inertia plus the box-approximated actuator inertia

This is a better dynamics estimate than omitting actuator mass, but it is still an engineering approximation.
