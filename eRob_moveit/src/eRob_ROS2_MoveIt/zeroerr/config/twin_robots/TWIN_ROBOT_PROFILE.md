# Twin Robot Profile --- Complete Technical Documentation

**Repository:** `fairino_ros2`\
**Branch documented:** `twin_robots`\
**Proven synchronized-execution milestone:** `twin-prepared-sync-v1`\
**Milestone commit:** `9dcedd3c2ea11bae31e73810a24ef5081f681cc4`\
**ROS 2:** Jazzy\
**Purpose:** Two independent 6-DOF eRob arms in one MoveIt/ros2_control
system, with isolated per-robot planning and synchronized
prepared-trajectory execution.

------------------------------------------------------------------------

## 1. Scope

The twin-robot profile runs two eRob arms in one ROS 2 / MoveIt 2 system
while deliberately keeping the generic motion stack single-robot.

The central architectural rule is:

> **Robot selection and scoping happen before a command enters the
> generic planner, validator, optimizer, or trajectory executor.**

This avoids making every generic motion component aware of `robot1` and
`robot2`. Each robot receives its own runtime context,
`RobotController`, backend and `LocalRuntimeGateway`. Twin-specific
composition lives above those components.

The profile currently supports:

-   two independently scoped 6-DOF robots;
-   one combined URDF and SRDF;
-   separate MoveIt planning groups;
-   separate trajectory controllers;
-   separate robot runtime/gateway instances;
-   concurrent independent robot execution;
-   concurrent path preparation;
-   plan-now / execute-later through `PreparedTrajectory`;
-   synchronized execution using one shared future ROS timestamp;
-   `live_anchor` and `require_exact` prepared-start policies;
-   pre-dispatch rejection when a cached trajectory no longer matches
    the live start state;
-   pair stop;
-   fake-hardware operation;
-   a real-hardware ros2_control topology for 12 EtherCAT slaves;
-   legacy flat runtime keys mapped to robot1 during migration.

Servo is intentionally disabled in the twin profile until Servo is
explicitly robot-aware.

------------------------------------------------------------------------

## 2. Current Proven State

The synchronized prepared-execution foundation was checkpointed as:

``` text
commit: 9dcedd3c2ea11bae31e73810a24ef5081f681cc4
tag:    twin-prepared-sync-v1
branch: twin_robots
```

The verified Stage 3 test demonstrated:

-   both robots prepared their Cartesian paths successfully;
-   preparation caused no robot motion;
-   both prepared starts matched the live joint states;
-   final controller goals were prepared before the common start
    timestamp was selected;
-   both controller dispatch gates were made ready before
    timing-critical dispatch;
-   both goals used the same future start stamp;
-   dispatch separation was approximately **0.522 ms**;
-   both goals were accepted before the common start;
-   both trajectories completed successfully;
-   both final joint states matched their prepared endpoints;
-   observed state-based motion-start skew was approximately **55.7
    ms**.

The state-based skew must not be confused with dispatch skew. State
monitoring was observed at approximately 10 Hz, so motion-start
detection is quantized by state publication timing. The controller
synchronization mechanism is the shared future `header.stamp`.

------------------------------------------------------------------------

## 3. High-Level Architecture

``` text
                         ROS 2 / MoveIt 2
                               |
                     Combined twin description
                               |
                +--------------+--------------+
                |                             |
          robot1_arm                     robot2_arm
                |                             |
       RobotRuntimeContext             RobotRuntimeContext
                |                             |
        RobotController                 RobotController
                |                             |
        Robot backend                   Robot backend
                |                             |
     LocalRuntimeGateway             LocalRuntimeGateway
                |                             |
                +-------------+---------------+
                              |
                      TwinLocalRuntime
                              |
              +---------------+----------------+
              |                                |
       independent commands             coordinated commands
              |                                |
       normal generic stack        prepare_pair / execute_pair
                                               |
                                  SynchronizedTrajectoryExecutor
                                               |
                                    shared future ROS stamp
```

`TwinLocalRuntime` is composition, not a replacement motion stack.

------------------------------------------------------------------------

## 4. Robot Description

### 4.1 Shared world

The twin URDF creates a common `world` link and instantiates the
reusable eRob arm macro twice:

``` text
world
 ├── robot1_base_link -> robot1 arm
 ├── robot2_base_link -> robot2 arm
 ├── table
 └── mounting_surface
```

### 4.2 Robot placement

The supplied twin URDF places the robots at:

``` text
Robot 1:
    xyz = (-0.7, 0, 0)
    yaw = 0

Robot 2:
    xyz = (+0.7, 0, 0)
    yaw = π
```

Robot 2 is therefore rotated 180° relative to Robot 1 so the two arms
face each other.

### 4.3 Shared environment geometry

The description contains a shared table and mounting surface, both
attached to `world`.

The current table geometry is:

``` text
2.0 m × 2.0 m × 0.03 m
```

The collision geometry is therefore part of the same MoveIt robot model
used by both arms.

------------------------------------------------------------------------

## 5. Naming and Robot Isolation

Every arm entity is prefixed.

### Robot 1

``` text
robot1_Joint_1 ... robot1_Joint_6
robot1_base_link
robot1_Link_1 ... robot1_Link_6
robot1_ee_link
robot1_tool0
robot1_tcp
```

### Robot 2

``` text
robot2_Joint_1 ... robot2_Joint_6
robot2_base_link
robot2_Link_1 ... robot2_Link_6
robot2_ee_link
robot2_tool0
robot2_tcp
```

This prefixing is fundamental. It allows one `/joint_states` stream and
one combined MoveIt model while still allowing each runtime context to
extract only the state belonging to its robot.

------------------------------------------------------------------------

## 6. MoveIt SRDF

The SRDF defines three planning groups.

### 6.1 `robot1_arm`

``` text
robot1_base_link -> robot1_ee_link
```

This is an independent 6-DOF chain.

### 6.2 `robot2_arm`

``` text
robot2_base_link -> robot2_ee_link
```

This is also an independent 6-DOF chain.

### 6.3 `dual_arms`

The composite group contains:

``` text
robot1_arm
robot2_arm
```

The composite group is useful for:

-   combined robot state;
-   collision checking;
-   future coordinated-planning experiments.

It is **not** configured as one 12-DOF TRAC-IK chain.

------------------------------------------------------------------------

## 7. Named Home State

Both robot groups use the same local joint-space home configuration:

``` text
Joint 1 = -1.57 rad
Joint 2 = -0.35 rad
Joint 3 = -0.70 rad
Joint 4 = -0.52 rad
Joint 5 = -1.57 rad
Joint 6 =  0.00 rad
```

The SRDF also defines a `home` state for `dual_arms` containing all 12
joint values.

------------------------------------------------------------------------

## 8. Kinematics

Both independent groups use TRAC-IK:

``` yaml
kinematics_solver: trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin
kinematics_solver_search_resolution: 0.002
kinematics_solver_timeout: 0.02
solve_type: Distance
kinematics_solver_attempts: 3
```

There is intentionally no 12-DOF TRAC-IK solver for `dual_arms`.

This matches the runtime architecture: each arm is normally planned
independently.

------------------------------------------------------------------------

## 9. Collision Model

The SRDF contains self-collision exclusions separately for `robot1_*`
and `robot2_*` links.

Adjacent links and selected link pairs that are known never to collide
are disabled from self-collision checking.

Because both robots exist in the same MoveIt model, inter-robot geometry
can remain visible to the planning scene rather than pretending each arm
exists in a separate world.

When modifying the SRDF, do not accidentally create broad
robot1-vs-robot2 collision exclusions merely to make a path plan.
Inter-arm collision awareness is important for real coordinated
operation.

------------------------------------------------------------------------

## 10. Runtime Profile

The twin runtime profile points to:

``` yaml
URDF_PATH: urdfs/twin_erob.urdf.xacro
SRDF_PATH: eRobo3.srdf
NUM_JOINTS: 12
ETHERCAT_EXPECTED_SLAVES: 12
```

It spawns:

``` yaml
CONTROLLERS_TO_SPAWN:
  - robot1_joint_trajectory_controller
  - robot2_joint_trajectory_controller
```

Servo is disabled:

``` yaml
SERVO_ENABLED: false
```

### 10.1 Primary robot

The profile currently declares:

``` yaml
PRIMARY_ROBOT: robot1
```

This supports migration of older code that still expects one flat set of
runtime keys.

### 10.2 Explicit robot configuration

`ROBOTS` contains per-robot values.

Robot 1:

``` text
planning group:      robot1_arm
base:                robot1_base_link
end effector:        robot1_ee_link
wrist:               robot1_Link_6
Cartesian source:    robot1_ee_link
collision tip:       robot1_tool0
trajectory action:   /robot1_joint_trajectory_controller/follow_joint_trajectory
```

Robot 2:

``` text
planning group:      robot2_arm
base:                robot2_base_link
end effector:        robot2_ee_link
wrist:               robot2_Link_6
Cartesian source:    robot2_ee_link
collision tip:       robot2_tool0
trajectory action:   /robot2_joint_trajectory_controller/follow_joint_trajectory
```

### 10.3 Legacy compatibility keys

The flat keys:

``` text
JOINT_NAMES
PLANNING_GROUP
BASE_LINK
EE_LINK
WRIST_LINK
CARTESIAN_SOURCE_LINK
COLLISION_TIP_LINK
ACTION_FOLLOW_TRAJECTORY
```

currently map to Robot 1.

Do not remove these until all legacy single-robot runtime components
have been migrated to explicit `RobotRuntimeContext` usage.

------------------------------------------------------------------------

## 11. ros2_control Architecture

The twin ros2_control description intentionally places both arms in
**one ros2_control System**.

### Fake hardware

The hardware plugin is:

``` text
mock_components/GenericSystem
```

Each joint exposes the fake-hardware command/state interfaces required
by the test system.

### Real hardware

The real architecture is:

``` text
controller_manager
       |
combined ros2_control System
       |
EthercatDriver
       |
IgH master 0
       |
slaves 0 ... 11
```

The supplied configuration uses:

``` text
master_id:              0
control_frequency:      1000
freeze_on_slave_fault:  true
timeout:                1000000000
```

### EtherCAT mapping

``` text
Robot 1:
  Joint 1 -> slave 0
  Joint 2 -> slave 1
  Joint 3 -> slave 2
  Joint 4 -> slave 3
  Joint 5 -> slave 4
  Joint 6 -> slave 5

Robot 2:
  Joint 1 -> slave 6
  Joint 2 -> slave 7
  Joint 3 -> slave 8
  Joint 4 -> slave 9
  Joint 5 -> slave 10
  Joint 6 -> slave 11
```

There is deliberately only one EtherCAT driver instance.

------------------------------------------------------------------------

## 12. ros2_control Controllers

The controller manager update rate is:

``` text
100 Hz
```

Controllers:

``` text
joint_state_broadcaster
robot1_joint_trajectory_controller
robot2_joint_trajectory_controller
```

Each trajectory controller commands only its own six prefixed joints
using the `position` command interface and reads position/velocity state
interfaces.

Expected active-controller output resembles:

``` text
robot2_joint_trajectory_controller  ... active
joint_state_broadcaster             ... active
robot1_joint_trajectory_controller  ... active
```

------------------------------------------------------------------------

## 13. MoveIt Controller Mapping

MoveIt is configured with two `FollowJointTrajectory` controllers.

### Robot 1

``` text
robot1_joint_trajectory_controller
action_ns: follow_joint_trajectory
joints: robot1_Joint_1 ... robot1_Joint_6
```

### Robot 2

``` text
robot2_joint_trajectory_controller
action_ns: follow_joint_trajectory
joints: robot2_Joint_1 ... robot2_Joint_6
```

Robot 1 is currently marked as the default controller in the supplied
MoveIt controller configuration; Robot 2 is not.

The runtime's explicit per-robot action path is what prevents a Robot 2
trajectory from being submitted to Robot 1's controller.

------------------------------------------------------------------------

## 14. `TwinLocalRuntime`

`TwinLocalRuntime` owns two normal robot runtimes in one Python process.

For each robot it constructs:

``` text
RobotRuntimeContext
RobotScopedStatePublisher
RobotController
backend
LocalRuntimeGateway
```

The runtime uses a `MultiThreadedExecutor`, with at least two executor
threads.

Conceptually:

``` python
with TwinLocalRuntime() as robots:
    r1 = robots.robot1
    r2 = robots.robot2
```

or:

``` python
r1 = robots.robot("robot1")
r2 = robots.robot("robot2")
```

The `gateways` property exposes a copy of the robot-name → gateway map.

------------------------------------------------------------------------

## 15. Per-Robot ROS Nodes and State

For each robot the runtime creates a scoped state publisher and
controller node.

Conceptually:

``` text
robot1_state_publisher
robot1_runtime

robot2_state_publisher
robot2_runtime
```

Each robot receives a state topic prefix:

``` text
/robot1
/robot2
```

and its own active TCP frame:

``` text
robot1_active_tcp
robot2_active_tcp
```

This allows both runtime instances to consume the combined system state
while publishing/operating in an explicitly scoped way.

------------------------------------------------------------------------

## 16. Runtime Readiness

`TwinLocalRuntime.wait_until_ready(timeout_s=15.0)` starts the executor
and waits until **all** gateways report:

``` python
gateway.is_motion_stack_ready()
```

The diagnostic method:

``` python
robots.readiness()
```

returns per-robot readiness and fault information.

Example shape:

``` python
{
    "robot1": {"ready": True, "fault": None},
    "robot2": {"ready": True, "fault": None},
}
```

Do not begin synchronized preparation/execution until both runtimes are
ready.

------------------------------------------------------------------------

## 17. Independent Motion

Independent robot motion should continue to use the normal gateway APIs.

The important rule is:

``` text
TwinLocalRuntime
     |
select robot
     |
LocalRuntimeGateway
     |
normal single-robot motion stack
```

There should not be separate `robot1_move_lin()` and `robot2_move_lin()`
implementations inside the generic planner.

For example, conceptually:

``` python
robots.robot1.move_lin(...)
robots.robot2.move_lin(...)
```

The exact gateway method should follow the current API.

------------------------------------------------------------------------

## 18. Plan-Now / Execute-Later

The choreography/synchronization foundation introduces a separation
between:

``` text
planning
```

and:

``` text
controller execution
```

The normal flow is:

``` text
Cartesian path request
       |
       v
prepare_path()
       |
       +-- IK / Cartesian planning
       +-- validation
       +-- optimization
       +-- trajectory capture
       |
       v
PreparedTrajectory
       |
       |   robot has NOT moved
       |
       v
execute_prepared()
       |
       v
controller goal
       |
       v
motion
```

This is necessary for synchronized multi-robot starts because both
trajectories must exist before either robot is allowed to move.

------------------------------------------------------------------------

## 19. `PreparedTrajectory`

A prepared trajectory is the reusable result of the planning phase.

Important concepts include:

-   trajectory points;
-   point count;
-   duration;
-   cached start joint positions;
-   cached end joint positions;
-   no-op state;
-   metadata;
-   start policy;
-   cyclic/closure metadata where applicable.

A `PreparedTrajectory` is not itself a controller goal.

Controller-specific final preparation still occurs before dispatch.

------------------------------------------------------------------------

## 20. `prepare_pair()`

`TwinLocalRuntime.prepare_pair()` accepts one `prepare_path()` kwargs
dictionary per robot:

``` python
prepared = robots.prepare_pair(
    robot1={...},
    robot2={...},
)
```

The method:

1.  starts the runtime;
2.  launches one preparation thread per robot;
3.  calls each robot gateway's existing `prepare_path()`;
4.  joins both preparation threads;
5.  fails if either preparation raises;
6.  fails if either returns a negative result code;
7.  accepts valid no-op preparations;
8.  returns the prepared trajectories.

Result:

``` python
{
    "robot1": PreparedTrajectory(...),
    "robot2": PreparedTrajectory(...),
}
```

### Critical guarantee

`prepare_pair()` is a **planning-only operation**.

It must not contact the trajectory controllers and must not move either
robot.

------------------------------------------------------------------------

## 21. Why Pair Preparation Is Concurrent

Independent planning can be expensive.

Serial planning would unnecessarily add Robot 1's planning latency to
Robot 2's planning latency.

Concurrent preparation also better matches the conceptual model:

``` text
          +--> prepare robot1 --+
START ----|                     |---- both ready
          +--> prepare robot2 --+
```

However, concurrent preparation is not what synchronizes the physical
start. Physical synchronization is provided later by the common
controller timestamp.

------------------------------------------------------------------------

## 22. Prepared Start Policies

Two policies are supported.

### 22.1 `live_anchor`

This is the compatibility/default behavior.

At execution time the first controller point is re-anchored to the
robot's current live joint state.

Use this when:

-   the prepared path may be executed from a slightly changed live
    state;
-   preserving the exact cached first point is not required;
-   compatibility with existing single-robot behavior is desired.

Important consequence:

> The cached first trajectory point is not necessarily the exact first
> point that executes.

### 22.2 `require_exact`

This is the important policy for reusable choreography.

The cached trajectory is preserved and the live robot state must match
the prepared first joint point within the configured tolerance.

If the difference exceeds:

``` text
EXECUTOR_PREPARED_START_TOL_RAD
```

the operation fails with:

``` text
MOTION_ERROR_PREPARED_START_MISMATCH = -15
```

The verified tolerance was:

``` text
0.01 rad
```

### Critical safety/semantic guarantee

A `require_exact` mismatch is a **hard pre-dispatch rejection**.

The rejected prepared trajectory must:

-   not become a dispatchable controller goal;
-   not call `send_goal_async()`;
-   not emit `goal_send`;
-   not move the robot.

This behavior was explicitly regression-tested after an earlier defect
was discovered and corrected.

------------------------------------------------------------------------

## 23. Why `require_exact` Matters for Choreography

A Cartesian pose can have more than one valid IK solution.

If a choreography is planned once and then replayed repeatedly,
returning only to the same Cartesian pose is not enough.

The robot must return to the **same intended joint-space start branch**.

Otherwise:

``` text
same Cartesian start pose
        |
        +--> IK branch A   <- trajectory was prepared here
        |
        +--> IK branch B   <- robot actually returned here
```

Blindly replaying the cached joint trajectory from branch B would be
incorrect and potentially unsafe.

For plan-once/replay choreography, the intended loop is therefore:

``` text
return to known choreography start
        |
verify exact joint-space branch
        |
require_exact passes
        |
execute cached trajectory
```

------------------------------------------------------------------------

## 24. Controller-Goal Preparation Split

The trajectory executor was split so synchronized execution does not
perform expensive serialized work after selecting the shared start
timestamp.

Conceptually there are three stages.

### Phase 1 --- controller goal preparation

Planning output is converted into the final prepared controller goal:

``` text
PreparedTrajectory
       |
start-policy validation
trajectory mutation/ramp handling
tolerances
FollowJointTrajectory goal construction
       |
PreparedControllerGoal
```

No timing-critical send occurs here.

### Phase 2 --- `ready_for_dispatch()`

Per-robot execution gates are prepared:

-   drive check;
-   execution lock acquisition;
-   action server readiness;
-   active execution state setup;
-   goal sequence assignment.

Still no action goal is sent and no shared start stamp is assigned.

### Phase 3 --- `send_prepared_goal()`

This is intentionally small:

``` text
assign shared header.stamp
send_goal_async()
register callbacks
```

The compatibility method:

``` text
send_prepared_controller_goal()
```

still wraps readiness + send for ordinary standalone execution.

------------------------------------------------------------------------

## 25. Synchronized Pair Execution

`TwinLocalRuntime.execute_prepared_pair()` routes the two trajectories
through the generic `SynchronizedTrajectoryExecutor`.

Conceptual call:

``` python
results = robots.execute_prepared_pair(
    prepared["robot1"],
    prepared["robot2"],
    blocking=True,
    start_policy="require_exact",
)
```

It can also accept an explicit `start_time` or an `offset_s`.

The method refuses coordinated execution if either robot is busy.

Synchronized pair execution is deliberately **not queued**. A
synchronized operation needs a controlled common start and therefore
cannot simply be inserted independently into two ordinary motion queues.

------------------------------------------------------------------------

## 26. Correct Synchronized Execution Ordering

The verified architecture is:

``` text
PreparedTrajectory R1        PreparedTrajectory R2
        |                            |
        +----------+  +--------------+
                   |  |
                   v  v
          prepare controller goals
                   |
          ALL goals must succeed
                   |
                   v
          ready execution gates
                   |
          ALL gates must succeed
                   |
                   v
       compute common future stamp
                   |
          +--------+--------+
          |                 |
          v                 v
      dispatch R1       dispatch R2
          |                 |
          +--------+--------+
                   |
          acceptance barrier
                   |
       both accepted in time
                   |
                   v
          common start stamp
                   |
             +-----+-----+
             |           |
             v           v
          Robot 1     Robot 2
```

The common start stamp must be selected **after** all expensive
controller-goal preparation and readiness work has completed.

------------------------------------------------------------------------

## 27. Shared Future Start Timestamp

Both `FollowJointTrajectory` goals receive the same future ROS
`header.stamp`.

That timestamp, not Python thread scheduling, is the actual
synchronization mechanism.

The synchronized executor uses either:

-   an explicitly supplied `start_time`; or
-   current shared ROS clock + supplied `offset_s`; or
-   the generic configured synchronized-execution start delay.

Because both controllers operate on the same machine/shared clock in the
current architecture, they can interpret the common timestamp
consistently.

------------------------------------------------------------------------

## 28. Concurrent Dispatch

Once both goals are completely prepared and both execution gates are
ready, the synchronized executor dispatches the goals concurrently.

This avoids the earlier failure mode:

``` text
send robot1
wait/work
send robot2
```

which consumed the future-start window.

The verified test measured approximately:

``` text
dispatch separation ≈ 0.522 ms
```

This is sufficiently small relative to the future timestamp because both
controllers independently wait for the same start stamp.

------------------------------------------------------------------------

## 29. Acceptance Barrier

Dispatch is not considered successful merely because `send_goal_async()`
was called.

Both goals must be accepted.

The synchronization executor tracks acceptance per entity and enforces
an acceptance barrier before the shared start.

Conceptually:

``` text
dispatch R1 ---> accepted? --+
                             +--> barrier success
dispatch R2 ---> accepted? --+
```

If a robot explicitly rejects before synchronized start, already
accepted peers are canceled according to the synchronized failure
policy.

The acceptance logic was made race-resistant so a fast callback cannot
be mistaken for a missing acceptance.

Instrumentation includes:

-   per-robot dispatch → acceptance latency;
-   acceptance margin before common start;
-   acceptance separation;
-   dispatch separation;
-   actual start-offset information.

------------------------------------------------------------------------

## 30. Busy-Robot Protection

Before coordinated execution, `TwinLocalRuntime` checks each gateway.

A robot is considered non-idle if:

-   its node reports `is_executing`; or
-   its motion queue reports queued work.

If either robot is busy:

``` text
execute_prepared_pair refused
```

This is intentional.

Do not silently queue synchronized choreography behind independent work.

------------------------------------------------------------------------

## 31. Pair Stop

`TwinLocalRuntime.stop_pair()` calls each gateway's existing
`stop_motion()` independently and returns a per-robot result dictionary.

Conceptually:

``` python
results = robots.stop_pair()
```

This is best-effort pair stopping. It does not replace the
controller-level synchronized acceptance/cancellation rules.

------------------------------------------------------------------------

## 32. Choreography Design

The intended choreography system should sit **above** the generic twin
runtime.

A choreography should capture user-selected robot poses/waypoints and
movement parameters, then plan the two robot trajectories before
execution.

Recommended conceptual data model:

``` yaml
name: expo_dance_01

start:
  robot1: ...
  robot2: ...

moves:
  - robot1:
      pose: ...
      vel: ...
      acc: ...
    robot2:
      pose: ...
      vel: ...
      acc: ...

  - robot1:
      pose: ...
      vel: ...
      acc: ...
    robot2:
      pose: ...
      vel: ...
      acc: ...

loop: true
```

The exact persistent schema should be defined by the choreography
application rather than baked into `TwinLocalRuntime`.

------------------------------------------------------------------------

## 33. Choreography Authoring Workflow

The planned UI workflow is:

``` text
Jog Robot 1
    |
capture pose
    |
Jog Robot 2
    |
capture pose
    |
set velocity / acceleration
    |
add choreography point
    |
repeat
    |
save choreography
```

The editor should allow:

-   independent Robot 1 jogging;
-   independent Robot 2 jogging;
-   capture of Cartesian poses;
-   ordered choreography points;
-   per-move velocity;
-   per-move acceleration;
-   save/load;
-   explicit start pose;
-   validation;
-   plan both robots;
-   execute only after both are ready.

The concrete choreography application should live in the twin
robot-system/application area of the higher-level platform rather than
polluting generic application code.

------------------------------------------------------------------------

## 34. Plan Once, Replay Many Times

For an expo dance, repeatedly replanning every loop is undesirable.

Preferred flow:

``` text
load choreography
       |
move both robots to defined start
       |
verify correct joint branches
       |
prepare robot1 choreography
prepare robot2 choreography
       |
both plans valid
       |
cache PreparedTrajectory pair
       |
execute synchronized
       |
return to exact start branch
       |
require_exact validation
       |
execute cached pair again
       |
repeat
```

This gives predictable repeatability and avoids planning jitter between
loops.

------------------------------------------------------------------------

## 35. Loop Closure

A reusable choreography should be designed as a closed or explicitly
resettable sequence.

Two useful approaches are:

### Closed choreography

The final trajectory returns naturally to the exact joint-space start
state.

``` text
START -> ... choreography ... -> START
```

This is ideal for seamless looping.

### Explicit return-to-start

The dance ends elsewhere, then a separately planned reset trajectory
returns both robots to the known choreography start.

Only after that reset completes and `require_exact` verifies the branch
should the cached dance be replayed.

------------------------------------------------------------------------

## 36. Velocity and Acceleration

Velocity and acceleration should be choreography data, not global
hard-coded constants.

Each authored movement should be able to specify its own:

``` text
vel
acc
```

The UI may provide defaults, but the stored choreography should preserve
the values used to create the planned trajectory.

Once a trajectory is prepared, replay should execute that prepared
timing rather than silently changing velocity/acceleration without
replanning.

------------------------------------------------------------------------

## 37. Fake-Hardware Environment

The verified development environment used the ZeroErr config package and
fake hardware.

Typical shell setup:

``` bash
cd ~/fairino_ros2

source /opt/ros/jazzy/setup.bash
source ~/fairino_ros2/install/setup.bash

export EROB_CONFIG_PACKAGE=zeroerr
export ZEROERR_USE_FAKE_HARDWARE=true
export ROS_DOMAIN_ID=42
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_LOCALHOST_ONLY=1
```

Note: `ROS_LOCALHOST_ONLY` is deprecated in Jazzy-era RMW configuration
but was still honored in the verified environment. If it is set, ROS
warns that it takes precedence over `ROS_AUTOMATIC_DISCOVERY_RANGE`.

------------------------------------------------------------------------

## 38. Build

The verified workspace build command used:

``` bash
cd ~/fairino_ros2
./build_zeroerr.sh
```

A successful build should complete both:

``` text
erob_moveit_runtime
zeroerr
```

After building:

``` bash
source ~/fairino_ros2/install/setup.bash
```

------------------------------------------------------------------------

## 39. Controller Verification

After launch:

``` bash
ros2 control list_controllers
```

Expected active controllers:

``` text
robot1_joint_trajectory_controller
robot2_joint_trajectory_controller
joint_state_broadcaster
```

Check combined joint state:

``` bash
ros2 topic echo /joint_states --once
```

Expected joint names:

``` text
robot1_Joint_1 ... robot1_Joint_6
robot2_Joint_1 ... robot2_Joint_6
```

The twin system therefore exposes 12 joint positions in the shared
joint-state message.

------------------------------------------------------------------------

## 40. Basic Twin Runtime Test

The existing twin test supports readiness and isolated jogging.

Example:

``` bash
ros2 run zeroerr twin_test.py --jog \
  --robot robot1 \
  --axis x \
  --step 20
```

It also supports concurrent jog testing.

Use `--help` to confirm the exact options in the installed build:

``` bash
ros2 run zeroerr twin_test.py --help
```

------------------------------------------------------------------------

## 41. Synchronized Choreography Test Script

The synchronized test/choreography companion introduced with the
prepared-execution work is:

``` text
zeroerr/scripts/twin_choreography_synced.py
```

It should be treated as a development/verification companion, not as the
final choreography authoring application.

The production choreography UI should consume the generic runtime API
rather than duplicate synchronization logic.

------------------------------------------------------------------------

## 42. Verified Regression Sequence

The synchronized architecture was developed and verified incrementally.

### Stage 1 --- executor split

Existing trajectory submission was separated into preparation and
sending while retaining the compatibility wrapper for existing callers.

Goal:

> Existing single-robot motion paths must continue to work unchanged.

### Stage 2 --- single prepared execution

Verified:

``` text
prepare_path()
```

returns a prepared trajectory without moving the robot.

Then:

``` text
execute_prepared()
```

executes it and reaches the prepared endpoint.

### Stage 2b --- `require_exact`

Two cases were verified.

Success:

``` text
live start == cached start
-> execute
-> success
```

Mismatch:

``` text
live start != cached start beyond tolerance
-> return -15
-> no controller dispatch
-> no robot movement
```

An early implementation detected `-15` but still sent the goal; this was
corrected so mismatch rejection now occurs before dispatch.

### Stage 3 --- synchronized pair execution

Verified:

-   concurrent preparation;
-   no motion during preparation;
-   exact cached starts;
-   final controller-goal preparation;
-   readiness gates;
-   common future timestamp;
-   concurrent dispatch;
-   acceptance barrier;
-   successful execution;
-   exact endpoints.

------------------------------------------------------------------------

## 43. Important Bugs Already Found and Fixed

### 43.1 Missing config import in twin runtime

An earlier `TwinLocalRuntime` revision referenced `config` without
importing it, causing:

``` text
NameError: name 'config' is not defined
```

The current runtime imports `config`.

### 43.2 `require_exact` detected mismatch but still dispatched

An earlier implementation returned `-15` but allowed a valid controller
goal to continue to dispatch.

Required behavior was corrected to:

``` text
mismatch
  |
  v
reject before dispatch
  |
  +-- no controller server send
  +-- no goal_send
  +-- no goal_accepted
  +-- no motion
```

### 43.3 Common timestamp selected too early

An earlier synchronized executor selected the common future timestamp
before expensive controller-goal preparation.

Preparation consumed the start window and both controllers rejected
stale goals.

Fixed ordering:

``` text
prepare all final goals
-> ready all dispatch gates
-> THEN compute common stamp
```

### 43.4 Serialized dispatch

An earlier version performed drive/server/send work serially per robot
after the common stamp was chosen.

This delayed Robot 2 until after the shared start.

The executor was split into readiness and timing-critical send phases,
then dispatch was made concurrent.

The resulting verified dispatch separation was approximately 0.522 ms.

### 43.5 Acceptance-barrier race/diagnostics

Earlier bookkeeping could report an acceptance deadline even after
goal-accepted callbacks had occurred.

Acceptance tracking was made authoritative/race-resistant and given a
bounded grace re-check.

------------------------------------------------------------------------

## 44. Generic vs Twin-Specific Responsibilities

### Generic motion stack owns

-   planning;
-   Cartesian path generation;
-   IK;
-   validation;
-   optimization;
-   trajectory representation;
-   prepared trajectory support;
-   controller-goal construction;
-   start policies;
-   execution locks;
-   action-client interaction;
-   generic synchronized N-entity execution.

### Twin layer owns

-   constructing exactly two scoped runtimes;
-   selecting Robot 1 vs Robot 2;
-   pair preparation;
-   pair idle checks;
-   invoking generic synchronized execution;
-   pair stop;
-   choreography-oriented composition.

### Choreography application should own

-   pose capture;
-   choreography data;
-   editing;
-   save/load;
-   velocity/acceleration authoring;
-   start/reset workflow;
-   loop logic;
-   user-facing plan/start/stop controls.

Do not move application/UI concerns into `TrajectoryExecutor`.

------------------------------------------------------------------------

## 45. Rules for Future Development

1.  **Do not fork the generic planner per robot.**

2.  **Do not add `if robot1` / `if robot2` logic deep inside generic
    planning and execution when a `RobotRuntimeContext` can provide the
    scoped value.**

3.  **Keep synchronized execution generic.**\
    `SynchronizedTrajectoryExecutor` should remain capable of
    coordinating N prepared entities rather than becoming a twin-only
    implementation.

4.  **Never dispatch one side if the other side failed preparation.**

5.  **Never silently queue a synchronized pair operation.**

6.  **For cached choreography replay, use joint-space start
    validation.**

7.  **Do not treat matching Cartesian pose as proof of matching IK
    branch.**

8.  **Do not select the common future timestamp before all expensive
    preparation/readiness work is complete.**

9.  **Do not rely on Python thread dispatch timing as the physical
    synchronization mechanism.**\
    The shared controller timestamp is authoritative.

10. **Do not weaken `require_exact` to make choreography easier to
    replay.**\
    Correct the return-to-start process instead.

11. **Keep old single-robot APIs backward compatible.**

12. **Regression-test paint and other single-robot profiles after
    generic executor changes.**

------------------------------------------------------------------------

## 46. Real-Hardware Considerations

Fake-hardware success does not by itself prove safe real-hardware
choreography.

Before real dual-arm execution, verify:

-   correct EtherCAT slave order 0--11;
-   correct drive-to-joint mapping;
-   both trajectory controllers;
-   hardware enable/fault handling;
-   common controller clock behavior;
-   inter-arm collision checking;
-   table/environment collision model;
-   emergency stop;
-   pair stop;
-   safe start pose;
-   low-speed first execution;
-   joint limits;
-   cable routing;
-   physical robot placement matches URDF transforms.

Start with low velocity/acceleration and large physical separation.

------------------------------------------------------------------------

## 47. EtherCAT Topology

The intended real-hardware topology is:

``` text
IgH EtherCAT master 0
 |
 +-- slave 0  robot1 Joint 1
 +-- slave 1  robot1 Joint 2
 +-- slave 2  robot1 Joint 3
 +-- slave 3  robot1 Joint 4
 +-- slave 4  robot1 Joint 5
 +-- slave 5  robot1 Joint 6
 |
 +-- slave 6  robot2 Joint 1
 +-- slave 7  robot2 Joint 2
 +-- slave 8  robot2 Joint 3
 +-- slave 9  robot2 Joint 4
 +-- slave 10 robot2 Joint 5
 +-- slave 11 robot2 Joint 6
```

The runtime profile expects 12 slaves.

A topology mismatch must be treated as a hardware configuration fault,
not worked around by changing robot prefixes.

------------------------------------------------------------------------

## 48. Troubleshooting

### `ros2: command not found`

Source ROS:

``` bash
source /opt/ros/jazzy/setup.bash
source ~/fairino_ros2/install/setup.bash
```

### Twin runtime `NameError: config`

Ensure the current `twin_local.py` contains:

``` python
import config
```

and rebuild/source the workspace.

### Controller not available

Check:

``` bash
ros2 control list_controllers
```

Both trajectory controllers must be active.

### Missing Robot 2 state

Check:

``` bash
ros2 topic echo /joint_states --once
```

All six `robot2_Joint_*` names must be present.

### Cartesian path fraction below 1.0

Do not immediately blame synchronization.

A preparation failure can be:

-   IK/reachability;
-   collision;
-   invalid path;
-   bad current state;
-   workspace boundary.

Synchronization is only entered after both trajectories prepare
successfully.

### Both goals rejected around common start

Inspect ordering and timestamps.

The required ordering is:

``` text
all final goals prepared
all dispatch gates ready
common stamp chosen
both goals dispatched
both accepted
shared start
```

Do not hide serialized preparation by simply making the start offset
enormous.

### `require_exact` returns `-15`

The robot is not at the cached prepared start joint state within
tolerance.

Do not bypass it.

Return the robot to the correct start branch, then retry.

### Observed motion-start skew looks tens of milliseconds

Check state publication frequency before modifying synchronization.

A 10 Hz observer can easily report tens of milliseconds of apparent skew
even when controller dispatch is sub-millisecond and both controllers
use the same start stamp.

------------------------------------------------------------------------

## 49. Known/Tracked Cleanup

Shutdown has produced Servo-related messages such as destruction being
requested after successful tests.

These messages were treated separately from the synchronized-execution
correctness work.

Because Servo is disabled for the twin profile, shutdown cleanup should
be investigated independently rather than changing proven
synchronization behavior to suppress unrelated shutdown logging.

------------------------------------------------------------------------

## 50. Git Milestones

### Concurrent twin runtime milestone

``` text
tag: twin-runtime-concurrent-v1
commit at tagging: af626a13e90c2d2920aae26b00db2e1d79b38fb5
```

Meaning:

> Independent concurrent robot execution proven.

### Prepared synchronized execution milestone

``` text
tag: twin-prepared-sync-v1
commit: 9dcedd3c2ea11bae31e73810a24ef5081f681cc4
```

Meaning:

> Plan-now / execute-later and synchronized two-robot execution proven.

Inspect:

``` bash
git show twin-prepared-sync-v1 --stat
```

Create a detached checkout for inspection:

``` bash
git checkout twin-prepared-sync-v1
```

To force the current branch back to that exact milestone, only when
intentionally discarding later work:

``` bash
git reset --hard twin-prepared-sync-v1
```

Use the latter carefully because it discards tracked working-tree
changes.

------------------------------------------------------------------------

## 51. Recommended Next Layer

The generic execution foundation should now be considered stable unless
regression tests expose a defect.

The next implementation layer should be:

``` text
Twin Choreography Domain
        |
        +-- choreography schema
        +-- pose capture
        +-- per-move vel/acc
        +-- start-pose definition
        +-- plan both
        +-- cached PreparedTrajectory pair
        +-- exact branch validation
        +-- loop/reset logic
        |
        v
Twin Choreography Application
        |
        +-- editor/setup UI
        +-- save/load
        +-- dashboard selection
        +-- PLAN
        +-- START
        +-- STOP
```

The higher-level platform can then expose a simple expo dashboard:

``` text
Choreography: [ Expo Dance 01 ▼ ]

Status:
    Robot 1 READY
    Robot 2 READY
    Choreography PLANNED

             [ START ]

             [ STOP ]
```

Planning should complete for both robots before START becomes
executable.

------------------------------------------------------------------------

## 52. Architectural Summary

The twin profile is intentionally not a 12-DOF monolithic robot
controller.

It is:

``` text
one physical/MoveIt world
        +
two independently scoped robot runtimes
        +
one generic motion stack per selected robot
        +
a generic synchronized prepared-execution coordinator
```

The key choreography invariant is:

> **Both trajectories must be successfully prepared before either robot
> is dispatched, and cached trajectories must only be replayed from
> their intended joint-space starting state.**

The key synchronization invariant is:

> **Both final controller goals are completely prepared and
> dispatch-ready before one common future ROS start timestamp is
> selected; both are then dispatched concurrently and must pass the
> acceptance barrier.**

That design preserves the existing single-robot architecture while
adding coordinated twin behavior as a composition layer above it.

------------------------------------------------------------------------

## 53. Quick Reference

### Build

``` bash
cd ~/fairino_ros2
./build_zeroerr.sh
source /opt/ros/jazzy/setup.bash
source ~/fairino_ros2/install/setup.bash
```

### Fake-hardware environment

``` bash
export EROB_CONFIG_PACKAGE=zeroerr
export ZEROERR_USE_FAKE_HARDWARE=true
export ROS_DOMAIN_ID=42
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_LOCALHOST_ONLY=1
```

### Check controllers

``` bash
ros2 control list_controllers
```

### Check state

``` bash
ros2 topic echo /joint_states --once
```

### Twin test

``` bash
ros2 run zeroerr twin_test.py --help
```

### Proven checkpoint

``` bash
git show twin-prepared-sync-v1 --stat
```

------------------------------------------------------------------------

## 54. Source Files Covered by This Document

This document was built from the supplied current twin-profile files and
the verified development history:

``` text
runtime_gateway/twin_local.py
twin_erob.urdf.xacro
eRobo3.srdf
eRobo3.ros2_control.xacro
initial_positions.yaml
joint_limits.yaml
kinematics.yaml
moveit_controllers.yaml
ros2_controllers.yaml
runtime.yaml
```

Where implementation details outside those supplied files are
described---especially the prepared/synchronized executor
internals---the descriptions reflect the behavior and verification
results established during development up to `twin-prepared-sync-v1`.

------------------------------------------------------------------------

**Document status:** Complete architecture/operation reference for the
current proven twin profile foundation.\
**Next documentation update:** after the persistent choreography schema
and platform choreography application are implemented.
