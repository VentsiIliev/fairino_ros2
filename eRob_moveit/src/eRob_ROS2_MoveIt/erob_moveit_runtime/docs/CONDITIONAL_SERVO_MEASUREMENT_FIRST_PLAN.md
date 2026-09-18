# Conditional Servo: Measurement-First Optimization Plan

**Status:** Pre-implementation measurement plan  
**Scope:** Current `ServoUntilConditionProcedure` and MoveIt Servo implementation  
**Principle:** Do not replace the current motion primitive until its latency, stop semantics, CPU cost, and retract gate have been measured on the real controller.

## 1. Objective

Measure the current condition-triggered Servo path before deciding whether to:

1. move condition and motion-boundary monitoring from Platform into ROS 2;
2. replace HTTP sensor polling with a persistent WebSocket sensor stream;
3. give the active Servo loop a bounded `SCHED_FIFO` priority below the controller;
4. retain MoveIt Servo or introduce a different interruptible motion primitive.

The first optimization candidate is deliberately smaller than an interruptible LIN implementation:

```text
Platform:
    owns PLC/Modbus and abstract sensor state
    sends sensor transitions over a persistent stream

ROS 2:
    owns one conditional Servo operation
    monitors fresh robot state and generic motion limits
    publishes zero, pauses Servo, confirms stop, and returns the result
```

No custom Cartesian path generator, sequential IK, trajectory replacement, adaptive braking, or dynamic collision-matrix implementation is authorized by this plan.

## 2. Desired measurements at a glance

The measurement phase must produce the following values for every trial.

### 2.1 Primary requested measurements

| Measurement | Start event | End event | Purpose |
|---|---|---|---|
| **Stop request transport time** | Platform begins sending the stop HTTP request (`T01`) | ROS REST route receives the request (`T02`) | Quantifies Platform-to-ROS HTTP and scheduling overhead. |
| **ROS stop-to-physical-stop time** | ROS REST route receives the stop request (`T02`) | Fresh measured state confirms that the robot is stopped (`T13`) | Measures the complete ROS/controller/mechanical stopping response. |
| **ROS stop-to-retract-allowed time** | ROS REST route receives the stop request (`T02`) | Platform declares retract eligible (`T14`) | Shows when the current implementation permits the next motion. |

Exact calculations:

```text
stop_request_sent_to_received_ms
    = (T02 - T01) / 1_000_000

stop_request_received_to_robot_stopped_ms
    = (T13 - T02) / 1_000_000

stop_request_received_to_retract_allowed_ms
    = (T14 - T02) / 1_000_000
```

### 2.2 Required stop/retract ordering measurements

These values determine whether the current handoff is safe and whether the retract gate belongs in ROS 2:

```text
robot_stopped_to_retract_allowed_ms
    = (T14 - T13) / 1_000_000

robot_stopped_to_retract_request_sent_ms
    = (T15 - T13) / 1_000_000

robot_stopped_to_retract_request_received_ms
    = (T16 - T13) / 1_000_000

robot_stopped_to_retract_accepted_ms
    = (T17 - T13) / 1_000_000
```

A negative value means the corresponding retract event occurred before measured stop confirmation. Report negative values as ordering violations; do not clamp them to zero.

### 2.3 Supporting measurements

The primary values cannot identify the source of delay without these supporting measurements:

| Measurement | Calculation | Question answered |
|---|---|---|
| Condition observation → stop send | `T01 - T00` | How much time Platform consumes after observing the condition. |
| Stop HTTP round trip | `T12 - T01` | How long Platform blocks on the current stop call. |
| ROS dispatch | `T05 - T02` | How much delay exists before Cartesian Servo begins stopping. |
| Request received → zero published | `T06 - T02` | How quickly the first zero command is emitted. |
| Pause-service dispatch | `T08 - T07` | ROS service/executor scheduling delay. |
| Pause callback → hold published | `T09 - T08` | C++ Servo stop/hold construction time. |
| Hold published → measured stop | `T13 - T09` | Controller plus physical settling time. |
| Stop response → retract allowed | `T14 - T12` | Platform work between stop completion and retract eligibility. |
| Retract send → ROS receive | `T16 - T15` | Retract HTTP transport delay. |
| Retract receive → accepted | `T17 - T16` | ROS planning/dispatch/controller acceptance delay. |

### 2.4 Context recorded with every measurement

Every result must include enough context to compare trials:

- `trace_id` and trial number;
- Servo command speed and direction;
- command and monitored reference frame;
- tool and user indices;
- Servo command publication rate;
- condition polling interval;
- retract motion type (`servo`, `fast_lin`, or `ptp`);
- whether preplanning was active;
- relevant process/thread CPU load;
- ROS middleware and kernel scheduling class;
- stop reason and whether the trial succeeded;
- trigger TCP pose, stopped TCP pose, and post-trigger travel distance;
- peak measured joint/TCP velocity before the stop request;
- joint-state sampling rate and maximum observed state age.

## 3. Current path being measured

The relevant current flow is:

```text
Platform condition poll detects ACTIVE
    -> ServoUntilConditionProcedure._stop_servo()
    -> HTTP POST /servojog/stop
    -> ROS REST route
    -> runtime_api.servo_jog_stop()
    -> backend.stop_servo_jog()
    -> ServoJog.stop_continuous_jog()
    -> CartesianServo.stop()
    -> publish zero / dwell / stop publisher
    -> SetBool /servo_node/pause_servo
    -> ZeroErr Servo publishes measured-state hold and disables processing
    -> HTTP response returns to Platform
    -> Platform reads contact pose
    -> Platform enters _retract()
    -> retract command is submitted
```

In the current Platform implementation, successful return from `_stop_servo()` allows `_retract()` to begin. This confirms that the pause request completed, but it is not yet evidence that measured robot velocity was zero. The measurements must establish the ordering of:

```text
pause confirmed
measured robot stopped
retract allowed
retract command received by ROS 2
```

## 4. Clock and correlation requirements

### 4.1 Clock

Use monotonic timestamps for all latency calculations:

- Python: `time.monotonic_ns()`
- C++: `std::chrono::steady_clock`

Platform and ROS 2 currently run on the same Linux host, so their monotonic timestamps share the host monotonic clock domain. Never use formatted wall-clock timestamps to calculate latency.

Every record may also contain wall-clock time for human log navigation, but wall-clock time is diagnostic only.

### 4.2 Trace identity

Assign one `trace_id` to every condition-triggered stop/retract cycle. Recommended format:

```text
servo-stop-<process-instance-id>-<monotonic-sequence>
```

Propagate it from Platform to ROS using an HTTP header such as:

```text
X-Motion-Trace-ID: servo-stop-a82f-000031
```

Include the same ID in structured timing records and, where practical, in the stop response. Do not correlate trials only by nearby wall-clock log lines.

If the existing `SetBool` pause service cannot carry the ID into the C++ node, maintain a ROS-side stop sequence and correlate it with the enclosing runtime timestamps. Do not change the service contract solely for the first measurement phase.

## 5. Required timestamp points

All timestamps are raw monotonic nanoseconds. Names include the component which records them.

| ID | Timestamp | Component | Meaning |
|---|---|---|---|
| T00 | `condition_observed_platform_ns` | Platform | Poll/read first returns the triggering condition. |
| T01 | `stop_http_send_platform_ns` | Platform HTTP adapter | Immediately before submitting `POST /servojog/stop`. |
| T02 | `stop_http_route_enter_ros_ns` | ROS REST server | First instruction inside the stop route. This is “stop request received.” |
| T03 | `stop_handler_enter_ros_ns` | Runtime handler | Entry into `servo_jog_stop()`. |
| T04 | `stop_backend_enter_ros_ns` | Backend/ServoJog | Entry into the local stop operation. |
| T05 | `cartesian_stop_enter_ros_ns` | Cartesian Servo | Entry into `_on_stop()`. |
| T06 | `zero_command_published_ros_ns` | Cartesian Servo | Zero Twist command publication completed. |
| T07 | `pause_request_sent_ros_ns` | Cartesian Servo | `/servo_node/pause_servo` request submitted. |
| T08 | `pause_callback_enter_servo_ns` | C++ Servo node | Pause service callback begins. |
| T09 | `measured_hold_published_servo_ns` | C++ Servo node | Measured-state hold is sent to the controller topic. |
| T10 | `pause_callback_complete_servo_ns` | C++ Servo node | Pause state and collision-monitor transition complete. |
| T11 | `stop_http_response_ros_ns` | ROS REST server | Stop HTTP response is ready to send. |
| T12 | `stop_http_response_platform_ns` | Platform | Platform receives and parses the response. |
| T13 | `measured_stop_confirmed_ros_ns` | ROS state monitor | Robot satisfies the defined stopped criterion. |
| T14 | `retract_allowed_platform_ns` | Platform procedure | Stop succeeded and code crosses into retract eligibility. Record before pose reads or callbacks. |
| T15 | `retract_command_send_platform_ns` | Platform adapter | Immediately before the Servo/PTP/Fast-LIN retract request is submitted. |
| T16 | `retract_command_received_ros_ns` | ROS REST server | First instruction in the selected retract endpoint. |
| T17 | `retract_motion_accepted_ros_ns` | ROS backend/controller interface | Retract is accepted for execution. |

T13 may occur before or after T12/T14. Records must therefore be collected asynchronously and joined by `trace_id` after the trial.

## 6. Primary measurements

### 6.1 Requested measurements

```text
stop transport latency
    = T02 - T01

ROS stop handling until physical stop
    = T13 - T02

stop request received until retract is allowed
    = T14 - T02
```

### 6.2 Additional measurements needed to interpret them

```text
condition polling delay proxy
    = T01 - T00

HTTP round trip
    = T12 - T01

ROS route and dispatch overhead
    = T05 - T02

zero-command latency after request receipt
    = T06 - T02

pause-service dispatch latency
    = T08 - T07

hold construction/publication latency
    = T09 - T08

pause completion latency
    = T10 - T08

physical settling after hold publication
    = T13 - T09

Platform processing before retract eligibility
    = T14 - T12

retract submission delay
    = T15 - T14

retract HTTP transport latency
    = T16 - T15

retract controller acceptance latency
    = T17 - T16
```

### 6.3 Most important safety/order metric

```text
retract eligibility margin
    = T14 - T13
```

Interpretation:

- positive: retract was allowed after measured stop;
- approximately zero: eligibility and measured stop coincide within sampling resolution;
- negative: current code allowed retract before measured stop was confirmed.

Also calculate:

```text
retract request margin  = T15 - T13
retract receive margin  = T16 - T13
retract accept margin   = T17 - T13
```

These distinguish an early Platform eligibility decision from an actually overlapping controller command.

## 7. Definition of “actually stopped”

Do not define physical stop as:

- HTTP response received;
- pause service success;
- zero command published;
- controller goal replaced;
- unchanged pose in one sample.

For the initial measurements, record two stop criteria:

### 7.1 Joint-velocity stop

```text
all(abs(measured_joint_velocity_rad_s[i]) <= threshold[i])
for N consecutive fresh samples
```

Initial diagnostic values, to be validated against encoder noise:

```yaml
joint_velocity_stop_threshold_rad_s: 0.01
stable_stop_sample_count: 3
maximum_joint_state_age_ms: 20
```

### 7.2 TCP-speed stop

Compute TCP linear and angular velocity from measured joint velocity and the Jacobian for the active group/tool:

```text
tcp_linear_speed_mm_s <= configured threshold
tcp_angular_speed_deg_s <= configured threshold
for N consecutive fresh samples
```

Record both criteria. The stricter/later timestamp should be used for `T13` until commissioning establishes the correct production definition.

The stop detector must consume the freshest controller joint-state source directly. It must not depend on the low-rate UI/state-publisher stream.

## 8. Measurement implementation constraints

The measurement patch should change observability only:

- no motion-profile changes;
- no frequency changes;
- no new sleep or dwell;
- no FIFO/affinity changes;
- no WebSocket sensor migration;
- no change to stop/retract ordering;
- no added blocking service calls;
- no formatted logging inside a future real-time critical section.

Prefer fixed-size in-memory trace records and emit them after stop/retract acceptance. If direct logging is initially used, measure its overhead with timing disabled versus enabled.

Structured record example:

```json
{
  "event": "conditional_servo_timing",
  "trace_id": "servo-stop-a82f-000031",
  "timestamp_name": "stop_http_route_enter_ros_ns",
  "monotonic_ns": 123456789012345,
  "trial": {
    "servo_speed_mm_s": 10.0,
    "publish_rate_hz": 50.0,
    "preplanning_active": true
  }
}
```

Record rejected and timeout trials as well; never silently exclude them from latency statistics.

## 9. Test matrix

Run stationary/dummy-condition trials first, followed by conservative real motion.

### 9.1 Load conditions

1. system idle except required robot stack;
2. normal Platform UI active;
3. actual preplanning active;
4. preplanning plus normal diagnostics;
5. remote desktop active, recorded separately rather than mixed silently.

### 9.2 Motion conditions

Start conservatively:

| Case | Servo speed | Trigger phase |
|---|---:|---|
| A | 5 mm/s | steady motion |
| B | 10 mm/s | steady motion |
| C | 20 mm/s | steady motion |
| D | 10 mm/s | shortly after start |
| E | 10 mm/s | near configured motion boundary |

Do not increase speed until stopping distance and drive behavior are repeatable.

### 9.3 Repetition

- at least 30 cycles per initial case;
- 100 cycles for cases used to make an architectural or RT decision;
- preserve every sample, not only aggregates.

For each metric report:

```text
count, failures, minimum, median, p90, p95, p99, maximum
```

Also capture per-thread CPU and scheduling data during each load condition:

```bash
pidstat -u -t -p <servo>,<runtime>,<platform>,<state-publisher>,<controller> 1
mpstat -P ALL 1
```

## 10. Generic ROS-side motion-boundary candidate

The future ROS-side conditional operation should not expose pickup-specific `minimum_z_mm` as its core abstraction.

Use typed termination guards:

```yaml
termination:
  external_condition:
    source: vacuum
    required_transition: inactive_to_active

  workspace_boundary:
    monitored_point:
      type: active_tool_tcp
      tool: 1
    reference_frame:
      type: user
      index: 2
    plane:
      normal: [0.0, 0.0, 1.0]
      offset_mm: 0.0
      allowed_side: positive

  maximum_projected_travel_mm: 60.0
  timeout_s: 5.0
```

The concrete minimum-Z rule is the half-space condition:

```text
[0, 0, 1] dot tcp_position_in_user_frame >= minimum_z
```

At operation arm time, ROS 2 should resolve and freeze:

- active tool/TCP transform;
- user/workobject transform;
- initial TCP pose;
- commanded direction;
- boundary plane;
- maximum projected travel;
- transform/configuration generation.

It should monitor raw joint state locally, perform FK with the selected tool, and evaluate the boundary in the selected fixed reference frame. A tool TCP may be the monitored point, but a moving tool frame should normally not be the boundary reference frame.

## 11. WebSocket candidate after baseline

After the HTTP baseline is measured, introduce a persistent sensor stream without changing motion ownership yet. Required fields:

```json
{
  "type": "sensor_state",
  "sensor": "vacuum",
  "state": "active",
  "stream_id": "6c7f...",
  "sequence": 18422,
  "detected_monotonic_ns": 123456789012345
}
```

Requirements:

- transitions plus a modest heartbeat, not high-rate unchanged-state spam;
- `ACTIVE`, `INACTIVE`, `ERROR`, `STALE`, and `DISCONNECTED` remain distinct;
- fresh `INACTIVE` is required before a new `ACTIVE` can trigger an operation;
- `stream_id + sequence` rejects stale/replayed state;
- first valid trigger is latched;
- disconnect/stale/error causes controlled abort;
- WebSocket compression disabled for these small messages;
- no reconnect, DNS, logging formatting, or allocation-heavy work in the trigger-to-stop path.

Measure the WebSocket path using the same T00/T01-style clock points before removing HTTP polling.

## 12. Real-time scheduling decision gate

`SCHED_FIFO` is a latency/jitter tool, not a CPU-usage optimization. Do not enable it merely because Servo consumes CPU.

Before testing FIFO:

1. prove Servo is paused during preplanning-only periods;
2. measure active and paused Servo thread CPU separately;
3. confirm scheduling is applied inside the actual Servo loop thread;
4. confirm the loop sleeps reliably and detect overruns;
5. keep Servo off the controller/EtherCAT CPU where possible;
6. establish baseline p95/p99/max latency under realistic load.

Candidate priority hierarchy for an experiment:

```text
EtherCAT/control critical loop     SCHED_FIFO 80
ros2_control update loop           SCHED_FIFO 50
conditional Servo critical loop    SCHED_FIFO 30-40
ordinary ROS/Platform/UI work       SCHED_OTHER
```

The exact Servo priority must be selected from runtime measurements. It must remain below the controller and must not starve DDS, network, state-update, or stop-service threads.

## 13. Decision criteria

### Keep current architecture with small optimizations when

- HTTP stop transport is small and low-jitter relative to physical stopping;
- retract is never accepted before measured stop;
- preplanning load does not violate the stop-latency budget;
- Servo CPU returns to low usage whenever paused.

### Move condition/boundary ownership into ROS 2 when

- Platform polling or scheduling materially dominates T00-to-T02;
- preplanning creates unacceptable p95/p99 stop jitter;
- Platform can allow or submit retract before measured stop;
- repeated pose HTTP reads materially affect CPU or boundary latency;
- atomic ownership of condition, boundary, stop, and retract gating simplifies failure handling.

### Test lower-priority FIFO only when

- the critical Servo thread is known and bounded;
- SCHED_OTHER scheduling delay materially contributes to T02-to-T09 jitter;
- CPU busy-loop or lifecycle problems have already been excluded;
- the test includes realistic preplanning, UI, DDS, and EtherCAT load.

### Reconsider interruptible LIN only when

- measured MoveIt Servo stopping distance cannot meet the requirement;
- the failure is intrinsic to Servo/controller handoff rather than HTTP/polling/scheduling;
- a controller-level replacement/braking design has its own safety and commissioning plan.

## 14. Execution phases

1. **Instrumentation review:** agree on timestamp definitions, velocity thresholds, trace format, and trial matrix.
2. **Observability-only patch:** add timestamps and stop detector without changing motion behavior.
3. **Stationary validation:** confirm clock comparability, trace completeness, and instrumentation overhead.
4. **Conservative robot trials:** run the matrix at 5/10/20 mm/s.
5. **Analysis:** calculate distributions and ordering margins, including negative retract margins.
6. **Architecture decision:** select the smallest justified change.
7. **WebSocket experiment:** measure persistent sensor delivery if polling is significant.
8. **ROS-owned conditional Servo prototype:** only if baseline supports it.
9. **FIFO experiment:** only after thread/CPU correctness and latency baseline are established.

## 15. Exit criteria for the measurement phase

The phase is complete when:

- every timestamp T00-T17 that applies to the selected retract type is captured or explicitly marked unavailable;
- the stopped criterion is documented and supported by fresh measured state;
- idle and preplanning-loaded distributions are available;
- stop and retract ordering is proven for at least 100 representative cycles;
- CPU use is attributed by process and thread;
- measurement overhead has been quantified;
- a written decision identifies which optimization is justified by the evidence.
