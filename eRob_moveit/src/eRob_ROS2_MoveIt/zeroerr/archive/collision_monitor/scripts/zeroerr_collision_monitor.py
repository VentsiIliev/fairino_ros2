#!/usr/bin/env python3

from __future__ import annotations

import csv
import json
import sys
from datetime import UTC, datetime
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np
import rclpy
from control_msgs.msg import DynamicJointState, InterfaceValue
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String

JOINT_NAMES = [f"Joint_{index}" for index in range(1, 7)]
INTERFACE_DEFAULTS = {
    "motor_actual_current": None,
    "following_error_actual": None,
    "torque_sensor": None,
    "statusword": None,
    "error_code": None,
    "mode_display": None,
}

PLOT_METRICS = {
    "position": lambda self, slave: slave.get("position"),
    "velocity": lambda self, slave: slave.get("velocity"),
    "following_error_actual": lambda self, slave: slave.get("following_error_actual"),
    "torque_sensor": lambda self, slave: slave.get("torque_sensor"),
    "friction_torque": lambda self, slave: self._friction_torque_by_joint.get(str(slave["joint"])),
    "measured_torque": lambda self, slave: self._measured_torque_by_joint.get(str(slave["joint"])),
    "expected_torque": lambda self, slave: self._expected_torque_by_joint.get(str(slave["joint"])),
    "torque_difference": lambda self, slave: self._torque_difference_by_joint.get(str(slave["joint"])),
    "external_torque": lambda self, slave: self._external_torque_by_joint.get(str(slave["joint"])),
    "contact_active": lambda self, slave: 1.0 if self._contact_active[str(slave["joint"])] else 0.0,
    "contact_latched": lambda self, slave: 1.0 if self._contact_latched[str(slave["joint"])] else 0.0,
    "dynamics_active": lambda self, slave: 1.0 if self._dynamics_active[str(slave["joint"])] else 0.0,
    "dynamics_latched": lambda self, slave: 1.0 if self._dynamics_latched[str(slave["joint"])] else 0.0,
}

DEFAULT_EFFORT_THRESHOLDS = [20.0, 20.0, 15.0, 8.0, 6.0, 4.0]
DEFAULT_FOLLOWING_ERROR_THRESHOLDS = [4000.0, 4000.0, 3500.0, 2000.0, 1500.0, 1200.0]
DEFAULT_COLLISION_CONFIG_PATH = str(
    Path(__file__).resolve().parents[1] / "config" / "collision_monitor_config.json"
)
DEFAULT_TORQUE_MODEL_PATH = str(
    Path(__file__).resolve().parents[1] / "config" / "torque_sensor_model.json"
)


class ZeroErrCollisionMonitor(Node):
    def __init__(self) -> None:
        super().__init__("zeroerr_collision_monitor")

        self.declare_parameter("slave_count", 6)
        self.declare_parameter("poll_period_sec", 2.0)
        self.declare_parameter("input_sample_period_sec", 0.0)
        self.declare_parameter("print_table", False)
        self.declare_parameter("confirm_cycles", 3)
        self.declare_parameter("effort_thresholds", DEFAULT_EFFORT_THRESHOLDS)
        self.declare_parameter(
            "following_error_thresholds",
            DEFAULT_FOLLOWING_ERROR_THRESHOLDS,
        )
        self.declare_parameter("use_inverse_dynamics", False)
        self.declare_parameter("dynamics_estimator_mode", "momentum_observer")
        self.declare_parameter("friction_coulomb_nm", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("friction_viscous_nm_per_rad_s", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("friction_velocity_deadband_rad_s", 0.01)
        self.declare_parameter("urdf_path", "")
        self.declare_parameter("base_link", "base_link")
        self.declare_parameter("tip_link", "tool0")
        self.declare_parameter("num_joints", 6)
        self.declare_parameter("external_torque_thresholds", [12.0, 12.0, 10.0, 8.0, 6.0, 5.0])
        self.declare_parameter("filter_alpha", 0.7)
        self.declare_parameter("include_gravity", False)
        self.declare_parameter("static_torque_bias_nm", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("warmup_sec", 8.0)
        self.declare_parameter("warmup_vel_threshold_rad_s", 0.02)
        self.declare_parameter("collision_config_path", DEFAULT_COLLISION_CONFIG_PATH)
        self.declare_parameter("torque_model_path", DEFAULT_TORQUE_MODEL_PATH)
        self.declare_parameter("torque_log_enabled", False)
        self.declare_parameter(
            "torque_log_path",
            str(Path(__file__).resolve().parents[1] / "data" / "torque_sensor_log.csv"),
        )
        self.declare_parameter("torque_log_min_velocity_rad_s", 0.01)
        self.declare_parameter("torque_log_min_acceleration_rad_s2", 0.05)
        self.declare_parameter("torque_log_idle_timeout_s", 2.0)

        self._slave_count = int(self.get_parameter("slave_count").value)
        self._print_table = bool(self.get_parameter("print_table").value)
        self._confirm_cycles = int(self.get_parameter("confirm_cycles").value)
        self._use_inverse_dynamics = bool(self.get_parameter("use_inverse_dynamics").value)
        self._dynamics_estimator_mode = str(
            self.get_parameter("dynamics_estimator_mode").value
        ).strip().lower()
        friction_coulomb_nm = list(self.get_parameter("friction_coulomb_nm").value)
        friction_viscous_nm_per_rad_s = list(
            self.get_parameter("friction_viscous_nm_per_rad_s").value
        )
        self._friction_velocity_deadband_rad_s = float(
            self.get_parameter("friction_velocity_deadband_rad_s").value
        )
        period = float(self.get_parameter("poll_period_sec").value)
        self._input_sample_period_sec = max(
            0.0,
            float(self.get_parameter("input_sample_period_sec").value),
        )
        self._last_joint_sample_time: Optional[float] = None
        self._last_dynamic_sample_time: Optional[float] = None
        effort_thresholds = list(self.get_parameter("effort_thresholds").value)
        following_error_thresholds = list(
            self.get_parameter("following_error_thresholds").value
        )
        external_torque_thresholds = list(
            self.get_parameter("external_torque_thresholds").value
        )
        self._effort_thresholds = {
            joint_name: float(effort_thresholds[index])
            for index, joint_name in enumerate(JOINT_NAMES[: self._slave_count])
        }
        self._following_error_thresholds = {
            joint_name: float(following_error_thresholds[index])
            for index, joint_name in enumerate(JOINT_NAMES[: self._slave_count])
        }
        self._external_torque_thresholds = {
            joint_name: float(external_torque_thresholds[index])
            for index, joint_name in enumerate(JOINT_NAMES[: self._slave_count])
        }
        self._friction_coulomb_nm = {
            joint_name: float(friction_coulomb_nm[index])
            for index, joint_name in enumerate(JOINT_NAMES[: self._slave_count])
        }
        self._friction_viscous_nm_per_rad_s = {
            joint_name: float(friction_viscous_nm_per_rad_s[index])
            for index, joint_name in enumerate(JOINT_NAMES[: self._slave_count])
        }

        self._latest_payload: Dict[str, Dict[str, Optional[float]]] = {
            joint_name: {
                "position": None,
                "velocity": None,
                "effort": None,
                **INTERFACE_DEFAULTS,
            }
            for joint_name in JOINT_NAMES[: self._slave_count]
        }
        self._joint_state_seen = False
        self._dynamic_state_seen = False
        self._joint_state_warned = False
        self._dynamic_state_warned = False
        self._contact_cycles = {joint_name: 0 for joint_name in JOINT_NAMES[: self._slave_count]}
        self._contact_latched = {joint_name: False for joint_name in JOINT_NAMES[: self._slave_count]}
        self._dynamics_cycles = {joint_name: 0 for joint_name in JOINT_NAMES[: self._slave_count]}
        self._dynamics_latched = {joint_name: False for joint_name in JOINT_NAMES[: self._slave_count]}
        self._contact_active = {joint_name: False for joint_name in JOINT_NAMES[: self._slave_count]}
        self._dynamics_active = {joint_name: False for joint_name in JOINT_NAMES[: self._slave_count]}
        self._contact_reason = {joint_name: "" for joint_name in JOINT_NAMES[: self._slave_count]}
        self._dynamics_reason = {joint_name: "" for joint_name in JOINT_NAMES[: self._slave_count]}
        self._expected_torque_by_joint = {
            joint_name: None for joint_name in JOINT_NAMES[: self._slave_count]
        }
        self._friction_torque_by_joint = {
            joint_name: None for joint_name in JOINT_NAMES[: self._slave_count]
        }
        self._measured_torque_by_joint = {
            joint_name: None for joint_name in JOINT_NAMES[: self._slave_count]
        }
        self._torque_difference_by_joint = {
            joint_name: None for joint_name in JOINT_NAMES[: self._slave_count]
        }
        self._external_torque_by_joint = {
            joint_name: None for joint_name in JOINT_NAMES[: self._slave_count]
        }
        self._learned_external_torque_filtered = {
            joint_name: None for joint_name in JOINT_NAMES[: self._slave_count]
        }
        self._last_velocity_vector: Optional[np.ndarray] = None
        self._last_velocity_timestamp: Optional[float] = None
        self._last_logged_velocity_by_joint = {
            joint_name: None for joint_name in JOINT_NAMES[: self._slave_count]
        }
        self._last_logged_timestamp: Optional[float] = None
        self._estimator: Optional[object] = None
        self._dynamics_enabled = False

        if self._use_inverse_dynamics:
            self._init_inverse_dynamics_model()

        self._collision_config_path = Path(
            str(self.get_parameter("collision_config_path").value)
        )
        self._load_persisted_config()
        self._filter_alpha = float(self.get_parameter("filter_alpha").value)
        self._torque_model_path = Path(str(self.get_parameter("torque_model_path").value))
        self._torque_model = self._load_torque_model(self._torque_model_path)
        self._torque_log_enabled = bool(self.get_parameter("torque_log_enabled").value)
        self._torque_log_path = Path(str(self.get_parameter("torque_log_path").value))
        self._torque_log_min_velocity_rad_s = float(
            self.get_parameter("torque_log_min_velocity_rad_s").value
        )
        self._torque_log_min_acceleration_rad_s2 = float(
            self.get_parameter("torque_log_min_acceleration_rad_s2").value
        )
        self._torque_log_idle_timeout_s = float(
            self.get_parameter("torque_log_idle_timeout_s").value
        )
        self._torque_log_header_written = False
        self._last_substantial_motion_timestamp: Optional[float] = None
        if self._torque_log_enabled:
            self._init_torque_log()

        self._json_pub = self.create_publisher(String, "/zeroerr/collision_monitor/json", 10)
        self._table_pub = self.create_publisher(String, "/zeroerr/collision_monitor/table", 10)
        self._state_pub = self.create_publisher(
            DynamicJointState,
            "/zeroerr/collision_monitor/state",
            10,
        )
        self._plot_publishers = {
            metric: self.create_publisher(
                JointState,
                f"/zeroerr/collision_monitor/plot/{metric}",
                qos_profile_sensor_data,
            )
            for metric in PLOT_METRICS
        }
        self._config_pub = self.create_publisher(
            String,
            "/zeroerr/collision_monitor/config_state",
            10,
        )

        self.create_subscription(
            JointState,
            "/joint_states",
            self._on_joint_states,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            DynamicJointState,
            "/dynamic_joint_states",
            self._on_dynamic_joint_states,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String,
            "/zeroerr/collision_monitor/config",
            self._on_config,
            10,
        )
        self.create_timer(period, self._publish_snapshot)

        self.get_logger().info(
            "[ZeroErrCollisionMonitor] Watching PDO-backed ROS interfaces from "
            "/joint_states and /dynamic_joint_states"
        )
        if self._dynamics_enabled:
            self.get_logger().info(
                "[ZeroErrCollisionMonitor] Inverse dynamics enabled with explicit ZeroErr model"
            )
        if self._torque_model is not None:
            self.get_logger().info(
                f"[ZeroErrCollisionMonitor] Learned torque model loaded from {self._torque_model_path}"
            )
        if self._input_sample_period_sec > 0.0:
            self.get_logger().info(
                "[ZeroErrCollisionMonitor] Input state processing throttled to "
                f"{1.0 / self._input_sample_period_sec:.1f} Hz"
            )

    def _on_joint_states(self, msg: JointState) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        if (
            self._input_sample_period_sec > 0.0
            and self._last_joint_sample_time is not None
            and now - self._last_joint_sample_time < self._input_sample_period_sec
        ):
            return
        self._last_joint_sample_time = now
        self._joint_state_seen = True
        for index, joint_name in enumerate(msg.name):
            if joint_name not in self._latest_payload:
                continue

            state = self._latest_payload[joint_name]
            if index < len(msg.position):
                state["position"] = msg.position[index]
            if index < len(msg.velocity):
                state["velocity"] = msg.velocity[index]
            if index < len(msg.effort):
                state["effort"] = msg.effort[index]

    def _on_dynamic_joint_states(self, msg: DynamicJointState) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        if (
            self._input_sample_period_sec > 0.0
            and self._last_dynamic_sample_time is not None
            and now - self._last_dynamic_sample_time < self._input_sample_period_sec
        ):
            return
        self._last_dynamic_sample_time = now
        self._dynamic_state_seen = True
        for index, joint_name in enumerate(msg.joint_names):
            if joint_name not in self._latest_payload or index >= len(msg.interface_values):
                continue

            interface_value = msg.interface_values[index]
            state = self._latest_payload[joint_name]
            for name, value in zip(interface_value.interface_names, interface_value.values):
                if name in INTERFACE_DEFAULTS:
                    state[name] = value

    def _on_config(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warning(
                f"[ZeroErrCollisionMonitor] Ignoring invalid config JSON: {exc}"
            )
            return

        updated = self._apply_runtime_config(payload)
        if updated:
            try:
                self._collision_config_path.parent.mkdir(parents=True, exist_ok=True)
                self._collision_config_path.write_text(
                    json.dumps(
                        {
                            "confirm_cycles": self._confirm_cycles,
                            "effort_thresholds": [
                                self._effort_thresholds[joint_name]
                                for joint_name in JOINT_NAMES[: self._slave_count]
                            ],
                            "following_error_thresholds": [
                                self._following_error_thresholds[joint_name]
                                for joint_name in JOINT_NAMES[: self._slave_count]
                            ],
                            "external_torque_thresholds": [
                                self._external_torque_thresholds[joint_name]
                                for joint_name in JOINT_NAMES[: self._slave_count]
                            ],
                            "friction_coulomb_nm": [
                                self._friction_coulomb_nm[joint_name]
                                for joint_name in JOINT_NAMES[: self._slave_count]
                            ],
                            "friction_viscous_nm_per_rad_s": [
                                self._friction_viscous_nm_per_rad_s[joint_name]
                                for joint_name in JOINT_NAMES[: self._slave_count]
                            ],
                            "friction_velocity_deadband_rad_s": self._friction_velocity_deadband_rad_s,
                            "dynamics_estimator_mode": self._dynamics_estimator_mode,
                            "use_inverse_dynamics": self._use_inverse_dynamics,
                        },
                        indent=2,
                        sort_keys=True,
                    ),
                    encoding="utf-8",
                )
            except Exception as exc:
                self.get_logger().warning(
                    f"[ZeroErrCollisionMonitor] Failed to persist config to {self._collision_config_path}: {exc}"
                )
            self.get_logger().info(
                "[ZeroErrCollisionMonitor] Runtime config updated"
            )

    def _apply_runtime_config(self, payload: Dict[str, object]) -> bool:
        updated = False
        confirm_cycles = payload.get("confirm_cycles")
        if confirm_cycles is not None:
            self._confirm_cycles = max(1, int(confirm_cycles))
            updated = True

        updated |= self._apply_threshold_update(
            payload.get("effort_thresholds"),
            self._effort_thresholds,
        )
        updated |= self._apply_threshold_update(
            payload.get("following_error_thresholds"),
            self._following_error_thresholds,
        )
        updated |= self._apply_threshold_update(
            payload.get("external_torque_thresholds"),
            self._external_torque_thresholds,
        )
        updated |= self._apply_threshold_update(
            payload.get("friction_coulomb_nm"),
            self._friction_coulomb_nm,
        )
        updated |= self._apply_threshold_update(
            payload.get("friction_viscous_nm_per_rad_s"),
            self._friction_viscous_nm_per_rad_s,
        )
        friction_velocity_deadband_rad_s = payload.get("friction_velocity_deadband_rad_s")
        if friction_velocity_deadband_rad_s is not None:
            self._friction_velocity_deadband_rad_s = max(
                0.0,
                float(friction_velocity_deadband_rad_s),
            )
            updated = True

        dynamics_estimator_mode = payload.get("dynamics_estimator_mode")
        if dynamics_estimator_mode is not None:
            mode = str(dynamics_estimator_mode).strip().lower()
            if mode in {"momentum_observer", "inverse_dynamics"}:
                if mode != self._dynamics_estimator_mode:
                    self._dynamics_estimator_mode = mode
                    self._reinitialize_estimator()
                updated = True

        use_inverse_dynamics = payload.get("use_inverse_dynamics")
        if use_inverse_dynamics is not None:
            enabled = bool(use_inverse_dynamics)
            if enabled != self._use_inverse_dynamics:
                self._use_inverse_dynamics = enabled
                self._clear_dynamics_collision_state()
                self._reinitialize_estimator()
            updated = True

        return updated

    def _clear_dynamics_collision_state(self) -> None:
        for joint_name in JOINT_NAMES[: self._slave_count]:
            self._dynamics_cycles[joint_name] = 0
            self._dynamics_latched[joint_name] = False
            self._dynamics_active[joint_name] = False
            self._dynamics_reason[joint_name] = ""

    def _load_persisted_config(self) -> None:
        path = self._collision_config_path
        if not path.exists():
            return
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
        except Exception as exc:
            self.get_logger().warning(
                f"[ZeroErrCollisionMonitor] Failed to load persisted config from {path}: {exc}"
            )
            return
        if self._apply_runtime_config(payload):
            self.get_logger().info(
                f"[ZeroErrCollisionMonitor] Loaded persisted config from {path}"
            )

    def _apply_threshold_update(
        self,
        values: Optional[List[float]],
        target: Dict[str, float],
    ) -> bool:
        if values is None:
            return False
        changed = False
        for index, joint_name in enumerate(JOINT_NAMES[: self._slave_count]):
            if index >= len(values):
                break
            target[joint_name] = float(values[index])
            changed = True
        return changed

    def _publish_snapshot(self) -> None:
        if not self._joint_state_seen and not self._joint_state_warned:
            self.get_logger().warning("[ZeroErrCollisionMonitor] Waiting for /joint_states...")
            self._joint_state_warned = True
        if not self._dynamic_state_seen and not self._dynamic_state_warned:
            self.get_logger().warning(
                "[ZeroErrCollisionMonitor] Waiting for /dynamic_joint_states..."
            )
            self._dynamic_state_warned = True

        stamp = self.get_clock().now().to_msg()
        payload = {
            "stamp": {
                "sec": stamp.sec,
                "nanosec": stamp.nanosec,
                "iso": datetime.now(UTC).isoformat(timespec="seconds"),
            },
            "slaves": [],
        }

        for slave_index, joint_name in enumerate(JOINT_NAMES[: self._slave_count]):
            state = dict(self._latest_payload[joint_name])
            state["joint"] = joint_name
            state["slave"] = slave_index
            state["friction_torque"] = self._compute_friction_torque(
                joint_name,
                state.get("velocity"),
            )
            self._friction_torque_by_joint[joint_name] = state["friction_torque"]
            state["measured_torque"] = self._compute_measured_torque(
                state.get("torque_sensor"),
            )
            self._measured_torque_by_joint[joint_name] = state["measured_torque"]
            payload["slaves"].append(state)

        self._update_external_torque_estimate(payload["slaves"])
        self._evaluate_collisions(payload["slaves"])
        self._inject_reasons(payload["slaves"])

        json_msg = String()
        json_msg.data = json.dumps(payload, sort_keys=True)
        self._json_pub.publish(json_msg)

        table_msg = String()
        table_msg.data = self._format_table(payload["slaves"])
        self._table_pub.publish(table_msg)
        self._state_pub.publish(self._build_state_msg(payload["slaves"]))
        self._publish_plot_topics(payload["slaves"], stamp)
        self._publish_config_state()

        if self._print_table:
            self.get_logger().info("\n" + table_msg.data)

        if self._torque_log_enabled:
            self._append_torque_log(payload["slaves"], stamp)

    def _evaluate_collisions(self, slaves: List[Dict[str, Optional[float]]]) -> None:
        for slave in slaves:
            joint_name = slave["joint"]
            effort = (
                abs(float(self._measured_torque_by_joint[joint_name]))
                if self._measured_torque_by_joint[joint_name] is not None
                else 0.0
            )
            following_error = (
                abs(float(slave["following_error_actual"]))
                if slave.get("following_error_actual") is not None
                else 0.0
            )
            external_torque = abs(self._external_torque_by_joint[joint_name]) if self._external_torque_by_joint[joint_name] is not None else 0.0
            self._contact_reason[joint_name] = (
                f"meas_tau {effort:.1f} > {self._effort_thresholds[joint_name]:.1f} and "
                f"foll_err {following_error:.0f} > {self._following_error_thresholds[joint_name]:.0f}"
            )
            self._dynamics_reason[joint_name] = (
                f"ext_tau {external_torque:.2f} > {self._external_torque_thresholds[joint_name]:.2f}"
            )

            if self._dynamics_enabled and self._external_torque_by_joint[joint_name] is not None:
                dynamics_active = external_torque >= self._external_torque_thresholds[joint_name]
                self._dynamics_active[joint_name] = dynamics_active
                self._contact_active[joint_name] = False
                self._contact_cycles[joint_name] = 0
                self._contact_latched[joint_name] = False
                if dynamics_active:
                    self._dynamics_cycles[joint_name] += 1
                else:
                    self._dynamics_cycles[joint_name] = 0
                    self._dynamics_latched[joint_name] = False

                if (
                    self._dynamics_cycles[joint_name] >= self._confirm_cycles
                    and not self._dynamics_latched[joint_name]
                ):
                    self._dynamics_latched[joint_name] = True
                    self.get_logger().warning(
                        "[ZeroErrCollisionMonitor] dynamics collision suspect on "
                        f"{joint_name}: external_torque={external_torque:.2f} >= "
                        f"{self._external_torque_thresholds[joint_name]:.2f}"
                    )
                continue

            exceeds_contact_thresholds = (
                effort >= self._effort_thresholds[joint_name]
                and following_error >= self._following_error_thresholds[joint_name]
            )
            self._contact_active[joint_name] = exceeds_contact_thresholds
            self._dynamics_active[joint_name] = False
            self._dynamics_cycles[joint_name] = 0
            self._dynamics_latched[joint_name] = False
            if exceeds_contact_thresholds:
                self._contact_cycles[joint_name] += 1
            else:
                self._contact_cycles[joint_name] = 0
                self._contact_latched[joint_name] = False

            if (
                self._contact_cycles[joint_name] >= self._confirm_cycles
                and not self._contact_latched[joint_name]
            ):
                self._contact_latched[joint_name] = True
                self.get_logger().warning(
                    "[ZeroErrCollisionMonitor] suspect collision on "
                    f"{joint_name}: effort={effort:.1f} >= {self._effort_thresholds[joint_name]:.1f}, "
                    f"following_error={following_error:.1f} >= "
                    f"{self._following_error_thresholds[joint_name]:.1f}"
                )

    def _inject_reasons(self, slaves: List[Dict[str, Optional[float]]]) -> None:
        for slave in slaves:
            joint_name = str(slave["joint"])
            slave["contact_reason"] = self._contact_reason[joint_name]
            slave["dynamics_reason"] = self._dynamics_reason[joint_name]

    def _init_inverse_dynamics_model(self) -> None:
        support = self._load_inverse_dynamics_support()
        if support is None:
            return
        external_estimator_class, momentum_estimator_class, model_class = support

        urdf_path = str(self.get_parameter("urdf_path").value)
        base_link = str(self.get_parameter("base_link").value)
        tip_link = str(self.get_parameter("tip_link").value)
        num_joints = int(self.get_parameter("num_joints").value)
        filter_alpha = float(self.get_parameter("filter_alpha").value)
        include_gravity = bool(self.get_parameter("include_gravity").value)
        static_torque_bias_nm = list(self.get_parameter("static_torque_bias_nm").value)
        warmup_sec = float(self.get_parameter("warmup_sec").value)
        warmup_vel_threshold_rad_s = float(self.get_parameter("warmup_vel_threshold_rad_s").value)

        if not urdf_path:
            self.get_logger().warning(
                "[ZeroErrCollisionMonitor] Inverse dynamics disabled: missing urdf_path"
            )
            return

        try:
            model = model_class(
                urdf_path=urdf_path,
                base_link=base_link,
                tip_link=tip_link,
                num_joints=num_joints,
                include_gravity=include_gravity,
                logger=self.get_logger(),
            )
            estimator_mode = self._dynamics_estimator_mode
            if estimator_mode == "momentum_observer":
                self._estimator = momentum_estimator_class(
                    model,
                    filter_alpha=filter_alpha,
                    static_torque_bias_nm=np.array(static_torque_bias_nm, dtype=float),
                    warmup_sec=warmup_sec,
                    warmup_vel_threshold_rad_s=warmup_vel_threshold_rad_s,
                )
            else:
                self._estimator = external_estimator_class(model, filter_alpha=filter_alpha)
            self._dynamics_enabled = True
            self.get_logger().info(
                f"[ZeroErrCollisionMonitor] Dynamics estimator mode: {estimator_mode}"
            )
        except Exception as exc:
            self._dynamics_enabled = False
            self.get_logger().warning(
                f"[ZeroErrCollisionMonitor] Inverse dynamics disabled: {exc}"
            )

    def _reinitialize_estimator(self) -> None:
        self._estimator = None
        self._dynamics_enabled = False
        self._last_velocity_vector = None
        self._last_velocity_timestamp = None
        for joint_name in JOINT_NAMES[: self._slave_count]:
            self._expected_torque_by_joint[joint_name] = None
            self._torque_difference_by_joint[joint_name] = None
            self._external_torque_by_joint[joint_name] = None
            self._learned_external_torque_filtered[joint_name] = None
        if self._use_inverse_dynamics:
            self._init_inverse_dynamics_model()

    def _load_torque_model(self, path: Path) -> Optional[Dict[str, np.ndarray]]:
        if not path.exists():
            self.get_logger().warning(
                f"[ZeroErrCollisionMonitor] Learned torque model not found: {path}"
            )
            return None
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
        except Exception as exc:
            self.get_logger().warning(
                f"[ZeroErrCollisionMonitor] Failed to load learned torque model from {path}: {exc}"
            )
            return None

        joints_payload = payload.get("joints", {})
        model: Dict[str, np.ndarray] = {}
        for joint_name in JOINT_NAMES[: self._slave_count]:
            joint_payload = joints_payload.get(joint_name)
            if not isinstance(joint_payload, dict) or joint_payload.get("skipped"):
                continue
            coefficients = joint_payload.get("coefficients")
            if not isinstance(coefficients, list) or len(coefficients) != 6:
                continue
            model[joint_name] = np.array(coefficients, dtype=float)

        if not model:
            self.get_logger().warning(
                f"[ZeroErrCollisionMonitor] Learned torque model at {path} has no usable joints"
            )
            return None
        return model

    def _predict_learned_torque(
        self,
        joint_name: str,
        position: float,
        velocity: float,
        acceleration: float,
    ) -> Optional[float]:
        if self._torque_model is None or joint_name not in self._torque_model:
            return None
        coeffs = self._torque_model[joint_name]
        features = np.array(
            [1.0, np.sin(position), np.cos(position), velocity, np.sign(velocity), acceleration],
            dtype=float,
        )
        return float(coeffs @ features)

    def _load_inverse_dynamics_support(self):
        try:
            from erob_moveit_runtime.safety.collision_detection.external_torque_estimator import (
                ExternalTorqueEstimator as estimator_class,
                MomentumObserverEstimator as momentum_estimator_class,
            )
            from erob_moveit_runtime.safety.collision_detection.inverse_dynamics_model import (
                KDLInverseDynamicsModel as model_class,
            )

            return estimator_class, momentum_estimator_class, model_class
        except ModuleNotFoundError:
            pass

        candidate_roots = [
            Path(__file__).resolve().parents[4] / "install" / "erob_moveit_runtime" / "lib",
            Path(__file__).resolve().parents[4] / "src" / "eRob_ROS2_MoveIt" / "erob_moveit_runtime" / "scripts",
        ]
        for candidate_root in candidate_roots:
            if not candidate_root.is_dir():
                continue
            candidate_root_str = str(candidate_root)
            if candidate_root_str not in sys.path:
                sys.path.insert(0, candidate_root_str)
            try:
                from erob_moveit_runtime.safety.collision_detection.external_torque_estimator import (
                    ExternalTorqueEstimator as estimator_class,
                    MomentumObserverEstimator as momentum_estimator_class,
                )
                from erob_moveit_runtime.safety.collision_detection.inverse_dynamics_model import (
                    KDLInverseDynamicsModel as model_class,
                )

                return estimator_class, momentum_estimator_class, model_class
            except ModuleNotFoundError:
                try:
                    from safety.collision_detection.external_torque_estimator import (
                        ExternalTorqueEstimator as estimator_class,
                        MomentumObserverEstimator as momentum_estimator_class,
                    )
                    from safety.collision_detection.inverse_dynamics_model import (
                        KDLInverseDynamicsModel as model_class,
                    )

                    return estimator_class, momentum_estimator_class, model_class
                except ModuleNotFoundError:
                    continue

        self._dynamics_enabled = False
        self.get_logger().warning(
            "[ZeroErrCollisionMonitor] Inverse dynamics disabled: erob_moveit_runtime collision modules not importable"
        )
        return None

    def _update_external_torque_estimate(self, slaves: List[Dict[str, Optional[float]]]) -> None:
        if self._torque_model is not None:
            self._update_external_torque_from_learned_model(slaves)
            return

        if not self._dynamics_enabled or self._estimator is None:
            return

        positions: List[float] = []
        velocities: List[float] = []
        measured_torques: List[float] = []
        for slave in slaves:
            if (
                slave.get("position") is None
                or slave.get("velocity") is None
            ):
                return
            measured_torque = self._compute_measured_torque(
                slave.get("torque_sensor"),
            )
            if measured_torque is None:
                return
            positions.append(float(slave["position"]))
            velocities.append(float(slave["velocity"]))
            measured_torques.append(float(measured_torque))

        velocity_vector = np.array(velocities, dtype=float)
        timestamp = self.get_clock().now().nanoseconds / 1e9
        if self._last_velocity_vector is None or self._last_velocity_timestamp is None:
            acceleration_vector = np.zeros_like(velocity_vector)
        else:
            dt = max(timestamp - self._last_velocity_timestamp, 1e-6)
            acceleration_vector = (velocity_vector - self._last_velocity_vector) / dt

        self._last_velocity_vector = velocity_vector
        self._last_velocity_timestamp = timestamp

        self._estimator.update(
            np.array(measured_torques, dtype=float),
            np.array(positions, dtype=float),
            velocity_vector,
            acceleration_vector,
        )

        filtered = self._estimator.filtered
        expected = self._estimator.expected_torque_history[-1] if self._estimator.expected_torque_history else None
        for index, joint_name in enumerate(JOINT_NAMES[: self._slave_count]):
            expected_torque = float(expected[index]) if expected is not None else None
            self._expected_torque_by_joint[joint_name] = expected_torque
            self._torque_difference_by_joint[joint_name] = (
                float(measured_torques[index]) - expected_torque if expected_torque is not None else None
            )
            self._external_torque_by_joint[joint_name] = float(filtered[index])

    def _update_external_torque_from_learned_model(
        self,
        slaves: List[Dict[str, Optional[float]]],
    ) -> None:
        positions: List[float] = []
        velocities: List[float] = []
        measured_torques: List[float] = []
        for slave in slaves:
            if slave.get("position") is None or slave.get("velocity") is None:
                return
            measured_torque = self._compute_measured_torque(slave.get("torque_sensor"))
            if measured_torque is None:
                return
            positions.append(float(slave["position"]))
            velocities.append(float(slave["velocity"]))
            measured_torques.append(float(measured_torque))

        velocity_vector = np.array(velocities, dtype=float)
        timestamp = self.get_clock().now().nanoseconds / 1e9
        if self._last_velocity_vector is None or self._last_velocity_timestamp is None:
            acceleration_vector = np.zeros_like(velocity_vector)
        else:
            dt = max(timestamp - self._last_velocity_timestamp, 1e-6)
            acceleration_vector = (velocity_vector - self._last_velocity_vector) / dt

        self._last_velocity_vector = velocity_vector
        self._last_velocity_timestamp = timestamp

        for index, joint_name in enumerate(JOINT_NAMES[: self._slave_count]):
            expected_torque = self._predict_learned_torque(
                joint_name,
                positions[index],
                velocities[index],
                float(acceleration_vector[index]),
            )
            self._expected_torque_by_joint[joint_name] = expected_torque
            if expected_torque is None:
                self._torque_difference_by_joint[joint_name] = None
                self._external_torque_by_joint[joint_name] = None
                self._learned_external_torque_filtered[joint_name] = None
                continue

            residual = float(measured_torques[index]) - expected_torque
            previous = self._learned_external_torque_filtered[joint_name]
            if previous is None:
                filtered = residual
            else:
                filtered = self._filter_alpha * previous + (1.0 - self._filter_alpha) * residual
            self._learned_external_torque_filtered[joint_name] = filtered
            self._torque_difference_by_joint[joint_name] = residual
            self._external_torque_by_joint[joint_name] = filtered

    def _format_table(self, slaves: List[Dict[str, Optional[float]]]) -> str:
        lines = [
            "slv joint     sens_tau(Nm) fric_tau(Nm) meas_tau(Nm) exp_tau(Nm) diff_tau(Nm) ext_tau(Nm) foll_err",
        ]
        for slave in slaves:
            joint_name = slave.get("joint", "")
            external_torque = self._external_torque_by_joint.get(slave.get("joint", ""))
            lines.append(
                "{slave:>3} {joint:<7} {torque_sensor:>12} {friction_torque:>12} "
                "{measured_torque:>12} {expected_torque:>11} {torque_difference:>12} {external_torque:>11} {following_error_actual:>8}".format(
                    slave=slave.get("slave", "NA"),
                    joint=joint_name or "NA",
                    torque_sensor=self._fmt_float(slave.get("torque_sensor"), 2),
                    friction_torque=self._fmt_float(self._friction_torque_by_joint.get(joint_name), 2),
                    measured_torque=self._fmt_float(self._measured_torque_by_joint.get(joint_name), 2),
                    expected_torque=self._fmt_float(self._expected_torque_by_joint.get(joint_name), 2),
                    torque_difference=self._fmt_float(self._torque_difference_by_joint.get(joint_name), 2),
                    external_torque=self._fmt_float(external_torque, 2),
                    following_error_actual=self._fmt_float(
                        slave.get("following_error_actual"),
                        0,
                    ),
                )
            )
        return "\n".join(lines)

    def _init_torque_log(self) -> None:
        self._torque_log_path.parent.mkdir(parents=True, exist_ok=True)
        self._torque_log_header_written = self._torque_log_path.exists() and self._torque_log_path.stat().st_size > 0
        self.get_logger().info(
            f"[ZeroErrCollisionMonitor] Torque logging enabled -> {self._torque_log_path}"
        )

    def _append_torque_log(self, slaves: List[Dict[str, Optional[float]]], stamp) -> None:
        timestamp = f"{stamp.sec}.{stamp.nanosec:09d}"
        timestamp_s = float(stamp.sec) + float(stamp.nanosec) / 1e9
        acceleration_by_joint: Dict[str, Optional[float]] = {}
        for slave in slaves:
            joint_name = str(slave["joint"])
            acceleration_by_joint[joint_name] = self._compute_joint_acceleration(
                joint_name,
                slave.get("velocity"),
                timestamp_s,
            )

        if not self._torque_log_motion_window_active(slaves, acceleration_by_joint, timestamp_s):
            self._last_logged_timestamp = timestamp_s
            for slave in slaves:
                joint_name = str(slave["joint"])
                velocity = slave.get("velocity")
                self._last_logged_velocity_by_joint[joint_name] = (
                    None if velocity is None else float(velocity)
                )
            return

        with self._torque_log_path.open("a", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            if not self._torque_log_header_written:
                writer.writerow(
                    [
                        "timestamp",
                        "joint",
                        "position_rad",
                        "velocity_rad_s",
                        "acceleration_rad_s2",
                        "torque_sensor_nm",
                        "measured_torque_nm",
                        "expected_torque_nm",
                        "external_torque_nm",
                        "following_error_actual",
                    ]
                )
                self._torque_log_header_written = True

            for slave in slaves:
                joint_name = str(slave["joint"])
                position = slave.get("position")
                velocity = slave.get("velocity")
                acceleration = acceleration_by_joint.get(joint_name)
                torque_sensor = slave.get("torque_sensor")
                following_error = slave.get("following_error_actual")
                if not self._torque_log_sample_is_valid(
                    position=position,
                    velocity=velocity,
                    acceleration=acceleration,
                    torque_sensor=torque_sensor,
                    following_error=following_error,
                ):
                    continue
                writer.writerow(
                    [
                        timestamp,
                        joint_name,
                        self._csv_value(position),
                        self._csv_value(velocity),
                        self._csv_value(acceleration),
                        self._csv_value(torque_sensor),
                        self._csv_value(self._measured_torque_by_joint.get(joint_name)),
                        self._csv_value(self._expected_torque_by_joint.get(joint_name)),
                        self._csv_value(self._external_torque_by_joint.get(joint_name)),
                        self._csv_value(following_error),
                    ]
                )

        self._last_logged_timestamp = timestamp_s
        for slave in slaves:
            joint_name = str(slave["joint"])
            velocity = slave.get("velocity")
            self._last_logged_velocity_by_joint[joint_name] = (
                None if velocity is None else float(velocity)
            )

    def _compute_joint_acceleration(
        self,
        joint_name: str,
        velocity: Optional[float],
        timestamp_s: float,
    ) -> Optional[float]:
        if velocity is None:
            return None
        previous_velocity = self._last_logged_velocity_by_joint.get(joint_name)
        if previous_velocity is None or self._last_logged_timestamp is None:
            return None
        dt = timestamp_s - self._last_logged_timestamp
        if dt <= 1e-6:
            return None
        return (float(velocity) - previous_velocity) / dt

    def _csv_value(self, value: Optional[float]) -> str:
        if value is None:
            return ""
        value_f = float(value)
        if not np.isfinite(value_f):
            return ""
        return f"{value_f:.9f}"

    def _torque_log_sample_is_valid(
        self,
        *,
        position: Optional[float],
        velocity: Optional[float],
        acceleration: Optional[float],
        torque_sensor: Optional[float],
        following_error: Optional[float],
    ) -> bool:
        required_values = [position, velocity, acceleration, torque_sensor, following_error]
        return all(value is not None and np.isfinite(float(value)) for value in required_values)

    def _torque_log_motion_window_active(
        self,
        slaves: List[Dict[str, Optional[float]]],
        acceleration_by_joint: Dict[str, Optional[float]],
        timestamp_s: float,
    ) -> bool:
        motion_detected = False
        for slave in slaves:
            joint_name = str(slave["joint"])
            velocity = slave.get("velocity")
            acceleration = acceleration_by_joint.get(joint_name)
            velocity_ok = (
                velocity is not None
                and np.isfinite(float(velocity))
                and abs(float(velocity)) >= self._torque_log_min_velocity_rad_s
            )
            acceleration_ok = (
                acceleration is not None
                and np.isfinite(float(acceleration))
                and abs(float(acceleration)) >= self._torque_log_min_acceleration_rad_s2
            )
            if velocity_ok or acceleration_ok:
                motion_detected = True
                break

        if motion_detected:
            self._last_substantial_motion_timestamp = timestamp_s
            return True

        if self._last_substantial_motion_timestamp is None:
            return False

        return (timestamp_s - self._last_substantial_motion_timestamp) <= self._torque_log_idle_timeout_s

    def _build_state_msg(self, slaves: List[Dict[str, Optional[float]]]) -> DynamicJointState:
        msg = DynamicJointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.joint_names = [str(slave["joint"]) for slave in slaves]

        for slave in slaves:
            interface_value = InterfaceValue()
            interface_value.interface_names = [
                "position",
                "velocity",
                "torque_sensor",
                "friction_torque",
                "measured_torque",
                "expected_torque",
                "torque_difference",
                "external_torque",
                "following_error_actual",
                "error_code",
                "statusword",
                "mode_display",
                "contact_active",
                "contact_cycles",
                "contact_latched",
                "dynamics_active",
                "dynamics_cycles",
                "dynamics_latched",
            ]
            interface_value.values = [
                self._state_value(slave.get("position")),
                self._state_value(slave.get("velocity")),
                self._state_value(slave.get("torque_sensor")),
                self._state_value(self._friction_torque_by_joint.get(str(slave["joint"]))),
                self._state_value(self._measured_torque_by_joint.get(str(slave["joint"]))),
                self._state_value(self._expected_torque_by_joint.get(str(slave["joint"]))),
                self._state_value(self._torque_difference_by_joint.get(str(slave["joint"]))),
                self._state_value(self._external_torque_by_joint.get(str(slave["joint"]))),
                self._state_value(slave.get("following_error_actual")),
                self._state_value(slave.get("error_code")),
                self._state_value(slave.get("statusword")),
                self._state_value(slave.get("mode_display")),
                self._state_value(1.0 if self._contact_active[str(slave["joint"])] else 0.0),
                self._state_value(float(self._contact_cycles[str(slave["joint"])])),
                self._state_value(1.0 if self._contact_latched[str(slave["joint"])] else 0.0),
                self._state_value(1.0 if self._dynamics_active[str(slave["joint"])] else 0.0),
                self._state_value(float(self._dynamics_cycles[str(slave["joint"])])),
                self._state_value(1.0 if self._dynamics_latched[str(slave["joint"])] else 0.0),
            ]
            msg.interface_values.append(interface_value)

        return msg

    def _state_value(self, value: Optional[float]) -> float:
        if value is None:
            return float("nan")
        return float(value)

    def _publish_config_state(self) -> None:
        msg = String()
        msg.data = json.dumps(
            {
                "confirm_cycles": self._confirm_cycles,
                "effort_thresholds": [
                    self._effort_thresholds[joint_name]
                    for joint_name in JOINT_NAMES[: self._slave_count]
                ],
                "following_error_thresholds": [
                    self._following_error_thresholds[joint_name]
                    for joint_name in JOINT_NAMES[: self._slave_count]
                ],
                "external_torque_thresholds": [
                    self._external_torque_thresholds[joint_name]
                    for joint_name in JOINT_NAMES[: self._slave_count]
                ],
                "dynamics_estimator_mode": self._dynamics_estimator_mode,
                "use_inverse_dynamics": self._use_inverse_dynamics,
                "friction_coulomb_nm": [
                    self._friction_coulomb_nm[joint_name]
                    for joint_name in JOINT_NAMES[: self._slave_count]
                ],
                "friction_viscous_nm_per_rad_s": [
                    self._friction_viscous_nm_per_rad_s[joint_name]
                    for joint_name in JOINT_NAMES[: self._slave_count]
                ],
                "friction_velocity_deadband_rad_s": self._friction_velocity_deadband_rad_s,
            },
            sort_keys=True,
        )
        self._config_pub.publish(msg)

    def _publish_plot_topics(self, slaves: List[Dict[str, Optional[float]]], stamp) -> None:
        joint_names = [str(slave["joint"]) for slave in slaves]
        for metric, publisher in self._plot_publishers.items():
            msg = JointState()
            msg.header.stamp = stamp
            msg.name = joint_names
            msg.position = [
                self._state_value(PLOT_METRICS[metric](self, slave))
                for slave in slaves
            ]
            publisher.publish(msg)

    def _compute_friction_torque(
        self,
        joint_name: str,
        velocity: Optional[float],
    ) -> Optional[float]:
        if velocity is None:
            return None
        velocity_f = float(velocity)
        if abs(velocity_f) < self._friction_velocity_deadband_rad_s:
            sign = 0.0
        else:
            sign = 1.0 if velocity_f > 0.0 else -1.0
        return (
            self._friction_coulomb_nm[joint_name] * sign
            + self._friction_viscous_nm_per_rad_s[joint_name] * velocity_f
        )

    def _compute_measured_torque(
        self,
        torque_sensor: Optional[float],
    ) -> Optional[float]:
        if torque_sensor is None or not np.isfinite(torque_sensor):
            return None
        return float(torque_sensor)

    def _fmt_float(self, value: Optional[float], decimals: int) -> str:
        if value is None:
            return "NA"
        return f"{value:.{decimals}f}"

    def _fmt_hex(self, value: Optional[float]) -> str:
        if value is None:
            return "NA"
        return f"0x{int(value):04X}"


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ZeroErrCollisionMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
