#!/usr/bin/env python3
from __future__ import annotations

import logging
from typing import Any, Callable

import config
from motion.planning.reachability import validate_pose_from_start
from runtime_gateway.base import RuntimeGateway
from utils.work_object import WorkObject


logger = logging.getLogger("erob_moveit_runtime_gateway")


def _to_jsonable(value):
    """Convert nested ROS / numpy-ish values into plain JSON-safe Python types."""
    if isinstance(value, dict):
        return {str(k): _to_jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_to_jsonable(v) for v in value]
    if hasattr(value, "item") and callable(getattr(value, "item")):
        try:
            return _to_jsonable(value.item())
        except Exception:
            pass
    return value


class LocalRuntimeGateway(RuntimeGateway):
    """Local in-process gateway.

    Preserves current behavior by calling the RobotController and
    IRobotBackend methods directly, with effectively no cost beyond a
    Python method call. This is the reference implementation for the
    future ROS-based gateway.
    """

    def __init__(
        self,
        robot: Any | None = None,
        node: Any | None = None,
        *,
        robot_getter: Callable[[], Any] | None = None,
        node_getter: Callable[[], Any] | None = None,
    ):
        self._robot = robot
        self._node = node
        self._robot_getter = robot_getter
        self._node_getter = node_getter

    @property
    def robot(self) -> Any | None:
        if self._robot_getter is not None:
            return self._robot_getter()
        return self._robot

    @property
    def node(self) -> Any | None:
        if self._node_getter is not None:
            return self._node_getter()
        return self._node

    # --- startup / readiness ----------------------------------------------

    def is_runtime_initialized(self) -> bool:
        return self.robot is not None and self.node is not None

    def is_motion_stack_ready(self) -> bool:
        node = self.node
        is_ready = getattr(node, "is_motion_stack_ready", None)
        return bool(is_ready() if callable(is_ready) else False)

    def get_motion_stack_fault_reason(self) -> str | None:
        node = self.node
        get_reason = getattr(node, "get_motion_stack_fault_reason", None)
        if not callable(get_reason):
            return None if self.is_motion_stack_ready() else "motion stack not ready"
        reason = get_reason()
        return str(reason) if reason else None

    def startup_status(self) -> dict:
        runtime_initialized = self.robot is not None and self.node is not None
        return {
            "ros2_active": runtime_initialized,
            "runtime_initialized": runtime_initialized,
            "motion_stack_ready": self.is_motion_stack_ready(),
            "motion_stack_fault": self.get_motion_stack_fault_reason(),
        }

    def runtime_state_snapshot(self) -> dict:
        robot = self.robot
        node = self.node
        if robot is None or node is None:
            return {
                "runtime_ready": False,
                "runtime_initialized": False,
                "startup": self.startup_status(),
            }
        drive_status = _to_jsonable(node.get_drive_operation_status())
        motion_interlock = _to_jsonable(node.get_motion_interlock_status())
        hardware_ready = bool(node.is_hardware_ready_for_motion())
        motion_stack_ready = bool(node.is_motion_stack_ready())
        return {
            "runtime_ready": motion_stack_ready,
            "runtime_initialized": True,
            "hardware_ready": hardware_ready,
            "hardware_fault": None if hardware_ready else node.get_hardware_fault_reason(),
            "motion_stack_ready": motion_stack_ready,
            "motion_stack_fault": None if motion_stack_ready else node.get_motion_stack_fault_reason(),
            "drive": drive_status,
            "motion_interlock": motion_interlock,
        }

    # --- status ------------------------------------------------------------

    def _runtime_status_dict(self) -> dict:
        node = self.node
        status_publisher = getattr(node, "status_publisher", None)
        get_status = getattr(status_publisher, "get_status_dict", None)
        return dict(get_status()) if callable(get_status) else {}

    def status(self) -> dict:
        robot = self.robot
        status = self._runtime_status_dict()
        get_ordered_status = getattr(robot, "get_ordered_motion_chain_status", None)
        if callable(get_ordered_status):
            status["ordered_motion_chain"] = _to_jsonable(get_ordered_status())
        status["success"] = True
        status.update(self.runtime_state_snapshot())
        return status

    def last_submitted_task_id(self) -> Any:
        node = getattr(self.robot, "node", self.node)
        return getattr(node, "last_submitted_task_id", None)

    def state_snapshot(self) -> dict:
        robot = self.robot
        node = self.node
        position = robot.get_current_position()
        flange_position = robot.get_current_flange_position()
        joints = robot.get_current_joints()
        velocity = robot.get_current_velocity()
        unavailable = []
        if position is None:
            unavailable.append("position")
        if flange_position is None:
            unavailable.append("flange_position")
        if joints is None:
            unavailable.append("joints")
        if velocity is None:
            unavailable.append("velocity")
        return {
            "success": not unavailable,
            "partial": bool(unavailable),
            "unavailable_fields": unavailable,
            "position": position,
            "flange_position": flange_position,
            "joints": joints,
            "velocity": velocity,
            "status": self._runtime_status_dict(),
            "active_tool": getattr(node, "active_tool_name", "TOOL_0"),
            "active_workobject": self.active_workobject(),
            "safety_walls": _to_jsonable(robot.get_safety_walls_status()),
            **self.runtime_state_snapshot(),
        }

    def state_kinematics(self) -> dict:
        robot = self.robot
        position = robot.get_current_position()
        joints = robot.get_current_joints()
        velocity = robot.get_current_velocity()
        acceleration = robot.get_current_acceleration()
        if position is None:
            return {"success": False, "error": "current position unavailable"}
        unavailable = []
        if joints is None:
            unavailable.append("joints")
        if velocity is None:
            unavailable.append("velocity")
        if acceleration is None:
            unavailable.append("acceleration")
        return {
            "success": not unavailable,
            "partial": bool(unavailable),
            "unavailable_fields": unavailable,
            "position": position,
            "joints": joints,
            "velocity": velocity,
            "acceleration": acceleration,
        }

    def ordered_motion_chain_status(self) -> dict:
        robot = self.robot
        get_ordered_status = getattr(robot, "get_ordered_motion_chain_status", None)
        if not callable(get_ordered_status):
            return {"supported": False, "active": False}
        status = _to_jsonable(get_ordered_status())
        if not isinstance(status, dict):
            return {"success": False, "error": "invalid ordered motion chain status"}
        return status

    # --- motion ------------------------------------------------------------

    def stop_motion(self) -> dict:
        logger.info("[runtime_gateway] Stopping motion")
        stop_result = self.robot.stop_motion()
        if not isinstance(stop_result, dict):
            stop_result = {
                "state": "ERROR",
                "result": -2,
                "success": False,
                "stopped": False,
                "error": f"Unexpected stop result type: {type(stop_result).__name__}",
            }
        return {
            "stop_state": stop_result.get("state", "ERROR"),
            "stopped": bool(stop_result.get("stopped", False)),
            "result": stop_result.get("result", -2),
            "success": bool(stop_result.get("success", False)),
            "queue_cleared": int(stop_result.get("queue_cleared", 0)),
            **({"error": stop_result["error"]} if stop_result.get("error") else {}),
        }

    def move_linear(
        self,
        position,
        tool=0,
        user=0,
        vel=30,
        acc=30,
        blocking=True,
        trajectory_optimizer=None,
        allow_collision_recovery=False,
    ) -> int:
        try:
            from motion.move_linear_timing import begin as begin_move_linear_timing
            begin_move_linear_timing(self.node, source="/move/linear")
        except Exception:
            pass
        return self.robot.move_liner(
            position,
            tool=tool,
            user=user,
            vel=vel,
            acc=acc,
            blocking=blocking,
            trajectory_optimizer=trajectory_optimizer,
            allow_collision_recovery=allow_collision_recovery,
        )

    def move_ptp(
        self,
        position,
        tool=0,
        user=0,
        vel=30,
        acc=30,
        blocking=True,
        trajectory_optimizer=None,
    ) -> int:
        return self.robot.move_ptp(
            position,
            tool=tool,
            user=user,
            vel=vel,
            acc=acc,
            blocking=blocking,
            trajectory_optimizer=trajectory_optimizer,
        )

    def execute_path(
        self,
        path,
        rx=None,
        ry=None,
        rz=None,
        vel=0.6,
        acc=0.4,
        blocking=True,
        trajectory_optimizer=None,
        orientation_mode="constant",
    ) -> int:
        return self.robot.execute_path(
            path,
            rx=rx,
            ry=ry,
            rz=rz,
            vel=vel,
            acc=acc,
            blocking=blocking,
            trajectory_optimizer=trajectory_optimizer,
            orientation_mode=orientation_mode,
        )

    def execute_sequence(self, segments, tool=0, user=0, blocking=True) -> int:
        return self.robot.execute_sequence(
            segments,
            tool=tool,
            user=user,
            blocking=blocking,
        )

    def execute_ordered_motion_chain(
        self,
        segments,
        tool=0,
        user=0,
        blocking=True,
        trajectory_optimizer=None,
    ) -> int:
        return self.robot.execute_ordered_motion_chain(
            segments=segments,
            tool=tool,
            user=user,
            blocking=blocking,
            trajectory_optimizer=trajectory_optimizer,
        )

    def prepare_ordered_motion_chain(self, segments, start_position, tool=0, user=0,
                                     trajectory_optimizer=None) -> dict:
        return self.robot.prepare_ordered_motion_chain(
            segments, start_position, tool, user, trajectory_optimizer
        )

    def execute_prepared_ordered_motion_chain(self, plan_id: str) -> dict:
        return self.robot.execute_prepared_ordered_motion_chain(plan_id)

    def discard_prepared_ordered_motion_chain(self, plan_id: str) -> dict:
        return self.robot.discard_prepared_ordered_motion_chain(plan_id)

    def prepared_ordered_motion_chain_status(self, plan_id: str) -> dict:
        return self.robot.get_prepared_ordered_motion_chain(plan_id)

    def unwind_joint6(self, blocking=True, queue_if_busy=True, vel=None, acc=None) -> int:
        return self.robot.unwind_joint6(
            blocking=blocking,
            queue_if_busy=queue_if_busy,
            vel=vel,
            acc=acc,
        )

    def jog(self, axis, direction, step, vel, acc, frame=None, tool=0, user=0) -> int:
        return self.robot.start_jog(
            axis,
            direction,
            step,
            vel,
            acc,
            frame=frame,
            tool=tool,
            user=user,
        )

    def joint_jog(self, joint, direction, step, vel, acc, blocking=True) -> int:
        return self.robot.joint_jog(
            joint,
            direction,
            step,
            vel,
            acc,
            blocking=blocking,
        )

    def servo_jog_start(
        self,
        axis,
        direction,
        vel,
        acc,
        frame=None,
        tool=0,
        user=0,
        linear_mm_s=None,
        angular_deg_s=None,
        disable_collision_checking=False,
    ) -> int:
        return self.robot.start_servo_jog(
            axis,
            direction,
            vel,
            acc,
            frame=frame,
            tool=tool,
            user=user,
            linear_mm_s=linear_mm_s,
            angular_deg_s=angular_deg_s,
            disable_collision_checking=disable_collision_checking,
        )

    def servo_jog_stop(self, *, restore_collision_checking: bool = True) -> int:
        return self.robot.stop_servo_jog(
            restore_collision_checking=restore_collision_checking
        )

    # --- position / kinematics queries ------------------------------------

    def get_current_position(self):
        return self.robot.get_current_position()

    def get_current_base_tcp_position(self):
        getter = getattr(self.robot, "get_current_base_tcp_position", None)
        return getter() if callable(getter) else self.robot.get_current_position(user_id=0)

    def get_current_flange_position(self):
        return self.robot.get_current_flange_position()

    def get_current_velocity(self):
        return self.robot.get_current_velocity()

    def get_current_acceleration(self):
        return self.robot.get_current_acceleration()

    # --- cartesian servo ---------------------------------------------------

    @property
    def _cartesian_servo(self):
        robot = self.robot
        return robot.cartesian_servo if robot is not None else None

    def cartesian_servo_available(self) -> bool:
        return self._cartesian_servo is not None

    def cartesian_servo_start(self, frame, tool, user):
        return self._cartesian_servo.start(frame=frame, tool=tool, user=user)

    def cartesian_servo_update(self, linear_mm_s, angular_deg_s):
        return self._cartesian_servo.update(
            linear_mm_s=linear_mm_s,
            angular_deg_s=angular_deg_s,
        )

    def cartesian_servo_stop(self):
        return self._cartesian_servo.stop()

    def cartesian_servo_status(self):
        servo = self._cartesian_servo
        return servo.get_status() if servo is not None else None

    # --- drives / interlocks ----------------------------------------------

    def set_drive_operation_enabled(self, enabled: bool) -> dict:
        return _to_jsonable(self.node.set_drive_operation_enabled(enabled))

    def drive_operation_status(self) -> dict:
        node = self.node
        hardware_ready = bool(node.is_hardware_ready_for_motion())
        return {
            **_to_jsonable(node.get_drive_operation_status()),
            "hardware_ready": hardware_ready,
            "hardware_fault": None if hardware_ready else node.get_hardware_fault_reason(),
        }

    def motion_interlock_status(self) -> dict:
        return _to_jsonable(self.node.get_motion_interlock_status())

    def reset_motion_interlock(self) -> dict:
        node = self.node
        result = _to_jsonable(node.reset_motion_interlock())
        hardware_ready = bool(node.is_hardware_ready_for_motion())
        return {
            **result,
            "hardware_ready": hardware_ready,
            "hardware_fault": None if hardware_ready else node.get_hardware_fault_reason(),
            "motion_interlock": _to_jsonable(node.get_motion_interlock_status()),
        }

    # --- safety walls ------------------------------------------------------

    def get_safety_walls_status(self):
        return self.robot.get_safety_walls_status()

    def enable_safety_walls(self):
        return self.robot.enable_safety_walls()

    def disable_safety_walls(self):
        return self.robot.disable_safety_walls()

    def get_motion_passage_status(self, passage_id=None):
        return self.robot.get_motion_passage_status(passage_id)

    def set_motion_passage_closed(self, passage_id, closed):
        return self.robot.set_motion_passage_closed(passage_id, closed)

    # --- tools / workobject ------------------------------------------------

    def tool_registry(self) -> dict:
        return config.get_tool_registry_snapshot()

    def active_tool_name(self) -> str:
        return str(getattr(self.node, "active_tool_name", "TOOL_0"))

    def set_active_tool(self, tool_name: str) -> str:
        self.node.set_tool(tool_name)
        return str(getattr(self.node, "active_tool_name", tool_name))

    def update_tool_registry(self, tool_id, name=None, transform=None, persist=False) -> dict:
        snapshot = config.update_tool_registry(
            tool_id=tool_id,
            name=name,
            transform=transform,
            persist=bool(persist),
        )
        resolved = config.resolve_tool_name(tool_id)
        if getattr(self.node, "active_tool_name", None) == resolved:
            self.node.set_tool(resolved)
        return snapshot

    def workobject_registry(self) -> dict:
        return config.get_workobject_registry_snapshot()

    def active_workobject(self) -> dict:
        robot = self.robot
        getter = getattr(robot, "get_active_workobject", None)
        if callable(getter):
            return _to_jsonable(getter())
        return {
            "user_id": 0,
            "workobject_name": "WOBJ_0",
            "origin": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        }

    def set_active_workobject(self, user_id: int) -> dict:
        robot = self.robot
        setter = getattr(robot, "set_active_workobject", None)
        if not callable(setter):
            raise RuntimeError("robot backend does not support active workobject")
        setter(int(user_id))
        return self.active_workobject()

    def update_workobject_registry(self, user_id, name=None, transform=None, persist=False) -> dict:
        snapshot = config.update_workobject_registry(
            user_id=user_id,
            name=name,
            transform=transform,
            persist=bool(persist),
        )
        resolved = config.resolve_workobject_name(user_id)
        values = getattr(config, "WORKOBJECT_REGISTRY", {}).get(resolved)
        if values is not None:
            self.robot.set_workobject(WorkObject(*values), user_id=int(user_id))
        active = self.active_workobject()
        if int(active.get("user_id", -1)) == int(user_id):
            self.set_active_workobject(int(user_id))
        return snapshot

    def set_workobject(self, origin, user_id=0) -> None:
        self.robot.set_workobject(WorkObject(*origin), user_id=user_id)

    # --- validation / IO ---------------------------------------------------

    def validate_pose(
        self,
        start_position,
        target_position,
        tool=0,
        user=0,
        start_joint_state_payload=None,
    ) -> dict:
        return _to_jsonable(
            validate_pose_from_start(
                self.node,
                self.robot,
                start_position=start_position,
                target_position=target_position,
                tool=tool,
                user=user,
                start_joint_state_payload=start_joint_state_payload,
            )
        )

    def set_digital_output(self, port: int, value: int) -> int:
        return self.robot.setDigitalOutput(port, value)
