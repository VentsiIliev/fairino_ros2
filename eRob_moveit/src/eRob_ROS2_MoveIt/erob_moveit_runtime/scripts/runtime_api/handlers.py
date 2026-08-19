#!/usr/bin/env python3
from __future__ import annotations

from dataclasses import dataclass
import logging
import time
import traceback
from typing import Any, Callable
from motion.servo.cartesian_servo.i_cartesian_servo import (
    CartesianServoResult,
)
import config
from rest.api_support import (
    MOTION_ERROR_DESCRIPTIONS,
    parse_cartesian_servo_start_request,
    parse_cartesian_servo_update_request,
    parse_execute_ordered_motion_chain_request,
    parse_execute_path_request,
    parse_execute_sequence_request,
    parse_joint_jog_request,
    parse_jog_request,
    parse_move_linear_request,
    parse_servo_jog_start_request,
)
from runtime_gateway.base import RuntimeGateway
from runtime_gateway.local import LocalRuntimeGateway


logger = logging.getLogger("erob_moveit_runtime_api")


@dataclass(frozen=True)
class ApiResponse:
    body: dict[str, Any]
    status: int = 200


def to_jsonable(value):
    """Convert nested ROS / numpy-ish values into plain JSON-safe Python types."""
    if isinstance(value, dict):
        return {str(k): to_jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [to_jsonable(v) for v in value]
    if hasattr(value, "item") and callable(getattr(value, "item")):
        try:
            return to_jsonable(value.item())
        except Exception:
            pass
    return value


def response_error(message: str, status_code: int, **extra) -> ApiResponse:
    return ApiResponse({"success": False, "error": message, **extra}, status_code)


def http_status_for_state(state: str | None, default: int = 500) -> int:
    if state in {"HARDWARE_NOT_READY"}:
        return 503
    if state in {"UNSUPPORTED"}:
        return 501
    return default


def as_dict(value, error_message: str) -> dict:
    value = to_jsonable(value)
    if isinstance(value, dict):
        return value
    return {"success": False, "error": error_message}


def motion_error(result: int, **extra) -> ApiResponse:
    description = MOTION_ERROR_DESCRIPTIONS.get(result, f"Unknown error code {result}")
    http_status = 503 if result in (-2, -5, -12) else 409 if result in (-13, -14) else 400 if result in (-3, -11) else 500
    body = {"result": result, "success": False, "error": description}
    body.update(extra)
    body["success"] = False
    return ApiResponse(body, http_status)


class RuntimeApi:
    """Transport-neutral robot runtime API.

    HTTP, command WebSocket, and future ROS2-native transports should call this
    layer instead of duplicating validation, backend calls, and response mapping.
    """

    def __init__(
        self,
        robot_getter: Callable[[], Any],
        node_getter: Callable[[], Any],
        startup_status_getter: Callable[[], dict[str, Any]],
        runtime_state_snapshot_getter: Callable[[], dict[str, Any]],
        *,
        port: int,
        gateway=None,
    ):
        self._robot_getter = robot_getter
        self._node_getter = node_getter
        self._startup_status_getter = startup_status_getter
        self._runtime_state_snapshot_getter = runtime_state_snapshot_getter
        self._gateway = gateway
        self._port = int(port)

    def _gateway_or_local(self) -> RuntimeGateway:
        if self._gateway is not None:
            return self._gateway
        return LocalRuntimeGateway(
            robot_getter=self._robot_getter,
            node_getter=self._node_getter,
        )

    def _task_id(self):
        return self._gateway_or_local().last_submitted_task_id()

    def _motion_result(self, result: int) -> ApiResponse:
        task_id = self._task_id()
        if result > 0:
            return ApiResponse({
                "result": result,
                "success": True,
                "queued": True,
                "queue_position": result,
                "task_id": task_id,
            }, 202)
        if result == 0:
            return ApiResponse({
                "result": result,
                "success": True,
                "queued": False,
                "task_id": task_id,
            }, 200)
        return motion_error(result)

    def _require_motion_stack_ready(self) -> ApiResponse | None:
        gateway = self._gateway_or_local()
        if not gateway.is_runtime_initialized():
            return response_error("robot runtime is still starting", 503)
        if gateway.is_motion_stack_ready():
            return None
        return response_error(
            "motion stack is not ready",
            503,
            motion_stack_ready=False,
            motion_stack_fault=gateway.get_motion_stack_fault_reason(),
            startup=self._startup_status_getter(),
            runtime=self._runtime_state_snapshot_getter(),
        )

    def move_linear(self, data: dict[str, Any] | None) -> ApiResponse:
        try:
            payload = parse_move_linear_request(data)
        except ValueError as exc:
            return response_error(str(exc), 400)
        not_ready = self._require_motion_stack_ready()
        if not_ready is not None:
            return not_ready

        logger.info("Received move/linear request with data %s", data)
        result = self._gateway_or_local().move_linear(
            payload["position"],
            tool=payload["tool"],
            user=payload["user"],
            vel=payload["vel"],
            acc=payload["acc"],
            blocking=payload["blocking"],
            trajectory_optimizer=payload["trajectory_optimizer"],
        )
        return self._motion_result(result)

    def move_ptp(self, data: dict[str, Any] | None) -> ApiResponse:
        try:
            payload = parse_move_linear_request(data)
        except ValueError as exc:
            return response_error(str(exc), 400)
        not_ready = self._require_motion_stack_ready()
        if not_ready is not None:
            return not_ready

        logger.info("Received move/ptp request with data %s", data)
        result = self._gateway_or_local().move_ptp(
            payload["position"],
            tool=payload["tool"],
            user=payload["user"],
            vel=payload["vel"],
            acc=payload["acc"],
            blocking=payload["blocking"],
            trajectory_optimizer=payload["trajectory_optimizer"],
        )
        return self._motion_result(result)

    def execute_path(self, data: dict[str, Any] | None) -> ApiResponse:
        try:
            payload = parse_execute_path_request(data)
        except ValueError as exc:
            return response_error(str(exc), 400)
        not_ready = self._require_motion_stack_ready()
        if not_ready is not None:
            return not_ready

        logger.info(
            f"Executing path with {len(payload['path'])} waypoints, vel={payload['vel']}, acc={payload['acc']}"
        )
        try:
            result = self._gateway_or_local().execute_path(
                payload["path"],
                rx=payload["rx"],
                ry=payload["ry"],
                rz=payload["rz"],
                vel=payload["vel"],
                acc=payload["acc"],
                blocking=payload["blocking"],
                trajectory_optimizer=payload["trajectory_optimizer"],
                orientation_mode=payload["orientation_mode"],
            )
        except Exception as exc:
            traceback.print_exc()
            logger.error(f"Error executing path: {exc}")
            return ApiResponse({"result": -1, "success": False, "error": str(exc)}, 500)
        return self._motion_result(result)

    def execute_sequence(self, data: dict[str, Any] | None) -> ApiResponse:
        try:
            payload = parse_execute_sequence_request(data)
        except ValueError as exc:
            return response_error(str(exc), 400)
        not_ready = self._require_motion_stack_ready()
        if not_ready is not None:
            return not_ready

        logger.info(f"Executing sequence with {len(payload['segments'])} segments")
        try:
            result = self._gateway_or_local().execute_sequence(
                payload["segments"],
                tool=payload["tool"],
                user=payload["user"],
                blocking=payload["blocking"],
            )
        except Exception as exc:
            traceback.print_exc()
            logger.error(f"Error executing sequence: {exc}")
            return ApiResponse({"result": -1, "success": False, "error": str(exc)}, 500)

        task_id = self._task_id()
        if result > 0:
            return ApiResponse({
                "result": result,
                "success": True,
                "accepted": True,
                "final": False,
                "state": "QUEUED",
                "queued": True,
                "queue_position": result,
                "task_id": task_id,
                "status_ws": "/ws/execution",
                "status_ws_port": int(getattr(config, "REST_WS_EXECUTION_PORT", self._port + 2)),
                "message": "sequence queued; subscribe to /ws/execution for current_task_id, last_completed_task_id, and last_completed_result",
            }, 202)
        if result == 0:
            if payload["blocking"]:
                return ApiResponse({
                    "result": result,
                    "success": True,
                    "accepted": True,
                    "final": True,
                    "state": "COMPLETED",
                    "queued": False,
                    "task_id": task_id,
                }, 200)
            return ApiResponse({
                "result": result,
                "success": True,
                "accepted": True,
                "final": False,
                "state": "ACCEPTED_ASYNC",
                "queued": False,
                "task_id": task_id,
                "status_ws": "/ws/execution",
                "status_ws_port": int(getattr(config, "REST_WS_EXECUTION_PORT", self._port + 2)),
                "message": "sequence accepted; planning/execution completes asynchronously, subscribe to /ws/execution for final result",
            }, 202)
        return motion_error(result, accepted=False, final=True, state="REJECTED", queued=False, task_id=task_id)

    def execute_ordered_motion_chain(self, data: dict[str, Any] | None) -> ApiResponse:
        try:
            payload = parse_execute_ordered_motion_chain_request(data)
        except ValueError as exc:
            return response_error(str(exc), 400)
        not_ready = self._require_motion_stack_ready()
        if not_ready is not None:
            return not_ready

        logger.info(
            f"Executing ordered motion chain with {len(payload['segments'])} segments"
        )
        try:
            result = self._gateway_or_local().execute_ordered_motion_chain(
                segments=payload["segments"],
                tool=payload["tool"],
                user=payload["user"],
                blocking=payload["blocking"],
                trajectory_optimizer=payload["trajectory_optimizer"],
            )
        except Exception as exc:
            traceback.print_exc()
            logger.error(f"Error executing ordered motion chain: {exc}")
            return ApiResponse({"result": -1, "success": False, "error": str(exc)}, 500)
        return self._motion_result(result)

    def unwind_joint6(self, data: dict[str, Any] | None) -> ApiResponse:
        payload = data or {}
        blocking = bool(payload.get("blocking", True))
        queue_if_busy = bool(payload.get("queue_if_busy", True))
        vel = payload.get("vel")
        acc = payload.get("acc")
        try:
            vel = None if vel is None else float(vel)
            acc = None if acc is None else float(acc)
        except (TypeError, ValueError):
            return ApiResponse({
                "result": -1,
                "success": False,
                "error": "vel and acc must be numeric when provided",
            }, 400)
        not_ready = self._require_motion_stack_ready()
        if not_ready is not None:
            return not_ready

        try:
            result = self._gateway_or_local().unwind_joint6(
                blocking=blocking,
                queue_if_busy=queue_if_busy,
                vel=vel,
                acc=acc,
            )
        except Exception as exc:
            traceback.print_exc()
            logger.error(f"Error executing explicit Joint_6 unwind: {exc}")
            return ApiResponse({"result": -1, "success": False, "error": str(exc)}, 500)
        return self._motion_result(result)

    def safety_walls_enabled(self) -> ApiResponse:
        body = as_dict(self._gateway_or_local().get_safety_walls_status(), "invalid safety wall status")
        success = not body.get("error")
        return ApiResponse({**body, "success": success, "enabled": bool(body.get("enabled", False))}, 200 if success else 503)

    def safety_walls_status(self) -> ApiResponse:
        body = as_dict(self._gateway_or_local().get_safety_walls_status(), "invalid safety wall status")
        success = not body.get("error")
        return ApiResponse({**body, "success": success}, 200 if success else 503)

    def enable_safety_walls(self) -> ApiResponse:
        body = as_dict(self._gateway_or_local().enable_safety_walls(), "invalid safety wall status")
        success = bool(body.get("enabled", False)) and not body.get("error")
        return ApiResponse({**body, "success": success}, 200 if success else 500)

    def disable_safety_walls(self) -> ApiResponse:
        body = as_dict(self._gateway_or_local().disable_safety_walls(), "invalid safety wall status")
        success = not bool(body.get("enabled", True)) and not body.get("error")
        return ApiResponse({**body, "success": success}, 200 if success else 500)

    def current_position(self) -> ApiResponse:
        pos = self._gateway_or_local().get_current_position()
        if pos is None:
            return response_error("current position unavailable", 503)
        return ApiResponse({"success": True, "position": pos})

    def flange_position(self) -> ApiResponse:
        pos = self._gateway_or_local().get_current_flange_position()
        if pos is None:
            return response_error("current flange position unavailable", 503)
        return ApiResponse({"success": True, "position": pos})

    def tool_registry(self) -> ApiResponse:
        return ApiResponse({"success": True, **self._gateway_or_local().tool_registry()})

    def active_tool(self) -> ApiResponse:
        return ApiResponse({"success": True, "tool_name": self._gateway_or_local().active_tool_name()})

    def set_active_tool(self, data: dict[str, Any] | None) -> ApiResponse:
        try:
            payload = data or {}
            if "tool_id" in payload:
                tool_name = config.resolve_tool_name(payload.get("tool_id"))
            else:
                tool_name = str(payload.get("name") or payload.get("tool_name") or "").strip()
                if not tool_name:
                    raise ValueError("tool_id or tool_name is required")
            tool_name = self._gateway_or_local().set_active_tool(tool_name)
            return ApiResponse({"success": True, "tool_name": tool_name})
        except ValueError as exc:
            return ApiResponse({"success": False, "error": str(exc)}, 400)
        except Exception as exc:
            logger.error(f"REST /tool/active exception: {exc}\n{traceback.format_exc()}")
            return ApiResponse({"success": False, "error": str(exc)}, 500)

    def update_tool_registry(self, tool_id: int, data: dict[str, Any] | None) -> ApiResponse:
        try:
            payload = data or {}
            snapshot = self._gateway_or_local().update_tool_registry(
                tool_id=tool_id,
                name=payload.get("name"),
                transform=payload.get("transform"),
                persist=bool(payload.get("persist", False)),
            )
            return ApiResponse({"success": True, **snapshot})
        except ValueError as exc:
            return ApiResponse({"success": False, "error": str(exc)}, 400)
        except Exception as exc:
            logger.error(f"REST /tool/registry exception: {exc}\n{traceback.format_exc()}")
            return ApiResponse({"success": False, "error": str(exc)}, 500)

    def workobject_registry(self) -> ApiResponse:
        return ApiResponse({"success": True, **self._gateway_or_local().workobject_registry()})

    def active_workobject(self) -> ApiResponse:
        return ApiResponse({"success": True, **self._gateway_or_local().active_workobject()})

    def set_active_workobject(self, data: dict[str, Any] | None) -> ApiResponse:
        try:
            payload = data or {}
            if "user_id" not in payload:
                raise ValueError("user_id is required")
            active = self._gateway_or_local().set_active_workobject(int(payload.get("user_id")))
            return ApiResponse({"success": True, **active})
        except ValueError as exc:
            return ApiResponse({"success": False, "error": str(exc)}, 400)
        except Exception as exc:
            logger.error(f"REST /workobject/active exception: {exc}\n{traceback.format_exc()}")
            return ApiResponse({"success": False, "error": str(exc)}, 500)

    def update_workobject_registry(self, user_id: int, data: dict[str, Any] | None) -> ApiResponse:
        try:
            payload = data or {}
            snapshot = self._gateway_or_local().update_workobject_registry(
                user_id=user_id,
                name=payload.get("name"),
                transform=payload.get("transform") or payload.get("origin"),
                persist=bool(payload.get("persist", False)),
            )
            return ApiResponse({"success": True, **snapshot})
        except ValueError as exc:
            return ApiResponse({"success": False, "error": str(exc)}, 400)
        except Exception as exc:
            logger.error(f"REST /workobject/registry exception: {exc}\n{traceback.format_exc()}")
            return ApiResponse({"success": False, "error": str(exc)}, 500)

    def validate_pose(self, data: dict[str, Any] | None) -> ApiResponse:
        try:
            payload = data or {}
            target_position = payload.get("target_position") or payload.get("position")
            start_position = payload.get("start_position")
            logger.info(
                "REST /reachability/pose request: "
                f"start={start_position} target={target_position} "
                f"tool={payload.get('tool', 0)} user={payload.get('user', 0)} "
                f"has_seed_joint_state={bool(payload.get('start_joint_state'))}"
            )
            if not target_position or len(target_position) != 6:
                return response_error("Invalid target_position format", 400, reachable=False)
            if start_position is None:
                start_position = self._gateway_or_local().get_current_position()
            if not start_position or len(start_position) != 6:
                return response_error("Invalid or unavailable start_position", 400, reachable=False)

            result = self._gateway_or_local().validate_pose(
                start_position=start_position,
                target_position=target_position,
                tool=payload.get("tool", 0),
                user=payload.get("user", 0),
                start_joint_state_payload=payload.get("start_joint_state"),
            )
            http_status = 200 if result.get("reachable") else 409 if result.get("reason") == "cartesian_path_partial" else 400
            log_fn = logger.info if result.get("reachable") else logger.warning
            log_fn(f"REST /reachability/pose response: http={http_status} result={result}")
            return ApiResponse({**result, "success": bool(result.get("reachable"))}, http_status)
        except Exception as exc:
            logger.error(f"REST /reachability/pose exception: {exc}\n{traceback.format_exc()}")
            return ApiResponse({
                "success": False,
                "reachable": False,
                "reason": "rest_handler_exception",
                "error": str(exc),
            }, 500)

    def current_velocity(self) -> ApiResponse:
        vel = self._gateway_or_local().get_current_velocity()
        if vel is None:
            return response_error("current velocity unavailable", 503)
        return ApiResponse({"success": True, "velocity": vel})

    def stop_motion(self) -> ApiResponse:
        return ApiResponse(self._gateway_or_local().stop_motion())

    def set_workobject(self, data: dict[str, Any] | None) -> ApiResponse:
        payload = data or {}
        origin = payload.get("origin")
        if not origin or len(origin) != 6:
            return response_error("Invalid origin format", 400)
        try:
            self._gateway_or_local().set_workobject(origin, user_id=payload.get("user_id", 0))
        except Exception as exc:
            logger.error(f"Workobject endpoint error: {exc}")
            return response_error(str(exc), 500)
        return ApiResponse({"success": True, "origin": origin, "user_id": payload.get("user_id", 0)})

    def status(self) -> ApiResponse:
        return ApiResponse(self._gateway_or_local().status())

    def ordered_motion_chain_status(self) -> ApiResponse:
        status = self._gateway_or_local().ordered_motion_chain_status()
        if status.get("supported") is False:
            return ApiResponse({"success": True, "supported": False, "active": False})
        success = not status.get("error")
        return ApiResponse({**status, "success": success}, 200 if success else 500)

    def state_snapshot(self) -> ApiResponse:
        return ApiResponse(self._gateway_or_local().state_snapshot())

    def state_kinematics(self) -> ApiResponse:
        result = self._gateway_or_local().state_kinematics()
        if result.get("error") is not None:
            return ApiResponse(result, 503)
        unavailable = result.get("unavailable_fields", [])
        return ApiResponse(result, 200 if not unavailable else 206)

    def jog(self, data: dict[str, Any] | None) -> ApiResponse:
        try:
            payload = parse_jog_request(data)
            not_ready = self._require_motion_stack_ready()
            if not_ready is not None:
                return not_ready
            result = self._gateway_or_local().jog(
                payload["axis"],
                payload["direction"],
                payload["step"],
                payload["vel"],
                payload["acc"],
                frame=payload["frame"],
                tool=payload["tool"],
                user=payload["user"],
            )
            if result == 0:
                return ApiResponse({"result": result, "success": True})
            return motion_error(result)
        except ValueError as exc:
            return ApiResponse({"result": -1, "success": False, "error": str(exc)}, 400)
        except Exception as exc:
            logger.error(f"Jog endpoint error: {exc}")
            return ApiResponse({"result": -1, "success": False, "error": str(exc)}, 500)

    def joint_jog(self, data: dict[str, Any] | None) -> ApiResponse:
        try:
            payload = parse_joint_jog_request(data)
            not_ready = self._require_motion_stack_ready()
            if not_ready is not None:
                return not_ready
            result = self._gateway_or_local().joint_jog(
                payload["joint"],
                payload["direction"],
                payload["step"],
                payload["vel"],
                payload["acc"],
                blocking=payload["blocking"],
            )
            if result == 0:
                return ApiResponse({"result": result, "success": True})
            return motion_error(result)
        except ValueError as exc:
            return ApiResponse({"result": -1, "success": False, "error": str(exc)}, 400)
        except Exception as exc:
            logger.error(f"Joint jog endpoint error: {exc}")
            return ApiResponse({"result": -1, "success": False, "error": str(exc)}, 500)

    def servo_jog_start(self, data: dict[str, Any] | None) -> ApiResponse:
        try:
            payload = parse_servo_jog_start_request(data)
            not_ready = self._require_motion_stack_ready()
            if not_ready is not None:
                return not_ready
            result = self._gateway_or_local().servo_jog_start(
                payload["axis"],
                payload["direction"],
                payload["vel"],
                payload["acc"],
                frame=payload["frame"],
                tool=payload["tool"],
                user=payload["user"],
                linear_mm_s=payload["linear_mm_s"],
                angular_deg_s=payload["angular_deg_s"],
            )
            if result == 0:
                return ApiResponse({"result": result, "success": True, "state": "running"})
            return motion_error(result)
        except ValueError as exc:
            return ApiResponse({"result": -1, "success": False, "error": str(exc)}, 400)
        except Exception as exc:
            logger.error(f"ServoJog start endpoint error: {exc}")
            return ApiResponse({"result": -1, "success": False, "error": str(exc)}, 500)

    def servo_jog_stop(self) -> ApiResponse:
        try:
            result = self._gateway_or_local().servo_jog_stop()
            if result == 0:
                return ApiResponse({"result": result, "success": True, "state": "stopped"})
            return motion_error(result)
        except Exception as exc:
            logger.error(f"ServoJog stop endpoint error: {exc}")
            return ApiResponse({"result": -1, "success": False, "error": str(exc)}, 500)

    def set_digital_output(self, data: dict[str, Any] | None) -> ApiResponse:
        try:
            payload = data or {}
            port = int(payload["port"])
            value = int(payload["value"])
        except KeyError as exc:
            return ApiResponse({"result": -1, "success": False, "error": f"Missing field: {exc.args[0]}"}, 400)
        except (TypeError, ValueError):
            return ApiResponse({"result": -1, "success": False, "error": "port and value must be integers"}, 400)

        if port < 0:
            return ApiResponse({"result": -1, "success": False, "error": "port must be >= 0"}, 400)
        if value not in (0, 1):
            return ApiResponse({"result": -1, "success": False, "error": "value must be 0 or 1"}, 400)

        try:
            result = self._gateway_or_local().set_digital_output(port, value)
            if result == 0:
                return ApiResponse({"result": 0, "success": True, "port": port, "value": value})
            return ApiResponse({"result": result, "success": False, "port": port, "value": value}, 500)
        except Exception as exc:
            logger.error(f"Digital output endpoint error: {exc}")
            return ApiResponse({"result": -1, "success": False, "error": str(exc)}, 500)

    def drive_command_response(self, command_result: dict, desired_enabled: bool) -> ApiResponse:
        command_result = to_jsonable(command_result)
        if not isinstance(command_result, dict):
            return ApiResponse({
                "success": False,
                "command_accepted": False,
                "desired_enabled": bool(desired_enabled),
                "error": f"unexpected drive command result type: {type(command_result).__name__}",
            }, 500)
        command_accepted = bool(command_result.get("success", False))
        if command_result.get("state") == "NOT_REQUIRED_FAKE_HARDWARE":
            return ApiResponse({
                **command_result,
                "success": True,
                "command_accepted": True,
                "desired_enabled": bool(desired_enabled),
                "verified": True,
                "verification_skipped": True,
                "message": "Drive enable verification is not required in fake hardware mode",
            }, 200)
        verify_timeout_s = max(float(getattr(config, "STARTUP_AUTO_ENABLE_DRIVES_VERIFY_TIMEOUT_S", 5.0)), 0.1)
        verify_deadline = time.monotonic() + verify_timeout_s
        drive_status = self._gateway_or_local().drive_operation_status()

        while command_accepted and time.monotonic() < verify_deadline:
            actual_enabled = bool(drive_status.get("actual_enabled", False))
            requested_enabled = bool(drive_status.get("requested_enabled", False))
            if desired_enabled and actual_enabled and requested_enabled:
                break
            if not desired_enabled and not actual_enabled and not requested_enabled:
                break
            time.sleep(0.05)
            drive_status = self._gateway_or_local().drive_operation_status()
            if not drive_status.get("success", True):
                break

        actual_enabled = bool(drive_status.get("actual_enabled", False))
        requested_enabled = bool(drive_status.get("requested_enabled", False))
        verified = actual_enabled and requested_enabled if desired_enabled else not actual_enabled and not requested_enabled
        response = {
            **drive_status,
            "success": bool(command_accepted and verified),
            "command_accepted": command_accepted,
            "desired_enabled": bool(desired_enabled),
            "request": command_result,
        }
        if response["success"]:
            return ApiResponse(response)
        if not command_accepted:
            response["error"] = str(command_result.get("error") or "drive command was rejected")
            return ApiResponse(response, http_status_for_state(command_result.get("state")))
        response["error"] = (
            "drive enable command accepted, but drives are not operation_enabled"
            if desired_enabled
            else "drive disable command accepted, but drives still report operation_enabled"
        )
        return ApiResponse(response, 202)

    def enable_drive_operation(self) -> ApiResponse:
        try:
            result = self._gateway_or_local().set_drive_operation_enabled(True)
            return self.drive_command_response(result, desired_enabled=True)
        except Exception as exc:
            logger.error(f"Drive enable endpoint error: {exc}")
            logger.exception("Drive enable endpoint error")
            return ApiResponse({"success": False, "error": str(exc)}, 500)

    def disable_drive_operation(self) -> ApiResponse:
        try:
            result = self._gateway_or_local().set_drive_operation_enabled(False)
            return self.drive_command_response(result, desired_enabled=False)
        except Exception as exc:
            logger.error(f"Drive disable endpoint error: {exc}")
            logger.exception("Drive disable endpoint error")
            return ApiResponse({"success": False, "error": str(exc)}, 500)

    def drive_operation_status(self) -> ApiResponse:
        try:
            return ApiResponse(self._gateway_or_local().drive_operation_status())
        except Exception as exc:
            logger.error(f"Drive status endpoint error: {exc}")
            logger.exception("Drive status endpoint error")
            return ApiResponse({"success": False, "error": str(exc)}, 500)

    def motion_interlock_status(self) -> ApiResponse:
        return ApiResponse({"success": True, **self._gateway_or_local().motion_interlock_status()})

    def reset_motion_interlock(self) -> ApiResponse:
        return ApiResponse(self._gateway_or_local().reset_motion_interlock())

    def _cartesian_servo_result(
            self,
            result: CartesianServoResult,
    ) -> ApiResponse:

        status = self._gateway_or_local().cartesian_servo_status()

        status_body = None
        if status is not None:
            status_body = {
                "state": status.state.value,
                "frame": status.frame.value if status.frame else None,
                "tool": status.tool,
                "command": (
                    {
                        "linear_mm_s": list(status.command.linear_mm_s),
                        "angular_deg_s": list(status.command.angular_deg_s),
                    }
                    if status.command is not None
                    else None
                ),
                "error": status.error,
            }

        body = {
            "success": result == CartesianServoResult.OK,
            "result": result.value,
            "servo": status_body,
        }

        if result == CartesianServoResult.OK:
            return ApiResponse(body, 200)

        if result in {
            CartesianServoResult.ALREADY_RUNNING,
            CartesianServoResult.NOT_STARTED,
        }:
            return ApiResponse(body, 409)

        if result in {
            CartesianServoResult.INVALID_FRAME,
            CartesianServoResult.INVALID_TOOL,
            CartesianServoResult.INVALID_COMMAND,
        }:
            return ApiResponse(body, 400)

        return ApiResponse(body, 500)

    def cartesian_servo_start(
        self,
        data: dict[str, Any] | None,
    ) -> ApiResponse:
        try:
            payload = parse_cartesian_servo_start_request(data)
        except ValueError as exc:
            return response_error(str(exc), 400)

        not_ready = self._require_motion_stack_ready()
        if not_ready is not None:
            return not_ready

        gateway = self._gateway_or_local()
        if not gateway.cartesian_servo_available():
            return response_error(
                "Cartesian Servo is not available",
                503,
            )

        logger.info(
            "Received Cartesian Servo start frame=%s tool=%s user=%s",
            payload["frame"].value,
            payload["tool"],
            payload["user"],
        )

        try:
            result = gateway.cartesian_servo_start(
                frame=payload["frame"],
                tool=payload["tool"],
                user=payload["user"],
            )
        except Exception as exc:
            logger.exception("Cartesian Servo start failed")
            return response_error(str(exc), 500)

        return self._cartesian_servo_result(result)

    def cartesian_servo_update(
        self,
        data: dict[str, Any] | None,
    ) -> ApiResponse:
        try:
            payload = parse_cartesian_servo_update_request(data)
        except ValueError as exc:
            return response_error(str(exc), 400)

        gateway = self._gateway_or_local()
        if not gateway.cartesian_servo_available():
            return response_error(
                "Cartesian Servo is not available",
                503,
            )

        try:
            result = gateway.cartesian_servo_update(
                linear_mm_s=payload["linear_mm_s"],
                angular_deg_s=payload["angular_deg_s"],
            )
        except Exception as exc:
            logger.exception("Cartesian Servo update failed")
            return response_error(str(exc), 500)

        return self._cartesian_servo_result(result)

    def cartesian_servo_stop(self) -> ApiResponse:
        gateway = self._gateway_or_local()
        if not gateway.cartesian_servo_available():
            return response_error(
                "Cartesian Servo is not available",
                503,
            )

        logger.info("Received Cartesian Servo stop")

        try:
            result = gateway.cartesian_servo_stop()
        except Exception as exc:
            logger.exception("Cartesian Servo stop failed")
            return response_error(str(exc), 500)

        return self._cartesian_servo_result(result)
