#!/usr/bin/env python3

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from ethercat_msgs.srv import GetSdo


ERROR_REGISTER_BITS = {
    0: "generic error",
    1: "current error",
    2: "voltage error",
    3: "temperature error",
    4: "communication error",
    5: "device-profile specific error",
}


DRIVE_ERROR_CODES = {
    0x0000: "no active drive error",
    0x2214: "motor current is over current",
    0x2250: "sum of motor three phase current exceeds the limit",
    0x2341: "U phase over current",
    0x2342: "V phase over current",
    0x2343: "W phase over current",
    0x3210: "bus voltage is overvoltage",
    0x3220: "bus voltage is undervoltage",
    0x4110: "power component temperature is too high",
    0x7121: "blocked motor rotation",
    0x7305: "load-side single-turn encoder read data is incorrect",
    0x7306: "motor-side single-turn encoder read data is incorrect",
    0x730D: "battery warning error",
    0x730F: "battery low voltage",
    0x7311: "sampled motor-end position error exceeds the limit",
    0x7314: "battery reconnection detected; reset load-side encoder to clear alarm",
    0x7315: "sampled load-side position error exceeds the limit",
    0x7350: "motor-side encoder type is not supported",
    0x7374: "multi-turn position error",
    0x7377: "reset pin error detected",
    0x737A: "load-side single-turn encoder startup error",
    0x737E: "motor-side single-turn encoder startup error",
    0x8130: "CAN heartbeat error",
    0x8400: "velocity error exceeds the limit value",
    0x8401: "motor velocity exceeds the limit value",
    0x8500: "position error exceeds the limit value",
    0xA000: "master station offline / EtherCAT communication abnormal",
    0xF004: "EtherCAT initialization error",
    0xF005: "STO function is activated",
    0xF006: "multi-turn circle count error",
    0xF008: "bus voltage below minimum allowable supply voltage (19V)",
}


@dataclass(frozen=True)
class SlaveFaultState:
    error_code: int
    error_register: int
    history_count: Optional[int]
    latest_history: Optional[int]


class ZeroErrErrorMonitor(Node):
    def __init__(self) -> None:
        super().__init__("zeroerr_error_monitor")

        self.declare_parameter("master_id", 0)
        self.declare_parameter("slave_count", 6)
        self.declare_parameter("poll_period_sec", 1.0)
        self.declare_parameter("log_zero_state_once", True)

        self._master_id = int(self.get_parameter("master_id").value)
        self._slave_count = int(self.get_parameter("slave_count").value)
        self._log_zero_state_once = bool(self.get_parameter("log_zero_state_once").value)
        period = float(self.get_parameter("poll_period_sec").value)

        self._client = self.create_client(GetSdo, "ethercat_manager/get_sdo")
        self._service_warned = False
        self._poll_in_progress = False
        self._disabled_due_to_abi_mismatch = False
        self._last_states: Dict[int, SlaveFaultState] = {}
        self._last_fault_signature: Dict[int, Tuple[int, int]] = {}
        self._zero_state_logged = False

        self.create_timer(period, self._poll_once)
        self.get_logger().info(
            f"[ZeroErrErrorMonitor] Watching 0x603F/0x1001/0x1003 for {self._slave_count} slaves "
            f"via ethercat_manager/get_sdo"
        )

    def _poll_once(self) -> None:
        if self._disabled_due_to_abi_mismatch:
            return
        if self._poll_in_progress:
            return
        if not self._client.service_is_ready():
            if not self._client.wait_for_service(timeout_sec=0.0) and not self._service_warned:
                self.get_logger().warning(
                    "[ZeroErrErrorMonitor] Waiting for ethercat_manager/get_sdo service..."
                )
                self._service_warned = True
            return

        self._service_warned = False
        self._poll_in_progress = True
        self._poll_next_slave(0)

    def _poll_next_slave(self, slave_position: int) -> None:
        if slave_position >= self._slave_count:
            self._poll_in_progress = False
            return

        self._read_fault_state(slave_position, lambda state: self._handle_state(slave_position, state))

    def _handle_state(self, slave_position: int, state: Optional[SlaveFaultState]) -> None:
        if state is None:
            self._poll_next_slave(slave_position + 1)
            return

        self._last_fault_signature[slave_position] = (state.error_code, state.error_register)
        previous = self._last_states.get(slave_position)
        if previous != state:
            self._last_states[slave_position] = state
            self._log_state_change(slave_position, previous, state)

        self._poll_next_slave(slave_position + 1)

    def _read_fault_state(self, slave_position: int, done_cb) -> None:
        self._read_sdo(slave_position, 0x603F, 0, "uint16",
                       lambda code: self._after_error_code(slave_position, code, done_cb))

    def _after_error_code(self, slave_position: int, error_code: Optional[int], done_cb) -> None:
        if error_code is None:
            done_cb(None)
            return

        self._read_sdo(slave_position, 0x1001, 0, "uint8",
                       lambda reg: self._after_error_register(slave_position, error_code, reg, done_cb))

    def _after_error_register(
        self,
        slave_position: int,
        error_code: int,
        error_register: Optional[int],
        done_cb,
    ) -> None:
        if error_register is None:
            done_cb(None)
            return

        last_signature = self._last_fault_signature.get(slave_position)
        current_signature = (error_code, error_register)
        cached_state = self._last_states.get(slave_position)
        should_refresh_history = cached_state is None or last_signature != current_signature

        if not should_refresh_history:
            done_cb(
                SlaveFaultState(
                    error_code=error_code,
                    error_register=error_register,
                    history_count=cached_state.history_count,
                    latest_history=cached_state.latest_history,
                )
            )
            return

        self._read_sdo(
            slave_position,
            0x1003,
            0,
            "uint8",
            lambda count: self._after_history_count(slave_position, error_code, error_register, count, done_cb),
        )

    def _after_history_count(
        self,
        slave_position: int,
        error_code: int,
        error_register: int,
        history_count: Optional[int],
        done_cb,
    ) -> None:
        if history_count is None:
            done_cb(SlaveFaultState(error_code, error_register, None, None))
            return

        if history_count <= 0:
            done_cb(SlaveFaultState(error_code, error_register, history_count, None))
            return

        self._read_sdo(
            slave_position,
            0x1003,
            1,
            "uint32",
            lambda latest: done_cb(SlaveFaultState(error_code, error_register, history_count, latest)),
        )

    def _read_sdo(self, slave_position: int, index: int, subindex: int, data_type: str, done_cb) -> None:
        request = GetSdo.Request()
        request.master_id = self._master_id
        request.slave_position = slave_position
        request.sdo_index = index
        request.sdo_subindex = subindex
        request.sdo_data_type = data_type

        future = self._client.call_async(request)
        future.add_done_callback(
            lambda fut: self._on_sdo_response(fut, slave_position, index, subindex, done_cb)
        )

    def _on_sdo_response(self, future, slave_position: int, index: int, subindex: int, done_cb) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warning(
                f"[ZeroErrErrorMonitor] SDO read failed for slave {slave_position} "
                f"0x{index:04X}:{subindex:02X}: {exc}"
            )
            done_cb(None)
            return

        if not response.success:
            if "ioctl() version magic is differing" in response.sdo_return_message:
                self._disabled_due_to_abi_mismatch = True
                self.get_logger().error(
                    "[ZeroErrErrorMonitor] Disabling error polling because ethercat_manager "
                    f"userspace ABI does not match the running EtherCAT master: "
                    f"{response.sdo_return_message}"
                )
                done_cb(None)
                return
            self.get_logger().warning(
                f"[ZeroErrErrorMonitor] SDO read failed for slave {slave_position} "
                f"0x{index:04X}:{subindex:02X}: {response.sdo_return_message}"
            )
            done_cb(None)
            return

        value = int(response.sdo_return_value)
        done_cb(value)

    def _log_state_change(
        self,
        slave_position: int,
        previous: Optional[SlaveFaultState],
        current: SlaveFaultState,
    ) -> None:
        prefix = f"[ZeroErrErrorMonitor] slave {slave_position}"
        error_text = self._translate_error_code(current.error_code)
        register_text = self._translate_error_register(current.error_register)
        history_text = self._translate_history(current.history_count, current.latest_history)
        has_active_fault = current.error_code != 0
        has_error_bits = current.error_register != 0

        if not has_active_fault and not has_error_bits:
            if self._log_zero_state_once and not self._zero_state_logged and previous is None:
                self.get_logger().info(
                    f"{prefix}: no active fault, error_register=0x00, {history_text}"
                )
                self._zero_state_logged = True
            elif previous is not None and (previous.error_code != 0 or previous.error_register != 0):
                self.get_logger().info(
                    f"{prefix}: fault cleared, error_code=0x0000 ({error_text}), "
                    f"error_register=0x00 ({register_text}), {history_text}"
                )
            return

        if not has_active_fault and has_error_bits:
            self.get_logger().warning(
                f"{prefix}: no active drive error, but error_register=0x{current.error_register:02X} "
                f"({register_text}), {history_text}"
            )
            return

        self.get_logger().error(
            f"{prefix}: error_code=0x{current.error_code:04X} ({error_text}), "
            f"error_register=0x{current.error_register:02X} ({register_text}), {history_text}"
        )

    def _translate_error_code(self, code: int) -> str:
        if code in DRIVE_ERROR_CODES:
            return DRIVE_ERROR_CODES[code]
        if code == 0:
            return DRIVE_ERROR_CODES[0]
        return "unknown drive error code"

    def _translate_error_register(self, value: int) -> str:
        if value == 0:
            return "no error bits set"
        labels = [label for bit, label in ERROR_REGISTER_BITS.items() if value & (1 << bit)]
        if labels:
            return ", ".join(labels)
        return "unmapped error register bits"

    def _translate_history(self, count: Optional[int], latest: Optional[int]) -> str:
        if count is None:
            return "history unavailable"
        if count == 0:
            return "history empty"
        if latest is None:
            return f"history_count={count}"
        latest_16 = latest & 0xFFFF
        latest_text = self._translate_error_code(latest_16)
        return f"history_count={count}, latest_history=0x{latest:08X} (0x{latest_16:04X}: {latest_text})"


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ZeroErrErrorMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
