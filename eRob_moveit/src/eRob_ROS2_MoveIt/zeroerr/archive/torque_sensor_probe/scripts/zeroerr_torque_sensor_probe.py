#!/usr/bin/env python3

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Dict, Optional

import rclpy
from ethercat_msgs.srv import GetSdo
from rclpy.node import Node


@dataclass(frozen=True)
class ProbeResult:
    dual_encoder_difference: Optional[int]
    torque_sensor_mnm: Optional[int]
    torque_sensor_ratio: Optional[int]


class ZeroErrTorqueSensorProbe(Node):
    def __init__(self) -> None:
        super().__init__("zeroerr_torque_sensor_probe")

        self.declare_parameter("master_id", 0)
        self.declare_parameter("slave_count", 6)

        self._master_id = int(self.get_parameter("master_id").value)
        self._slave_count = int(self.get_parameter("slave_count").value)
        self._client = self.create_client(GetSdo, "ethercat_manager/get_sdo")
        self._service_warned = False
        self._results: Dict[int, ProbeResult] = {}

    def run(self) -> None:
        if not self._client.service_is_ready():
            if not self._client.wait_for_service(timeout_sec=2.0):
                self.get_logger().error(
                    "[ZeroErrTorqueSensorProbe] ethercat_manager/get_sdo service is not available"
                )
                return
        self.get_logger().info(
            "[ZeroErrTorqueSensorProbe] Reading VTS objects "
            "0x2241:00, 0x3B69:00, 0x3B6A:00"
        )
        self._probe_slave(0)

    def _probe_slave(self, slave_position: int) -> None:
        if slave_position >= self._slave_count:
            self._log_summary()
            return
        self._read_sdo(
            slave_position,
            0x2241,
            0,
            "int32",
            lambda dual: self._after_dual_encoder(slave_position, dual),
        )

    def _after_dual_encoder(self, slave_position: int, dual_encoder_difference: Optional[int]) -> None:
        self._read_sdo(
            slave_position,
            0x3B69,
            0,
            "int32",
            lambda torque: self._after_torque_sensor(slave_position, dual_encoder_difference, torque),
        )

    def _after_torque_sensor(
        self,
        slave_position: int,
        dual_encoder_difference: Optional[int],
        torque_sensor_mnm: Optional[int],
    ) -> None:
        self._read_sdo(
            slave_position,
            0x3B6A,
            0,
            "uint16",
            lambda ratio: self._after_torque_ratio(
                slave_position,
                dual_encoder_difference,
                torque_sensor_mnm,
                ratio,
            ),
        )

    def _after_torque_ratio(
        self,
        slave_position: int,
        dual_encoder_difference: Optional[int],
        torque_sensor_mnm: Optional[int],
        torque_sensor_ratio: Optional[int],
    ) -> None:
        self._results[slave_position] = ProbeResult(
            dual_encoder_difference=dual_encoder_difference,
            torque_sensor_mnm=torque_sensor_mnm,
            torque_sensor_ratio=torque_sensor_ratio,
        )
        self._probe_slave(slave_position + 1)

    def _read_sdo(
        self,
        slave_position: int,
        index: int,
        subindex: int,
        data_type: str,
        done_cb: Callable[[Optional[int]], None],
    ) -> None:
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

    def _on_sdo_response(
        self,
        future,
        slave_position: int,
        index: int,
        subindex: int,
        done_cb: Callable[[Optional[int]], None],
    ) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warning(
                f"[ZeroErrTorqueSensorProbe] SDO read failed for slave {slave_position} "
                f"0x{index:04X}:{subindex:02X}: {exc}"
            )
            done_cb(None)
            return

        if not response.success:
            self.get_logger().warning(
                f"[ZeroErrTorqueSensorProbe] SDO read failed for slave {slave_position} "
                f"0x{index:04X}:{subindex:02X}: {response.sdo_return_message}"
            )
            done_cb(None)
            return

        done_cb(int(response.sdo_return_value))

    def _log_summary(self) -> None:
        for slave_position in range(self._slave_count):
            result = self._results.get(
                slave_position,
                ProbeResult(None, None, None),
            )
            self.get_logger().info(
                f"[ZeroErrTorqueSensorProbe] slave {slave_position}: "
                f"dual_encoder_difference={self._fmt(result.dual_encoder_difference)} "
                f"torque_sensor_mNm={self._fmt(result.torque_sensor_mnm)} "
                f"torque_sensor_Nm={self._fmt_nm(result.torque_sensor_mnm)} "
                f"torque_sensor_ratio={self._fmt(result.torque_sensor_ratio)}"
            )
        rclpy.shutdown()

    def _fmt(self, value: Optional[int]) -> str:
        return "NA" if value is None else str(value)

    def _fmt_nm(self, value_mnm: Optional[int]) -> str:
        if value_mnm is None:
            return "NA"
        return f"{value_mnm / 1000.0:.3f}"


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ZeroErrTorqueSensorProbe()
    try:
        node.run()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
