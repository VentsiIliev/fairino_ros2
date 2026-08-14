#!/usr/bin/env python3
from __future__ import annotations

from setproctitle import setproctitle

setproctitle("zeroerr_drive_diagnostics")

import json
import time
from typing import Callable, Dict, Optional, Tuple

import rclpy
from ethercat_msgs.srv import GetSdo
from rclpy.node import Node
from std_msgs.msg import String


SdoSpec = Tuple[str, int, int, str, float]


class ZeroErrDriveDiagnostics(Node):
    """Low-rate SDO diagnostics kept out of the ros2_control PDO loop."""

    _SDO_SPECS: Tuple[SdoSpec, ...] = (
        ("following_error_actual", 0x60F4, 0, "int32", 1.0),
        ("torque_sensor", 0x3B69, 0, "int32", 0.001),
        ("motor_actual_current", 0x6078, 0, "int16", 1.0),
    )

    def __init__(self) -> None:
        super().__init__("zeroerr_drive_diagnostics")

        self.declare_parameter("master_id", 0)
        self.declare_parameter("slave_count", 6)
        self.declare_parameter("poll_period_sec", 5.0)
        self.declare_parameter("topic_name", "/zeroerr/drive_diagnostics")

        self._master_id = int(self.get_parameter("master_id").value)
        self._slave_count = int(self.get_parameter("slave_count").value)
        self._poll_period_sec = max(0.5, float(self.get_parameter("poll_period_sec").value))
        self._topic_name = str(self.get_parameter("topic_name").value)

        self._client = self.create_client(GetSdo, "ethercat_manager/get_sdo")
        self._publisher = self.create_publisher(String, self._topic_name, 10)
        self._scan_in_progress = False
        self._service_warned = False

        self.create_timer(self._poll_period_sec, self._start_scan)
        self.get_logger().info(
            f"[ZeroErrDriveDiagnostics] Publishing low-rate SDO diagnostics on {self._topic_name} "
            f"every {self._poll_period_sec:.3f}s"
        )

    def _start_scan(self) -> None:
        if self._scan_in_progress:
            return
        if not self._client.service_is_ready():
            if not self._client.wait_for_service(timeout_sec=0.05):
                if not self._service_warned:
                    self.get_logger().warning(
                        "[ZeroErrDriveDiagnostics] Waiting for ethercat_manager/get_sdo service"
                    )
                    self._service_warned = True
                return
        self._service_warned = False
        self._scan_in_progress = True
        self._scan_started_monotonic = time.monotonic()
        self._results: Dict[int, Dict[str, Optional[float]]] = {}
        self._read_next(0, 0)

    def _read_next(self, slave_position: int, spec_index: int) -> None:
        if slave_position >= self._slave_count:
            self._publish_results()
            self._scan_in_progress = False
            return
        if spec_index >= len(self._SDO_SPECS):
            self._read_next(slave_position + 1, 0)
            return

        name, index, subindex, data_type, factor = self._SDO_SPECS[spec_index]
        self._read_sdo(
            slave_position,
            index,
            subindex,
            data_type,
            lambda raw: self._after_read(slave_position, spec_index, name, raw, factor),
        )

    def _after_read(
        self,
        slave_position: int,
        spec_index: int,
        name: str,
        raw_value: Optional[int],
        factor: float,
    ) -> None:
        joint = self._results.setdefault(slave_position, {})
        joint[name] = None if raw_value is None else float(raw_value) * factor
        self._read_next(slave_position, spec_index + 1)

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
            self.get_logger().debug(
                f"[ZeroErrDriveDiagnostics] SDO read failed for slave {slave_position} "
                f"0x{index:04X}:{subindex:02X}: {exc}"
            )
            done_cb(None)
            return

        if not response.success:
            self.get_logger().debug(
                f"[ZeroErrDriveDiagnostics] SDO read failed for slave {slave_position} "
                f"0x{index:04X}:{subindex:02X}: {response.sdo_return_message}"
            )
            done_cb(None)
            return

        done_cb(int(response.sdo_return_value))

    def _publish_results(self) -> None:
        elapsed_s = time.monotonic() - self._scan_started_monotonic
        joints = []
        for slave_position in range(self._slave_count):
            data = self._results.get(slave_position, {})
            joints.append({
                "slave_position": slave_position,
                "joint_name": f"Joint_{slave_position + 1}",
                "following_error_actual": data.get("following_error_actual"),
                "torque_sensor": data.get("torque_sensor"),
                "motor_actual_current": data.get("motor_actual_current"),
            })

        msg = String()
        msg.data = json.dumps({
            "stamp_sec": self.get_clock().now().nanoseconds / 1e9,
            "scan_elapsed_s": elapsed_s,
            "source": "ethercat_manager/get_sdo",
            "joints": joints,
        }, separators=(",", ":"))
        self._publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ZeroErrDriveDiagnostics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
