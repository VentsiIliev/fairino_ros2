#!/usr/bin/env python3
"""Block until required ROS graph entities are available."""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--service", action="append", default=[])
    parser.add_argument("--topic", action="append", default=[])
    parser.add_argument("--poll-period", type=float, default=0.1)
    args = parser.parse_args()

    rclpy.init()
    node = Node("zeroerr_startup_readiness")
    required_services = set(args.service)
    required_topics = set(args.topic)
    received_topics = set()
    subscriptions = {}
    last_missing = None

    try:
        while rclpy.ok():
            services = {name for name, _ in node.get_service_names_and_types()}
            topic_types = dict(node.get_topic_names_and_types())
            for topic in required_topics - subscriptions.keys():
                types = topic_types.get(topic, [])
                if not types:
                    continue
                try:
                    msg_type = get_message(types[0])
                except (AttributeError, ImportError, ModuleNotFoundError, ValueError) as exc:
                    node.get_logger().warning(
                        f"Cannot load message type {types[0]} for {topic}: {exc}"
                    )
                    continue
                subscriptions[topic] = node.create_subscription(
                    msg_type,
                    topic,
                    lambda _msg, name=topic: received_topics.add(name),
                    qos_profile_sensor_data,
                )
            missing_services = sorted(required_services - services)
            missing_topics = sorted(required_topics - received_topics)
            missing = (tuple(missing_services), tuple(missing_topics))

            if not missing_services and not missing_topics:
                node.get_logger().info("Required ROS interfaces are ready")
                return 0

            if missing != last_missing:
                details = []
                if missing_services:
                    details.append("services=" + ",".join(missing_services))
                if missing_topics:
                    details.append("topics=" + ",".join(missing_topics))
                node.get_logger().info("Waiting for " + " ".join(details))
                last_missing = missing

            rclpy.spin_once(node, timeout_sec=args.poll_period)
            time.sleep(args.poll_period)
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 1


if __name__ == "__main__":
    sys.exit(main())
