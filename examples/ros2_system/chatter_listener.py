#!/usr/bin/env python3
"""Small ROS2 subscriber used to observe messages propagated by a monitor."""

from __future__ import annotations

import argparse

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ChatterListener(Node):
    def __init__(self, topic: str):
        super().__init__("rosmonitoring_example_chatter_listener")
        self.create_subscription(String, topic, self.callback, 10)

    def callback(self, message: String) -> None:
        self.get_logger().info(f"received {message.data!r}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/chatter")
    args = parser.parse_args()

    rclpy.init()
    node = ChatterListener(args.topic)
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
