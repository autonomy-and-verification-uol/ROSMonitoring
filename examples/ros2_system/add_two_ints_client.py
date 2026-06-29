#!/usr/bin/env python3
"""Small ROS2 client for the service-filtering example."""

from __future__ import annotations

import argparse

import rclpy
from example_interfaces.srv import AddTwoInts
from rclpy.node import Node


class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__("rosmonitoring_example_add_two_ints_client")
        self.client = self.create_client(AddTwoInts, "/add_two_ints_mon")

    def call(self, a: int, b: int) -> int:
        if not self.client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("/add_two_ints_mon is not available")
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if not future.done() or future.result() is None:
            raise RuntimeError("service call did not complete")
        return int(future.result().sum)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("a", type=int)
    parser.add_argument("b", type=int)
    args = parser.parse_args()

    rclpy.init()
    node = AddTwoIntsClient()
    try:
        result = node.call(args.a, args.b)
        node.get_logger().info(f"response sum={result}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
