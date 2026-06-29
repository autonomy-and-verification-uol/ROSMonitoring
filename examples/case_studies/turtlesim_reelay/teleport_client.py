#!/usr/bin/env python3
"""Call turtlesim TeleportAbsolute through the ROSMonitoring service proxy."""

from __future__ import annotations

import argparse

import rclpy
from rclpy.node import Node
from turtlesim.srv import TeleportAbsolute


class TeleportClient(Node):
    def __init__(self, service: str) -> None:
        super().__init__("turtlesim_case_study_teleport_client")
        self.client = self.create_client(TeleportAbsolute, service)

    def call(self, x: float, y: float, theta: float) -> None:
        if not self.client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("service is not available")
        request = TeleportAbsolute.Request()
        request.x = float(x)
        request.y = float(y)
        request.theta = float(theta)
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if not future.done():
            raise RuntimeError("service call timed out")
        if future.exception() is not None:
            raise future.exception()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("x", type=float)
    parser.add_argument("y", type=float)
    parser.add_argument("theta", type=float, nargs="?", default=0.0)
    parser.add_argument("--service", default="/turtle1/teleport_absolute_mon")
    args = parser.parse_args()

    rclpy.init()
    node = TeleportClient(args.service)
    try:
        node.call(args.x, args.y, args.theta)
        node.get_logger().info(f"teleport request completed: x={args.x:.2f}, y={args.y:.2f}, theta={args.theta:.2f}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
