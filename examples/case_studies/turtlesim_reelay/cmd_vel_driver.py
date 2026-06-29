#!/usr/bin/env python3
"""Publish finite turtlesim velocity traces through the ROSMonitoring proxy."""

from __future__ import annotations

import argparse
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


TRACES = {
    "safe": [(0.5, 0.0), (0.5, 0.6), (0.3, -0.6), (0.0, 0.0)],
    "unsafe_velocity": [(1.0, 0.0), (3.2, 0.0), (0.5, 0.0), (0.0, 0.0)],
    "unsafe_turn": [(0.8, 0.0), (0.8, 3.8), (0.4, 0.0), (0.0, 0.0)],
}


class Driver(Node):
    def __init__(self, topic: str, trace: list[tuple[float, float]], period: float) -> None:
        super().__init__("turtlesim_case_study_driver")
        self.publisher = self.create_publisher(Twist, topic, 10)
        self.trace = trace
        self.period = period

    def wait_for_subscriber(self, timeout: float) -> bool:
        deadline = time.time() + timeout
        while rclpy.ok() and self.publisher.get_subscription_count() == 0 and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.publisher.get_subscription_count() > 0

    def run_trace(self) -> None:
        for linear_x, angular_z in self.trace:
            message = Twist()
            message.linear.x = float(linear_x)
            message.angular.z = float(angular_z)
            self.publisher.publish(message)
            self.get_logger().info(f"published linear.x={linear_x:.2f}, angular.z={angular_z:.2f}")
            time.sleep(self.period)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/turtle1/cmd_vel_mon")
    parser.add_argument("--trace", choices=sorted(TRACES), default="safe")
    parser.add_argument("--period", type=float, default=1.0)
    parser.add_argument("--wait-for-subscriber", type=float, default=8.0)
    args = parser.parse_args()

    rclpy.init()
    node = Driver(args.topic, TRACES[args.trace], args.period)
    try:
        node.wait_for_subscriber(args.wait_for_subscriber)
        time.sleep(0.5)
        node.run_trace()
        time.sleep(0.5)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
