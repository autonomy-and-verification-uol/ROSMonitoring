#!/usr/bin/env python3
"""Publish stamped messages on two topics out of source-time order."""

from __future__ import annotations

import argparse
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class OrderedMultiPublisher(Node):
    def __init__(self, wait_for_subscriber: float, settle_time: float):
        super().__init__("rosmonitoring_example_ordered_multi_publisher")
        self.alpha = self.create_publisher(PoseStamped, "/ordered_alpha", 10)
        self.beta = self.create_publisher(PoseStamped, "/ordered_beta", 10)
        self.wait_for_subscriber = wait_for_subscriber
        self.settle_time = settle_time

    def wait_until_ready(self) -> None:
        deadline = time.monotonic() + self.wait_for_subscriber
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.alpha.get_subscription_count() > 0 and self.beta.get_subscription_count() > 0:
                time.sleep(self.settle_time)
                return

    def publish(self, publisher, sec: int, frame_id: str) -> None:
        message = PoseStamped()
        message.header.stamp.sec = sec
        message.header.frame_id = frame_id
        message.pose.orientation.w = 1.0
        publisher.publish(message)
        self.get_logger().info(f"published {frame_id} stamp={sec}")

    def run(self) -> None:
        self.wait_until_ready()
        sequence = [
            (self.alpha, 20, "alpha-later"),
            (self.beta, 10, "beta-earlier"),
            (self.alpha, 30, "alpha-watermark"),
        ]
        for publisher, sec, frame_id in sequence:
            self.publish(publisher, sec, frame_id)
            rclpy.spin_once(self, timeout_sec=0.2)
            time.sleep(0.8)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--wait-for-subscriber", type=float, default=5.0)
    parser.add_argument("--settle-time", type=float, default=2.0)
    args = parser.parse_args()

    rclpy.init()
    node = OrderedMultiPublisher(args.wait_for_subscriber, args.settle_time)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
