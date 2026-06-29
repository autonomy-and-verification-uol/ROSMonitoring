#!/usr/bin/env python3
"""Small ROS2 publisher used by the ROSMonitoring topic examples."""

from __future__ import annotations

import argparse
import itertools
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ChatterTalker(Node):
    def __init__(self, topic: str, messages: list[str], period: float, once: bool):
        super().__init__("rosmonitoring_example_chatter_talker")
        self.publisher = self.create_publisher(String, topic, 10)
        self.once = once
        self.done = False
        self.sent = 0
        self.total = len(messages)
        self.messages = iter(messages) if once else itertools.cycle(messages)
        self.timer = self.create_timer(period, self.publish_next)

    def wait_for_subscriber(self, timeout: float) -> bool:
        deadline = time.time() + timeout
        while rclpy.ok() and self.publisher.get_subscription_count() == 0 and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.publisher.get_subscription_count() > 0

    def publish_next(self) -> None:
        try:
            value = next(self.messages)
        except StopIteration:
            self.done = True
            self.timer.cancel()
            return
        message = String()
        message.data = value
        self.publisher.publish(message)
        self.sent += 1
        self.get_logger().info(f"published {message.data!r}")
        if self.once and self.sent >= self.total:
            self.done = True
            self.timer.cancel()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/chatter_mon")
    parser.add_argument("--messages", nargs="+", default=["hello", "hello", "drop", "hello"])
    parser.add_argument("--period", type=float, default=1.0)
    parser.add_argument("--once", action="store_true", help="publish the message sequence once and exit")
    parser.add_argument("--wait-for-subscriber", type=float, default=5.0)
    parser.add_argument("--settle-time", type=float, default=2.0)
    args = parser.parse_args()

    rclpy.init()
    node = ChatterTalker(args.topic, args.messages, args.period, args.once)
    try:
        if node.wait_for_subscriber(args.wait_for_subscriber):
            time.sleep(args.settle_time)
        if args.once:
            while rclpy.ok() and not node.done:
                rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(0.2)
        else:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
