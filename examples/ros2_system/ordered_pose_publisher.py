#!/usr/bin/env python3
"""Publish PoseStamped messages out of source-time order."""

from __future__ import annotations

import time
import argparse

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class OrderedPosePublisher(Node):
    def __init__(self):
        super().__init__("rosmonitoring_example_ordered_pose_publisher")
        self.publisher = self.create_publisher(PoseStamped, "/ordered_pose", 10)

    def wait_for_subscriber(self, timeout: float) -> bool:
        deadline = time.time() + timeout
        while rclpy.ok() and self.publisher.get_subscription_count() == 0 and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.publisher.get_subscription_count() > 0

    def publish_pose(self, sec: int, frame_id: str) -> None:
        message = PoseStamped()
        message.header.stamp.sec = sec
        message.header.stamp.nanosec = 0
        message.header.frame_id = frame_id
        message.pose.orientation.w = 1.0
        self.publisher.publish(message)
        self.get_logger().info(f"published source stamp {sec} ({frame_id})")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--wait-for-subscriber", type=float, default=5.0)
    parser.add_argument("--settle-time", type=float, default=2.0)
    args = parser.parse_args()

    rclpy.init()
    node = OrderedPosePublisher()
    try:
        if node.wait_for_subscriber(args.wait_for_subscriber):
            time.sleep(args.settle_time)
        for sec, frame_id in [(20, "later"), (10, "earlier"), (30, "middle"), (50, "watermark")]:
            node.publish_pose(sec, frame_id)
            time.sleep(1.0)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
