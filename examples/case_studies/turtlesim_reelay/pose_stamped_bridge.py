#!/usr/bin/env python3
"""Bridge turtlesim/Pose to PoseStamped so ordered monitoring has timestamps."""

from __future__ import annotations

import math

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose


class PoseStampedBridge(Node):
    def __init__(self) -> None:
        super().__init__("turtlesim_pose_stamped_bridge")
        self.publisher = self.create_publisher(PoseStamped, "/turtle1/pose_stamped", 10)
        self.create_subscription(Pose, "/turtle1/pose", self.callback, 10)

    def callback(self, pose: Pose) -> None:
        stamped = PoseStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = "turtlesim"
        stamped.pose.position.x = float(pose.x)
        stamped.pose.position.y = float(pose.y)
        stamped.pose.orientation.z = math.sin(float(pose.theta) / 2.0)
        stamped.pose.orientation.w = math.cos(float(pose.theta) / 2.0)
        self.publisher.publish(stamped)


def main() -> None:
    rclpy.init()
    node = PoseStampedBridge()
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
