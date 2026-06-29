#!/usr/bin/env python3
from __future__ import annotations

import rclpy
from example_interfaces.srv import AddTwoInts
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class AddTwoIntsServer(Node):
    def __init__(self) -> None:
        super().__init__("tutorial_add_two_ints_server")
        self.create_service(AddTwoInts, "/add_two_ints", self.callback)

    def callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f"served: {request.a} + {request.b} = {response.sum}")
        return response


def main() -> None:
    rclpy.init()
    node = AddTwoIntsServer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
