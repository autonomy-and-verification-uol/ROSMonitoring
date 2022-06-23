import rclpy
from rclpy.node import Node

from rosmonitoring_interfaces.msg import MonitorError


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            MonitorError,                                              # CHANGE
            'testmonerror',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
            self.get_logger().info('I heard: "{0}, {1}, {2}, {3}"'.format(msg.m_topic,msg.m_content,msg.m_property,msg.m_time))


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()