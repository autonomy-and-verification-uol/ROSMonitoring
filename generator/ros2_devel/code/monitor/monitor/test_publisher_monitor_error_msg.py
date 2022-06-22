import rclpy
from rclpy.node import Node

from rosmonitoring_interfaces.msg import MonitorError


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(MonitorError, 'testmonerror', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = MonitorError()
        msg.m_topic='test'
        msg.m_content='Hello World: %d' % self.i
        msg.m_property='F "Hello World"'
        msg.m_time = float(self.get_clock().now().to_msg().sec)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "{0}, {1}, {2}, {3}"'.format(msg.m_topic,msg.m_content,msg.m_property,msg.m_time))
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()