#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64


class UtilityManager(Node):

    def __init__(self):
        super().__init__('utility_manager')
        self.publisher_ = self.create_publisher(Float64, 'system_utility', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Float64()
        msg.data = 5.5
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = UtilityManager()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()