#!/usr/bin/env python3
import sys

from altruism_msgs.srv import GetBlackboard

import rclpy
from rclpy.node import Node


class BlackboardGetClient(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(GetBlackboard, '/get_blackboard')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GetBlackboard.Request()

    def send_request(self):
        self.req.key_name = "the_answer"
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()

    minimal_client = BlackboardGetClient()
    response = minimal_client.send_request()
    minimal_client.get_logger().info('Result of it' + str(response.key_value))
    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()