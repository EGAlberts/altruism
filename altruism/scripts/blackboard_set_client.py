#!/usr/bin/env python3
import sys

from altruism_msgs.srv import SetBlackboard

import rclpy
from rclpy.node import Node


class BlackboardSetClient(Node):

    def __init__(self):
        super().__init__('minimal_set_blackboard_client_async')
        self.cli = self.create_client(SetBlackboard, '/set_blackboard')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetBlackboard.Request()

    def send_request(self):
        self.req.script_code = "the_answer:='frompython' "
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()

    minimal_client = BlackboardSetClient()
    response = minimal_client.send_request()
    minimal_client.get_logger().info('Result of it' + str(response.success))
    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()