#!/usr/bin/env python3
import time
import rclpy
from rclpy.action import ActionServer, ActionClient
from rclpy.node import Node

from altruism_msgs.action import SLAM
from std_msgs.msg import Float64
from nav2_msgs.action import NavigateToPose
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from slam_toolbox.srv import SaveMap
from copy import deepcopy
from itertools import cycle

class SaveMapClient(Node):

    def __init__(self):
        super().__init__('save_map_client')
        self.cli = self.create_client(SaveMap, '/slam_toolbox/save_map')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SaveMap.Request()

    def send_request(self):
        string_msg = String(data='some_mappp')

        self.req.name = string_msg
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()

    minimal_client = SaveMapClient()
    response = minimal_client.send_request()
    minimal_client.get_logger().info(
        'Result %d' %
        (response.result))

    minimal_client.destroy_node()
    rclpy.shutdown()




if __name__ == '__main__':
    main()