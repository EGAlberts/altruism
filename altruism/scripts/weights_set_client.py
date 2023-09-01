#!/usr/bin/env python3
import sys

from altruism_msgs.srv import SetWeights
from altruism_msgs.msg import NFR
import rclpy
from rclpy.node import Node


class SetWeightsClient(Node):

    def __init__(self):
        super().__init__('minimal_set_weights_client_async')
        self.cli = self.create_client(SetWeights, '/set_weights')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetWeights.Request()

    def send_request(self):
        name_weights = sys.argv[1:]

        for i in range(0,len(name_weights),2):
            nfr = NFR()
            nfr_name, nfr_weight = name_weights[i:i+2]
            nfr.nfr_name = nfr_name
            print(nfr_weight)
            nfr.weight = float(nfr_weight)
            self.req.nfrs_to_update.append(nfr)
    
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()
    minimal_client = SetWeightsClient()
    response = minimal_client.send_request()
    minimal_client.get_logger().info('Result of it' + str(response.success))
    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()