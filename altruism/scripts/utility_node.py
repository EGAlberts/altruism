#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from altruism_msgs.srv import GetNFR
from std_msgs.msg import Float64
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor


class UtilityManager(Node):

    def __init__(self):
        super().__init__('utility_manager')
        self.publisher_ = self.create_publisher(Float64, 'system_utility', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        exclusive_group = MutuallyExclusiveCallbackGroup()

        self.cli = self.create_client(GetNFR, '/get_nfr', callback_group=exclusive_group)
        self.req = GetNFR.Request()

    def timer_callback(self):        
        #I should probably make functions like these into utilities, like as members of a subclass of Node..
        self.get_logger().info("Calling NFR service client...")
        
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
        response = self.cli.call(self.req)

        self.get_logger().info('Result of it ' + str(response.nfrs_in_tree))


        weight_sum = sum([nfr.weight for nfr in response.nfrs_in_tree])

        utility_value = 0.0
        for nfr in response.nfrs_in_tree:
            utility_value += (nfr.weight/weight_sum) * nfr.metric

        
        msg = Float64()
        msg.data = utility_value
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

    


    

def main(args=None):
    rclpy.init()

    util_manage_node = UtilityManager()

    mt_executor = MultiThreadedExecutor()
    mt_executor.add_node(util_manage_node)
    
    mt_executor.spin()
   

    
    
    util_manage_node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()