#!/usr/bin/env python3
import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from altruism_msgs.action import Random
from altruism_msgs.msg import AdaptationState
from std_msgs.msg import Float64

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue
import numpy as np
class RandomActionServer(Node):

    def __init__(self):
        super().__init__('random_action_server')
        self._action_server = ActionServer(
            self,
            Random,
            'random',
            self.execute_callback,
            callback_group=MutuallyExclusiveCallbackGroup())
        
        self.subscription = self.create_subscription(
            AdaptationState,
            'system_adaptation_state',
            self.listener_callback,
            10, 
            callback_group=MutuallyExclusiveCallbackGroup())
        
        self.cli = self.create_client(SetParameters, '/identify_action_server/set_parameters', callback_group=MutuallyExclusiveCallbackGroup())
        self.req = SetParameters.Request()
        self.time_to_adapt = False
        self.get_logger().info('Random-based Action server created...')

    

    def send_set_parameters(self):
        while not self.cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('set_param service not available, waiting again...')
        response = self.cli.call(self.req)

        if(not all([res.successful for res in response.results])):
            self.get_logger().warning('One or more requests to set a parameter were unsuccessful in the Bandit, see reason(s):' + str(response.results))

        self.get_logger().info('Param set...')



    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        

        while True:
            while not self.time_to_adapt:
                self.get_logger().info('Random-based action server is waiting for the moment to adapt',throttle_duration_sec=5)
            
            
            random_config = np.random.choice(self.possible_configs)

            self.req.parameters = random_config.configuration_parameters

            self.send_set_parameters()

            feedback_msg = Random.Feedback()
            feedback_msg.chosen_adaptation = str(self.current_pic_rate)
        
            goal_handle.publish_feedback(feedback_msg)
            
            self.time_to_adapt = False

            



        # goal_handle.succeed()

        # result = Random.Result()

        # return result
    
    def listener_callback(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg.data)
        if(msg is not None):
            self.possible_configs = msg.system_possible_configurations
            self.time_to_adapt = True
        




def main(args=None):
    rclpy.init(args=args)

    random_action_server = RandomActionServer()
    
    mt_executor = MultiThreadedExecutor()
    mt_executor.add_node(random_action_server)
    mt_executor.spin()

    random_action_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()