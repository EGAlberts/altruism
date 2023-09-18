#!/usr/bin/env python3
import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from masced_bandits.bandit_options import initialize_arguments
from masced_bandits.bandits import init_bandit

from altruism_msgs.action import Bandit
from std_msgs.msg import Float64

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue

class BanditActionServer(Node):

    def __init__(self):
        super().__init__('bandit_action_server')
        self._action_server = ActionServer(
            self,
            Bandit,
            'bandit',
            self.execute_callback,
            callback_group=MutuallyExclusiveCallbackGroup())
        
        self.subscription = self.create_subscription(
            Float64,
            'system_utility',
            self.listener_callback,
            10, 
            callback_group=MutuallyExclusiveCallbackGroup())
        
        self.cli = self.create_client(SetParameters, '/identify_action_server/set_parameters', callback_group=MutuallyExclusiveCallbackGroup())
        self.req = SetParameters.Request()
        self.param = Parameter()
        self.param.name = 'pic_rate'


        self.req.parameters = [self.param]

        initialize_arguments([1,3,5,7], 0)
        self.ucb_instance = init_bandit(name='UCB')
        self.get_logger().info('Action server created...')


    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        temp_count = 0 #temporarily making it so that the bandit actually finishes
        do_bandit = True
        self.reward = -1
        while do_bandit:
            next_arm = self.ucb_instance.get_next_arm(self.reward)
            feedback_msg = Bandit.Feedback()
            feedback_msg.chosen_arm = str(next_arm)

            param_value = ParameterValue()
            param_value.type = 2 #Integer
            param_value.integer_value = next_arm
            self.param.value = param_value

            while not self.cli.wait_for_service(timeout_sec=1.0):
                    self.get_logger().info('set_param service not available, waiting again...')
            response = self.cli.call(self.req)

            self.get_logger().info('Param set...')

            

            goal_handle.publish_feedback(feedback_msg)
            
                
            self.reward = -1
            
            

            temp_count = temp_count+1
            time.sleep(1)
            if(temp_count == 60): do_bandit = False

            self.get_logger().info('Start waiting...')
            
            
            while self.reward == -1: #waiting for the subscriber to receive a new reward
                pass

            self.get_logger().info('...done waiting')


        goal_handle.succeed()

        result = Bandit.Result()
        result.average_reward = 0.0
        return result
    
    def listener_callback(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg.data)
        if(msg.data is not None): self.reward = msg.data


def main(args=None):
    rclpy.init(args=args)

    bandit_action_server = BanditActionServer()
    
    mt_executor = MultiThreadedExecutor()
    mt_executor.add_node(bandit_action_server)
    mt_executor.spin()

    bandit_action_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()