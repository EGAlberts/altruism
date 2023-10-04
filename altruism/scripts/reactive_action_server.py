#!/usr/bin/env python3
import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from altruism_msgs.action import Reactive
from altruism_msgs.msg import AdaptationState
from std_msgs.msg import Float64

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue

class ReactiveActionServer(Node):

    def __init__(self):
        super().__init__('reactive_action_server')
        self._action_server = ActionServer(
            self,
            Reactive,
            'reactive',
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
        self.qr_values = {}
        self.config_dict = {}
        self.prev_configurations_msg = None
        self.current_pic_rate = 3 #presumption
        self.declare_parameter('energy_high_threshold', 0.8)
        self.declare_parameter('mission_low_threshold', 0.2)

        self.energy_high_threshold = self.get_parameter('energy_high_threshold').get_parameter_value().double_value
        self.mission_low_threshold = self.get_parameter('mission_low_threshold').get_parameter_value().double_value

        self.qrs_updated = False
        self.get_logger().info('Reactive-based Action server created...')
        self.first_msg_received = False
    

    def send_set_parameters(self):
        while not self.cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('set_param service not available, waiting again...')
        response = self.cli.call(self.req)

        self.get_logger().info('Reactive is changing parameter to ' + str(self.current_pic_rate) )

        if(not all([res.successful for res in response.results])):
            self.get_logger().warning('One or more requests to set a parameter were unsuccessful in the Bandit, see reason(s):' + str(response.results))

        self.get_logger().info('Param set...')


    def increase_picture_rate(self):
        self.get_logger().info('Increasing Picture rate')
        
        param_values = list(self.config_dict.keys())

        new_param_value = min([pic_rate_value for pic_rate_value in param_values if pic_rate_value > self.current_pic_rate], default=-1)

        if(new_param_value == -1): return #nothing can be done
        
        self.req.parameters = [self.config_dict[new_param_value]]

        self.current_pic_rate = new_param_value

        self.send_set_parameters()

        


    def decrease_picture_rate(self):
        self.get_logger().info('Lowering Picture rate')

        param_values = list(self.config_dict.keys())

        new_param_value = max([pic_rate_value for pic_rate_value in param_values if pic_rate_value < self.current_pic_rate], default=-1)

        if(new_param_value == -1): return #nothing can be done
        
        self.req.parameters = [self.config_dict[new_param_value]]

        self.current_pic_rate = new_param_value

        self.send_set_parameters()

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        


        while not self.first_msg_received:
            self.get_logger().info('Reactive-based action server is waiting for an initial message',throttle_duration_sec=5)
        
        while True:
            while not self.qrs_updated:
                self.get_logger().info('Reactive-based action server is waiting for a message',throttle_duration_sec=5)

            if self.qr_values['EnergyNFR'] < self.energy_high_threshold:
                self.decrease_picture_rate()
            elif(self.qr_values['MissionNFR'] >= self.mission_low_threshold):
                self.increase_picture_rate()


            feedback_msg = Reactive.Feedback()
            feedback_msg.chosen_adaptation = str(self.current_pic_rate)
        
            goal_handle.publish_feedback(feedback_msg)

            self.qrs_updated = False
                
            

        



        # goal_handle.succeed()

        # result = Reactive.Result()

        # return result
    
    def listener_callback(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg.data)
        if(msg is not None):
            self.possible_configs = msg.system_possible_configurations
            for qr in msg.qr_values:
                self.qr_values[qr.name] = qr.qr_fulfilment
                self.qrs_updated = True

                
            if((self.first_msg_received is False) and (len(msg.system_possible_configurations) != 0)):                 
                for config in msg.system_possible_configurations:
                    self.config_dict[config.configuration_parameters[0].value.integer_value] = config.configuration_parameters[0]
                self.first_msg_received = True






def main(args=None):
    rclpy.init(args=args)

    reactive_action_server = ReactiveActionServer()
    
    mt_executor = MultiThreadedExecutor()
    mt_executor.add_node(reactive_action_server)
    mt_executor.spin()

    reactive_action_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()