#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from altruism_msgs.srv import GetNFR, GetVariableParams
from rcl_interfaces.msg import Parameter
from std_msgs.msg import Float64
from altruism_msgs.msg import AdaptationState, Configuration, QRValue
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from itertools import product

class UtilityManager(Node):

    def __init__(self):
        super().__init__('utility_manager')
        self.publisher_ = self.create_publisher(AdaptationState, 'system_adaptation_state', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        exclusive_group = MutuallyExclusiveCallbackGroup()

        self.cli_nfr = self.create_client(GetNFR, '/get_nfr', callback_group=exclusive_group)
        self.cli_var = self.create_client(GetVariableParams, '/get_variable_params', callback_group=exclusive_group)

        self.req_nfr = GetNFR.Request()
        self.req_var = GetVariableParams.Request()



    def get_system_utility(self):
        self.get_logger().info("Calling NFR service client...")
        
        while not self.cli_nfr.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
        response = self.cli_nfr.call(self.req_nfr)

        self.get_logger().info('Result of it ' + str(response.nfrs_in_tree))


        weight_sum = sum([nfr.weight for nfr in response.nfrs_in_tree])

        utility_value = 0.0

        all_the_qrs = []
        for nfr in response.nfrs_in_tree:
            qr_val = QRValue()
            qr_val.name = nfr.nfr_name
            qr_val.qr_fulfilment = (nfr.weight/weight_sum) * nfr.metric
            all_the_qrs.append(qr_val)

        return all_the_qrs
    
    def get_system_vars(self):
        self.get_logger().info("Calling VariableParams service client...")
        
        while not self.cli_var.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
        response = self.cli_var.call(self.req_var)

        self.get_logger().info('Result of it ' + str(response.variables_in_tree))

        possible_configurations = []

        list_of_list_param = []
        for knob in response.variables_in_tree.variable_parameters:
            decomposed = []
            #knob is a msg of type VariableParameter is name-potential_values pair representing a thing that can change about the current state of the system.
            for pos_val in knob.possible_values:
                param = Parameter()
                param.name = knob.name
                param.value = pos_val
                decomposed.append(param)
            list_of_list_param.append(decomposed)
        
        self.get_logger().info('list of list param ' + str(list_of_list_param))

        possible_configurations = list(product(*list_of_list_param))

        #here's where you'd apply constraints to remove invalid configurations

        print(len(possible_configurations))
        print(possible_configurations)
        config_list = []

        for possible_config in possible_configurations:
            if(len(possible_config) != 0):
                config_msg = Configuration()
                config_msg.configuration_parameters = list(possible_config)
                config_list.append(config_msg)

        #The arms should consists of a list of Parameter, name value pairs of each parameter given.
        return config_list



    def timer_callback(self):        
        #I should probably make functions like these into utilities, like as members of a subclass of Node..




        
        msg = AdaptationState()
        msg.qr_values = self.get_system_utility()
        msg.system_possible_configurations = self.get_system_vars()
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)
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