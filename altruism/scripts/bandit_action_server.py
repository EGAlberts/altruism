#!/usr/bin/env python3
import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from masced_bandits.bandit_options import initialize_arguments, bandit_args
from masced_bandits.bandits import init_bandit
from masced_bandits.utilities import truncate


from altruism_msgs.action import Bandit
from altruism_msgs.msg import AdaptationState
from std_msgs.msg import Float64
import sys
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue
import csv
import os
import numpy as np

HYPERPARAM_PARAM = "hyperparameters"

session_names = [
    "Quantum",
    "Nebula",
    "Odyssey",
    "Cipher",
    "Zenith",
    "Synapse",
    "Apex",
    "Cosmos",
    "Genesis",
    "Spectrum",
    "Infinity",
    "Aegis",
    "Velocity",
    "Orion",
    "Phoenix",
    "Catalyst",
    "Elysium",
    "Helix",
    "Serenity",
    "Pinnacle",
    "Eclipse",
    "Pandora",
    "Polaris",
    "Nova",
    "Aether",
    "Metropolis",
    "Aurora",
    "Equinox",
    "Radiance",
    "Celestial",
    "Paragon",
    "Quasar",
    "Epoch",
    "Luminary",
    "QuantumCore",
    "Nebulous",
    "Ascendant",
    "Zenithal",
    "Heliosphere",
    "Astrolabe",
    "Empyrean",
    "Utopia",
    "Ethereal",
    "Synthwave",
    "Solstice",
    "Stardust",
    "Apexia",
    "Temporal",
    "AstraNova",
    "Perigee"
]




chosen_arms = []





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
            AdaptationState,
            'system_adaptation_state',
            self.listener_callback,
            10, 
            callback_group=MutuallyExclusiveCallbackGroup())
        

        self.declare_parameter(HYPERPARAM_PARAM,[""])
        self.hyper_parameters = self.get_parameter(HYPERPARAM_PARAM).get_parameter_value().string_array_value
        
        self.session_name = np.random.choice(session_names)
        # self.cli = self.create_client(SetParameters, '/identify_action_server/set_parameters', callback_group=MutuallyExclusiveCallbackGroup())
        self.req = SetParameters.Request()
        
        csv_file = 'bandit_report.csv'
        file = open(csv_file, mode='a', newline='')
        self.writer = csv.writer(file)

        file_exists = os.path.isfile(csv_file)

        if not file_exists:
            self.writer.writerow(['Session_Name', 'Timestamp', 'Adaptation Chosen', 'Reward', 'Bounds']) 

        file.close()
        bandit_args["dynamic_bounds"] = True
        bandit_args["bounds"] = [0,0.000000000001]

        print(bandit_args["bounds"])

        self.configuration_dict = {}
        self.prev_configurations_msg = None
        self.arm_change_flag = False
        self.get_logger().info('Action server created...')
        self.first_msg_received = False
        self.hyper_param_kwargs = {}
        self.qr_values = []
        self.set_parameter_client_dict = {}

    def create_set_param_client(self, node_name):
        self.set_parameter_client_dict[node_name] = self.create_client(SetParameters, '/' + node_name + '/set_parameters', callback_group=MutuallyExclusiveCallbackGroup())

    def set_parameters_of_node(self, node_name):
        if node_name not in self.set_parameter_client_dict: 
            self.create_set_param_client(node_name)

        client = self.set_parameter_client_dict[node_name]

        while not client.wait_for_service(timeout_sec=1.0):
                    self.get_logger().info('set_param service not available, waiting again...')
        
        response = client.call(self.req)
        
        if(not all([res.successful for res in response.results])):
            self.get_logger().warning('One or more requests to set a parameter were unsuccessful in the Bandit, see reason(s):' + str(response.results))
        
    



    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        while not self.first_msg_received:
            self.get_logger().info('Bandit action server is waiting for an initial message ' + str(bandit_args["bounds"]),throttle_duration_sec=5)
        

        self.get_logger().info(str(list(self.configuration_dict.keys())))


        for i in range(0,len(self.hyper_parameters),2):
            self.hyper_param_kwargs[self.hyper_parameters[i]] = self.hyper_parameters[i+1]
        #since python 3.7 the order given by .keys() is guaranteed to be insertion order, this means any new arms should be added added to a len+1 index in the dictionary and not get confused for others.
        bandit_args["arms"] = list(self.configuration_dict.keys())
        bandit_args["initial_configuration"] = bandit_args["arms"][0]
        #initialize_arguments(list(self.configuration_dict.keys()), 0, bounds = (0,0.0001), dynamic_bounds=True) #this 0 is an assumption that can only be resolved by giving the initial configuration to this node somehow.

        
        self.bandit_instance = init_bandit(name=str(goal_handle.request.bandit_name), **self.hyper_param_kwargs)


        temp_count = 0 #temporarily making it so that the bandit actually finishes
        do_bandit = True
        while do_bandit:

            if(self.arm_change_flag):
                self.bandit_instance.arms = list(self.configuration_dict.keys()) #The chosen bandit might not properly support the change in arms.
                self.arm_change_flag = False
            next_arm = self.bandit_instance.get_next_arm(self.reward)
            feedback_msg = Bandit.Feedback()
            feedback_msg.chosen_arm = str(next_arm)

            self.get_logger().info("Bandit adapting to " + str(next_arm))

            file = open('bandit_report.csv', mode='a', newline='')
            writer = csv.writer(file)
            row = [self.session_name, str(time.time()), str(next_arm), str(self.reward), str(bandit_args["bounds"]), str(self.qr_values), str(goal_handle.request.bandit_name), str(self.hyper_param_kwargs)]
            writer.writerow(row)
            file.close()
            self.get_logger().info("Wrote row " + str(row))

            
            self.req.parameters = self.configuration_dict[next_arm].configuration_parameters


            self.set_parameters_of_node(self.configuration_dict[next_arm].node_name)

          
            
            #self.get_logger().info('Param set...')

            

            goal_handle.publish_feedback(feedback_msg)
            
                
            self.reward = -1
            

            #self.get_logger().info('Start waiting...')
            
            
            while self.reward == -1: #waiting for the subscriber to receive a new reward
                feedback_msg = Bandit.Feedback()
                goal_handle.publish_feedback(feedback_msg)
                self.get_logger().info("Bandit waiting for next reward ", throttle_duration_sec=4)


            self.get_logger().info('...done waiting')


        goal_handle.succeed()

        result = Bandit.Result()
        result.average_reward = 0.0
        return result
    
    def listener_callback(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg.data)
        if(msg is not None):
            self.reward = 0 
            #for qr in msg.qr_values: self.reward+=qr.qr_fulfilment 
            self.qr_values = [qr.qr_fulfilment for qr in msg.qr_values ]
            qr_product =  np.prod(self.qr_values)

            self.reward = qr_product + ((np.sqrt(1-qr_product) - (1 -  qr_product)) * 4)
            
            self.get_logger().info('Reward and bounds before truncating ' + str(self.reward) + str(bandit_args["bounds"]))

            self.reward, _, _ = truncate(self.reward)
            
            self.possible_configs = msg.system_possible_configurations

            if(self.prev_configurations_msg != msg.system_possible_configurations):
                #New configurations have been added OR its the first time.
                # self.get_logger().info('Hashable thing happened...')

                self.make_hashable()

                self.prev_configurations_msg = msg.system_possible_configurations
                
                self.arm_change_flag = True
                
            #else:
                #self.get_logger().info('Hashable thing didnt happen...',throttle_duration_sec=10)
                

            if((self.first_msg_received is False) and (len(msg.system_possible_configurations) != 0)): 
                # self.get_logger().info('Supposedly wasnt empty..'+ str(msg.system_possible_configurations))
                
                
                
                self.first_msg_received = True


    def make_hashable(self):
        for config_msg in self.possible_configs:
            string_repr = " ".join([str((param.name, param.value)) for param in config_msg.configuration_parameters])
            if string_repr not in self.configuration_dict:
                self.configuration_dict[string_repr] = config_msg
        





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