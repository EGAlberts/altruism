#!/usr/bin/env python3
import sys

from altruism_msgs.srv import SetBlackboard
from sensor_msgs.msg import BatteryState
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import rclpy
from rclpy.node import Node
import time
from rclpy.executors import MultiThreadedExecutor
from collections import deque
#Essentially this is client to the setBlackboard service with some logic around it to incorporate different
#published data about the system.
import time
import threading
class SystemReflection(Node):
    
    def __init__(self):
        print("Initializing the node...")
        #self.node_name = 'system_reflection'
        super().__init__('system_reflection')
        
        timer_cb_group = MutuallyExclusiveCallbackGroup()
        exclusive_group = MutuallyExclusiveCallbackGroup()
        sub_group = MutuallyExclusiveCallbackGroup()

        self.cli = self.create_client(SetBlackboard, '/set_blackboard', callback_group=exclusive_group)
        self.subscription = self.create_subscription(BatteryState,'/battery/status',self.battery_state_cb,10, callback_group=sub_group)
        self.req = SetBlackboard.Request()

        self.time_monitor_timer = self.create_timer(2, self.do_stuff, callback_group=timer_cb_group)
        self.battery_state = None
        self.state_queue = deque(maxlen=20) #just picked a random length, can be adjusted..
        self.battery_state_lock = threading.Lock()

    def send_request(self, script):
        print("sending req")
        with self.battery_state_lock:
            while not self.cli.wait_for_service(timeout_sec=1.0):
                    self.get_logger().info('service not available, waiting again...')
            self.req.script_code = script 
            self.future = self.cli.call_async(self.req)
            # while rclpy.spin_until_future_complete(self, self.future):
            #     self.get_logger().info("Waiting for future to complete")
            # return self.future.result()
        
    
    def battery_state_cb(self,msg):
        self.state_queue.append(msg)

    def do_stuff(self):
        #print('lpp'+ str(i))
        self.get_logger().info("Processing battery state and calling service client...")
        
        script = self.process_bt_state()
        print("\nscript is " + script + "\n\n")
        if(script != ""):
            self.req.script_code = script 
            response = self.cli.call(self.req)

            self.get_logger().info('Result of it ' + str(response.success))

            
            self.battery_state = None

        

    def process_bt_state(self):
        #float32 voltage          # Voltage in Volts (Mandatory)
        #float32 temperature      # Temperature in Degrees Celsius (If unmeasured NaN)
        #float32 current          # Negative when discharging (A)  (If unmeasured NaN)
        #float32 charge           # Current charge in Ah  (If unmeasured NaN)
        #float32 capacity         # Capacity in Ah (last full capacity)  (If unmeasured NaN)
        #float32 design_capacity  # Capacity in Ah (design capacity)  (If unmeasured NaN)
        #float32 percentage       # Charge percentage on 0 to 1 range  (If unmeasured NaN)
        
        try:
            bt_state = self.state_queue[-1]
            assignment_string = ""
            fields = ["voltage", "temperature", "current", "charge", "capacity", "design_capacity", "percentage"] #paramterize?
            for field in fields:
                assignment_string+= field
                assignment_string+= ":="
                value = getattr(bt_state,field) #same as doing bt_state.voltage etc. etc.
                if type(value) == str:
                    assignment_string+= "'" + value + "'"
                else:
                    assignment_string+= str(value)
                assignment_string+= "; "
            
            assignment_string = assignment_string[:-2]
            return assignment_string
        except IndexError:
            #nothing in the queue (yet?)
            return ""

def main():
    rclpy.init()

    sys_reflec_node = SystemReflection()

    mt_executor = MultiThreadedExecutor()
    mt_executor.add_node(sys_reflec_node)
    
    mt_executor.spin()
   

    
    
    sys_reflec_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()