#!/usr/bin/env python3
import time
import rclpy
from rclpy.action import ActionServer, ActionClient
from rclpy.node import Node

from altruism_msgs.action import Identify
from std_msgs.msg import Float64
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from darknet_ros_msgs.action import CheckForObjects
from copy import deepcopy
from itertools import cycle


class IdentifyActionServer(Node):

    def __init__(self):
        super().__init__('identify_action_server')
        self._action_server = ActionServer(
            self,
            Identify,
            'identify',
            self.execute_callback,
            callback_group=MutuallyExclusiveCallbackGroup())
        
        self.check_obj_acclient = ActionClient(self, CheckForObjects, 'checkForObjectsActionName', callback_group = MutuallyExclusiveCallbackGroup())
        print(type(self.check_obj_acclient))
        self.subscription = self.create_subscription(Image,'/camera/image_raw',self.tb_image_cb,10, callback_group = MutuallyExclusiveCallbackGroup())
        self.tb_image = None
        self.counter = 0
        self.prev_done = True
        self.ID_fdback_msg = Identify.Feedback()
        self.ID_fdback_msg.obj_idd.object_names = []
        self.ID_fdback_msg.obj_idd.probabilities = []
        self.ID_fdback_msg.obj_idd.object_detected = False
        #bool object_detected
        #string object_name
        #float32 probability
        self.get_logger().info('ID Action server created...')

    def tb_image_cb(self,msg):
        self.tb_image = msg
        self.counter+=1
        if(self.counter % 1000 == 0): self.get_logger().info('1000th Msg Received!!')
        if self.counter > 200000: self.counter = 0


    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        self.get_logger().info('Its me, hi, Im a ID action server')

        while True:
            print("gonna wait 10 seconds now and then try to send the image to identify")
            time.sleep(10)        


            
            self.ID_fdback_msg.obj_idd.stamp = self.get_clock().now().to_msg()
            goal_handle.publish_feedback(self.ID_fdback_msg)
            if(self.prev_done is True):
                self.prev_done = False
                self.send_goal()
            else:
                self.get_logger().info("Still procesing prev image")

    def send_goal(self):
        #ros2 topic pub /goal_pose geometry_msgs/PoseStamped "{header: {stamp: {sec: 0}, frame_id: 'map'}, pose: {position: {x: 1.9, y: 0.7, z: 0.0}, orientation: {w: 1.0}}}"

        self.get_logger().info("Sending the goal")

        """Send a `CheckForObjects` action request."""
        self.get_logger().debug("Waiting for 'CheckForObjects' action server")
        while not self.check_obj_acclient.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("'CheckForObjects' action server was not available, will try again")
            

        goal_msg = CheckForObjects.Goal()
        goal_msg.id = 1
        while self.tb_image is None:
            self.get_logger().info("No image received yet to send, waiting...")
        #self.get_logger().info("Uhh:" + str(type(self.tb_image)))

        goal_msg.image = self.tb_image

        send_goal_future = self.check_obj_acclient.send_goal_async(goal_msg, self._feedbackCallback)
        
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future,timeout_sec=20)
        self.ID_fdback_msg.obj_idd.object_names = []
        self.ID_fdback_msg.obj_idd.probabilities = []
        self.ID_fdback_msg.obj_idd.object_detected = False
        result = get_result_future.result().result
        if(len(result.bounding_boxes.bounding_boxes) > 0):
            self.ID_fdback_msg.obj_idd.object_detected = True
            
            for box_index, box in enumerate(result.bounding_boxes.bounding_boxes):
                self.ID_fdback_msg.obj_idd.object_names.append(box.class_id)
                self.ID_fdback_msg.obj_idd.probabilities.append(box.probability)
                self.get_logger().info('Box: {2} Result: {0} Probability: {1}'.format(box.class_id, str(box.probability), box_index))
        else:
                self.get_logger().info('nothing detected')

        self.prev_done = True


        return True


    def _feedbackCallback(self, msg):
        self.get_logger().debug('Received action feedback message')
        self.feedback = msg.feedback
        return
    
def main(args=None):
    rclpy.init(args=args)

    id_action_server = IdentifyActionServer()
    
    mt_executor = MultiThreadedExecutor()
    mt_executor.add_node(id_action_server)
    mt_executor.spin()

    id_action_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()