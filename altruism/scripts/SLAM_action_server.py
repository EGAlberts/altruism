#!/usr/bin/env python3
import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from altruism_msgs.action import SLAM
from std_msgs.msg import Float64

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped

#This is just a placeholder SLAM action server. The real SLAM will happen in an action server for the real world.
class SLAMActionServer(Node):

    def __init__(self):
        super().__init__('slam_action_server')
        self._action_server = ActionServer(
            self,
            SLAM,
            'slam',
            self.execute_callback,
            callback_group=MutuallyExclusiveCallbackGroup())
        
        self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)

        self.get_logger().info('SLAM Action server created...')


    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        self.get_logger().info('Its me, hi, Im a SLAM action server')
    
        while True:
            feedback_msg = SLAM.Feedback()

            feedback_msg.progress = 0.0

            goal_handle.publish_feedback(feedback_msg)

            self.send_goal()
            
            time.sleep(60)

    def send_goal(self):
        #ros2 topic pub /goal_pose geometry_msgs/PoseStamped "{header: {stamp: {sec: 0}, frame_id: 'map'}, pose: {position: {x: 1.9, y: 0.7, z: 0.0}, orientation: {w: 1.0}}}"

        self.get_logger().info("Sending the goal")

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.pose.position.x = 0.2
        msg.pose.position.y = 0.0
        msg.pose.position.z = 0.0

        msg.pose.orientation.w = 1.0

        
        self.publisher_.publish(msg)


    
def main(args=None):
    rclpy.init(args=args)

    slam_action_server = SLAMActionServer()
    
    mt_executor = MultiThreadedExecutor()
    mt_executor.add_node(slam_action_server)
    mt_executor.spin()

    slam_action_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()