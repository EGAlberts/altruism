#!/usr/bin/env python3
import time
import rclpy
from rclpy.action import ActionServer, ActionClient
from rclpy.node import Node

from altruism_msgs.action import SLAM
from std_msgs.msg import Float64
from nav2_msgs.action import NavigateToPose
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped
from copy import deepcopy
from itertools import cycle

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
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        route = [
        [-1.5, -2.5],
        [-1.0, -2.0],
        [-0.5, -2.0],
        [0.0, -2.0],
        [0.5, -2.0],
        [1.0, -2.0],
        [1.5, -1.5],
        [2.0, -1.0],
        [2.0, 0.0],
        [2.0, 1.0 ],
        [1.0, 2.0],
        [-1.0, 2.0], #bottom left corner
        [-2.0, 1.0],
        [-1.75, 0.45],
        [0.75, 0.45],
        [0.75, -0.7],
        [-2.0, -0.7]]

        route_poses = []
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.orientation.w = 1.0
        for pt in route:
            pose.pose.position.x = pt[0]
            pose.pose.position.y = pt[1]
            route_poses.append(deepcopy(pose))

        self.route_to_follow = cycle(route_poses)
        self.get_logger().info('SLAM Action server created...')


    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        self.get_logger().info('Its me, hi, Im a SLAM action server')
        while True:
            print("gonna wait 10 seconds now and then send the goal to move")
            #time.sleep(10)        
            feedback_msg = SLAM.Feedback()

            feedback_msg.progress = 0.0

            goal_handle.publish_feedback(feedback_msg)

            self.send_goal()
            
    def send_goal(self):
        #ros2 topic pub /goal_pose geometry_msgs/PoseStamped "{header: {stamp: {sec: 0}, frame_id: 'map'}, pose: {position: {x: 1.9, y: 0.7, z: 0.0}, orientation: {w: 1.0}}}"

        self.get_logger().info("Sending the goal")
        # we start somewhere here: x: -2.0, y: -1.0, z: 0.0
        # x:  -1.5, y: -2.5, z: 0.0
        # -1.0, y: -2.0, z: 0.0
        # -0.5, y: -2.0, z: 0.0
        # x: 0.0, y: -2.0, z: 0.0
        # x: 0.5 y: -2.0, z: 0.0
        # x: 1.0 y: -2.0, z: 0.0
        # x: 1.5 y: -1.5, z: 0.0
        # x: 2.0 y: -1.0, z: 0.0
        # x: 2.0 y: 0.0, z: 0.0 w: 0.5
        # x: 2.0 y: 1.0 
        # x: 1.0 y: 2.0
        #x: -1.0, y: 2.0 bottom left corner
        # x: -2.0 y: 1.0
        # x: -1.75 y: 0.45
        # x: 0.75 y: 0.45
        # x: 0.75 y: -0.7
        # x: -2.0 y: -0.7

        self.goToPose(next(self.route_to_follow))
        #self.publisher_.publish(next(self.route_to_follow))
    def _feedbackCallback(self, msg):
        self.get_logger().debug('Received action feedback message')
        self.feedback = msg.feedback
        return
    def goToPose(self, pose, behavior_tree=''):
        """Send a `NavToPose` action request."""
        self.get_logger().debug("Waiting for 'NavigateToPose' action server")
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("'NavigateToPose' action server not available, waiting...")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        goal_msg.behavior_tree = behavior_tree

        self.get_logger().info('Navigating to goal: ' + str(pose.pose.position.x) + ' ' +
                  str(pose.pose.position.y) + '...')
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg,
                                                                   self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.get_logger().error('Goal to ' + str(pose.pose.position.x) + ' ' +
                       str(pose.pose.position.y) + ' was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, self.result_future,timeout_sec=20)
        return True
    
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