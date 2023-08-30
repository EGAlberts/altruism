#!/usr/bin/env python3
import time
import rclpy
from rclpy.action import ActionServer, ActionClient
from rclpy.node import Node
import math
import numpy as np
from altruism_msgs.action import SLAM
from std_msgs.msg import Float64
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import LoadMap
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped
from copy import deepcopy
from itertools import cycle

#This is just a placeholder SLAM action server. The real SLAM will happen in an action server for the real world.

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

class SLAMActionServer(Node):

    def __init__(self):
        super().__init__('slam_action_server')
        self._action_server = ActionServer(
            self,
            SLAM,
            'slam',
            self.execute_callback,
            callback_group=MutuallyExclusiveCallbackGroup())
        
        #self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)
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

        for point_i in range(len(route)): #this is to give a more natural angle for the robot to end up at
            if(point_i == 0):
                route[point_i].append(0.017)

                continue
            new_point = route[point_i]
            old_point = route[point_i-1]
            heading_angle = math.atan2(new_point[1] - old_point[1], new_point[0] - old_point[0])
            route[point_i].append(heading_angle)



        route_poses = []

        self.initial_time = time.time()

        pose = PoseStamped()
        pose.header.frame_id = 'map'
  
        #pose.pose.orientation.w = 1.0
        for pt in route:
            pose.pose.position.x = pt[0]
            pose.pose.position.y = pt[1]
            q = quaternion_from_euler(0, 0, pt[2])
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            route_poses.append(deepcopy(pose))

        self.nav2_feedback = None
        self.route_to_follow = iter(route_poses * 2)
        self.get_logger().info('SLAM Action server created...')

    
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        self.get_logger().info('Its me, hi, Im a SLAM action server')
        goal_success = True
        while goal_success:
            print("gonna wait 10 seconds now and then send the goal to move")
            #time.sleep(10)        
            feedback_msg = SLAM.Feedback()
            if(self.nav2_feedback is not None):
                feedback_msg.current_pose = self.nav2_feedback.current_pose

            goal_handle.publish_feedback(feedback_msg)

            goal_success = self.send_goal()

        goal_handle.succeed()
        result = SLAM.Result()

        result.time_elapsed = time.time() - self.initial_time

        return result
        

            
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

        return self.goToPose(next(self.route_to_follow, None))
        #self.publisher_.publish(next(self.route_to_follow))
    def _feedbackCallback(self, msg):
        self.get_logger().debug('Received nav2 action feedback message')
        self.nav2_feedback = msg.feedback
        return
    def goToPose(self, pose, behavior_tree=''):
        """Send a `NavToPose` action request."""
        if pose is None:
            return False

        pose.header.stamp = self.get_clock().now().to_msg()

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