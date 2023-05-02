#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from altruism_msgs.action import BehaviorTree


class TreeActionClient(Node):

    def __init__(self):
        super().__init__('tree_action_client')
        self._action_client = ActionClient(self, BehaviorTree, 'start_tree')

    def send_goal(self):
        goal_msg = BehaviorTree.Goal()
    
        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.is_success))
        rclpy.shutdown()

    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.node_status))



def main(args=None):
    rclpy.init(args=args)

    action_client = TreeActionClient()

    future = action_client.send_goal()

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()