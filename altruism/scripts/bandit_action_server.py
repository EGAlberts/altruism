#!/usr/bin/env python3
import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from masced_bandits.bandit_options import initialize_arguments
from masced_bandits.bandits import init_bandit

from altruism_msgs.action import Bandit


class BanditActionServer(Node):

    def __init__(self):
        super().__init__('bandit_action_server')
        self._action_server = ActionServer(
            self,
            Bandit,
            'bandit',
            self.execute_callback)

        initialize_arguments(["Arm1","Arm2"], 0)
        self.ucb_instance = init_bandit(name='UCB')
        self.get_logger().info('Action server created...')


    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Bandit.Feedback()
        feedback_msg.chosen_arm = "None"

        do_bandit = True
        while do_bandit:
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal_handle.succeed()

        result = Bandit.Result()
        result.average_reward = 0
        return result


def main(args=None):
    rclpy.init(args=args)

    bandit_action_server = BanditActionServer()

    rclpy.spin(bandit_action_server)


if __name__ == '__main__':
    main()