from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='altruism',
            executable='SLAM_action_server.py',
            name='slam_action_server'
        ),
        Node(
            package='altruism',
            executable='bandit_action_server.py',
            name='bandit_action_server'
        ),
    ])