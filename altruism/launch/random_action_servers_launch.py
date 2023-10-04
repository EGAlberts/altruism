from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    config_file = os.path.join(
    get_package_share_directory('altruism'),
    'config',
    'altruism_config.yaml'
    )
    return LaunchDescription([
        Node(
            package='altruism',
            executable='SLAM_action_server.py',
            name='slam_action_server',
            parameters=[config_file,{}]
        ),
        Node(
            package='altruism',
            executable='ID_action_server.py',
            name='identify_action_server',
            parameters=[config_file,{}]
        ),
        Node(
            package='altruism',
            executable='random_action_server.py',
            name='random_action_server',
            parameters=[config_file,{}]
        ),
    ])