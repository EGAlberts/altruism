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
            executable='system_reflection.py',
            name='system_reflection_node',
            parameters=[config_file,{}]
        ),
        Node(
            package='altruism',
            executable='adaptation_manager_node.py',
            name='adaptation_manager_node',
            parameters=[config_file,{}]
        ),
    ])