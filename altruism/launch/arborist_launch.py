from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
def generate_launch_description():
    config_file = os.path.join(
    get_package_share_directory('altruism'),
    'config',
    'altruism_config.yaml'
    )

    experiment_name_arg = DeclareLaunchArgument(
        'experiment_name',
        default_value='none',
        description='Name to give to the experiment in the CSV reporting afterwards.'
    )
    experiment_name = LaunchConfiguration('experiment_name')

    tree_name_arg = DeclareLaunchArgument(
    'tree_name',
    default_value='none',
    description='Name of behavior tree xml file'
    )
    tree_name = LaunchConfiguration('tree_name')



    return LaunchDescription([
        experiment_name_arg,
        tree_name_arg,
        Node(
            package='altruism',
            executable='arborist',
            name='arborist_node',
            parameters=[config_file,{
                'experiment_name': experiment_name,
                'bt_filename': tree_name
            }]
        ),
    ])