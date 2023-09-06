import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument

import random
def generate_launch_description():
    gui = LaunchConfiguration('gui')
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Run with gui (true/false)')

    tb3_path = get_package_share_directory('turtlebot3_gazebo')
    tb3_launch_path = os.path.join(
        tb3_path, 'launch', 'turtlebot3_world.launch.py')
    
    this_path = get_package_share_directory('altruism')
    custom_world_launch_path = os.path.join(
        this_path, 'launch', 'custom_world.launch.py')
    


    possible_poses = [(-0.0,0.5),(1.0,0.50), (0.11,-0.61)]
    random.seed(1337)
    x_random,y_random = random.choice(possible_poses)
    box_x_pose = LaunchConfiguration('box_x_pose', default=str(x_random))
    box_y_pose = LaunchConfiguration('box_y_pose', default=str(y_random))
    custom_world =  IncludeLaunchDescription(
        AnyLaunchDescriptionSource(custom_world_launch_path),
        launch_arguments={
           'gui': gui
        }.items())
    model_name = 'fire_hydrant_small'
    spawn_box = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'object_to_find_1',
            '-database', model_name,
            '-x', box_x_pose,
            '-y', box_y_pose,
            '-z', '-0.14'
        ],
        output='screen',
    )
    return LaunchDescription([
        custom_world,
        spawn_box,
        gui_arg,
    ])
