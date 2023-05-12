import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import AnyLaunchDescriptionSource

def generate_launch_description():

    nav2_path = get_package_share_directory('nav2_bringup')
    nav2_launch_path = os.path.join(
        nav2_path, 'launch', 'navigation_launch.py')
    
    slam_tb_path = get_package_share_directory('slam_toolbox')
    slam_tb_launch_path = os.path.join(
        slam_tb_path, 'launch', 'online_async_launch.py')
    
    nav2 =  IncludeLaunchDescription(AnyLaunchDescriptionSource(nav2_launch_path))
    slam_tb =  IncludeLaunchDescription(AnyLaunchDescriptionSource(slam_tb_launch_path))


    return LaunchDescription([
        nav2,
        slam_tb,
    ])