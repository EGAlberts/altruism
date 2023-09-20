#!/bin/bash
source ~/.bashrc
source ~/nav2simple_ws/install/setup.bash
source /usr/share/gazebo/setup.sh

export TURTLEBOT3_MODEL=waffle
export GAZEBO_PLUGIN_PATH=$HOME/nav2simple_ws/build/gazebo_ros_battery:$GAZEBO_PLUGIN_PATH

ros2 launch altruism spawn_tb3.launch.py gui:='true'
