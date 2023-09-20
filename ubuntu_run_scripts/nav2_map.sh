#!/bin/bash
source ~/.bashrc
source ~/nav2simple_ws/install/setup.bash

ros2 launch nav2_bringup bringup_launch.py use_sim_time:=False map:=/home/ega/nav2simple_ws/map_1694604036.yaml
