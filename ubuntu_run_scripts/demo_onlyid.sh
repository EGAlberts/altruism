#!/bin/bash

gnome-terminal -- ./id_action_server.sh
sleep 5
gnome-terminal -- ./darknet.sh
sleep 10
gnome-terminal -- ./gazebo_tb3.sh 
sleep 15
gnome-terminal -- ./nav2_map.sh 
sleep 15
gnome-terminal -- ./arborist.sh 
sleep 10
gnome-terminal -- ./sysrefl.sh 
sleep 10
gnome-terminal -- ./tree_action.sh 
