#!/bin/bash

gnome-terminal -- ./action_servers.sh
sleep 5
gnome-terminal -- ./darknet.sh
sleep 10
gnome-terminal -- ./gazebo_tb3.sh 
sleep 15
gnome-terminal -- ./nav2.sh 
sleep 15
gnome-terminal -- ./arborist.sh 
sleep 10
gnome-terminal -- ./sysrefl.sh 
sleep 10
gnome-terminal -- ./tree_action.sh 
