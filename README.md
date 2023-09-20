# Altruism
Behavior Trees to meet Quality Requirements through Multi-armed Bandits at runtime in ROS 2.

This is a WIP project where in the coming months we'll develop a framework for a general and reusable software architectures to meet quality requirements e.g. safety, reliability, energy efficiency applied to robotics. 

[ICRA Poster Alberts[1140].pdf](https://github.com/EGAlberts/altruism/files/11591028/ICRA.Poster.Alberts.1140.pdf)

## Dependencies 
1. [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html)
2. [NAV2](https://navigation.ros.org/getting_started/index.html#installation)
3. [Gazebo classic](http://classic.gazebosim.org/)

## Setting up 
1. Download release BehaviorTree.CPP-4.1.1 from https://github.com/BehaviorTree/BehaviorTree.CPP/releases/tag/4.1.1
2. Clone particular commit from BehaviorTree.ROS2: `git clone https://github.com/BehaviorTree/BehaviorTree.ROS2/tree/9ed45e92bee2d694789a4db6703234a1da0b8681`
3. Download darknet and darknet-ros by cloning darknet-ROS recursively: `git clone --recurse-submodules -j8 https://github.com/EGAlberts/darknet_ros.git`
4. Install masced_bandits python library: `pip install masced-bandits`
