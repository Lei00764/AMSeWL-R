#!/bin/bash


cd /home/parallels/catkin_ws

source install_isolated/setup.bash

# 执行 ROS 程序，play 3D 包
roslaunch cartographer_ros demo_backpack_3d.launch bag_filename:=${HOME}/Downloads/b3-2016-04-05-14-14-00.bag

