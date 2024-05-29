#!/bin/bash


cd /home/parallels/catkin_ws

source install_isolated/setup.bash

# 执行 ROS 程序，play 2D 包
roslaunch cartographer_ros demo_backpack_2d.launch bag_filename:=${HOME}/Downloads/cartographer_paper_deutsches_museum.bag

