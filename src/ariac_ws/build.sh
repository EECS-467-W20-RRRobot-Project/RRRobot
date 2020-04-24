#!/bin/bash

catkin_make clean &&
catkin_make &&
catkin_make install &&

source install/setup.bash
roslaunch ur10_moveit_config move_group.launch arm_namespace:=/ariac/arm1