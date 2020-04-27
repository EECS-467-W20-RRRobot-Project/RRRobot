#!/bin/bash

cd /app/rrrobot_ws
# catkin_make clean
catkin_make
# catkin_make install
source devel/setup.bash
rosrun rrrobot arm_controller_node
