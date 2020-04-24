#!/bin/bash

cd /app/rrrobot_ws
catkin_make clean && catkin_make
source devel/setup.bash
rosrun rrrobot arm_controller_node
