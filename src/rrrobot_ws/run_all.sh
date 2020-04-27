#!/bin/bash

# Build Workspace
cd /app/rrrobot_ws
catkin_make clean
catkin_make
source devel/setup.bash
export GAZEBO_RESOURCE_PATH=$(pwd)/world:$GAZEBO_RESOURCE_PATH

# ARIAC Environment
cd /app/rrrobot_ws/src/rrrobot/scripts
./rrrobot_run_no_build.sh &
sleep 25
rosparam set /use_sim_time true

# CV Model
cd /app/rrrobot_ws/src/rrrobot/src
python3 cv_model.py &
sleep 10

# Arm Controller Node
cd /app/rrrobot_ws
rosrun rrrobot arm_controller_node &
sleep 3

# Depth Camera Node
rosrun rrrobot depth_camera_node &
sleep 3

# RRRobot Node
rosrun rrrobot rrrobot_node &
sleep 3

# Object Spawner
rosrun rrrobot object_spawner_node &
sleep 3

# Start Competition
source /opt/ros/melodic/setup.bash
rosservice call /ariac/start_competition
sleep 1
rosservice call /ariac/conveyor/control "power: 100"
sleep 1
rosservice call /ariac/arm1/gripper/control "enable: false"
sleep 1
rosservice call /ariac/arm1/gripper/control "enable: true"
sleep 1
rosservice call /ariac/arm1/gripper/control "enable: false"
