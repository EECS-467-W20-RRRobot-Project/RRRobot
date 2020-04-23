#!/bin/bash

cd /app/rrrobot_ws/

catkin_make clean
catkin_make
catkin_make install

cd /app/rrrobot_ws/src/rrrobot/scripts

sudo cp ../launch/rrrobot.launch /opt/ros/melodic/share/osrf_gear/launch
sudo cp ../../../world/gear.py /opt/ros/melodic/lib/osrf_gear/gear.py
sudo cp ../../../world/gear.world.template /opt/ros/melodic/share/osrf_gear/worlds/gear.world.template

roslaunch osrf_gear rrrobot.launch
