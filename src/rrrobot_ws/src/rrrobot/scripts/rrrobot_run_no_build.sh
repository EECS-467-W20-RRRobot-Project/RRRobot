#!/bin/bash

sudo cp ../launch/rrrobot.launch /opt/ros/melodic/share/osrf_gear/launch
sudo cp ../../../world/gear.py /opt/ros/melodic/lib/osrf_gear/gear.py
sudo cp ../../../world/gear.world.template /opt/ros/melodic/share/osrf_gear/worlds/gear.world.template

roslaunch osrf_gear rrrobot.launch
