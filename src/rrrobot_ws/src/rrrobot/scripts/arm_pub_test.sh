#!/bin/bash

rostopic pub /target_pose geometry_msgs/Pose '{position: {x: 0.875, y: 0.75, z: 1.5}, orientation: {x: 0, y: 0, z: 0, w: 0}}'
