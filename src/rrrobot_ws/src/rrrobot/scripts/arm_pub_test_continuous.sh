#!/bin/bash

#rostopic pub /target_pose geometry_msgs/Pose '{position: {x: 0.875, y: 0.75, z: 1.5}, orientation: {x: 0, y: 0, z: 0, w: 0}}'
rostopic pub /arm_controller/destination rrrobot/arm_command "grab_location:
  position:
    x: 1.2
    y: 0.2
    z: 1.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 0.0
drop_location:
  position:
    x: -0.3
    y: 1.15
    z: 1.5
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 0.0"
