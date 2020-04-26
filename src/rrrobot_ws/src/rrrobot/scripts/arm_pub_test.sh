#!/bin/bash

source /app/rrrobot_ws/devel/setup.bash

rostopic pub /arm_controller/destination rrrobot/arm_command "grab_location:
  position:
    x: 1.22003614902
    y: 1.22740316391
    z: 0.97
  orientation:
    x: 0.0
    y: 0.707
    z: 0.0
    w: 0.707
drop_location:
  position:
    x: -0.3
    y: 0.383
    z: 1.0
  orientation:
    x: 0.0
    y: 0.707
    z: 0.0
    w: 0.707" \
-1
