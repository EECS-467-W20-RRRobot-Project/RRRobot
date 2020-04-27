#!/bin/bash

rostopic pub /ariac/arm1/arm/command trajectory_msgs/JointTrajectory    "{joint_names: \
        ['linear_arm_actuator_joint',  'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'], \
     points: [ \
{time_from_start: {secs: 5}, \
        positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}, \
]}" -1
