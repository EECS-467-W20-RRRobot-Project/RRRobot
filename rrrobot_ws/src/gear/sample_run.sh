#!/bin/bash

# Start Competition
rosservice call /ariac/start_competition

# Wait
sleep 2

# Move arm1 over a gasket part
rostopic pub /ariac/arm1/arm/command trajectory_msgs/JointTrajectory    "{joint_names: \
        ['linear_arm_actuator_joint',  'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'], \
     points: [ \
{time_from_start: {secs: 2}, \
        positions: [0.15, 3.14,  -1.570,  2.14, 3.1, -1.59, 0.126]}, \
{time_from_start: {secs: 4}, \
        positions: [-0.35, 3.14,  -0.6,  2.3, 3.0, -1.59, 0.126]}, \
{time_from_start: {secs: 6}, \
        positions: [-0.35, 3.14,  -0.5,  2.3, 3.05, -1.59, 0.126]}, \
]}" -1

# Wait for arm to move
sleep 8

# Turn gripper suction on
rosservice call /ariac/arm1/gripper/control "enable: true"

# Wait for suction
sleep 2

# Move part to AGV1's tray
rostopic pub /ariac/arm1/arm/command trajectory_msgs/JointTrajectory    "{joint_names: \
        ['linear_arm_actuator_joint',  'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'], \
     points: [ \
{time_from_start: {secs: 2}, \
        positions: [0.0, 3.14,  -1.570,  2.14, 3.27, -1.51, 0.0]}, \
{time_from_start: {secs: 5}, \
        positions: [1.0, 1.85,  0,  -0.38, 1.57, -1.51, 0.00]}, \
{time_from_start: {secs: 7}, \
        positions: [1.0, 1.507,  0,  -0.38, 0.38, -1.51, 0.00]}, \
{time_from_start: {secs: 10}, \
        positions: [1.18, 1.507,  0.38,  -0.38, 1.55, 1.75, 0.127]}, \
]}" -1

# Wait for arm to move
sleep 12

# Turn gripper suction off
rosservice call /ariac/arm1/gripper/control "enable: false"

# Wait for suction
sleep 2

# Return to starting position
rostopic pub /ariac/arm1/arm/command trajectory_msgs/JointTrajectory    "{joint_names: \
        ['linear_arm_actuator_joint',  'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'], \
     points: [ \
{time_from_start: {secs: 5}, \
        positions: [0.0, 3.14,  -1.570,  2.14, 3.27, -1.51, 0.0]}, \
]}" -1

# Wait for arm to move
sleep 7
