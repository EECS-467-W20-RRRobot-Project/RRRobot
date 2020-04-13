# GEAR Simulation

## Running GEAR Container

1. Go to `docker_env` folder
   - `cd /PATH/TO/RRRobot/docker_env`
2. Start GEAR docker container
   - `docker-compose run --rm gear`

## Building & Running Sample Environment

1. Source ROS Setup
   - `source /opt/ros/melodic/setup.bash`
<!-- 2. Go to rrrobot_ws folder in container
   - `cd /app/rrrobot_ws`
3. Build package
   - `catkin_make clean`
   - `catkin_make`
   - `catkin_make install` -->
2. Run sample environment
   - `roslaunch osrf_gear sample_environment.launch`

## Controlling Sample Environment

While sample environment is running, open a new terminal (see tips & tricks section of [home](./home.md)) on the running docker container. The following sections provide a brief overview of the commands that can be used to control various aspects of the simulation. For a full list, check out the [ARIAC 2019 Wiki](https://bitbucket.org/osrf/ariac/wiki/2019/tutorials/gear_interface).

[sample_run.sh](../rrrobot_ws/src/gear/sample_run.sh) moves arm 1 over the gasket part, picks it up, moves it to AGV1's tray, drops it, and moves back to the starting position.

### Start Competition

`rosservice call /ariac/start_competition`

### Controlling Arms

#### Gripper

- Turn gripper suction on
  - `rosservice call /ariac/arm1/gripper/control "enable: true"`
- Turn gripper suction off
  - `rosservice call /ariac/arm1/gripper/control "enable: false"`

#### Joints

Move `arm1` over a gasket part

```
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
```

Move part to `AGV1`'s tray

```
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
```

Return to starting position

```
rostopic pub /ariac/arm1/arm/command trajectory_msgs/JointTrajectory    "{joint_names: \
        ['linear_arm_actuator_joint',  'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'], \
     points: [ \
{time_from_start: {secs: 5}, \
        positions: [0.0, 3.14,  -1.570,  2.14, 3.27, -1.51, 0.0]}, \
]}" -1
```
