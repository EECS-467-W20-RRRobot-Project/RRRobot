# GEAR Simulation <!-- omit in toc -->

## Page Links <!-- omit in toc -->
- [Home](home.md)
- [Documentation](documentation.md)
- [GEAR](gear.md)

## Table of Contents <!-- omit in toc -->
- [Running GEAR Container](#running-gear-container)
- [Building & Running Simulation](#building--running-simulation)
- [Controlling Sample Environment](#controlling-sample-environment)
  - [Start Competition](#start-competition)
  - [Controlling Arms](#controlling-arms)
- [Running Full Simulation](#running-full-simulation)
  - [Manual Process for Testing](#manual-process-for-testing)
  - [Semi-Automated Process](#semi-automated-process)

## Running GEAR Container

1. On your local system, go to `docker_env` folder
   - `cd /PATH/TO/RRRobot/docker_env`
2. Start GEAR docker container
   - `docker-compose run --rm gear`

## Building & Running Simulation

1. Source ROS Setup
   - `source /opt/ros/melodic/setup.bash`
2. Run Simulation
   - Sample Environment
     - `roslaunch osrf_gear sample_environment.launch`
     - See [controlling sample environment](#controlling-sample-environment) for more information.
   - Final Simulation
     - See [running full simulation](#running-full-simulation) for more information.


## Controlling Sample Environment

While sample environment is running, open a new terminal (see tips & tricks section of [home](home.md)) on the running docker container. The following sections provide a brief overview of the commands that can be used to control various aspects of the simulation. For a full list, check out the [ARIAC 2019 Wiki](https://bitbucket.org/osrf/ariac/wiki/2019/tutorials/gear_interface).

[sample_run.sh](https://github.com/EECS-467-W20-RRRobot-Project/RRRobot/blob/master/src/rrrobot_ws/src/rrrobot/scripts/sample_run.sh) moves arm 1 over the gasket part, picks it up, moves it to AGV1's tray, drops it, and moves back to the starting position.

### Start Competition

`rosservice call /ariac/start_competition`

### Controlling Arms

#### Gripper <!-- omit in toc -->

- Turn gripper suction on
  - `rosservice call /ariac/arm1/gripper/control "enable: true"`
- Turn gripper suction off
  - `rosservice call /ariac/arm1/gripper/control "enable: false"`

#### Joints <!-- omit in toc -->

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

## Running Full Simulation

### Manual Process for Testing

#### Terminal 1: ARIAC Environment <!-- omit in toc -->

- `cd /app/rrrobot_ws/src/rrrobot/scripts`
- `./rrrobot_run_no_build.sh`

#### Terminal 2: Build RRRobot Package & Arm Controller Node <!-- omit in toc -->

- `cd /app/rrrobot_ws`
- `catkin_make clean`
- `catkin_make`
- `source devel/setup.bash`
- `rosrun rrrobot arm_controller_node`

#### Terminal 3: CV Model <!-- omit in toc -->

- `cd /app/rrrobot_ws/src/rrrobot/src/`
- `python3 cv_model.py`

#### Terminal 4: Depth Camera Node (for getting pick up location) <!-- omit in toc -->

- `cd /app/rrrobot_ws/`
- `source devel/setup.bash`
- `rosrun rrrobot depth_camera_node`

#### Terminal 5: Run the main rrrobot node <!-- omit in toc -->

- `cd /app/rrrobot_ws/`
- `source devel/setup.bash`
- `rosrun rrrobot rrrobot_node`

#### Terminal 6: Run the node to spawn random objects onto the conveyor belt <!-- omit in toc -->

- `cd /app/rrrobot_ws/`
- `source devel/setup.bash`
- `rosrun rrrobot object_spawner_node`

### Semi-Automated Process

#### Terminal 1 <!-- omit in toc -->

In terminal 1, run the script that will build all components and run the nodes. You may need to adjust the sleep times in [run_all.sh](https://github.com/EECS-467-W20-RRRobot-Project/RRRobot/blob/master/src/rrrobot_ws/run_all.sh) based on your system's performance such that there is enough time for each component to finish before proceeding.

- `cd /app/rrrobot_ws/`
- `./run_all.sh`

#### Terminal 2 <!-- omit in toc -->

When a new item is spawned after the arm drops off the previous item, the conveyor belt must be manually started again using `rosservice call /ariac/conveyor/control "power: 100"` so that the item moves to the depth camera and the process can continue. You can do this in a new terminal since terminal 1 is running all of the simulation nodes.

#### Troubleshooting <!-- omit in toc -->

If you encounter issues with newline characters when attempting to run scripts, use the `dos2unix` tool to convert them to linux style newline characters.
These errors usually look like `/bin/bash^M: bad interpreter: No such file or directory` or `env: python\r: No such file or directory`.

- `sudo apt-get install -y dos2unix`
- `dos2unix <file_path>`
