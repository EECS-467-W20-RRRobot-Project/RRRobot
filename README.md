# RRRobot! <!-- omit in toc -->

## Table of Contents <!-- omit in toc -->
- [Contributors](#contributors)
- [Documents](#documents)
- [Introduction](#introduction)
- [Development Guide](#development-guide)
  - [Docker Compose Services](#docker-compose-services)
  - [GUI Support](#gui-support)
  - [Running Docker Containers](#running-docker-containers)

## Contributors

### Project Team <!-- omit in toc -->

- Sravan Balaji ([balajsra@umich.edu](mailto:balajsra@umich.edu))
- Chenxi Gu ([chenxgu@umich.edu](mailto:chenxgu@umich.edu))
- Jake Johnson ([thejakej@umich.edu](mailto:thejakej@umich.edu))
- Derek Witcpalek ([dwitcpa@umich.edu](mailto:dwitcpa@umich.edu))

### EECS 467 W20 Course Staff <!-- omit in toc -->

- Prof. Chad Jenkins ([ocj@umich.edu](mailto:ocj@umich.edu))
- Xiaotong Chen ([cxt@umich.edu](mailto:cxt@umich.edu))
- Jana Pavlasek ([pavlasek@umich.edu](mailto:pavlasek@umich.edu))

## Documents

1. [Project Poster](Documents/1.%20Project%20Poster.pdf)
2. [Project Proposal](Documents/2.%20Project%20Proposal.pdf)

## Introduction

Repository for UMICH EECS 467: Autonomous Robotics (W20) RRRobot! project.

To get started, you will need to install Docker on your system. Information on what Docker is, how to install it, and how to use it can be found in [Getting Started with Docker](https://sravanbalaji.com/Web%20Pages/blog_docker.html).

## Development Guide

At this point, you should have Docker Desktop, Toolbox, or Engine setup on your host system. In this section, you will find a list of the different docker-compose services available and how to run them.

### Docker Compose Services

#### ros-dev <!-- omit in toc -->

ros-dev is the development container for ROS. This is used to provide all the necessary dependencies for working through the ROS tutorials and creating ROS packages.

#### ros-turtlesim <!-- omit in toc -->

ros-turtlesim is a container that tests whether GUI support is working. This will install the ros tutorial packages and run the turtlesim GUI with keyboard control. If X forwarding is setup correctly, you should see a window open up with a turtle on a blue background. Use the arrow keys in the container terminal to move it around and press `q` to quit.

#### ros <!-- omit in toc -->

ros is the production container for all ROS nodes we create. This is meant to run our final code and interface with the gazebo container for simulation.

#### gazebo <!-- omit in toc -->

Similarly to ros, gazebo is the production container for gazebo. This is meant to run our final simulation.

### GUI Support

If using VcXsrv for Windows to enable GUI applications, run XLaunch using [config.xlaunch](utils/config.xlaunch). This will enable the following settings:

- Display Settings
  - Multiple Windows
  - Display Number: -1
- Client Startup
  - Start No Client
- Extra Settings
  - [x] Clipboard
    - [x] Primary Selection
  - [ ] Native OpenGL
  - [x] Disable Access Control

The main thing to note here is that `Native OpenGL` is disabled and `Disable Access Control` is enabled. When running Gazebo GUI applications through a Docker Container in WSL, enabling `Native OpenGL` resulted in the following errors:

```
libGL error: No matching fbConfigs or visuals found
libGL error: failed to load driver: swrast
libGL error: No matching fbConfigs or visuals found
libGL error: failed to load driver: swrast
```

As mentioned in [No libGL libraries when running Gazebo from ROS](https://github.com/microsoft/WSL/issues/3644#issuecomment-434556680), this can be resolved by disabling `No native OpenGL` in the VcXsrv configuration.

Additionally, be sure to update the `IP_ADDRESS` variable in [.env](src/.env) with your computer's IP Address to enable X forwarding.

### Running Docker Containers

1. Start Docker Machine (default)
    - WSL: `docker-machine.exe start default`
    - Other: `docker-machine start default`
2. Set Docker Machine Environment
    - WSL: `docker-machine.exe env`
    - Other: `docker-machine env`
3. Navigate to src folder
    - `cd /PATH/TO/rrrobot/src`
4. Use Docker Compose to run a service (refer to [docker-compose.yml](src/docker-compose.yml) or [Docker Compose Services](#docker-compose-services))
   - `docker-compose run --rm <service_name>`


### Building the simulation environment

1. Build the drivers for the simulation
   - `cd /home/rrrobot/rrrobot_src/src/simulation_env/`
   - `source build.sh`


### Running the simulation

1. Start ros master node
   - `roscore &`
2. Run the gazebo simulator - this will bring up gazebo with a robotic arm
   - `gazebo /home/rrrobot/rrrobot_src/world/rrrobot.world`
3. Run control and perception programs
