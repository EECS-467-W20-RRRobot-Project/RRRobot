# Documentation <!-- omit in toc -->

## Page Links <!-- omit in toc -->
- [Home](home.md)
- [Documentation](documentation.md)
- [GEAR](gear.md)

## Table of Contents <!-- omit in toc -->
- [Directory Structure](#directory-structure)
  - [docker_env](#dockerenv)
  - [docs](#docs)
  - [src](#src)
    - [src/rrrobot_ws/src/gazebo_models](#srcrrrobotwssrcgazebomodels)
    - [src/rrrobot_ws/src/rrrobot](#srcrrrobotwssrcrrrobot)
    - [src/rrrobot_ws/world](#srcrrrobotwsworld)
  - [utils](#utils)

## Directory Structure

In the RRRobot folder of our repository we have the following folders.

### docker_env

This contains the [docker-compose.yml](https://github.com/EECS-467-W20-RRRobot-Project/RRRobot/blob/master/docker_env/docker-compose.yml) file and folders for each docker service described in the [home](home.md) page. **gear** is the docker image used for our final simulation.

### docs

This contains the markdown files used to generate the project website pages and project documents like the poster and proposal.

### src

This contains the rrrobot_ws catkin workspace for our ROS development. Inside the `rrrobot_ws` folder is a [run_all.sh](https://github.com/EECS-467-W20-RRRobot-Project/RRRobot/blob/master/src/rrrobot_ws/run_all.sh) that will run the final simulation. See the [GEAR](gear.md) page for full instructions on using this.

In `rrrobot_ws/src`, we have a `gazebo_models` and `rrrobot` folder.

#### src/rrrobot_ws/src/gazebo_models

`gazebo_models`, as the name implies, contains 3D models we used in our testing. These include a fanuc robot arm with gripper, paper bag, plastic bottle, pop can, etc. For our final simulation, we decided to use simple cubes to represent trash (red) and recycling (green) items to make the ur10 arm's vacuum suction gripper work well. We also have the images that are passed to the CV model for classification in the `recycling_images` and `trash_images` folders. `model_mappings.txt` defines the model to use, location of image, and ground truth classification in a comma separated format.

#### src/rrrobot_ws/src/rrrobot

Our ROS package contains several folders.

`config` contains the configuration `.yaml` files that we use to overwrite ARIAC's default configuration. These files remove a bunch of the parts that we don't need and specify that we only need 1 depth camera.

`include` contains the header files for the names of topics that are used by our ROS nodes and the class definition for `ArmRepresentation`.

`launch` contains the launch file that we use to overwrite ARIAC's default launch file.

`msg` contains the definitions for the custom ROS messages used by our nodes.

`scripts` contains the bash scripts we used throughout testing. These include sending the arm to the home position, publishing a test message for the arm controller to listen to, and running the modified version of the GEAR simulation environment.

`src` contains the source code for our ROS nodes.

- `arm_controller_node.cpp` receives a pose from rrrobot_node to pick up the object, then uses inverse kinematics to determine the joint positions that will achieve the desired end-effector pose.
- `arm_representation_node.cpp` interfaces with [KDL](http://docs.ros.org/melodic/api/orocos_kdl/html/index.html) to perform inverse and forward kinematics on our ur10 robot arm model.
- `cv_model.py` uses `pytorch_pretrain_model.pt` to classify images that it receives and send the classification to rrrobot_node.
- `depth_camera_node.cpp` takes depth camera information and determines an end-effector pose for the robot arm to reach so it can pick up the object.
- `image_display.py` is a test script that uses [matplotlib](https://matplotlib.org/) to display images.
- `model_insertion_plugin.cpp` and `object_spawner_node.cpp` interface with gazebo to spawn the items in the simulation so the arm can sort them.
- `rrrobot_node.cpp` is the main node that handles interactions between the different nodes.

`test` contains some ROS nodes that we created to test forward and inverse kinematics.

`CMakeLists.txt` is used by our build system, [CMake](https://cmake.org), to build and compile our ROS package.

`package.xml` defines the metadata for our ROS package.

#### src/rrrobot_ws/world

This folder contains the world files that we use to overwrite ARIAC's defaults.

In `gear.py` we have disabled all but two of the bins as we only need to classify recycling and trash.

### utils

This contains config.xlaunch which is used by VcXsrv to automatically start the server with the desired settings (see [home](home.md) page for more information).
