# ROS Development Dockerfile

# Use official image for ROS Melodic Morenia
FROM ros:melodic

# Install dependencies for development
RUN sudo apt-get update && apt-get install -y \
        # ROS Tutorial Dependencies
        ros-melodic-ros-tutorials \
        ros-melodic-rqt \
        ros-melodic-rqt-common-plugins \
        ros-melodic-turtlesim \
        # C++ Development: g++ compiler, etc.
        build-essential

CMD ["/bin/bash"]
