# GEAR Dockerfile

# Use official image for Gazebo 9.x 
FROM gazebo:gzserver9

RUN apt-get update
# Install packages required for developing with gazebo
RUN apt-get install -y libgazebo9-dev
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-get install -y curl
RUN curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | apt-key add -
RUN apt-get update
# Install packages required for developing with ROS
RUN apt-get install -y ros-melodic-desktop-full
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
RUN apt install -y python-rosdep
RUN rosdep init
RUN rosdep update
# user id 1000 should be the same as the host user, so that you can access files
# from inside the docker container and also on the host
RUN useradd -u 1000 rrrobot
# set to no password
RUN passwd --delete rrrobot
# add to sudo users
RUN usermod -aG sudo rrrobot
# set the entry point
WORKDIR /app/rrrobot_ws
RUN mkdir /home/rrrobot
RUN chown -R rrrobot:rrrobot /home/rrrobot

# Initialize the environment in .bashrc
RUN echo "source /opt/ros/melodic/setup.bash" >> /home/rrrobot/.bashrc
RUN echo "source /usr/share/gazebo/setup.sh" >> /home/rrrobot/.bashrc
RUN echo "export GAZEBO_MODEL_PATH=/app/rrrobot_ws/src/gazebo_models:\$GAZEBO_MODEL_PATH" >> /home/rrrobot/.bashrc
RUN echo "export GAZEBO_PLUGIN_PATH=/opt/ros/melodic/lib:/app/rrrobot_ws/devel/lib:\$GAZEBO_PLUGIN_PATH" >> /home/rrrobot/.bashrc

USER rrrobot

# Install Gazebo Environment for Agile Robotics (GEAR)
RUN sudo apt-get update && \
    sudo apt-get install -y \
        wget

RUN sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable bionic main" > /etc/apt/sources.list.d/gazebo-stable.list' && \
    wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - && \
    sudo apt-get update && \
    sudo apt-get install -y \
        ariac3

# CV Model Dependencies
RUN sudo apt-get update && \
    sudo apt-get install -y \
        python3-pip

RUN pip3 install -U \
        numpy \
        torch \
        torchvision \
        Pillow \
        pyyaml \
        rospkg

CMD ["/bin/bash"]
