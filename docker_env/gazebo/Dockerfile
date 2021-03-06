# Gazebo Dockerfile

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
RUN apt-get install -y ros-melodic-pid
# user id 1000 should be the same as the host user, so that you can access files
# from inside the docker container and also on the host
RUN useradd -u 1000 rrrobot
# set to no password
RUN passwd --delete rrrobot
# add to sudo users
RUN usermod -aG sudo rrrobot
# set the entry point
WORKDIR /home/rrrobot
RUN chown -R rrrobot:rrrobot /home/rrrobot

# Initialize the environment in .bashrc
RUN echo "source /opt/ros/melodic/setup.bash" >> /home/rrrobot/.bashrc
RUN echo "source /usr/share/gazebo/setup.sh" >> /home/rrrobot/.bashrc
RUN echo "export GAZEBO_MODEL_PATH=/home/rrrobot/rrrobot_src/src/gazebo_models:\$GAZEBO_MODEL_PATH" >> /home/rrrobot/.bashrc
RUN echo "export GAZEBO_PLUGIN_PATH=/home/rrrobot/rrrobot_src/lib:\$GAZEBO_PLUGIN_PATH" >> /home/rrrobot/.bashrc

USER rrrobot

CMD ["/bin/bash"]
