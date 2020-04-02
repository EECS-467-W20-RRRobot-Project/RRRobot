#docker run -i -h rrrobot-env -t eecs467:rrrobot bash

xhost +local:docker #rrrobot-env
docker run -it --rm --device=/dev/input/event4 --device=/dev/dri:/dev/dri -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v$PWD/gazebo_models:/root/.gazebo/models -h rrrobot-env eecs467:rrrobot
xhost -local:docker #rrrobot-env
