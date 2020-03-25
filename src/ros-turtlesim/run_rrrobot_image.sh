#docker run -i -h rrrobot-env -t eecs467:rrrobot bash

xhost +local:docker #rrrobot-env
docker run -it --rm --privileged --device=/dev/dri:/dev/dri -e DISPLAY=$DISPLAY -v $PWD:/app -v /tmp/.X11-unix:/tmp/.X11-unix -h rrrobot-env eecs467:rrrobot
xhost -local:docker #rrrobot-env
