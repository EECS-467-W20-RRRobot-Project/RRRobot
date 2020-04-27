xhost +local:docker #rrrobot-env
docker run -it --rm --device=/dev/input/event4 --device=/dev/dri:/dev/dri -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v $PWD/rrrobot_src:/home/rrrobot/rrrobot_src -h rrrobot-env eecs467:rrrobot
xhost -local:docker #rrrobot-env
