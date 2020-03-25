#docker run -i -h rrrobot-env -t eecs467:rrrobot bash

#xhost +local:docker
#docker run -it --privileged --security-opt seccomp=unconfined -v $SSH_AUTH_SOCK:/ssh-agent --env SSH_AUTH_SOCK=/ssh-agent --device=/dev/dri:/dev/dri -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --env QT_X11_NO_MITSHM=1 -h rrrobot-env eecs467:rrrobot
#xhost -local:docker

../run_rrrobot_image.sh
