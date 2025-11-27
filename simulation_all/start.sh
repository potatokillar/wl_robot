#!/bin/bash
source env.sh
echo $DOCKER_RUN_FLAG
# 在远程界面，该值没有；在原生界面，该值是:1
export DISPLAY=$DISPLAY
echo "setting display to $DISPLAY"
xhost +
docker run  $DOCKER_RUN_FLAG \
            -v /tmp/.X11-unix:/tmp/.X11-unix \
            -e DISPLAY=$DISPLAY  \
            $DOCKER_IMG  \
            bash -c 'source ~/.bashrc && ./script/start.sh "$@"' -- "$@"
xhost -
