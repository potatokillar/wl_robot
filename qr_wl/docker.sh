#!/bin/bash
source script/env.sh

if [ $# == 0 ]; then
    docker run $DOCKER_RUN_FLAG  $DOCKER_IMG bash 
else
    export DISPLAY=$DISPLAY
    echo "setting display to $DISPLAY"
    xhost +
    docker run $DOCKER_RUN_FLAG \
            -v /tmp/.X11-unix:/tmp/.X11-unix \
            -e DISPLAY=$DISPLAY \
            --env="NVIDIA_DRIVER_CAPABILITIES=all" \
            $DOCKER_IMG bash
fi
