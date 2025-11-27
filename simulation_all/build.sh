#!/bin/bash
source env.sh

docker run $DOCKER_RUN_FLAG $DOCKER_IMG bash -c 'source ~/.bashrc && ./script/build.sh "$@" && exit' -- "$@"
