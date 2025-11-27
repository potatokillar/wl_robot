#!/bin/bash
source script/env.sh
docker run $DOCKER_RUN_FLAG $DOCKER_IMG bash -c 'source ~/.bashrc && ./script/start.sh "$@" && exit' -- "$@"