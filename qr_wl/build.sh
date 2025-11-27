#!/bin/bash
source script/env.sh

# 检查 Docker 镜像是否存在
if ! docker images | grep -q "$(echo $DOCKER_IMG | cut -d: -f1)" 2>/dev/null; then
    echo "Warning: Docker image not found: $DOCKER_IMG"
    echo "Please run ./setup_environment.sh to configure environment and download the image"
    echo "Or manually pull: docker pull $DOCKER_IMG"
    exit 1
fi

docker run $DOCKER_RUN_FLAG $DOCKER_IMG bash -c 'source ~/.bashrc && ./script/build.sh "$@" && exit' -- "$@"

# docker编译出的权限和当前用户可能不同，修改
#sudo chown -R $(whoami):$(id -g -n) ./build
#sudo chown -R $(whoami):$(id -g -n) ./onnx_model
#sudo chown -R $(whoami):$(id -g -n) ./log
