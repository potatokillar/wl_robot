#!/bin/bash

ARCH=$(uname -m)

# Harbor 配置
HARBOR_REGISTRY=192.168.1.93
HARBOR_PROJECT=iiri

# Docker 镜像配置 - 使用 latest 标签自动获取最新版本
DOCKER_IMG_X86=${HARBOR_REGISTRY}/${HARBOR_PROJECT}/build_x86_ros2:latest
DOCKER_IMG_ARM=${HARBOR_REGISTRY}/${HARBOR_PROJECT}/build_arm_ros2:latest
# 创建环境变量文件， docker env文件不会自动的将shell命令转化，而直接写则不支持空格
echo "USER_NAME=$(whoami)" >> env.tmp
echo "USER_UID=$(id -u)" >> env.tmp
echo "USER_GID=$(id -g)" >> env.tmp
echo "USER_HOME=/home/$(whoami)" >> env.tmp
echo "USER_GROUPS=$(id -Gn) dialout" >> env.tmp 
echo "ROS_LOCALHOST_ONLY=1" >> env.tmp
echo "ROS_DOMAIN_ID=0">> env.tmp
echo "RCUTILS_COLORIZED_OUTPUT=1" >> env.tmp
echo "PULSE_SERVER=unix:/run/user/native" >> env.tmp

# 此处不带-it参数，按需添加
# 在 CI 环境中使用随机容器名，避免命名冲突
if [ -n "$CI" ]; then
    CONTAINER_NAME="iiri-ros-$$-$(date +%s)"
    # 在 Jenkins 容器中运行时，需要将 Jenkins 容器内的路径转换为宿主机路径
    # Jenkins 容器挂载: /home/wl/jenkins_data -> /var/jenkins_home
    WORKSPACE_DIR=$(pwd | sed 's|/var/jenkins_home|/home/wl/jenkins_data|')
else
    CONTAINER_NAME="iiri-ros"
    WORKSPACE_DIR=$(pwd)
fi

DOCKER_RUN_FLAG="-v "${WORKSPACE_DIR}:$(pwd)" \
                 -w "$(pwd)" \
                 --env-file env.tmp \
                 -v /dev:/dev \
                 -v /home/$(whoami)/.ros:/home/$(whoami)/.ros \
                 -v /home/$(whoami)/.gazebo:/home/$(whoami)/.gazebo \
                 --rm --network=host --privileged \
                 -v /run/user/1000/pulse/native:/run/pulse/native \
                 --name $CONTAINER_NAME"

if [ $ARCH == "x86_64" ]; then
    # 仅在非 CI 环境中启用 GPU
    if [ -z "$CI" ]; then
        DOCKER_RUN_FLAG="$DOCKER_RUN_FLAG --gpus=all"
    fi
    DOCKER_IMG=$DOCKER_IMG_X86
elif [ $ARCH == "aarch64" ]; then
    if [ $(uname -n) != "raspberry" ] && [ $(uname -n) != "raspberrypi" ]; then
        # 仅在非 CI 环境中启用 NVIDIA runtime
        if [ -z "$CI" ]; then
            DOCKER_RUN_FLAG="$DOCKER_RUN_FLAG --runtime=nvidia"
        fi
    fi
    DOCKER_IMG=$DOCKER_IMG_ARM
fi

export DOCKER_RUN_FLAG
export DOCKER_IMG
export DOCKER_IMG_X86
export DOCKER_IMG_ARM
export HARBOR_REGISTRY
export HARBOR_PROJECT