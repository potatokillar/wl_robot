#!/bin/bash

# 创建环境变量， docker并不会自动的将shell命令转化，也不支持空格
echo "USER_NAME=$(whoami)" > env.tmp
echo "USER_UID=$(id -u)" >> env.tmp
echo "USER_GID=$(id -g)" >> env.tmp
echo "USER_HOME=/$(whoami)" >> env.tmp
echo "USER_GROUPS=$(id -Gn)" >> env.tmp 
echo "NVIDIA_DRIVER_CAPABILITIES=all" >> env.tmp

DOCKER_IMG=registry.cn-hangzhou.aliyuncs.com/iiri/build_x86_arm_ros1
DOCKER_RUN_FLAG="-v "$(pwd):$(pwd)" \
                 -w "$(pwd)" \
                 --env-file env.tmp \
                 -v "$HOME"/.ros:"$HOME"/.ros \
                 -v "$HOME"/.gazebo:"$HOME"/.gazebo \
                 --gpus=all \
                 -it --rm --network=host --privileged" 