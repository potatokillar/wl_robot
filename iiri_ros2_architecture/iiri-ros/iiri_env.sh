#!/bin/bash

ARCH=$(uname -m)

DOCKER_IMG_X86=registry.cn-hangzhou.aliyuncs.com/iiri/build_x86_ros2
DOCKER_IMG_ARM=registry.cn-hangzhou.aliyuncs.com/iiri/build_arm_ros2

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
DOCKER_RUN_FLAG="-v "$(pwd):$(pwd)" \
                 -w "$(pwd)" \
                 --env-file env.tmp \
                 -v /dev:/dev \
                 -v /home/$(whoami)/.ros:/home/$(whoami)/.ros \
                 -v /home/$(whoami)/.gazebo:/home/$(whoami)/.gazebo \
                 --rm --network=host --privileged \
                 -v /run/user/1000/pulse/native:/run/pulse/native \
                 --name iiri-ros"

if [ $ARCH == "x86_64" ]; then
    DOCKER_RUN_FLAG="$DOCKER_RUN_FLAG --gpus=all"
    DOCKER_IMG=registry.cn-hangzhou.aliyuncs.com/iiri/build_x86_ros2
elif [ $ARCH == "aarch64" ]; then
    if [ $(uname -n) != "raspberry" ] && [ $(uname -n) != "raspberrypi" ]; then
        DOCKER_RUN_FLAG="$DOCKER_RUN_FLAG --runtime=nvidia"
    fi
    DOCKER_IMG=registry.cn-hangzhou.aliyuncs.com/iiri/build_arm_ros2 
fi

export DOCKER_RUN_FLAG
export DOCKER_IMG
export DOCKER_IMG_ARM