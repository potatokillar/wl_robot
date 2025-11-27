#!/bin/bash

# 创建环境变量， docker并不会自动的将shell命令转化，也不支持空格
echo "USER_NAME=$(whoami)" > env.tmp
echo "USER_UID=$(id -u)" >> env.tmp
echo "USER_GID=$(id -g)" >> env.tmp
echo "USER_HOME=/home/$(whoami)" >> env.tmp
echo "USER_GROUPS=$(id -Gn)" >> env.tmp

# Docker 镜像配置
# 如果镜像不存在，请运行 ./setup_environment.sh 自动配置环境和下载镜像
# Harbor 仓库地址：192.168.1.93 (用户名: admin, 密码: Westlake1234)
DOCKER_IMG=192.168.1.93/iiri/build_x86_arm_ros1:latest

# 检测是否有 GPU 可用
GPU_FLAG=""
PLATFORM_FLAG=""

# 检测系统架构
ARCH=$(uname -m)
if [ "$ARCH" = "aarch64" ] || [ "$ARCH" = "arm64" ]; then
    # ARM64 架构（如 Orin），使用 runtime=nvidia 而不是 --gpus
    if command -v nvidia-smi &> /dev/null && nvidia-smi &> /dev/null; then
        GPU_FLAG="--runtime=nvidia"
        echo "检测到 NVIDIA GPU (ARM64)，使用 runtime=nvidia"
    fi
    # 在 ARM64 上运行 amd64 镜像需要 qemu，但这里我们直接使用 native arm64
    # PLATFORM_FLAG="--platform linux/arm64"
else
    # x86_64 架构，使用传统的 --gpus 参数
    if command -v nvidia-smi &> /dev/null && nvidia-smi &> /dev/null; then
        GPU_FLAG="--gpus=all"
        echo "检测到 NVIDIA GPU (x86_64)，启用 GPU 支持"
    fi
fi

DOCKER_RUN_FLAG="-v "$(pwd):$(pwd)" \
                 -w "$(pwd)" \
                 -v "$HOME"/.ssh:/home/$(whoami)/.ssh \
                 -v /dev:/dev \
                 --env-file env.tmp \
                 -i --rm --network=host --privileged ${GPU_FLAG} ${PLATFORM_FLAG}" 

