#!/bin/bash
source iiri_env.sh

# 加载 Docker 工具函数（如果存在）
if [ -f "script/docker_utils.sh" ]; then
    source script/docker_utils.sh
fi

# 分层架构编译脚本 - 适配当前环境编译所有层
# 基于原wl_ros的build.sh，但适配分层架构

# 无参数时，设置默认编译架构
if [ $ARCH == "x86_64" ]; then
    BUILD_ARCH="x86"
elif [ $ARCH == "aarch64" ]; then
    BUILD_ARCH="arm"
fi

# 显示使用帮助
if [ "$1" == "help" ] || [ "$1" == "-h" ] || [ "$1" == "--help" ]; then
    echo "Usage: $0 [-c] [x86|arm]"
    echo ""
    echo "Options:"
    echo "  -c                Clean build cache before building"
    echo "  x86|arm           Target architecture (auto-detected if not specified)"
    echo ""
    echo "This script builds all layers in the correct order:"
    echo "  1. core_layer"
    echo "  2. hardware_layer"
    echo "  3. perception_layer"
    echo "  4. intelligence_layer"
    echo "  5. application_layer"
    echo ""
    echo "Examples:"
    echo "  $0                # Build all layers for current arch"
    echo "  $0 x86            # Build all layers for x86"
    echo "  $0 -c arm         # Clean build all layers for ARM"
    echo ""
    echo "For more granular control, use build_layered.sh"
    exit 0
fi

# 有参数时，设置想要编译的架构
if [ $# == 1 ];then
    if [ "$1" == 'x86' ]; then
        BUILD_ARCH="x86"
    elif [ "$1" == 'arm' ]; then
        BUILD_ARCH="arm"
    else
        echo "Unknown parameter: $1"
        echo "Use '$0 help' for usage information"
        exit 1
    fi
fi

# 处理-c参数
if [ $# == 2 ];then
    if [ "$1" != '-c' ]; then
        echo "option error"
        exit 1
    fi

    if [ "$2" == 'x86' ]; then
        BUILD_ARCH="x86"
    elif [ "$2" == 'arm' ]; then
        BUILD_ARCH="arm"
    else
        echo "Unknown architecture: $2"
        exit 1
    fi
fi

# 增加-it参数
DOCKER_RUN_FLAG="$DOCKER_RUN_FLAG -it"

# 根据目标选择不同的启动镜像
if [ $BUILD_ARCH == "x86" ]; then
    DOCKER_IMG=$DOCKER_IMG_X86
elif [ $BUILD_ARCH == "arm" ]; then
    DOCKER_IMG=$DOCKER_IMG_ARM
fi

# 检查并确保 Docker 镜像存在
if command -v check_image_exists &> /dev/null; then
    if ! check_image_exists "$DOCKER_IMG"; then
        echo -e "\033[1;33m[WARN]\033[0m Docker 镜像不存在: $DOCKER_IMG"
        echo -e "\033[0;34m[INFO]\033[0m 正在尝试自动拉取镜像..."

        if command -v pull_image &> /dev/null; then
            if ! pull_image "$DOCKER_IMG"; then
                echo -e "\033[0;31m[ERROR]\033[0m 镜像拉取失败"
                echo ""
                echo "请尝试以下方法："
                echo "  1. 登录 Harbor:"
                echo "     docker login $HARBOR_REGISTRY"
                echo ""
                echo "  2. 使用一键脚本:"
                echo "     ./setup-and-build.sh"
                echo ""
                echo "  3. 手动拉取镜像:"
                echo "     docker pull $DOCKER_IMG"
                exit 1
            fi
        else
            echo -e "\033[0;31m[ERROR]\033[0m 无法自动拉取镜像"
            echo "请运行: ./setup-and-build.sh --pull-only"
            exit 1
        fi
    fi
else
    # 如果工具函数不可用，直接检查镜像
    if ! docker image inspect "$DOCKER_IMG" &> /dev/null; then
        echo -e "\033[1;33m[WARN]\033[0m Docker 镜像不存在: $DOCKER_IMG"
        echo ""
        echo "请先拉取镜像："
        echo "  1. 使用一键脚本（推荐）:"
        echo "     ./setup-and-build.sh"
        echo ""
        echo "  2. 或手动拉取:"
        echo "     docker login $HARBOR_REGISTRY"
        echo "     docker pull $DOCKER_IMG"
        exit 1
    fi
fi

# 构建传递给容器内脚本的参数
CONTAINER_ARGS=""
if [ "$1" == "-c" ] || [ "$2" == "-c" ]; then
    CONTAINER_ARGS="$CONTAINER_ARGS -c"
fi

# 显示编译信息
echo "=== ROS2 Layered Build - All Layers ==="
echo "Architecture: $BUILD_ARCH"
echo "Docker Image: $DOCKER_IMG"
if [ -n "$CONTAINER_ARGS" ]; then
    echo "Clean Build: Yes"
fi
echo "Building all layers in order..."
echo "=================================="

# 编译所有层 - 调用容器内的分层编译脚本
docker run $DOCKER_RUN_FLAG $DOCKER_IMG bash -c "source ~/.bashrc && ./script/build_layered.sh $CONTAINER_ARGS && exit"