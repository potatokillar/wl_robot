#!/bin/bash
source iiri_env.sh

DOCKER_RUN_FLAG="$DOCKER_RUN_FLAG -it"

# 添加vcstool安装脚本
VCSTOOL_INSTALL="apt-get update && apt-get install -y python3-vcstool && echo 'vcstool installed successfully'"

if [ $# == 0 ]; then
    echo "参数错误，参数必须是run或者exec，参考例子如下"
    echo "./docker.sh run [arm]         # 启动容器"
    echo "./docker.sh exec <容器ID>      # 进入容器"
    echo "./docker.sh setup             # 初始化vcstool工作空间"
    exit 1
elif [ $1 == "run" ]; then
    echo $DOCKER_RUN_FLAG
    if [ -z "$DISPLAY" ]; then
        echo "启动无界面版本，若想启动有界面版本，请在ubuntu原生桌面下启动"  
        if [ "$2" = "arm" ]; then
            docker run $DOCKER_RUN_FLAG $DOCKER_IMG_ARM bash
        else
            docker run $DOCKER_RUN_FLAG $DOCKER_IMG bash
        fi
    else
        export DISPLAY=$DISPLAY
        echo "启动有界面版本"
        xhost +
        if [ "$2" = "arm" ]; then
            docker run $DOCKER_RUN_FLAG \
                -v /tmp/.X11-unix:/tmp/.X11-unix \
                -e DISPLAY=$DISPLAY \
                --env="NVIDIA_DRIVER_CAPABILITIES=all" \
                $DOCKER_IMG_ARM bash
        else
            docker run $DOCKER_RUN_FLAG \
                -v /tmp/.X11-unix:/tmp/.X11-unix \
                -e DISPLAY=$DISPLAY \
                --env="NVIDIA_DRIVER_CAPABILITIES=all" \
                $DOCKER_IMG bash
        fi
        xhost -
    fi
elif [ $1 == "exec" ]; then
    if [ -z "$2" ]; then
        echo "错误：exec 命令需要一个容器ID作为第二个参数"
        echo "示例: ./docker.sh exec <容器ID>"
        exit 1
    fi
    docker exec -it -u "$USER:$USER"  $2 bash
elif [ $1 == "setup" ]; then
    echo "正在初始化vcstool工作空间..."

    # 检查是否存在.repos文件
    if [ ! -f ".repos" ]; then
        echo "错误：未找到.repos配置文件"
        echo "请确保当前目录包含.repos文件"
        exit 1
    fi

    # 显示当前配置
    echo "当前vcstool配置:"
    cat .repos

    echo ""
    echo "提示：进入容器后，运行以下命令来导入代码："
    echo "  mkdir -p src"
    echo "  vcs import src < .repos"
    echo "  ./build.sh"

    # 启动容器并安装vcstool
    if [ -z "$DISPLAY" ]; then
        if [ "$2" = "arm" ]; then
            docker run $DOCKER_RUN_FLAG -e "VCSTOOL_INSTALL=$VCSTOOL_INSTALL" \
                --entrypoint bash $DOCKER_IMG_ARM \
                -c "eval \$VCSTOOL_INSTALL && exec bash"
        else
            docker run $DOCKER_RUN_FLAG -e "VCSTOOL_INSTALL=$VCSTOOL_INSTALL" \
                --entrypoint bash $DOCKER_IMG \
                -c "eval \$VCSTOOL_INSTALL && exec bash"
        fi
    else
        export DISPLAY=$DISPLAY
        xhost +
        if [ "$2" = "arm" ]; then
            docker run $DOCKER_RUN_FLAG \
                -v /tmp/.X11-unix:/tmp/.X11-unix \
                -e DISPLAY=$DISPLAY \
                -e "VCSTOOL_INSTALL=$VCSTOOL_INSTALL" \
                --env="NVIDIA_DRIVER_CAPABILITIES=all" \
                --entrypoint bash $DOCKER_IMG_ARM \
                -c "eval \$VCSTOOL_INSTALL && exec bash"
        else
            docker run $DOCKER_RUN_FLAG \
                -v /tmp/.X11-unix:/tmp/.X11-unix \
                -e DISPLAY=$DISPLAY \
                -e "VCSTOOL_INSTALL=$VCSTOOL_INSTALL" \
                --env="NVIDIA_DRIVER_CAPABILITIES=all" \
                --entrypoint bash $DOCKER_IMG \
                -c "eval \$VCSTOOL_INSTALL && exec bash"
        fi
        xhost -
    fi
fi

