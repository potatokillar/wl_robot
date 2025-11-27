#!/bin/bash
source iiri_env.sh

# 加载 Docker 工具函数（如果存在）
if [ -f "script/docker_utils.sh" ]; then
    source script/docker_utils.sh
fi

# 分层编译脚本 - 容器外部使用
# 支持编译指定层级或所有层级

# 无参数时，设置默认编译架构
if [ $ARCH == "x86_64" ]; then
    BUILD_ARCH="x86"
elif [ $ARCH == "aarch64" ]; then
    BUILD_ARCH="arm"
fi

# 显示使用帮助
if [ "$1" == "help" ] || [ "$1" == "-h" ] || [ "$1" == "--help" ]; then
    echo "Usage: $0 [-c] [x86|arm] [layer_name]"
    echo ""
    echo "Options:"
    echo "  -c                Clean build cache before building"
    echo "  x86|arm           Target architecture (auto-detected if not specified)"
    echo ""
    echo "Layer names:"
    echo "  core_layer        Build core layer only"
    echo "  hardware_layer    Build core + hardware layers"
    echo "  perception_layer  Build up to perception layer"
    echo "  intelligence_layer Build up to intelligence layer"
    echo "  application_layer Build all layers (default)"
    echo ""
    echo "Examples:"
    echo "  $0                          # Build all layers for current arch"
    echo "  $0 x86                      # Build all layers for x86"
    echo "  $0 -c arm                   # Clean build all layers for ARM"
    echo "  $0 perception_layer         # Build up to perception layer"
    echo "  $0 -c x86 core_layer        # Clean build core layer for x86"
    exit 0
fi

# 处理参数 - 按照原build.sh的逻辑
CLEAN_BUILD=false
TARGET_LAYER=""
USE_CERES="OFF"

# 1个参数的情况
if [ $# == 1 ]; then
    if [ "$1" == 'x86' ]; then
        BUILD_ARCH="x86"
    elif [ "$1" == 'arm' ]; then
        BUILD_ARCH="arm"
    elif [ "$1" == 'core_layer' ] || [ "$1" == 'hardware_layer' ] || [ "$1" == 'perception_layer' ] || [ "$1" == 'intelligence_layer' ] || [ "$1" == 'application_layer' ]; then
        TARGET_LAYER="$1"
    else
        echo "Unknown parameter: $1"
        echo "Use '$0 help' for usage information"
        exit 1
    fi
fi

# 2个参数的情况
if [ $# == 2 ]; then
    if [ "$1" == '-c' ]; then
        CLEAN_BUILD=true
        if [ "$2" == 'x86' ]; then
            BUILD_ARCH="x86"
        elif [ "$2" == 'arm' ]; then
            BUILD_ARCH="arm"
        elif [ "$2" == 'core_layer' ] || [ "$2" == 'hardware_layer' ] || [ "$2" == 'perception_layer' ] || [ "$2" == 'intelligence_layer' ] || [ "$2" == 'application_layer' ]; then
            TARGET_LAYER="$2"
        else
            echo "Unknown parameter: $2"
            exit 1
        fi
    elif [ "$1" == 'x86' ] || [ "$1" == 'arm' ]; then
        BUILD_ARCH="$1"
        if [ "$2" == 'core_layer' ] || [ "$2" == 'hardware_layer' ] || [ "$2" == 'perception_layer' ] || [ "$2" == 'intelligence_layer' ] || [ "$2" == 'application_layer' ]; then
            TARGET_LAYER="$2"
        else
            echo "Unknown layer: $2"
            exit 1
        fi
    else
        echo "Invalid parameter combination"
        exit 1
    fi
fi

# 3个参数的情况
if [ $# == 3 ]; then
    if [ "$1" == '-c' ]; then
        CLEAN_BUILD=true
        if [ "$2" == 'x86' ] || [ "$2" == 'arm' ]; then
            BUILD_ARCH="$2"
            if [ "$3" == 'core_layer' ] || [ "$3" == 'hardware_layer' ] || [ "$3" == 'perception_layer' ] || [ "$3" == 'intelligence_layer' ] || [ "$3" == 'application_layer' ]; then
                TARGET_LAYER="$3"
            else
                echo "Unknown layer: $3"
                exit 1
            fi
        else
            echo "Unknown architecture: $2"
            exit 1
        fi
    else
        echo "Invalid parameter combination"
        exit 1
    fi
fi

# 参数过多的情况
if [ $# -gt 3 ]; then
    echo "Too many parameters"
    echo "Use '$0 help' for usage information"
    exit 1
fi

# 增加-it参数（仅在有 TTY 时）
# 如果 stdin 是终端设备，或者不是 CI 环境，使用 -it
if [ -t 0 ] && [ -z "$CI" ]; then
    DOCKER_RUN_FLAG="$DOCKER_RUN_FLAG -it"
fi

# 根据目标选择不同的启动镜像
if [ $BUILD_ARCH == "x86" ]; then
    DOCKER_IMG=$DOCKER_IMG_X86
elif [ $BUILD_ARCH == "arm" ]; then
    DOCKER_IMG=$DOCKER_IMG_ARM
fi

# 检查并确保 Docker 镜像存在（在非 CI 环境中）
if [ -z "$CI" ]; then
    if command -v check_image_exists &> /dev/null; then
        if ! check_image_exists "$DOCKER_IMG"; then
            echo -e "\033[1;33m[WARN]\033[0m Docker 镜像不存在: $DOCKER_IMG"
            echo -e "\033[0;34m[INFO]\033[0m 正在尝试自动拉取镜像..."

            if command -v pull_image &> /dev/null; then
                if ! pull_image "$DOCKER_IMG"; then
                    echo -e "\033[0;31m[ERROR]\033[0m 镜像拉取失败"
                    echo ""
                    echo "请尝试以下方法："
                    echo "  1. 使用一键脚本:"
                    echo "     ./setup-and-build.sh"
                    echo ""
                    echo "  2. 登录并手动拉取:"
                    echo "     docker login $HARBOR_REGISTRY"
                    echo "     docker pull $DOCKER_IMG"
                    exit 1
                fi
            fi
        fi
    else
        # 如果工具函数不可用，直接检查镜像
        if ! docker image inspect "$DOCKER_IMG" &> /dev/null; then
            echo -e "\033[1;33m[WARN]\033[0m Docker 镜像不存在: $DOCKER_IMG"
            echo ""
            echo "请先拉取镜像："
            echo "  ./setup-and-build.sh --pull-only"
            exit 1
        fi
    fi
fi

# CI ARM 构建：添加 Docker 资源限制以降低 QEMU 负载
DOCKER_RESOURCE_FLAGS=""
if [ -n "$CI" ] && [ "$BUILD_ARCH" == "arm" ]; then
    DOCKER_RESOURCE_FLAGS="--memory=8g --shm-size=2g"
    echo "CI ARM 构建：启用 Docker 资源限制（8GB 内存，2GB 共享内存）"
fi

# 传递目标架构和 CI 标志到容器内（用于 CI ARM Debug 模式检测）
DOCKER_ENV_FLAGS="-e TARGET_ARCH=$BUILD_ARCH -e CI=$CI"

# 构建传递给容器内脚本的参数
CONTAINER_ARGS=""
if [ "$CLEAN_BUILD" = true ]; then
    CONTAINER_ARGS="$CONTAINER_ARGS -c"
fi
if [ "$USE_CERES" = "ON" ]; then
    CONTAINER_ARGS="$CONTAINER_ARGS --ceres"
fi
if [ -n "$TARGET_LAYER" ]; then
    CONTAINER_ARGS="$CONTAINER_ARGS $TARGET_LAYER"
fi

# 显示编译信息
echo "=== ROS2 Layered Build (External) ==="
echo "Architecture: $BUILD_ARCH"
echo "Docker Image: $DOCKER_IMG"
echo "Ceres Optimization: $USE_CERES"
if [ -n "$TARGET_LAYER" ]; then
    echo "Target Layer: $TARGET_LAYER"
else
    echo "Target Layer: All layers"
fi
if [ "$CLEAN_BUILD" = true ]; then
    echo "Clean Build: Yes"
fi
if [ -n "$DOCKER_RESOURCE_FLAGS" ]; then
    echo "Docker Resources: 8GB memory, 2GB shm"
fi
echo "=================================="

# ============================================================
# 前端预构建（在 Docker 外部进行）
# ============================================================
# 如果要编译 application_layer 且存在前端源码，先在宿主机上构建前端
if [ -z "$TARGET_LAYER" ] || [ "$TARGET_LAYER" == "application_layer" ]; then
    FRONTEND_SRC_DIR="src/application_layer/src/dev_server/frontend_src"
    FRONTEND_SCRIPT="$FRONTEND_SRC_DIR/build_frontend.sh"

    if [ -d "$FRONTEND_SRC_DIR" ] && [ -f "$FRONTEND_SCRIPT" ]; then
        # 检查宿主机是否有 Node.js 和 npm
        if command -v node &> /dev/null && command -v npm &> /dev/null; then
            NODE_VERSION=$(node --version)
            NPM_VERSION=$(npm --version)
            echo ""
            echo "=== 前端预构建（宿主机） ==="
            echo "Node.js: $NODE_VERSION"
            echo "npm: $NPM_VERSION"
            echo "位置: $FRONTEND_SRC_DIR"
            echo "开始构建..."

            # 执行前端构建
            if bash "$FRONTEND_SCRIPT"; then
                echo "✓ 前端构建成功"
                echo "============================"
            else
                echo "⚠️  前端构建失败，但将继续 Docker 编译（后端仍可用）"
                echo "============================"
            fi
        else
            echo ""
            echo "=== 前端构建跳过 ==="
            echo "原因: 宿主机未安装 Node.js 或 npm"
            echo "影响: 最终部署包将不包含 Web 界面"
            echo "=========================="
        fi
    fi
fi

# 调试信息：显示 Docker 命令
if [ -n "$CI" ]; then
    echo "DEBUG: Current directory: $(pwd)"
    echo "DEBUG: Files in current directory:"
    ls -la
    echo "DEBUG: DOCKER_RUN_FLAG: $DOCKER_RUN_FLAG"
    echo "DEBUG: DOCKER_IMG: $DOCKER_IMG"
    echo "DEBUG: Checking files in container:"
    docker run $DOCKER_RUN_FLAG $DOCKER_IMG bash -c "pwd && ls -la && ls -la script/" || true
fi

# 编译 - 调用容器内的分层编译脚本
docker run $DOCKER_RUN_FLAG $DOCKER_ENV_FLAGS $DOCKER_RESOURCE_FLAGS $DOCKER_IMG bash -c "source ~/.bashrc && ./script/build_layered.sh $CONTAINER_ARGS && exit"