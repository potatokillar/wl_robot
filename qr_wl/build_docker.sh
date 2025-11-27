#!/bin/bash
#=============================================================================
# qr_wl 增强版 Docker 构建脚本
# 功能：使用 Docker 容器编译项目，支持 x86 和 ARM 架构
# 用法：./build_docker.sh [x86|arm|all] [-c|--clean] [--debug|--release]
# 作者：唐文浩
# 日期：2025-10-14
#=============================================================================

set -e  # 遇到错误立即退出

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 打印函数
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_header() {
    echo ""
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}$1${NC}"
    echo -e "${GREEN}========================================${NC}"
}

# 显示使用说明
show_usage() {
    echo "qr_wl Docker Build Script"
    echo ""
    echo "Usage: $0 [architecture] [options]"
    echo ""
    echo "Arguments:"
    echo "  architecture  Target architecture: x86, arm, or all (default: all)"
    echo ""
    echo "Options:"
    echo "  -c, --clean   Clean build cache before building"
    echo "  --debug       Build in debug mode"
    echo "  --release     Build in release mode (default)"
    echo "  -h, --help    Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0                    # Build both x86 and ARM"
    echo "  $0 x86               # Build x86 only"
    echo "  $0 arm --clean       # Clean and build ARM"
    echo "  $0 all --debug       # Build both in debug mode"
    echo ""
}

# 默认值
ARCH="all"
CLEAN_FLAG=""
BUILD_TYPE="Release"

# 参数解析
while [[ $# -gt 0 ]]; do
    case $1 in
        -c|--clean)
            CLEAN_FLAG="-c"
            shift
            ;;
        --debug)
            BUILD_TYPE="Debug"
            shift
            ;;
        --release)
            BUILD_TYPE="Release"
            shift
            ;;
        -h|--help)
            show_usage
            exit 0
            ;;
        x86|arm|all)
            ARCH=$1
            shift
            ;;
        *)
            print_error "Unknown option: $1"
            show_usage
            exit 1
            ;;
    esac
done

print_header "qr_wl Docker Build System"
print_info "Architecture: ${ARCH}"
print_info "Build Type: ${BUILD_TYPE}"
if [ -n "$CLEAN_FLAG" ]; then
    print_info "Clean Build: Yes"
else
    print_info "Clean Build: No"
fi

# 检查 Docker 是否安装
if ! command -v docker &> /dev/null; then
    print_error "Docker is not installed!"
    print_info "Please install Docker first: https://docs.docker.com/get-docker/"
    exit 1
fi

# 检查 Docker 服务是否运行
if ! docker info &> /dev/null; then
    print_error "Docker is not running!"
    print_info "Please start Docker service: sudo systemctl start docker"
    exit 1
fi

# 加载环境变量
print_info "Loading environment..."
source script/env.sh

# 检查 Docker 镜像是否存在
print_info "Checking Docker image..."
if ! docker images | grep -q "$(echo $DOCKER_IMG | cut -d: -f1)"; then
    print_warning "Docker image not found: $DOCKER_IMG"
    print_info "Please run ./setup_environment.sh to configure environment and download the image"
    print_info "Or manually pull the image: docker pull $DOCKER_IMG"

    read -p "Do you want to pull the image now? (y/N): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        print_info "Pulling Docker image..."
        if docker pull $DOCKER_IMG; then
            print_success "Image pulled successfully!"
        else
            print_error "Failed to pull image. Please run ./setup_environment.sh for complete setup"
            exit 1
        fi
    else
        print_error "Cannot proceed without Docker image"
        exit 1
    fi
else
    print_success "Docker image found"
fi

# 添加 --pull 选项支持
if [[ "$*" == *"--pull"* ]]; then
    print_info "Force pulling latest Docker image..."
    docker pull $DOCKER_IMG || print_warning "Failed to pull latest image, using cached version"
fi

# 构建参数
BUILD_ARGS=""
if [ -n "$CLEAN_FLAG" ]; then
    BUILD_ARGS="$CLEAN_FLAG"
fi
if [ "$ARCH" != "all" ]; then
    BUILD_ARGS="$BUILD_ARGS $ARCH"
fi

# Docker 构建命令
print_info "Starting Docker build..."
print_info "Docker Image: $DOCKER_IMG"

# 运行 Docker 容器进行构建
docker run $DOCKER_RUN_FLAG $DOCKER_IMG bash -c \
    "source ~/.bashrc && \
     export CMAKE_BUILD_TYPE=$BUILD_TYPE && \
     ./script/build.sh $BUILD_ARGS && \
     exit" || {
    print_error "Docker build failed!"
    exit 1
}

# 修复权限问题
print_info "Fixing file permissions..."
if [ -d "./build" ]; then
    sudo chown -R $(whoami):$(id -g -n) ./build 2>/dev/null || true
fi
if [ -d "./onnx_model" ]; then
    sudo chown -R $(whoami):$(id -g -n) ./onnx_model 2>/dev/null || true
fi
if [ -d "./log" ]; then
    sudo chown -R $(whoami):$(id -g -n) ./log 2>/dev/null || true
fi

# 验证构建结果
print_info "Verifying build outputs..."
BUILD_SUCCESS=true

if [ "$ARCH" == "all" ] || [ "$ARCH" == "x86" ]; then
    if [ -f "build/x64/output/qr" ]; then
        print_success "x86 build successful: build/x64/output/qr"
    else
        print_warning "x86 build output not found"
        BUILD_SUCCESS=false
    fi
fi

if [ "$ARCH" == "all" ] || [ "$ARCH" == "arm" ]; then
    if [ -f "build/arm64/output/qr" ]; then
        print_success "ARM build successful: build/arm64/output/qr"
    else
        print_warning "ARM build output not found"
        BUILD_SUCCESS=false
    fi
fi

# 显示构建结果
if [ "$BUILD_SUCCESS" = true ]; then
    print_header "Build Completed Successfully!"

    # 显示文件信息
    if [ "$ARCH" == "all" ] || [ "$ARCH" == "x86" ]; then
        if [ -f "build/x64/output/qr" ]; then
            echo ""
            echo "x86 Binary Information:"
            ls -lh build/x64/output/qr
            file build/x64/output/qr
        fi
    fi

    if [ "$ARCH" == "all" ] || [ "$ARCH" == "arm" ]; then
        if [ -f "build/arm64/output/qr" ]; then
            echo ""
            echo "ARM Binary Information:"
            ls -lh build/arm64/output/qr
            file build/arm64/output/qr
        fi
    fi

    echo ""
    print_info "Next steps:"
    print_info "  1. Test the binary: ./build/*/output/qr config/<config>.toml"
    print_info "  2. Create deployment package: ./deploy_package_qr.sh $ARCH"
else
    print_error "Build failed or incomplete!"
    print_info "Please check the build logs for errors."
    exit 1
fi