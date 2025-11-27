#!/bin/bash

#=============================================================================
# Jenkins ARM 模拟支持安装脚本
# 用途：在 x86 Jenkins 服务器上启用 ARM Docker 容器支持
# 作者：唐文浩
# 日期：2025-10-11
#=============================================================================

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

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

# 检查是否为 root
check_root() {
    if [ "$EUID" -ne 0 ]; then
        print_error "此脚本需要 root 权限，请使用 sudo"
        exit 1
    fi
}

# 检查是否为 x86 架构
check_architecture() {
    local arch=$(uname -m)
    if [ "$arch" != "x86_64" ]; then
        print_error "此脚本仅用于 x86_64 架构的主机"
        print_info "当前架构: $arch"
        exit 1
    fi
    print_success "架构检查通过: $arch"
}

# 安装 QEMU
install_qemu() {
    print_header "安装 QEMU 和依赖"

    print_info "更新软件包列表..."
    apt-get update -qq

    print_info "安装 qemu-user-static 和 binfmt-support..."
    apt-get install -y qemu-user-static binfmt-support

    if [ $? -eq 0 ]; then
        print_success "QEMU 安装成功"
    else
        print_error "QEMU 安装失败"
        exit 1
    fi
}

# 启用多架构支持
enable_multiarch() {
    print_header "启用 Docker 多架构支持"

    print_info "配置 binfmt_misc..."
    docker run --rm --privileged multiarch/qemu-user-static --reset -p yes

    if [ $? -eq 0 ]; then
        print_success "多架构支持已启用"
    else
        print_error "多架构支持启用失败"
        exit 1
    fi
}

# 验证 ARM 支持
verify_arm_support() {
    print_header "验证 ARM 支持"

    print_info "测试 ARM64 Alpine 镜像..."
    local result=$(docker run --rm arm64v8/alpine uname -m 2>&1)

    if [ "$result" == "aarch64" ]; then
        print_success "ARM64 支持验证成功 ✓"
        print_info "输出: $result"
    else
        print_warning "ARM64 支持验证失败"
        print_info "输出: $result"
        return 1
    fi

    echo ""
    print_info "测试项目 ARM 构建镜像..."
    local build_result=$(docker run --rm 192.168.1.93/iiri/build_arm_ros2:v1.4.2 uname -m 2>&1)

    if [ "$build_result" == "aarch64" ]; then
        print_success "项目 ARM 镜像验证成功 ✓"
        print_info "输出: $build_result"
    else
        print_warning "项目 ARM 镜像验证失败"
        print_info "输出: $build_result"
        print_info "可能需要先拉取镜像"
    fi
}

# 显示配置信息
show_configuration() {
    print_header "配置信息"

    echo -e "${BLUE}主机架构:${NC} $(uname -m)"
    echo -e "${BLUE}QEMU 版本:${NC} $(qemu-aarch64-static --version | head -1)"
    echo -e "${BLUE}Docker 版本:${NC} $(docker --version)"
    echo ""

    print_info "支持的架构:"
    ls /proc/sys/fs/binfmt_misc/ | grep -E "qemu" | while read arch; do
        echo "  - $arch"
    done
}

# 主函数
main() {
    print_header "Jenkins ARM 模拟支持安装"

    check_root
    check_architecture

    echo ""
    install_qemu

    echo ""
    enable_multiarch

    echo ""
    verify_arm_support

    echo ""
    show_configuration

    echo ""
    print_header "安装完成"
    print_success "Jenkins 现在可以构建 ARM 包了！"
    print_info ""
    print_info "下一步："
    print_info "1. 在 Jenkins Web UI 中触发 ARM 构建"
    print_info "   URL: http://192.168.1.93:8080/job/iiri-layered-build-ci/"
    print_info "   参数: ARCHITECTURE = arm"
    print_info ""
    print_info "2. ARM 构建会比 x86 慢 3-5 倍（正常现象）"
    print_info ""
    print_warning "注意：此配置在主机重启后会失效，需要重新运行："
    print_warning "  docker run --rm --privileged multiarch/qemu-user-static --reset -p yes"
    print_info ""
    print_info "如需开机自动配置，请将上述命令添加到 /etc/rc.local 或 systemd 服务"
}

# 执行主函数
main "$@"
