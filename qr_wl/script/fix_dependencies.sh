#!/bin/bash
# fix_dependencies.sh - 自动解决四足机器人程序的依赖缺失问题
# 适用于ARM64 (树莓派) 和 x86_64 系统

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 日志函数
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 检测系统架构
detect_architecture() {
    ARCH=$(uname -m)
    log_info "检测到系统架构: $ARCH"
    
    case $ARCH in
        aarch64)
            SYSTEM_ARCH="arm64"
            ;;
        x86_64)
            SYSTEM_ARCH="x64"
            ;;
        *)
            log_error "不支持的系统架构: $ARCH"
            exit 1
            ;;
    esac
}

# 检测操作系统
detect_os() {
    if [ -f /etc/os-release ]; then
        . /etc/os-release
        OS=$ID
        OS_VERSION=$VERSION_ID
        log_info "检测到操作系统: $OS $OS_VERSION"
    else
        log_error "无法检测操作系统"
        exit 1
    fi
}

# 创建必要的目录
create_directories() {
    local lib_dir="/usr/local/lib"
    local include_dir="/usr/local/include"
    
    sudo mkdir -p "$lib_dir"
    sudo mkdir -p "$include_dir"
    log_info "创建库目录: $lib_dir, $include_dir"
}

# 安装基础依赖
install_base_dependencies() {
    log_info "安装基础依赖包..."
    
    case $OS in
        ubuntu|debian)
            sudo apt-get update
            sudo apt-get install -y wget curl build-essential cmake pkg-config
            ;;
        *)
            log_warning "未知操作系统，跳过基础依赖安装"
            ;;
    esac
}

# 安装 ONNX Runtime
install_onnxruntime() {
    log_info "安装 ONNX Runtime 1.13.1..."
    
    local base_url="https://github.com/microsoft/onnxruntime/releases/download/v1.13.1"
    local onnx_dir="/usr/local/onnxruntime"
    local lib_dir="/usr/local/lib"
    local include_dir="/usr/local/include"
    
    case $SYSTEM_ARCH in
        arm64)
            local package_name="onnxruntime-linux-aarch64-1.13.1.tgz"
            ;;
        x64)
            local package_name="onnxruntime-linux-x64-1.13.1.tgz"
            ;;
    esac
    
    # 检查是否已安装
    if [ -f "$lib_dir/libonnxruntime.so.1.13.1" ]; then
        log_success "ONNX Runtime 已安装"
        return 0
    fi
    
    # 下载和安装
    local temp_dir=$(mktemp -d)
    cd "$temp_dir"
    
    log_info "下载 $package_name..."
    
    # 尝试多种下载方式以解决SSL问题
    local download_success=0
    
    # 方法1: 使用wget with SSL options
    if wget --no-check-certificate --timeout=30 --tries=3 -O "$package_name" "$base_url/$package_name" 2>/dev/null; then
        download_success=1
    else
        log_warning "wget下载失败，尝试使用curl..."
        
        # 方法2: 使用curl with SSL options
        if curl -k -L --connect-timeout 30 --retry 3 -o "$package_name" "$base_url/$package_name" 2>/dev/null; then
            download_success=1
        else
            log_warning "curl下载失败，尝试更新SSL证书..."
            
            # 方法3: 更新SSL证书后重试
            sudo apt-get update && sudo apt-get install -y ca-certificates
            
            if wget --ca-certificate=/etc/ssl/certs/ca-certificates.crt --timeout=30 --tries=3 -O "$package_name" "$base_url/$package_name" 2>/dev/null; then
                download_success=1
            elif curl --cacert /etc/ssl/certs/ca-certificates.crt -L --connect-timeout 30 --retry 3 -o "$package_name" "$base_url/$package_name" 2>/dev/null; then
                download_success=1
            fi
        fi
    fi
    
    if [ $download_success -eq 0 ]; then
        log_error "下载 ONNX Runtime 失败，已尝试所有下载方式"
        log_error "请检查网络连接和防火墙设置"
        return 1
    fi
    
    log_success "ONNX Runtime 下载完成"
    
    log_info "解压 ONNX Runtime..."
    tar -xzf "$package_name"
    
    local extracted_dir=$(find . -maxdepth 1 -type d -name "onnxruntime*" | head -1)
    if [ -z "$extracted_dir" ]; then
        log_error "找不到解压的 ONNX Runtime 目录"
        return 1
    fi
    
    # 安装库文件
    sudo cp "$extracted_dir/lib/libonnxruntime.so"* "$lib_dir/"
    sudo cp -r "$extracted_dir/include"/* "$include_dir/"
    
    # 创建符号链接
    cd "$lib_dir"
    if [ -f "libonnxruntime.so.1.13.1" ]; then
        sudo ln -sf libonnxruntime.so.1.13.1 libonnxruntime.so.1
        sudo ln -sf libonnxruntime.so.1.13.1 libonnxruntime.so
    fi
    
    # 清理临时文件
    rm -rf "$temp_dir"
    
    log_success "ONNX Runtime 安装完成"
}

# 安装 qpOASES
install_qpoases() {
    log_info "安装 qpOASES 3.2..."
    
    local lib_dir="/usr/local/lib"
    local include_dir="/usr/local/include"
    
    # 检查是否已安装
    if [ -f "$lib_dir/libqpOASES.so.3.2" ]; then
        log_success "qpOASES 已安装"
        return 0
    fi
    
    # 下载和编译安装
    local temp_dir=$(mktemp -d)
    cd "$temp_dir"
    
    log_info "下载 qpOASES 源码..."
    
    # 尝试多种下载方式以解决SSL问题
    local download_success=0
    local qpoases_url="https://github.com/coin-or/qpOASES/archive/refs/tags/releases/3.2.1.tar.gz"
    
    # 方法1: 使用wget with SSL options
    if wget --no-check-certificate --timeout=30 --tries=3 -O qpOASES-3.2.1.tar.gz "$qpoases_url" 2>/dev/null; then
        download_success=1
    else
        log_warning "wget下载qpOASES失败，尝试使用curl..."
        
        # 方法2: 使用curl with SSL options
        if curl -k -L --connect-timeout 30 --retry 3 -o qpOASES-3.2.1.tar.gz "$qpoases_url" 2>/dev/null; then
            download_success=1
        else
            log_warning "curl下载qpOASES失败，尝试使用SSL证书..."
            
            # 方法3: 使用SSL证书重试
            if wget --ca-certificate=/etc/ssl/certs/ca-certificates.crt --timeout=30 --tries=3 -O qpOASES-3.2.1.tar.gz "$qpoases_url" 2>/dev/null; then
                download_success=1
            elif curl --cacert /etc/ssl/certs/ca-certificates.crt -L --connect-timeout 30 --retry 3 -o qpOASES-3.2.1.tar.gz "$qpoases_url" 2>/dev/null; then
                download_success=1
            fi
        fi
    fi
    
    if [ $download_success -eq 0 ]; then
        log_error "下载 qpOASES 失败，已尝试所有下载方式"
        log_error "请检查网络连接和防火墙设置"
        return 1
    fi
    
    log_success "qpOASES 源码下载完成"
    
    log_info "解压和编译 qpOASES..."
    tar -xzf qpOASES-3.2.1.tar.gz
    cd qpOASES-releases-3.2.1
    
    # 编译
    mkdir -p build
    cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local
    make -j$(nproc)
    sudo make install
    
    # 创建符号链接
    cd "$lib_dir"
    if [ -f "libqpOASES.so" ]; then
        sudo ln -sf libqpOASES.so libqpOASES.so.3.2
        sudo ln -sf libqpOASES.so libqpOASES.so.3
    fi
    
    # 清理临时文件
    rm -rf "$temp_dir"
    
    log_success "qpOASES 安装完成"
}

# 更新动态链接库缓存
update_ldconfig() {
    log_info "更新动态链接库缓存..."
    
    # 添加库路径到 ld.so.conf
    echo "/usr/local/lib" | sudo tee -a /etc/ld.so.conf.d/usr-local.conf > /dev/null
    
    # 更新缓存
    sudo ldconfig
    
    log_success "动态链接库缓存更新完成"
}

# 验证依赖是否解决
verify_dependencies() {
    log_info "验证依赖库安装状态..."
    
    local qr_binary=""
    
    # 查找 qr 可执行文件
    if [ -f "/home/$(whoami)/bin/qr" ]; then
        qr_binary="/home/$(whoami)/bin/qr"
    elif [ -f "/home/$(whoami)/autorun/qr" ]; then
        qr_binary="/home/$(whoami)/autorun/qr"
    elif [ -f "./qr" ]; then
        qr_binary="./qr"
    else
        log_warning "找不到 qr 可执行文件，跳过验证"
        return 0
    fi
    
    log_info "检查 $qr_binary 的依赖..."
    
    # 检查依赖
    local missing_deps=0
    
    if ldd "$qr_binary" | grep -q "libonnxruntime.so.1.13.1 => not found"; then
        log_error "libonnxruntime.so.1.13.1 仍然缺失"
        missing_deps=$((missing_deps + 1))
    else
        log_success "libonnxruntime.so.1.13.1 已找到"
    fi
    
    if ldd "$qr_binary" | grep -q "libqpOASES.so.3.2 => not found"; then
        log_error "libqpOASES.so.3.2 仍然缺失"
        missing_deps=$((missing_deps + 1))
    else
        log_success "libqpOASES.so.3.2 已找到"
    fi
    
    if [ $missing_deps -eq 0 ]; then
        log_success "所有依赖库已正确安装！"
        return 0
    else
        log_error "仍有 $missing_deps 个依赖库缺失"
        return 1
    fi
}

# 主函数
main() {
    log_info "=== 四足机器人依赖修复脚本 ==="
    log_info "开始修复依赖缺失问题..."
    
    # 检查是否以root身份运行部分命令
    if [ "$EUID" -eq 0 ]; then
        log_warning "请不要以root身份运行此脚本"
        exit 1
    fi
    
    # 检测系统信息
    detect_architecture
    detect_os
    
    # 创建必要目录
    create_directories
    
    # 安装基础依赖
    install_base_dependencies
    
    # 安装 ONNX Runtime
    if ! install_onnxruntime; then
        log_error "ONNX Runtime 安装失败"
        exit 1
    fi
    
    # 安装 qpOASES
    if ! install_qpoases; then
        log_error "qpOASES 安装失败"
        exit 1
    fi
    
    # 更新动态链接库缓存
    update_ldconfig
    
    # 验证安装结果
    if verify_dependencies; then
        log_success "=== 依赖修复完成！==="
        log_info "现在可以尝试运行 qr 程序了"
    else
        log_error "=== 依赖修复失败 ==="
        log_info "请检查错误信息并手动解决剩余问题"
        exit 1
    fi
}

# 脚本入口
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi
