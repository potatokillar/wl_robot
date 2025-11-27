#!/bin/bash

# IIRI ROS2 Architecture Build Script Auto-completion Installer
# 支持永久安装bash补全功能

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMPLETION_SCRIPT="$SCRIPT_DIR/build_layered_completion.sh"
COMPLETION_NAME="build_layered"

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 打印带颜色的消息
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

# 检查补全脚本是否存在
check_completion_script() {
    if [[ ! -f "$COMPLETION_SCRIPT" ]]; then
        print_error "补全脚本不存在: $COMPLETION_SCRIPT"
        exit 1
    fi
}

# 检测系统环境并选择安装方式
detect_install_method() {
    # 方法1: 系统级安装 (需要sudo权限)
    if [[ -d "/etc/bash_completion.d" ]] && [[ -w "/etc/bash_completion.d" || $(id -u) -eq 0 ]]; then
        INSTALL_METHOD="system"
        INSTALL_DIR="/etc/bash_completion.d"
        return
    fi
    
    # 方法2: 用户级安装 (bash-completion目录)
    USER_COMPLETION_DIR="$HOME/.local/share/bash-completion/completions"
    if [[ -d "$(dirname "$USER_COMPLETION_DIR")" ]]; then
        INSTALL_METHOD="user_completion"
        INSTALL_DIR="$USER_COMPLETION_DIR"
        return
    fi
    
    # 方法3: 添加到 ~/.bashrc
    INSTALL_METHOD="bashrc"
    INSTALL_DIR="$HOME/.bashrc"
}

# 检查是否已安装
check_installed() {
    case "$INSTALL_METHOD" in
        "system")
            [[ -f "$INSTALL_DIR/$COMPLETION_NAME" ]]
            ;;
        "user_completion")
            [[ -f "$INSTALL_DIR/$COMPLETION_NAME" ]]
            ;;
        "bashrc")
            grep -q "source.*build_layered_completion.sh" "$INSTALL_DIR" 2>/dev/null
            ;;
    esac
}

# 安装补全功能
install_completion() {
    print_info "检测安装环境..."
    detect_install_method
    
    print_info "使用安装方式: $INSTALL_METHOD"
    print_info "安装目录: $INSTALL_DIR"
    
    # 检查是否已安装
    if check_installed; then
        print_warning "补全功能已安装，是否重新安装? (y/N)"
        read -r response
        if [[ ! "$response" =~ ^[Yy]$ ]]; then
            print_info "取消安装"
            return 0
        fi
    fi
    
    case "$INSTALL_METHOD" in
        "system")
            print_info "安装到系统目录 (可能需要sudo权限)..."
            if sudo cp "$COMPLETION_SCRIPT" "$INSTALL_DIR/$COMPLETION_NAME"; then
                print_success "已安装到系统目录: $INSTALL_DIR/$COMPLETION_NAME"
            else
                print_error "系统目录安装失败，尝试用户目录安装"
                INSTALL_METHOD="user_completion"
                USER_COMPLETION_DIR="$HOME/.local/share/bash-completion/completions"
                INSTALL_DIR="$USER_COMPLETION_DIR"
                install_completion
                return
            fi
            ;;
        "user_completion")
            print_info "安装到用户补全目录..."
            mkdir -p "$INSTALL_DIR"
            if cp "$COMPLETION_SCRIPT" "$INSTALL_DIR/$COMPLETION_NAME"; then
                print_success "已安装到用户目录: $INSTALL_DIR/$COMPLETION_NAME"
            else
                print_error "用户目录安装失败，尝试bashrc方式"
                INSTALL_METHOD="bashrc"
                INSTALL_DIR="$HOME/.bashrc"
                install_completion
                return
            fi
            ;;
        "bashrc")
            print_info "添加到 ~/.bashrc..."
            if ! grep -q "source.*build_layered_completion.sh" "$INSTALL_DIR" 2>/dev/null; then
                echo "" >> "$INSTALL_DIR"
                echo "# IIRI ROS2 Architecture build script auto-completion" >> "$INSTALL_DIR"
                echo "if [[ -f \"$COMPLETION_SCRIPT\" ]]; then" >> "$INSTALL_DIR"
                echo "    source \"$COMPLETION_SCRIPT\"" >> "$INSTALL_DIR"
                echo "fi" >> "$INSTALL_DIR"
                print_success "已添加到 ~/.bashrc"
            else
                print_warning "~/.bashrc 中已存在补全配置"
            fi
            ;;
    esac
    
    print_success "补全功能安装完成！"
    print_info "请重新打开终端或运行 'source ~/.bashrc' 使补全功能生效"
}

# 卸载补全功能
uninstall_completion() {
    print_info "检测已安装的补全功能..."
    
    local uninstalled=false
    
    # 检查系统目录
    if [[ -f "/etc/bash_completion.d/$COMPLETION_NAME" ]]; then
        print_info "发现系统级安装，正在卸载..."
        if sudo rm -f "/etc/bash_completion.d/$COMPLETION_NAME"; then
            print_success "已从系统目录卸载"
            uninstalled=true
        else
            print_error "系统目录卸载失败"
        fi
    fi
    
    # 检查用户补全目录
    USER_COMPLETION_DIR="$HOME/.local/share/bash-completion/completions"
    if [[ -f "$USER_COMPLETION_DIR/$COMPLETION_NAME" ]]; then
        print_info "发现用户级安装，正在卸载..."
        if rm -f "$USER_COMPLETION_DIR/$COMPLETION_NAME"; then
            print_success "已从用户目录卸载"
            uninstalled=true
        else
            print_error "用户目录卸载失败"
        fi
    fi
    
    # 检查 ~/.bashrc
    if grep -q "source.*build_layered_completion.sh" "$HOME/.bashrc" 2>/dev/null; then
        print_info "发现 ~/.bashrc 中的配置，正在移除..."
        # 创建备份
        cp "$HOME/.bashrc" "$HOME/.bashrc.bak.$(date +%Y%m%d_%H%M%S)"
        # 移除相关行
        sed -i '/# IIRI ROS2 Architecture build script auto-completion/,+3d' "$HOME/.bashrc"
        print_success "已从 ~/.bashrc 移除配置"
        uninstalled=true
    fi
    
    if [[ "$uninstalled" == "true" ]]; then
        print_success "补全功能卸载完成！"
        print_info "请重新打开终端使更改生效"
    else
        print_warning "未发现已安装的补全功能"
    fi
}

# 显示帮助信息
show_help() {
    echo "IIRI ROS2 Architecture Build Script Auto-completion Installer"
    echo ""
    echo "用法: $0 [选项]"
    echo ""
    echo "选项:"
    echo "  install     安装补全功能 (默认)"
    echo "  uninstall   卸载补全功能"
    echo "  status      显示安装状态"
    echo "  help        显示此帮助信息"
    echo ""
    echo "安装方式优先级:"
    echo "  1. 系统级: /etc/bash_completion.d/ (需要sudo权限)"
    echo "  2. 用户级: ~/.local/share/bash-completion/completions/"
    echo "  3. 配置文件: ~/.bashrc"
}

# 显示安装状态
show_status() {
    print_info "检查补全功能安装状态..."
    
    local installed=false
    
    # 检查系统目录
    if [[ -f "/etc/bash_completion.d/$COMPLETION_NAME" ]]; then
        print_success "✓ 系统级安装: /etc/bash_completion.d/$COMPLETION_NAME"
        installed=true
    fi
    
    # 检查用户补全目录
    USER_COMPLETION_DIR="$HOME/.local/share/bash-completion/completions"
    if [[ -f "$USER_COMPLETION_DIR/$COMPLETION_NAME" ]]; then
        print_success "✓ 用户级安装: $USER_COMPLETION_DIR/$COMPLETION_NAME"
        installed=true
    fi
    
    # 检查 ~/.bashrc
    if grep -q "source.*build_layered_completion.sh" "$HOME/.bashrc" 2>/dev/null; then
        print_success "✓ ~/.bashrc 配置已添加"
        installed=true
    fi
    
    if [[ "$installed" == "false" ]]; then
        print_warning "补全功能未安装"
        return 1
    fi
    
    return 0
}

# 主函数
main() {
    check_completion_script
    
    case "${1:-install}" in
        "install")
            install_completion
            ;;
        "uninstall")
            uninstall_completion
            ;;
        "status")
            show_status
            ;;
        "help"|"--help"|"-h")
            show_help
            ;;
        *)
            print_error "未知选项: $1"
            show_help
            exit 1
            ;;
    esac
}

# 运行主函数
main "$@"