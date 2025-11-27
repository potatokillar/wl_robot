#!/bin/bash

# IIRI ROS2 分层架构一键安装脚本
# 功能：安装基础依赖、导入所有分层代码、配置vcstool环境

set -e

# 颜色定义
RED='\\033[0;31m'
GREEN='\\033[0;32m'
YELLOW='\\033[1;33m'
BLUE='\\033[0;34m'
PURPLE='\\033[0;35m'
NC='\\033[0m' # No Color

# 打印带颜色的消息
print_msg() {
    echo -e "${GREEN}[INSTALL]${NC} $1"
}

print_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_step() {
    echo -e "${PURPLE}[STEP]${NC} $1"
}

# 显示横幅
show_banner() {
    echo -e "${PURPLE}"
    echo "╔═══════════════════════════════════════════════════════════════╗"
    echo "║                    IIRI ROS2 分层架构                        ║"
    echo "║                      一键安装脚本                            ║"
    echo "║                                                               ║"
    echo "║  功能: 安装基础依赖 + 导入代码 + 配置vcstool环境              ║"
    echo "╚═══════════════════════════════════════════════════════════════╝"
    echo -e "${NC}"
}

# 显示帮助信息
show_help() {
    echo "用法: $0 [选项]"
    echo ""
    echo "功能: 安装系统依赖、配置 vcstool 环境、导入分层代码"
    echo ""
    echo "选项:"
    echo "  --no-deps           跳过依赖安装"
    echo "  --config [配置]     指定vcstool配置 (main|devel|stable，默认main)"
    echo "  --help              显示此帮助信息"
    echo ""
    echo "示例:"
    echo "  $0                           # 标准安装（依赖+代码导入）"
    echo "  $0 --config devel            # 使用开发分支"
    echo "  $0 --no-deps                 # 跳过依赖安装，只导入代码"
    echo "  $0 --no-deps --config stable # 跳过依赖安装，使用稳定版本"
    echo ""
    echo "环境要求:"
    echo "  - Ubuntu 20.04/22.04"
    echo "  - 网络连接到 192.168.1.55"
    echo ""
    echo "完成后续步骤:"
    echo "  本脚本只准备环境和导入代码，不包含编译。"
    echo "  安装完成后，请运行以下命令完成初始化和编译："
    echo ""
    echo "  ./setup-and-build.sh         # Harbor登录 + 拉取镜像 + 编译"
    echo ""
    echo "或者分步执行:"
    echo "  docker login 192.168.1.93    # Harbor 登录"
    echo "  ./build.sh                   # 编译项目"
}

# 检查系统环境
check_environment() {
    print_step "检查系统环境..."

    # 检查操作系统
    if [[ ! -f /etc/os-release ]]; then
        print_error "无法确定操作系统版本"
        exit 1
    fi

    . /etc/os-release
    if [[ "$ID" != "ubuntu" ]]; then
        print_warn "未在Ubuntu系统上测试，可能存在兼容性问题"
    fi

    print_info "操作系统: $PRETTY_NAME"

    # 检查架构
    local arch=$(uname -m)
    print_info "系统架构: $arch"

    # 检查网络连接
    print_info "检查网络连接..."
    if ! ping -c 1 -W 3 192.168.1.55 >/dev/null 2>&1; then
        print_warn "无法连接到GitLab服务器 (192.168.1.55)"
        print_warn "请确保网络连接正常"
    else
        print_info "网络连接正常"
    fi
}

# 安装系统依赖
install_dependencies() {
    print_step "安装系统依赖..."

    # 更新包列表
    print_info "更新包列表..."
    sudo apt-get update

    # 安装基础依赖
    print_info "安装基础依赖..."
    sudo apt-get install -y \
        curl \
        git \
        python3-pip \
        python3-vcstool \
        build-essential \
        cmake \
        software-properties-common

    # 安装基本开发工具（ROS2在Docker镜像中提供）
    print_info "安装基本开发工具..."
    sudo apt-get install -y \
        python3-argcomplete

    # rosdep初始化（如果需要）
    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
        print_info "初始化rosdep..."
        sudo rosdep init || print_warn "rosdep已初始化或权限不足"
    fi
    rosdep update || print_warn "rosdep更新失败，可能需要网络或权限"

    print_msg "依赖安装完成！"
}

# 配置vcstool
setup_vcstool() {
    local config_type="$1"

    print_step "配置vcstool (使用${config_type}配置)..."

    # 检查配置文件是否存在
    local repos_file=".repos"
    if [[ "$config_type" != "main" ]]; then
        repos_file=".repos.${config_type}"
    fi

    if [[ ! -f "$repos_file" ]]; then
        print_error "配置文件 $repos_file 不存在"
        exit 1
    fi

    # 切换配置
    if [[ "$config_type" != "main" ]]; then
        print_info "切换到 $config_type 配置..."
        cp "$repos_file" .repos
    fi

    print_info "当前vcstool配置:"
    cat .repos | head -20

    print_msg "vcstool配置完成！"
}

# 导入代码
import_code() {
    print_step "导入分层代码..."

    # 检查vcstool是否可用
    if ! command -v vcs &> /dev/null; then
        print_error "vcstool未安装"
        exit 1
    fi

    # 创建src目录
    mkdir -p src

    # 导入代码
    print_info "正在从远程仓库导入代码..."
    vcs import src < .repos

    print_msg "代码导入完成！"

    # 显示导入的仓库
    print_info "已导入的仓库:"
    vcs status src
}

# 包依赖说明
package_dependencies_info() {
    print_step "包依赖说明..."

    if [[ -d "src" ]]; then
        print_info "ROS2包依赖在Docker环境中自动处理"
        print_info "如需在本地安装依赖，运行:"
        print_info "  rosdep install --from-paths src --ignore-src -r -y"
    else
        print_warn "src目录不存在，请先运行代码导入"
    fi
}


# 下一步操作提示
next_steps_info() {
    print_step "下一步操作..."

    echo ""
    print_info "环境准备完成！接下来需要："
    echo ""
    print_info "【推荐】一键完成 Harbor 登录 + 镜像拉取 + 编译："
    print_info "  ./setup-and-build.sh"
    echo ""
    print_info "【或】分步执行："
    print_info "  1. Harbor 登录:      docker login 192.168.1.93"
    print_info "  2. 拉取镜像:         docker pull 192.168.1.93/iiri/build_x86_ros2:latest"
    print_info "  3. 编译项目:         ./build.sh"
    echo ""
    print_info "【可选】使用 Docker 环境："
    print_info "  ./docker.sh setup             # 启动Docker并配置vcstool环境"
    print_info "  ./docker.sh run               # 启动普通容器"
    echo ""
}

# 显示完成信息
show_completion() {
    print_step "环境安装完成！"

    echo -e "${GREEN}"
    echo "╔═══════════════════════════════════════════════════════════════╗"
    echo "║                   环境安装成功！                             ║"
    echo "╠═══════════════════════════════════════════════════════════════╣"
    echo "║  已完成:                                                      ║"
    echo "║  ✓ 系统依赖安装                                               ║"
    echo "║  ✓ vcstool 配置                                               ║"
    echo "║  ✓ 分层代码导入                                               ║"
    echo "║                                                               ║"
    echo "║  下一步: 完成 Docker 镜像准备和编译                           ║"
    echo "║                                                               ║"
    echo "║  【推荐】运行一键脚本:                                         ║"
    echo "║    ./setup-and-build.sh                                      ║"
    echo "║                                                               ║"
    echo "║  【或】手动执行:                                               ║"
    echo "║    1. docker login 192.168.1.93                              ║"
    echo "║    2. ./build.sh                                             ║"
    echo "╚═══════════════════════════════════════════════════════════════╝"
    echo -e "${NC}"

    echo ""
    print_info "工作空间位置: $(pwd)"
    print_info "配置文件: $(pwd)/.repos"
    print_info "分层代码: $(pwd)/src/"
    print_info ""
    print_info "日常管理脚本:"
    print_info "  • 代码同步: ./sync.sh status | pull | push"
    print_info "  • 版本管理: ./release.sh validate | create | list"
    print_info "  • 构建编译: ./build.sh | ./build_layered.sh"
    print_info ""
    print_info "查看团队指南: cat VCSTOOL_TEAM_GUIDE.md"
}

# 安装 bash 自动补全
install_bash_completion() {
    local completion_script="$PROJECT_DIR/completions/build_layered_completion.sh"

    # 检查补全脚本是否存在
    if [ ! -f "$completion_script" ]; then
        print_warning "未找到补全脚本: $completion_script"
        return 1
    fi

    print_step "设置 bash 自动补全"

    # 确保脚本可执行
    chmod +x "$completion_script"

    # 加载到当前会话
    if source "$completion_script" 2>/dev/null; then
        print_info "✓ 补全功能已加载到当前会话"
    fi

    # 检查是否已添加到 ~/.bashrc
    if grep -q "build_layered_completion.sh" ~/.bashrc 2>/dev/null; then
        print_info "✓ 自动补全已存在于 ~/.bashrc"
        return 0
    fi

    # 询问是否永久添加
    echo ""
    echo -e "${YELLOW}是否将 bash 自动补全永久添加到 ~/.bashrc？${NC}"
    echo -e "${BLUE}提示: 这将允许您在新终端中使用 Tab 补全 build_layered.sh 的参数${NC}"
    echo -e "  示例: ./build_layered.sh [TAB][TAB] 将显示所有可用选项"
    echo ""
    read -p "添加到 ~/.bashrc? (y/n): " -r response

    if [[ "$response" =~ ^[Yy]$ ]]; then
        {
            echo ""
            echo "# Auto-completion for build_layered.sh"
            echo "source \"$completion_script\""
        } >> ~/.bashrc

        echo -e "${GREEN}✓ 已添加到 ~/.bashrc${NC}"
        echo -e "${BLUE}提示: 在新终端中自动生效，或运行 'source ~/.bashrc' 立即生效${NC}"
    else
        print_info "已跳过 ~/.bashrc 配置（补全仅在当前会话有效）"
    fi

    echo ""
}

# 主函数
main() {
    local skip_deps=false
    local config_type="main"

    # 解析参数
    while [[ $# -gt 0 ]]; do
        case $1 in
            --no-deps)
                skip_deps=true
                shift
                ;;
            --config)
                config_type="$2"
                shift 2
                ;;
            --help|-h)
                show_help
                exit 0
                ;;
            *)
                print_error "未知参数: $1"
                show_help
                exit 1
                ;;
        esac
    done

    # 验证配置类型
    if [[ ! "$config_type" =~ ^(main|devel|stable)$ ]]; then
        print_error "无效的配置类型: $config_type"
        print_info "有效选项: main, devel, stable"
        exit 1
    fi

    # 显示横幅
    show_banner

    # 检查环境
    check_environment

    # 安装依赖
    if [[ "$skip_deps" == false ]]; then
        install_dependencies
    else
        print_info "跳过依赖安装"
    fi

    # 配置vcstool
    setup_vcstool "$config_type"

    # 导入代码
    import_code

    # 包依赖说明
    package_dependencies_info

    # 显示下一步操作
    next_steps_info

    # 安装 bash 自动补全
    install_bash_completion

    # 显示完成信息
    show_completion
}

# 运行主函数
main "$@"