#!/bin/bash

# IIRI ROS2 一键初始化和编译脚本
# 功能: Harbor登录 + 镜像拉取 + vcstool导入 + 编译

set -e

# 加载环境配置
source iiri_env.sh
source script/docker_utils.sh

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
NC='\033[0m' # No Color

# 打印函数
print_step() {
    echo -e "${PURPLE}[STEP]${NC} $1"
}

print_msg() {
    echo -e "${GREEN}[SETUP]${NC} $1"
}

# 显示横幅
show_banner() {
    echo -e "${PURPLE}"
    echo "╔═══════════════════════════════════════════════════════════════╗"
    echo "║            IIRI ROS2 一键初始化和编译脚本                    ║"
    echo "║                                                               ║"
    echo "║  功能: Harbor登录 + 镜像拉取 + 代码导入 + 编译               ║"
    echo "╚═══════════════════════════════════════════════════════════════╝"
    echo -e "${NC}"
}

# 显示帮助信息
show_help() {
    echo "用法: $0 [选项] [架构]"
    echo ""
    echo "选项:"
    echo "  --skip-login        跳过 Harbor 登录"
    echo "  --skip-pull         跳过镜像拉取（假设镜像已存在）"
    echo "  --skip-import       跳过 vcstool 代码导入"
    echo "  --skip-build        跳过编译（只做准备工作）"
    echo "  --pull-only         只拉取镜像，不做其他操作"
    echo "  --force             强制拉取最新镜像（即使本地已有）"
    echo "  -c, --clean         清理构建缓存"
    echo "  --help, -h          显示此帮助信息"
    echo ""
    echo "架构:"
    echo "  x86                 x86_64 架构（默认自动检测）"
    echo "  arm                 ARM/aarch64 架构"
    echo ""
    echo "示例:"
    echo "  $0                           # 完整流程（登录+拉取+导入+编译）"
    echo "  $0 --skip-login              # 跳过登录，其他步骤正常"
    echo "  $0 --pull-only               # 只拉取镜像"
    echo "  $0 --skip-import             # 跳过代码导入（代码已存在时）"
    echo "  $0 --force                   # 强制拉取最新镜像"
    echo "  $0 -c x86                    # 清理构建并指定 x86 架构"
    echo ""
    echo "新人首次使用推荐:"
    echo "  $0                           # 一键完成所有步骤"
    echo ""
    echo "老用户更新镜像:"
    echo "  $0 --pull-only --force       # 只拉取最新镜像"
}

# 主函数
main() {
    local skip_login=false
    local skip_pull=false
    local skip_import=false
    local skip_build=false
    local pull_only=false
    local force_pull=false
    local clean_build=false
    local target_arch=""

    # 解析参数
    while [[ $# -gt 0 ]]; do
        case $1 in
            --skip-login)
                skip_login=true
                shift
                ;;
            --skip-pull)
                skip_pull=true
                shift
                ;;
            --skip-import)
                skip_import=true
                shift
                ;;
            --skip-build)
                skip_build=true
                shift
                ;;
            --pull-only)
                pull_only=true
                skip_import=true
                skip_build=true
                shift
                ;;
            --force)
                force_pull=true
                shift
                ;;
            -c|--clean)
                clean_build=true
                shift
                ;;
            x86|arm)
                target_arch="$1"
                shift
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

    # 显示横幅
    show_banner

    # 步骤 1: 检查 Docker
    print_step "1. 检查 Docker 环境"
    if ! check_docker_installed; then
        exit 1
    fi
    print_msg "Docker 环境正常"
    echo ""

    # 步骤 2: Harbor 登录
    if [ "$skip_login" == false ]; then
        print_step "2. Harbor 登录"
        if ! harbor_login "$HARBOR_REGISTRY"; then
            print_warn "Harbor 登录失败，但继续执行"
            print_info "如果后续拉取失败，请手动登录:"
            print_info "  docker login $HARBOR_REGISTRY"
        fi
        echo ""
    else
        print_info "跳过 Harbor 登录"
        echo ""
    fi

    # 确定架构
    if [ -z "$target_arch" ]; then
        if [ "$ARCH" == "x86_64" ]; then
            target_arch="x86"
        elif [ "$ARCH" == "aarch64" ]; then
            target_arch="arm"
        fi
    fi

    # 确定镜像
    local docker_image=""
    if [ "$target_arch" == "x86" ]; then
        docker_image="$DOCKER_IMG_X86"
    elif [ "$target_arch" == "arm" ]; then
        docker_image="$DOCKER_IMG_ARM"
    fi

    print_info "目标架构: $target_arch"
    print_info "Docker 镜像: $docker_image"
    echo ""

    # 步骤 3: 拉取镜像
    if [ "$skip_pull" == false ]; then
        print_step "3. 拉取 Docker 镜像"
        if [ "$force_pull" == true ]; then
            print_info "强制拉取最新镜像..."
            if ! pull_image "$docker_image" "true"; then
                print_error "镜像拉取失败"
                exit 1
            fi
        else
            if ! ensure_image "$docker_image" "true"; then
                print_error "镜像准备失败"
                exit 1
            fi
        fi
        echo ""
    else
        print_info "跳过镜像拉取"
        # 仍然检查镜像是否存在
        if ! check_image_exists "$docker_image"; then
            print_error "镜像不存在且跳过拉取，无法继续"
            print_info "请移除 --skip-pull 参数或手动拉取镜像"
            exit 1
        fi
        echo ""
    fi

    # 如果只拉取镜像，到此结束
    if [ "$pull_only" == true ]; then
        print_msg "镜像拉取完成！"
        print_info "现在可以运行编译:"
        print_info "  ./build.sh $target_arch"
        exit 0
    fi

    # 步骤 4: vcstool 导入代码
    if [ "$skip_import" == false ]; then
        print_step "4. 导入分层代码"

        if [ ! -d "src" ] || [ -z "$(ls -A src 2>/dev/null)" ]; then
            print_info "src 目录为空，正在导入代码..."

            # 检查 vcstool 是否安装
            if ! command -v vcs &> /dev/null; then
                print_warn "vcstool 未安装，尝试安装..."
                if ! sudo apt-get install -y python3-vcstool; then
                    print_error "vcstool 安装失败"
                    print_info "请手动安装: sudo apt-get install python3-vcstool"
                    exit 1
                fi
            fi

            # 检查 .repos 文件
            if [ ! -f ".repos" ]; then
                print_error ".repos 文件不存在"
                exit 1
            fi

            # 导入代码
            mkdir -p src
            if vcs import src < .repos; then
                print_msg "代码导入成功"
            else
                print_error "代码导入失败"
                exit 1
            fi
        else
            print_info "src 目录已存在代码，跳过导入"
            print_info "如需重新导入，请删除 src 目录或运行: ./sync.sh clean"
        fi
        echo ""
    else
        print_info "跳过代码导入"
        echo ""
    fi

    # 步骤 5: 编译项目
    if [ "$skip_build" == false ]; then
        print_step "5. 编译项目"

        local build_cmd="./build.sh"
        if [ "$clean_build" == true ]; then
            build_cmd="$build_cmd -c"
        fi
        if [ -n "$target_arch" ]; then
            build_cmd="$build_cmd $target_arch"
        fi

        print_info "执行编译命令: $build_cmd"
        echo ""

        if $build_cmd; then
            echo ""
            print_msg "编译成功！"
        else
            echo ""
            print_error "编译失败"
            exit 1
        fi
    else
        print_info "跳过编译"
        echo ""
    fi

    # 显示完成信息
    echo -e "${GREEN}"
    echo "╔═══════════════════════════════════════════════════════════════╗"
    echo "║                     初始化完成！                             ║"
    echo "╠═══════════════════════════════════════════════════════════════╣"
    echo "║  环境准备完成，您可以：                                       ║"
    echo "║                                                               ║"
    echo "║  • 运行编译: ./build.sh                                      ║"
    echo "║  • 查看状态: ./sync.sh status                                ║"
    echo "║  • 更新代码: ./sync.sh pull                                  ║"
    echo "║  • 更新镜像: $0 --pull-only --force       ║"
    echo "╚═══════════════════════════════════════════════════════════════╝"
    echo -e "${NC}"
}

# 运行主函数
main "$@"
