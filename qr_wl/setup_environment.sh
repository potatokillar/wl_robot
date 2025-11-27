#!/bin/bash
#=============================================================================
# qr_wl 环境配置脚本
# 功能：自动配置 Docker 环境，登录 Harbor 仓库，拉取编译镜像
# 作者：唐文浩
# 日期：2025-10-15
#
# Harbor 仓库信息：
#   地址：192.168.1.93
#   用户名：admin
#   密码：Westlake1234
#=============================================================================

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
MAGENTA='\033[0;35m'
NC='\033[0m' # No Color

# Harbor 配置（硬编码）
HARBOR_REGISTRY="192.168.1.93"
HARBOR_USER="admin"
HARBOR_PASSWORD="Westlake1234"
DOCKER_IMAGE="192.168.1.93/iiri/build_x86_arm_ros1:latest"

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
    echo -e "${CYAN}========================================${NC}"
    echo -e "${CYAN}$1${NC}"
    echo -e "${CYAN}========================================${NC}"
}

print_step() {
    echo ""
    echo -e "${MAGENTA}>>> $1${NC}"
}

# 显示欢迎信息
show_welcome() {
    clear
    echo -e "${GREEN}"
    echo "============================================"
    echo "       qr_wl 环境配置向导 v1.0"
    echo "============================================"
    echo -e "${NC}"
    echo "此脚本将自动完成以下任务："
    echo "  1. 检查 Docker 安装状态"
    echo "  2. 配置 Harbor 不安全仓库"
    echo "  3. 登录 Harbor 仓库"
    echo "  4. 拉取编译所需的 Docker 镜像"
    echo "  5. 验证环境配置"
    echo ""
    echo -e "${YELLOW}Harbor 仓库信息：${NC}"
    echo "  地址：$HARBOR_REGISTRY"
    echo "  用户：$HARBOR_USER"
    echo "  密码：******* (已配置)"
    echo ""
    read -p "按 Enter 键开始配置，或按 Ctrl+C 退出... "
}

# 检查是否以 root 运行
check_root() {
    if [[ $EUID -eq 0 ]]; then
        print_warning "检测到以 root 用户运行"
        print_info "建议使用普通用户运行此脚本"
        read -p "是否继续？(y/N): " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            exit 1
        fi
    fi
}

# 检查 Docker 安装
check_docker_installation() {
    print_step "步骤 1: 检查 Docker 安装"

    if ! command -v docker &> /dev/null; then
        print_error "Docker 未安装！"
        print_info "请先安装 Docker："
        echo ""
        echo "  # Ubuntu/Debian:"
        echo "  curl -fsSL https://get.docker.com | sh"
        echo "  sudo usermod -aG docker \$USER"
        echo "  newgrp docker"
        echo ""
        exit 1
    fi

    print_success "Docker 已安装"
    docker --version
}

# 检查 Docker 服务
check_docker_service() {
    print_step "步骤 2: 检查 Docker 服务"

    if ! docker info &> /dev/null; then
        print_warning "Docker 服务未运行或无权限访问"

        # 尝试启动 Docker 服务
        print_info "尝试启动 Docker 服务..."
        if command -v systemctl &> /dev/null; then
            sudo systemctl start docker 2>/dev/null || true
            sleep 2
        fi

        # 再次检查
        if ! docker info &> /dev/null; then
            print_error "无法访问 Docker 服务"
            print_info "请执行以下命令："
            echo "  sudo systemctl start docker"
            echo "  sudo usermod -aG docker \$USER"
            echo "  newgrp docker"
            exit 1
        fi
    fi

    print_success "Docker 服务正常运行"
}

# 配置 Harbor 不安全仓库
configure_insecure_registry() {
    print_step "步骤 3: 配置 Harbor 不安全仓库"

    DAEMON_FILE="/etc/docker/daemon.json"
    NEED_RESTART=false

    # 检查当前配置
    if [ -f "$DAEMON_FILE" ]; then
        if grep -q "\"$HARBOR_REGISTRY\"" "$DAEMON_FILE" 2>/dev/null; then
            print_success "Harbor 仓库已在 Docker 配置中"
        else
            print_info "需要添加 Harbor 到不安全仓库列表"
            NEED_RESTART=true

            # 备份原配置
            sudo cp "$DAEMON_FILE" "${DAEMON_FILE}.bak.$(date +%Y%m%d_%H%M%S)"

            # 更新配置
            if grep -q "insecure-registries" "$DAEMON_FILE"; then
                # 已有 insecure-registries，添加新仓库
                sudo python3 -c "
import json
with open('$DAEMON_FILE', 'r') as f:
    config = json.load(f)
if 'insecure-registries' not in config:
    config['insecure-registries'] = []
if '$HARBOR_REGISTRY' not in config['insecure-registries']:
    config['insecure-registries'].append('$HARBOR_REGISTRY')
with open('$DAEMON_FILE', 'w') as f:
    json.dump(config, f, indent=2)
"
            else
                # 没有 insecure-registries，创建新的
                sudo python3 -c "
import json
import os
config = {}
if os.path.exists('$DAEMON_FILE'):
    with open('$DAEMON_FILE', 'r') as f:
        try:
            config = json.load(f)
        except:
            config = {}
config['insecure-registries'] = ['$HARBOR_REGISTRY']
with open('$DAEMON_FILE', 'w') as f:
    json.dump(config, f, indent=2)
"
            fi
        fi
    else
        print_info "创建 Docker daemon 配置文件"
        NEED_RESTART=true
        echo '{
  "insecure-registries": ["'$HARBOR_REGISTRY'"]
}' | sudo tee "$DAEMON_FILE" > /dev/null
    fi

    # 重启 Docker 服务
    if [ "$NEED_RESTART" = true ]; then
        print_info "重启 Docker 服务以应用配置..."
        if command -v systemctl &> /dev/null; then
            sudo systemctl daemon-reload
            sudo systemctl restart docker
            sleep 5
        else
            print_warning "请手动重启 Docker 服务"
        fi
        print_success "Docker 配置已更新"
    fi

    # 显示最终配置
    print_info "当前 Docker daemon 配置："
    cat "$DAEMON_FILE" 2>/dev/null || echo "无法读取配置文件"
}

# 检查网络连接
check_network() {
    print_step "步骤 4: 检查与 Harbor 的网络连接"

    if ping -c 1 -W 3 "$HARBOR_REGISTRY" &> /dev/null; then
        print_success "网络连接正常"
    else
        print_error "无法连接到 Harbor 服务器 ($HARBOR_REGISTRY)"
        print_info "请检查："
        echo "  1. Harbor 服务器是否运行"
        echo "  2. 网络连接是否正常"
        echo "  3. 防火墙设置"
        exit 1
    fi

    # 检查 Harbor HTTP 服务
    if command -v curl &> /dev/null; then
        if curl -s --connect-timeout 5 "http://$HARBOR_REGISTRY/api/v2.0/systeminfo" &> /dev/null; then
            print_success "Harbor 服务响应正常"
        else
            print_warning "Harbor 服务可能未完全启动，继续尝试..."
        fi
    fi
}

# 登录 Harbor
login_harbor() {
    print_step "步骤 5: 登录 Harbor 仓库"

    print_info "使用预配置的账号登录 Harbor..."

    # 使用 --password-stdin 更安全
    if echo "$HARBOR_PASSWORD" | docker login "$HARBOR_REGISTRY" -u "$HARBOR_USER" --password-stdin 2>/dev/null; then
        print_success "Harbor 登录成功！"
    else
        print_error "Harbor 登录失败"
        print_info "请检查："
        echo "  1. 用户名和密码是否正确"
        echo "  2. Harbor 服务是否正常"
        echo "  3. Docker daemon 配置是否正确"
        exit 1
    fi
}

# 拉取 Docker 镜像
pull_docker_image() {
    print_step "步骤 6: 拉取编译镜像"

    print_info "镜像：$DOCKER_IMAGE"
    print_info "这可能需要几分钟时间，请耐心等待..."

    if docker pull "$DOCKER_IMAGE"; then
        print_success "镜像拉取成功！"

        # 显示镜像信息
        echo ""
        print_info "镜像信息："
        docker images --format "table {{.Repository}}:{{.Tag}}\t{{.ID}}\t{{.Size}}\t{{.CreatedAt}}" | grep -E "REPOSITORY|$DOCKER_IMAGE" || true
    else
        print_error "镜像拉取失败"
        print_info "请检查："
        echo "  1. Harbor 登录是否成功"
        echo "  2. 镜像名称是否正确"
        echo "  3. 网络连接是否稳定"
        exit 1
    fi
}

# 验证环境
verify_environment() {
    print_step "步骤 7: 验证环境配置"

    VERIFY_SUCCESS=true

    # 检查 Docker
    if docker ps &> /dev/null; then
        print_success "✓ Docker 运行正常"
    else
        print_error "✗ Docker 访问异常"
        VERIFY_SUCCESS=false
    fi

    # 检查 Harbor 配置
    if docker info 2>/dev/null | grep -q "$HARBOR_REGISTRY"; then
        print_success "✓ Harbor 不安全仓库已配置"
    else
        print_warning "⚠ Harbor 配置可能未生效"
    fi

    # 检查镜像
    if docker images | grep -q "build_x86_arm_ros1"; then
        print_success "✓ 编译镜像已下载"
    else
        print_error "✗ 编译镜像未找到"
        VERIFY_SUCCESS=false
    fi

    # 测试运行
    print_info "测试 Docker 镜像运行..."
    if docker run --rm "$DOCKER_IMAGE" echo "Docker 镜像测试成功" &> /dev/null; then
        print_success "✓ Docker 镜像可以正常运行"
    else
        print_warning "⚠ Docker 镜像运行测试失败（可能需要特殊参数）"
    fi

    if [ "$VERIFY_SUCCESS" = true ]; then
        print_header "环境配置完成！"
        echo ""
        echo -e "${GREEN}恭喜！所有环境配置已完成。${NC}"
        echo ""
        echo "你现在可以使用以下命令编译项目："
        echo -e "${CYAN}  ./build_docker.sh        # Docker 编译（推荐）${NC}"
        echo -e "${CYAN}  ./build.sh               # 传统编译方式${NC}"
        echo ""
        echo "编译特定架构："
        echo -e "${CYAN}  ./build_docker.sh x86    # 编译 x86 版本${NC}"
        echo -e "${CYAN}  ./build_docker.sh arm    # 编译 ARM 版本${NC}"
        echo ""
        echo "其他有用的命令："
        echo -e "${CYAN}  docker images            # 查看已下载的镜像${NC}"
        echo -e "${CYAN}  ./docker.sh              # 进入 Docker 开发环境${NC}"
    else
        print_error "环境配置未完全成功，请检查错误信息"
        exit 1
    fi
}

# 清理函数
cleanup() {
    print_warning "脚本被中断"
    exit 1
}

# 主函数
main() {
    # 设置中断处理
    trap cleanup INT

    # 显示欢迎信息
    show_welcome

    # 检查权限
    check_root

    # 执行配置步骤
    check_docker_installation
    check_docker_service
    configure_insecure_registry
    check_network
    login_harbor
    pull_docker_image
    verify_environment

    print_success "环境配置脚本执行完成！"
}

# 脚本入口
main "$@"