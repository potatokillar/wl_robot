#!/bin/bash

# Docker 工具函数库
# 提供 Docker 镜像管理的通用函数

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 打印函数
print_msg() {
    echo -e "${GREEN}[DOCKER]${NC} $1"
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

# 检查 Docker 是否安装
check_docker_installed() {
    if ! command -v docker &> /dev/null; then
        print_error "Docker 未安装"
        print_info "请先安装 Docker: https://docs.docker.com/get-docker/"
        return 1
    fi

    # 检查 Docker 服务是否运行
    if ! docker info &> /dev/null; then
        print_error "Docker 服务未运行"
        print_info "请启动 Docker 服务: sudo systemctl start docker"
        return 1
    fi

    return 0
}

# Harbor 登录
harbor_login() {
    local registry="${1:-192.168.1.93}"
    local username="${2:-admin}"
    local password="${3:-Westlake1234}"

    print_msg "正在登录 Harbor 镜像仓库..."
    print_info "Registry: $registry"
    print_info "Username: $username"

    # 检查是否已登录
    if docker info 2>/dev/null | grep -q "$registry"; then
        print_msg "已登录 Harbor，跳过"
        return 0
    fi

    # 执行登录（使用默认凭证）
    if echo "$password" | docker login "$registry" -u "$username" --password-stdin 2>/dev/null; then
        print_msg "Harbor 登录成功"
        return 0
    else
        print_error "Harbor 登录失败"
        print_warn "默认凭证无效，请检查 Harbor 服务器状态或联系管理员"
        return 1
    fi
}

# 检查镜像是否存在
check_image_exists() {
    local image="$1"

    if docker image inspect "$image" &> /dev/null; then
        return 0
    else
        return 1
    fi
}

# 拉取镜像
pull_image() {
    local image="$1"
    local force="${2:-false}"

    print_msg "正在拉取镜像: $image"

    # 如果镜像已存在且不强制拉取
    if check_image_exists "$image" && [ "$force" != "true" ]; then
        print_info "镜像已存在，跳过拉取"
        print_info "使用 --force 参数强制更新镜像"
        return 0
    fi

    # 拉取镜像
    if docker pull "$image"; then
        print_msg "镜像拉取成功"
        return 0
    else
        print_error "镜像拉取失败"
        print_warn "请检查："
        print_warn "  1. 网络连接是否正常"
        print_warn "  2. 是否已登录 Harbor (docker login ${HARBOR_REGISTRY})"
        print_warn "  3. 镜像名称是否正确"
        return 1
    fi
}

# 确保镜像存在（不存在则拉取）
ensure_image() {
    local image="$1"
    local auto_pull="${2:-true}"

    if check_image_exists "$image"; then
        print_info "镜像已存在: $image"

        # 检查镜像年龄
        local image_age=$(docker image inspect "$image" --format='{{.Created}}')
        print_info "镜像创建时间: $image_age"
        print_info "提示: 使用 --force 参数可以强制更新镜像"
        return 0
    fi

    print_warn "镜像不存在: $image"

    if [ "$auto_pull" == "true" ]; then
        print_info "正在自动拉取镜像..."
        if pull_image "$image"; then
            return 0
        else
            print_error "自动拉取失败"
            return 1
        fi
    else
        print_error "请手动拉取镜像:"
        print_info "  docker pull $image"
        print_info "或运行:"
        print_info "  ./setup-and-build.sh --pull-only"
        return 1
    fi
}

# 获取镜像信息
get_image_info() {
    local image="$1"

    if ! check_image_exists "$image"; then
        print_error "镜像不存在: $image"
        return 1
    fi

    print_info "镜像信息:"
    docker image inspect "$image" --format='  ID: {{.Id}}
  创建时间: {{.Created}}
  大小: {{.Size}} bytes
  架构: {{.Architecture}}
  OS: {{.Os}}'
}

# 列出本地 IIRI 镜像
list_iiri_images() {
    local registry="${HARBOR_REGISTRY:-192.168.1.93}"
    local project="${HARBOR_PROJECT:-iiri}"

    print_info "本地 IIRI 镜像列表:"
    docker images "${registry}/${project}/*" --format "table {{.Repository}}\t{{.Tag}}\t{{.Size}}\t{{.CreatedAt}}"
}

# 清理旧镜像
cleanup_old_images() {
    local registry="${HARBOR_REGISTRY:-192.168.1.93}"
    local project="${HARBOR_PROJECT:-iiri}"
    local keep_latest="${1:-true}"

    print_msg "正在清理旧镜像..."

    if [ "$keep_latest" == "true" ]; then
        print_info "保留 latest 标签的镜像"
        docker images "${registry}/${project}/*" --format "{{.ID}} {{.Tag}}" | \
            grep -v "latest" | \
            awk '{print $1}' | \
            xargs -r docker rmi 2>/dev/null || true
    else
        print_warn "将删除所有本地 IIRI 镜像"
        docker images "${registry}/${project}/*" --format "{{.ID}}" | \
            xargs -r docker rmi 2>/dev/null || true
    fi

    print_msg "清理完成"
}

# 测试 Harbor 连接
test_harbor_connection() {
    local registry="${HARBOR_REGISTRY:-192.168.1.93}"

    print_msg "测试 Harbor 连接..."
    print_info "Registry: http://$registry"

    # 测试网络连通性
    if ! ping -c 1 -W 3 "$registry" &> /dev/null; then
        print_error "无法连接到 Harbor 服务器"
        print_warn "请检查网络连接"
        return 1
    fi

    print_msg "网络连接正常"

    # 测试 HTTP 访问
    if curl -s -o /dev/null -w "%{http_code}" "http://$registry" | grep -q "200\|302\|401"; then
        print_msg "Harbor 服务正常"
        return 0
    else
        print_warn "Harbor 服务可能未运行"
        return 1
    fi
}

# 显示镜像使用帮助
show_image_help() {
    echo "Docker 镜像管理工具"
    echo ""
    echo "可用的镜像:"
    echo "  x86: ${DOCKER_IMG_X86:-192.168.1.93/iiri/build_x86_ros2:latest}"
    echo "  ARM: ${DOCKER_IMG_ARM:-192.168.1.93/iiri/build_arm_ros2:latest}"
    echo ""
    echo "常用命令:"
    echo "  登录 Harbor:"
    echo "    docker login ${HARBOR_REGISTRY:-192.168.1.93}"
    echo ""
    echo "  拉取镜像:"
    echo "    docker pull ${DOCKER_IMG_X86:-192.168.1.93/iiri/build_x86_ros2:latest}"
    echo ""
    echo "  查看本地镜像:"
    echo "    docker images ${HARBOR_REGISTRY:-192.168.1.93}/${HARBOR_PROJECT:-iiri}/*"
    echo ""
    echo "  一键拉取脚本:"
    echo "    ./setup-and-build.sh --pull-only"
}

# 导出函数（可选）
export -f check_docker_installed
export -f harbor_login
export -f check_image_exists
export -f pull_image
export -f ensure_image
export -f get_image_info
export -f list_iiri_images
export -f cleanup_old_images
export -f test_harbor_connection
export -f show_image_help
