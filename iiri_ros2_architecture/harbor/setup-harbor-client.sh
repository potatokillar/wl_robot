#!/bin/bash

# Harbor 仓库自动配置脚本
# 适用于 Harbor 服务器：192.168.1.93 (HTTP模式)
# 管理员账号：admin
# 管理员密码：Westlake1234

set -e

# 配置变量
HARBOR_IP="192.168.1.93"
HARBOR_DOMAIN="harbor.local"
ADMIN_USER="admin"
ADMIN_PASSWORD="Westlake1234"

# 颜色输出
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

# 检查是否为root用户
check_root() {
    if [[ $EUID -eq 0 ]]; then
        log_warning "检测到以root用户运行，某些操作可能不需要sudo"
        SUDO=""
    else
        SUDO="sudo"
    fi
}

# 检查Docker是否已安装
check_docker() {
    if ! command -v docker &> /dev/null; then
        log_error "Docker未安装，请先安装Docker"
        exit 1
    fi

    if ! docker info &> /dev/null; then
        log_error "Docker服务未运行或当前用户无权限访问Docker"
        log_info "请确保Docker服务运行并将当前用户加入docker组："
        log_info "sudo systemctl start docker"
        log_info "sudo usermod -aG docker \$USER"
        log_info "然后重新登录或运行: newgrp docker"
        exit 1
    fi

    log_success "Docker检查通过"
}

# 检测操作系统
detect_os() {
    if [[ "$OSTYPE" == "linux-gnu"* ]]; then
        OS="linux"
    elif [[ "$OSTYPE" == "darwin"* ]]; then
        OS="macos"
    elif [[ "$OSTYPE" == "msys" ]] || [[ "$OSTYPE" == "cygwin" ]]; then
        OS="windows"
    else
        OS="unknown"
    fi
    log_info "检测到操作系统: $OS"
}

# 检查网络连通性
check_network() {
    log_info "检查与Harbor服务器的网络连通性..."

    if ping -c 1 -W 5 "$HARBOR_IP" &> /dev/null; then
        log_success "网络连通性检查通过"
    else
        log_error "无法连接到Harbor服务器 $HARBOR_IP"
        log_info "请检查："
        log_info "1. Harbor服务器是否运行"
        log_info "2. 网络连接是否正常"
        log_info "3. 防火墙设置是否允许80端口"
        exit 1
    fi

    # 检查80端口是否可访问
    if command -v curl &> /dev/null; then
        if curl -s --connect-timeout 10 "http://$HARBOR_IP/" | grep -q "Harbor"; then
            log_success "Harbor服务检查通过"
        else
            log_warning "Harbor服务可能未正常运行，但将继续配置"
        fi
    fi
}

# 配置hosts解析（可选）
setup_hosts() {
    local hosts_file
    if [[ "$OS" == "linux" ]] || [[ "$OS" == "macos" ]]; then
        hosts_file="/etc/hosts"
    elif [[ "$OS" == "windows" ]]; then
        hosts_file="/c/Windows/System32/drivers/etc/hosts"
    else
        log_warning "未知操作系统，跳过hosts配置"
        return
    fi

    log_info "配置hosts解析（可选）..."

    read -p "是否配置hosts解析 $HARBOR_DOMAIN -> $HARBOR_IP? (y/N): " setup_hosts_choice
    if [[ "$setup_hosts_choice" =~ ^[Yy]$ ]]; then
        if grep -q "$HARBOR_DOMAIN" "$hosts_file" 2>/dev/null; then
            log_warning "hosts文件中已存在 $HARBOR_DOMAIN 的解析"
            log_info "当前解析："
            grep "$HARBOR_DOMAIN" "$hosts_file"
            read -p "是否覆盖现有解析? (y/N): " overwrite_choice
            if [[ "$overwrite_choice" =~ ^[Yy]$ ]]; then
                $SUDO sed -i "/$HARBOR_DOMAIN/d" "$hosts_file"
                echo "$HARBOR_IP $HARBOR_DOMAIN" | $SUDO tee -a "$hosts_file" > /dev/null
                log_success "hosts解析已更新"
            fi
        else
            echo "$HARBOR_IP $HARBOR_DOMAIN" | $SUDO tee -a "$hosts_file" > /dev/null
            log_success "hosts解析已添加"
        fi
    else
        log_info "跳过hosts配置，将使用IP地址访问"
    fi
}

# 检查jq工具
check_jq() {
    if ! command -v jq &> /dev/null; then
        log_warning "未找到jq工具，正在安装..."
        if command -v apt-get &> /dev/null; then
            $SUDO apt-get update && $SUDO apt-get install -y jq
        elif command -v yum &> /dev/null; then
            $SUDO yum install -y jq
        elif command -v brew &> /dev/null; then
            brew install jq
        else
            log_error "无法自动安装jq，请手动安装后重试"
            log_info "Ubuntu/Debian: sudo apt-get install jq"
            log_info "CentOS/RHEL: sudo yum install jq"
            log_info "macOS: brew install jq"
            return 1
        fi

        # 再次检查安装是否成功
        if ! command -v jq &> /dev/null; then
            log_error "jq安装失败"
            return 1
        else
            log_success "jq安装成功"
        fi
    fi
    return 0
}

# 配置Docker不安全仓库
setup_docker_insecure() {
    log_info "配置Docker不安全仓库..."

    # 检查jq工具
    if ! check_jq; then
        log_error "需要jq工具来处理JSON配置，请安装后重试"
        return 1
    fi

    local daemon_json="/etc/docker/daemon.json"
    local registries='["'$HARBOR_IP'", "'$HARBOR_DOMAIN'"]'

    # 检查daemon.json是否存在
    if [[ ! -f "$daemon_json" ]]; then
        log_info "创建Docker daemon.json配置文件"
        echo '{
  "insecure-registries": '$registries'
}' | $SUDO tee "$daemon_json" > /dev/null
    else
        # 检查是否已配置
        if $SUDO cat "$daemon_json" | jq -e '.["insecure-registries"]' &> /dev/null; then
            log_info "daemon.json中已存在insecure-registries配置"

            # 检查是否已包含我们的仓库
            if $SUDO cat "$daemon_json" | jq -e ".\"insecure-registries\" | index(\"$HARBOR_IP\")" &> /dev/null; then
                log_success "Harbor IP已在不安全仓库列表中"
            else
                log_info "添加Harbor IP到不安全仓库列表"
                $SUDO cp "$daemon_json" "$daemon_json.backup"
                # 使用临时文件避免文件截断问题
                local temp_file=$(mktemp)
                if $SUDO cat "$daemon_json" | jq ".\"insecure-registries\" += [\"$HARBOR_IP\", \"$HARBOR_DOMAIN\"]" > "$temp_file"; then
                    # 验证生成的JSON是否有效
                    if jq empty "$temp_file" &>/dev/null; then
                        $SUDO mv "$temp_file" "$daemon_json"
                        log_success "成功添加Harbor IP到不安全仓库列表"
                    else
                        log_error "生成的JSON无效，保持原配置不变"
                        rm -f "$temp_file"
                        return 1
                    fi
                else
                    log_error "jq处理失败，保持原配置不变"
                    rm -f "$temp_file"
                    return 1
                fi
            fi
        else
            log_info "添加insecure-registries配置到现有daemon.json"
            $SUDO cp "$daemon_json" "$daemon_json.backup"
            # 使用临时文件避免文件截断问题
            local temp_file=$(mktemp)
            if $SUDO cat "$daemon_json" | jq ". += {\"insecure-registries\": $registries}" > "$temp_file"; then
                # 验证生成的JSON是否有效
                if jq empty "$temp_file" &>/dev/null; then
                    $SUDO mv "$temp_file" "$daemon_json"
                    log_success "成功添加不安全仓库配置"
                else
                    log_error "生成的JSON无效，保持原配置不变"
                    rm -f "$temp_file"
                    return 1
                fi
            else
                log_error "jq处理失败，保持原配置不变"
                rm -f "$temp_file"
                return 1
            fi
        fi
    fi

    log_success "Docker daemon.json配置完成"
}

# 重启Docker服务
restart_docker() {
    log_info "重启Docker服务以应用配置..."

    if [[ "$OS" == "linux" ]]; then
        $SUDO systemctl daemon-reload
        $SUDO systemctl restart docker

        # 等待Docker服务启动
        sleep 3

        if $SUDO systemctl is-active --quiet docker; then
            log_success "Docker服务重启成功"
        else
            log_error "Docker服务重启失败"
            exit 1
        fi
    else
        log_warning "请手动重启Docker服务"
        if [[ "$OS" == "macos" ]]; then
            log_info "macOS: 在Docker Desktop中点击重启"
        elif [[ "$OS" == "windows" ]]; then
            log_info "Windows: 在Docker Desktop中点击重启"
        fi

        read -p "Docker重启完成后按Enter继续..."
    fi
}

# 登录Harbor
login_harbor() {
    log_info "登录Harbor仓库..."

    # 选择使用IP还是域名
    local harbor_url
    if grep -q "$HARBOR_DOMAIN" "/etc/hosts" 2>/dev/null; then
        read -p "使用域名登录($HARBOR_DOMAIN)还是IP($HARBOR_IP)? (d/I): " url_choice
        if [[ "$url_choice" =~ ^[Dd]$ ]]; then
            harbor_url="$HARBOR_DOMAIN"
        else
            harbor_url="$HARBOR_IP"
        fi
    else
        harbor_url="$HARBOR_IP"
    fi

    log_info "尝试登录 $harbor_url..."

    if echo "Westlake1234" | sudo docker login 192.168.1.93 -u admin --password-stdin; then
        log_success "Harbor登录成功"
        return 0
    else
        log_error "Harbor登录失败"
        log_info "请检查："
        log_info "1. Harbor服务是否正常运行"
        log_info "2. 账号密码是否正确"
        log_info "3. 网络连接是否正常"
        return 1
    fi
}

# 创建示例项目和推送测试
demo_usage() {
    read -p "是否运行使用示例? (y/N): " demo_choice
    if [[ ! "$demo_choice" =~ ^[Yy]$ ]]; then
        return
    fi

    log_info "运行Harbor使用示例..."

    # 确定harbor地址
    local harbor_url
    if grep -q "$HARBOR_DOMAIN" "/etc/hosts" 2>/dev/null; then
        harbor_url="$HARBOR_DOMAIN"
    else
        harbor_url="$HARBOR_IP"
    fi

    log_info "拉取测试镜像..."
    docker pull alpine:3.19

    log_info "为Harbor打标签..."
    docker tag alpine:3.19 "$harbor_url/library/alpine:3.19"

    log_info "推送镜像到Harbor..."
    if docker push "$harbor_url/library/alpine:3.19"; then
        log_success "镜像推送成功!"
        log_info "你可以在Harbor UI中查看: http://$HARBOR_IP/"

        log_info "测试拉取镜像..."
        docker rmi "$harbor_url/library/alpine:3.19" || true
        if docker pull "$harbor_url/library/alpine:3.19"; then
            log_success "镜像拉取成功!"
        else
            log_error "镜像拉取失败"
        fi
    else
        log_error "镜像推送失败，请检查项目权限"
        log_info "确保在Harbor UI中创建了 'library' 项目，或使用其他已存在的项目"
    fi
}

# 显示配置总结
show_summary() {
    log_info "配置总结："
    echo "----------------------------------------"
    echo "Harbor服务器: http://$HARBOR_IP/"
    echo "管理员账号: $ADMIN_USER"
    echo "管理员密码: $ADMIN_PASSWORD"
    echo ""
    echo "使用方法："
    echo "1. 登录: docker login $HARBOR_IP -u $ADMIN_USER -p $ADMIN_PASSWORD"
    echo "2. 推送镜像示例:"
    echo "   docker tag <本地镜像> $HARBOR_IP/<项目名>/<仓库名>:<标签>"
    echo "   docker push $HARBOR_IP/<项目名>/<仓库名>:<标签>"
    echo "3. 拉取镜像示例:"
    echo "   docker pull $HARBOR_IP/<项目名>/<仓库名>:<标签>"
    echo ""
    echo "Web UI: http://$HARBOR_IP/"
    echo "----------------------------------------"
}

# 主函数
main() {
    log_info "Harbor 客户端自动配置脚本"
    log_info "服务器: $HARBOR_IP, 账号: $ADMIN_USER"
    echo ""

    # 检查前置条件
    check_root
    detect_os
    check_docker
    check_network


    # 配置步骤
    setup_hosts
    setup_docker_insecure
    restart_docker

    # 登录测试
    if login_harbor; then
        demo_usage
        show_summary
        log_success "Harbor客户端配置完成!"
    else
        log_error "配置过程中出现错误，请检查日志"
        exit 1
    fi
}

# 显示帮助信息
show_help() {
    echo "Harbor 客户端自动配置脚本"
    echo ""
    echo "用法: $0 [选项]"
    echo ""
    echo "选项:"
    echo "  -h, --help    显示帮助信息"
    echo "  --skip-demo   跳过演示示例"
    echo ""
    echo "功能:"
    echo "- 检查Docker环境"
    echo "- 配置不安全仓库设置"
    echo "- 可选配置hosts解析"
    echo "- 自动登录Harbor"
    echo "- 运行推送/拉取示例"
    echo ""
    echo "服务器信息:"
    echo "  IP: $HARBOR_IP"
    echo "  账号: $ADMIN_USER"
    echo "  密码: $ADMIN_PASSWORD"
}

# 处理命令行参数
while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            show_help
            exit 0
            ;;
        --skip-demo)
            SKIP_DEMO=1
            shift
            ;;
        *)
            log_error "未知参数: $1"
            show_help
            exit 1
            ;;
    esac
done

# 执行主函数
main
