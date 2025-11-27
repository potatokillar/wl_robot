#!/bin/bash

# Jenkins 任务创建脚本
# 用法: ./create-jenkins-job.sh

set -e

# 配置
JENKINS_URL="http://192.168.1.93:8080"
JENKINS_USER="admin"
JENKINS_TOKEN="westlake"  # 建议使用 API Token 而不是密码
JOB_NAME="iiri-layered-build-ci"
CONFIG_FILE="$(dirname "$0")/job-config.xml"

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

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

# 检查配置文件
if [ ! -f "$CONFIG_FILE" ]; then
    print_error "配置文件不存在: $CONFIG_FILE"
    exit 1
fi

print_info "Jenkins 服务器: $JENKINS_URL"
print_info "任务名称: $JOB_NAME"
print_info "配置文件: $CONFIG_FILE"

# 获取 crumb (CSRF 保护)
print_info "获取 Jenkins Crumb..."
CRUMB=$(curl -s --user "$JENKINS_USER:$JENKINS_TOKEN" \
    "$JENKINS_URL/crumbIssuer/api/json" | grep -oP '(?<="crumb":")[^"]*')

if [ -z "$CRUMB" ]; then
    print_warn "无法获取 Crumb，Jenkins 可能未启用 CSRF 保护"
    CRUMB_HEADER=""
else
    print_success "成功获取 Crumb"
    CRUMB_HEADER="Jenkins-Crumb: $CRUMB"
fi

# 检查任务是否已存在
print_info "检查任务是否已存在..."
JOB_EXISTS=$(curl -s -o /dev/null -w "%{http_code}" \
    --user "$JENKINS_USER:$JENKINS_TOKEN" \
    "$JENKINS_URL/job/$JOB_NAME/api/json")

if [ "$JOB_EXISTS" = "200" ]; then
    print_warn "任务 '$JOB_NAME' 已存在"
    read -p "是否要更新现有任务? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        print_info "取消操作"
        exit 0
    fi

    # 更新任务
    print_info "更新任务配置..."
    HTTP_CODE=$(curl -s -o /dev/null -w "%{http_code}" \
        -X POST \
        --user "$JENKINS_USER:$JENKINS_TOKEN" \
        -H "$CRUMB_HEADER" \
        -H "Content-Type: application/xml" \
        --data-binary "@$CONFIG_FILE" \
        "$JENKINS_URL/job/$JOB_NAME/config.xml")

    if [ "$HTTP_CODE" = "200" ]; then
        print_success "任务更新成功!"
    else
        print_error "任务更新失败! HTTP 状态码: $HTTP_CODE"
        exit 1
    fi
else
    # 创建新任务
    print_info "创建新任务..."
    HTTP_CODE=$(curl -s -o /dev/null -w "%{http_code}" \
        -X POST \
        --user "$JENKINS_USER:$JENKINS_TOKEN" \
        -H "$CRUMB_HEADER" \
        -H "Content-Type: application/xml" \
        --data-binary "@$CONFIG_FILE" \
        "$JENKINS_URL/createItem?name=$JOB_NAME")

    if [ "$HTTP_CODE" = "200" ]; then
        print_success "任务创建成功!"
    else
        print_error "任务创建失败! HTTP 状态码: $HTTP_CODE"
        exit 1
    fi
fi

# 显示任务 URL
print_success "任务已配置完成!"
echo ""
echo "任务 URL: $JENKINS_URL/job/$JOB_NAME"
echo ""
print_info "后续步骤:"
echo "1. 访问 Jenkins 任务页面查看配置"
echo "2. 确保 Git 凭据 'git-cred' 已配置"
echo "3. 点击 'Build with Parameters' 测试构建"
echo "4. 配置邮件或其他通知方式（可选）"
echo ""
print_info "提示: 首次构建可能需要下载 Docker 镜像，耗时较长"
