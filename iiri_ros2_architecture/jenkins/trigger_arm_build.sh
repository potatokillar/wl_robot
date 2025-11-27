#!/bin/bash

#=============================================================================
# Jenkins ARM 构建触发脚本
# 用途：通过命令行触发 Jenkins ARM 构建（带 CSRF 保护支持）
# 作者：唐文浩
# 日期：2025-10-11
#=============================================================================

set -e

# Jenkins 配置
JENKINS_URL="http://192.168.1.93:8080"
JENKINS_USER="admin"
JENKINS_TOKEN="westlake"  # 建议使用 API Token 而非密码
JOB_NAME="iiri-layered-build-ci"

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

print_header() {
    echo ""
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}$1${NC}"
    echo -e "${GREEN}========================================${NC}"
}

print_header "触发 Jenkins ARM 构建"

# 1. 获取 Jenkins Crumb（CSRF Token）
print_info "获取 Jenkins CSRF Token..."
CRUMB=$(curl -s -u "$JENKINS_USER:$JENKINS_TOKEN" \
    "$JENKINS_URL/crumbIssuer/api/xml?xpath=concat(//crumbRequestField,\":\",//crumb)")

if [ -z "$CRUMB" ]; then
    print_error "无法获取 CSRF Token"
    print_info "请检查："
    print_info "  1. Jenkins 服务是否运行"
    print_info "  2. 用户名和密码是否正确"
    print_info "  3. Jenkins URL 是否正确"
    exit 1
fi

print_success "CSRF Token 已获取"

# 2. 触发构建
print_info "触发 ARM Release 构建（Orin 硬件编译）..."
RESPONSE=$(curl -s -w "\n%{http_code}" -u "$JENKINS_USER:$JENKINS_TOKEN" \
    -H "$CRUMB" \
    -X POST "$JENKINS_URL/job/$JOB_NAME/buildWithParameters?ARCHITECTURE=arm&BUILD_MODE=release&ENABLE_CERES=false")

# 提取 HTTP 状态码
HTTP_CODE=$(echo "$RESPONSE" | tail -n1)
BODY=$(echo "$RESPONSE" | head -n-1)

if [ "$HTTP_CODE" -eq 201 ]; then
    print_success "ARM Release 构建已成功触发！"
    echo ""
    print_info "构建参数："
    print_info "  Job: $JOB_NAME"
    print_info "  Architecture: ARM"
    print_info "  Build Mode: Release (Orin 硬件编译 @ 192.168.1.61)"
    print_info "  Ceres: Disabled"
    print_info "  URL: $JENKINS_URL/job/$JOB_NAME/"
    echo ""
    print_info "监控构建进度："
    print_info "  方式 1: 访问 Jenkins Web UI"
    print_info "    $JENKINS_URL/job/$JOB_NAME/"
    print_info ""
    print_info "  方式 2: 使用命令行查看最新构建日志"
    print_info "    ./jenkins/watch_build.sh"
    echo ""
else
    print_error "触发构建失败（HTTP $HTTP_CODE）"
    print_info "响应内容："
    echo "$BODY"
    exit 1
fi

print_header "✅ 完成"
