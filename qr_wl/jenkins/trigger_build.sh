#!/bin/bash

#=============================================================================
# Jenkins 构建触发脚本
# 功能：触发 qr-wl-build-ci Jenkins 任务
# 用法：./trigger_build.sh <architecture> <build_mode>
# 作者：参考 iiri_ros2_architecture
# 日期：2025-10-15
#=============================================================================

# Jenkins 配置
JENKINS_URL="http://192.168.1.93:8080"
JOB_NAME="qr-wl-build-ci"
JENKINS_USER="admin"
JENKINS_TOKEN="westlake"

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 打印函数
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 显示使用说明
show_usage() {
    echo "qr_wl Jenkins Build Trigger"
    echo ""
    echo "Usage: $0 <architecture>"
    echo ""
    echo "Arguments:"
    echo "  architecture  Target architecture: x86 or arm"
    echo "                - x86: x86_64 architecture"
    echo "                - arm: ARM64 cross-compilation"
    echo ""
    echo "Examples:"
    echo "  $0 x86                      # x86 build"
    echo "  $0 arm                      # ARM cross-compilation"
    echo ""
}

# 检查参数
if [ $# -ne 1 ]; then
    print_error "Invalid number of arguments"
    show_usage
    exit 1
fi

ARCHITECTURE=$1

# 验证参数
if [[ ! "$ARCHITECTURE" =~ ^(x86|arm)$ ]]; then
    print_error "Invalid architecture: $ARCHITECTURE"
    print_error "Must be 'x86' or 'arm'"
    exit 1
fi

# 构建参数
PARAMS="ARCHITECTURE=${ARCHITECTURE}"

print_info "======================================"
print_info "Triggering Jenkins Build"
print_info "======================================"
print_info "Job: ${JOB_NAME}"
print_info "Architecture: ${ARCHITECTURE}"
print_info "======================================"
echo ""

# 禁用代理（内网访问）
unset http_proxy https_proxy

# 临时文件
TEMP_DIR=$(mktemp -d)
COOKIE_FILE="${TEMP_DIR}/jenkins_cookies.txt"

# 清理函数
cleanup() {
    rm -rf "${TEMP_DIR}"
}
trap cleanup EXIT

# 获取 CSRF crumb
print_info "Getting CSRF crumb token..."
CRUMB_RESPONSE=$(curl -s -c "${COOKIE_FILE}" --user "${JENKINS_USER}:${JENKINS_TOKEN}" \
    "${JENKINS_URL}/crumbIssuer/api/json" 2>/dev/null)

if [ -z "$CRUMB_RESPONSE" ]; then
    print_error "Failed to get CSRF crumb token"
    exit 1
fi

CRUMB=$(echo "$CRUMB_RESPONSE" | python3 -c "import sys, json; d=json.load(sys.stdin); print(d.get('crumb', ''))" 2>/dev/null)
CRUMB_FIELD=$(echo "$CRUMB_RESPONSE" | python3 -c "import sys, json; d=json.load(sys.stdin); print(d.get('crumbRequestField', 'Jenkins-Crumb'))" 2>/dev/null)

if [ -z "$CRUMB" ]; then
    print_error "Failed to parse CSRF crumb"
    exit 1
fi

# 触发构建
print_info "Sending build request to Jenkins..."
RESPONSE=$(curl -s -w "\n%{http_code}" -X POST \
    "${JENKINS_URL}/job/${JOB_NAME}/buildWithParameters?${PARAMS}" \
    --user "${JENKINS_USER}:${JENKINS_TOKEN}" \
    -b "${COOKIE_FILE}" \
    -H "${CRUMB_FIELD}:${CRUMB}")

# 分离响应体和状态码
HTTP_BODY=$(echo "$RESPONSE" | head -n -1)
HTTP_CODE=$(echo "$RESPONSE" | tail -n 1)

# 检查响应
if [ "$HTTP_CODE" == "201" ]; then
    print_success "Build triggered successfully!"
    echo ""
    print_info "📊 View build status:"
    print_info "   ${JENKINS_URL}/job/${JOB_NAME}"
    echo ""
    print_info "💡 Monitor build progress:"
    print_info "   ./jenkins/watch_latest_build.sh"
    echo ""
elif [ "$HTTP_CODE" == "200" ]; then
    print_success "Build request accepted!"
    echo ""
    print_info "📊 View build status:"
    print_info "   ${JENKINS_URL}/job/${JOB_NAME}"
    echo ""
else
    print_error "Failed to trigger build (HTTP $HTTP_CODE)"
    if [ -n "$HTTP_BODY" ]; then
        echo ""
        print_error "Response:"
        echo "$HTTP_BODY"
    fi
    echo ""
    print_info "💡 Troubleshooting:"
    print_info "  1. Check Jenkins is running: ${JENKINS_URL}"
    print_info "  2. Verify job name: ${JOB_NAME}"
    print_info "  3. Check credentials: ${JENKINS_USER}"
    exit 1
fi
