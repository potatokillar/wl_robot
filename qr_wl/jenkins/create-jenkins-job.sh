#!/bin/bash

#=============================================================================
# Jenkins 任务创建脚本
# 功能：自动创建 qr-wl-build-ci Jenkins 任务
# 用法：./create-jenkins-job.sh
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

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

echo ""
print_info "======================================"
print_info "Jenkins Job Creator - qr_wl"
print_info "======================================"
echo ""

# 检查 Jenkins 是否可访问
print_info "Checking Jenkins availability..."
HTTP_CODE=$(curl -s -o /dev/null -w "%{http_code}" --user "${JENKINS_USER}:${JENKINS_TOKEN}" "${JENKINS_URL}")

if [ "$HTTP_CODE" != "200" ]; then
    print_error "Cannot connect to Jenkins (HTTP $HTTP_CODE)"
    print_error "URL: ${JENKINS_URL}"
    echo ""
    print_info "Please check:"
    print_info "  1. Jenkins is running"
    print_info "  2. URL is correct"
    print_info "  3. Credentials are valid"
    exit 1
fi

print_success "Jenkins is accessible"
echo ""

# 检查任务是否已存在
print_info "Checking if job '${JOB_NAME}' already exists..."
JOB_CHECK=$(curl -s -o /dev/null -w "%{http_code}" --user "${JENKINS_USER}:${JENKINS_TOKEN}" "${JENKINS_URL}/job/${JOB_NAME}")

if [ "$JOB_CHECK" == "200" ]; then
    print_warning "Job '${JOB_NAME}' already exists!"
    echo ""
    read -p "Do you want to update the configuration? (y/N): " -n 1 -r
    echo ""

    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        print_info "Operation cancelled"
        exit 0
    fi

    print_info "Updating job configuration..."
    ACTION="update"
else
    print_info "Creating new job '${JOB_NAME}'..."
    ACTION="create"
fi

echo ""

# 创建/更新任务配置
print_info "====================================="
print_info "Creating Jenkins Job Configuration"
print_info "====================================="
echo ""
print_info "Job Name: ${JOB_NAME}"
print_info "Job Type: Pipeline (from SCM)"
print_info "Git Repo: http://192.168.1.55/ontology/qr_wl.git"
print_info "Git Branch: */main"
print_info "Script Path: Jenkinsfile"
echo ""

# 获取脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_FILE="${SCRIPT_DIR}/job-config.xml"

# 检查配置文件是否存在
if [ ! -f "${CONFIG_FILE}" ]; then
    print_error "Configuration file not found: ${CONFIG_FILE}"
    exit 1
fi

# 临时文件
TEMP_DIR=$(mktemp -d)
COOKIE_FILE="${TEMP_DIR}/jenkins_cookies.txt"
CRUMB_FILE="${TEMP_DIR}/jenkins_crumb.json"
RESPONSE_FILE="${TEMP_DIR}/jenkins_response.txt"

# 清理函数
cleanup() {
    rm -rf "${TEMP_DIR}"
}
trap cleanup EXIT

# 禁用代理（内网访问）
unset http_proxy https_proxy

# 获取 CSRF crumb
print_info "Getting CSRF crumb token..."
CRUMB_RESPONSE=$(curl -s -c "${COOKIE_FILE}" --user "${JENKINS_USER}:${JENKINS_TOKEN}" \
    "${JENKINS_URL}/crumbIssuer/api/json" 2>/dev/null)

if [ -z "$CRUMB_RESPONSE" ]; then
    print_error "Failed to get CSRF crumb"
    exit 1
fi

CRUMB=$(echo "$CRUMB_RESPONSE" | python3 -c "import sys, json; d=json.load(sys.stdin); print(d.get('crumb', ''))" 2>/dev/null)
CRUMB_FIELD=$(echo "$CRUMB_RESPONSE" | python3 -c "import sys, json; d=json.load(sys.stdin); print(d.get('crumbRequestField', 'Jenkins-Crumb'))" 2>/dev/null)

if [ -z "$CRUMB" ]; then
    print_error "Failed to parse CSRF crumb"
    exit 1
fi

print_success "Got crumb token"

# 创建或更新任务
if [ "$ACTION" == "create" ]; then
    print_info "Creating Jenkins job via API..."
    HTTP_STATUS=$(curl -s -o "${RESPONSE_FILE}" -w "%{http_code}" \
        -X POST "${JENKINS_URL}/createItem?name=${JOB_NAME}" \
        --user "${JENKINS_USER}:${JENKINS_TOKEN}" \
        -b "${COOKIE_FILE}" \
        -H "${CRUMB_FIELD}:${CRUMB}" \
        -H "Content-Type: application/xml" \
        --data-binary @"${CONFIG_FILE}" 2>/dev/null)
else
    print_info "Updating Jenkins job configuration via API..."
    HTTP_STATUS=$(curl -s -o "${RESPONSE_FILE}" -w "%{http_code}" \
        -X POST "${JENKINS_URL}/job/${JOB_NAME}/config.xml" \
        --user "${JENKINS_USER}:${JENKINS_TOKEN}" \
        -b "${COOKIE_FILE}" \
        -H "${CRUMB_FIELD}:${CRUMB}" \
        -H "Content-Type: application/xml" \
        --data-binary @"${CONFIG_FILE}" 2>/dev/null)
fi

# 检查结果
if [ "$HTTP_STATUS" == "200" ]; then
    print_success "Jenkins job ${ACTION}d successfully!"
    echo ""
    print_info "Job URL: ${JENKINS_URL}/job/${JOB_NAME}"
    echo ""
    print_info "You can now:"
    print_info "  1. Trigger build: ./jenkins/trigger_build.sh x86 debug"
    print_info "  2. View job: ${JENKINS_URL}/job/${JOB_NAME}"
    print_info "  3. Monitor builds: ./jenkins/watch_latest_build.sh"
    echo ""

    # 打开浏览器
    if command -v xdg-open &> /dev/null; then
        print_info "Opening Jenkins job in browser..."
        xdg-open "${JENKINS_URL}/job/${JOB_NAME}" 2>/dev/null &
    elif command -v open &> /dev/null; then
        print_info "Opening Jenkins job in browser..."
        open "${JENKINS_URL}/job/${JOB_NAME}" 2>/dev/null &
    fi
else
    print_error "Failed to ${ACTION} Jenkins job (HTTP ${HTTP_STATUS})"
    echo ""
    print_info "Response:"
    cat "${RESPONSE_FILE}"
    echo ""
    exit 1
fi

echo ""
print_success "Setup completed!"
echo ""
