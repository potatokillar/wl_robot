#!/bin/bash

#=============================================================================
# Jenkins 最新构建监控脚本
# 用途：实时监控最新 Jenkins 构建的状态和日志
# 作者：唐文浩
# 日期：2025-10-11
#=============================================================================

set -e

# Jenkins 配置
JENKINS_URL="http://192.168.1.93:8080"
JENKINS_USER="admin"
JENKINS_TOKEN="westlake"
JOB_NAME="iiri-layered-build-ci"

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
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

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_header() {
    echo ""
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}$1${NC}"
    echo -e "${GREEN}========================================${NC}"
}

print_header "监控 Jenkins 最新构建"

# 获取最新构建号
print_info "获取最新构建信息..."
JOB_INFO=$(curl -s -u "$JENKINS_USER:$JENKINS_TOKEN" \
    "$JENKINS_URL/job/$JOB_NAME/api/json")

LAST_BUILD_NUMBER=$(echo "$JOB_INFO" | grep -o '"lastBuild":{"number":[0-9]*' | grep -o '[0-9]*')

if [ -z "$LAST_BUILD_NUMBER" ]; then
    print_error "无法获取最新构建号"
    exit 1
fi

print_success "找到构建 #$LAST_BUILD_NUMBER"

# 获取构建详细信息
BUILD_INFO=$(curl -s -u "$JENKINS_USER:$JENKINS_TOKEN" \
    "$JENKINS_URL/job/$JOB_NAME/$LAST_BUILD_NUMBER/api/json")

# 解析构建信息
BUILDING=$(echo "$BUILD_INFO" | grep -o '"building":[^,]*' | cut -d':' -f2)
RESULT=$(echo "$BUILD_INFO" | grep -o '"result":"[^"]*"' | cut -d'"' -f4)

# 获取构建参数
PARAMS=$(curl -s -u "$JENKINS_USER:$JENKINS_TOKEN" \
    "$JENKINS_URL/job/$JOB_NAME/$LAST_BUILD_NUMBER/api/json?tree=actions[parameters[name,value]]")

ARCHITECTURE=$(echo "$PARAMS" | grep -o '"name":"ARCHITECTURE","value":"[^"]*"' | cut -d'"' -f8)
BUILD_MODE=$(echo "$PARAMS" | grep -o '"name":"BUILD_MODE","value":"[^"]*"' | cut -d'"' -f8)
ENABLE_CERES=$(echo "$PARAMS" | grep -o '"name":"ENABLE_CERES","value":"[^"]*"' | cut -d'"' -f8)

echo ""
echo -e "${CYAN}构建参数：${NC}"
echo "  • 架构: ${ARCHITECTURE:-未知}"
echo "  • 构建模式: ${BUILD_MODE:-未知}"
echo "  • Ceres 优化: ${ENABLE_CERES:-未知}"

echo ""
if [ "$BUILDING" = "true" ]; then
    print_info "构建状态: 正在运行中..."
    echo ""
    print_info "查看实时日志："
    echo "  $JENKINS_URL/job/$JOB_NAME/$LAST_BUILD_NUMBER/console"
    echo ""
    print_info "5 秒后开始显示最新日志..."
    sleep 5

    echo ""
    print_header "最新构建日志（最后 100 行）"
    curl -s -u "$JENKINS_USER:$JENKINS_TOKEN" \
        "$JENKINS_URL/job/$JOB_NAME/$LAST_BUILD_NUMBER/consoleText" | tail -100

    echo ""
    print_info "构建仍在进行中，使用以下命令持续监控："
    echo "  watch -n 5 'curl -s -u $JENKINS_USER:$JENKINS_TOKEN $JENKINS_URL/job/$JOB_NAME/$LAST_BUILD_NUMBER/consoleText | tail -50'"

elif [ -n "$RESULT" ]; then
    if [ "$RESULT" = "SUCCESS" ]; then
        print_success "构建状态: 成功 ✓"
    elif [ "$RESULT" = "FAILURE" ]; then
        print_error "构建状态: 失败 ✗"
        echo ""
        print_info "查看完整日志："
        echo "  $JENKINS_URL/job/$JOB_NAME/$LAST_BUILD_NUMBER/console"
        echo ""
        print_info "显示错误日志（最后 200 行）..."
        sleep 2
        echo ""
        curl -s -u "$JENKINS_USER:$JENKINS_TOKEN" \
            "$JENKINS_URL/job/$JOB_NAME/$LAST_BUILD_NUMBER/consoleText" | tail -200
    elif [ "$RESULT" = "ABORTED" ]; then
        print_warning "构建状态: 已中止"
    else
        echo -e "${YELLOW}构建状态: $RESULT${NC}"
    fi
else
    print_warning "构建状态: 未知"
fi

echo ""
print_header "完成"
