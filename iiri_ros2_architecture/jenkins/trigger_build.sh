#!/bin/bash

# Jenkins 构建触发脚本
# 用法: ./trigger_build.sh [x86|arm] [true|false]
#   参数1: 架构 (x86 或 arm，默认 x86)
#   参数2: 是否启用 Ceres (true 或 false，默认 false)

set -e

# 配置
JENKINS_URL="http://192.168.1.93:8080"
JENKINS_USER="admin"
JENKINS_TOKEN="westlake"
JOB_NAME="iiri-layered-build-ci"

# 参数
ARCH="${1:-x86}"
ENABLE_CERES="${2:-false}"

# 颜色
GREEN='\033[0;32m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${BLUE}[INFO]${NC} 触发 Jenkins 构建..."
echo -e "${BLUE}[INFO]${NC} 任务: $JOB_NAME"
echo -e "${BLUE}[INFO]${NC} 架构: $ARCH"
echo -e "${BLUE}[INFO]${NC} Ceres 优化: $ENABLE_CERES"

# 获取 crumb
CRUMB=$(curl -s --user "$JENKINS_USER:$JENKINS_TOKEN" \
    "$JENKINS_URL/crumbIssuer/api/json" | grep -oP '(?<="crumb":")[^"]*' || echo "")

if [ -n "$CRUMB" ]; then
    CRUMB_HEADER="Jenkins-Crumb: $CRUMB"
else
    CRUMB_HEADER=""
fi

# 触发构建
HTTP_CODE=$(curl -s -o /dev/null -w "%{http_code}" \
    -X POST \
    --user "$JENKINS_USER:$JENKINS_TOKEN" \
    -H "$CRUMB_HEADER" \
    "$JENKINS_URL/job/$JOB_NAME/buildWithParameters?ARCHITECTURE=$ARCH&ENABLE_CERES=$ENABLE_CERES")

if [ "$HTTP_CODE" = "201" ]; then
    echo -e "${GREEN}[SUCCESS]${NC} 构建已触发!"
    echo ""
    echo "查看构建状态: $JENKINS_URL/job/$JOB_NAME/"
    echo "查看控制台输出: $JENKINS_URL/job/$JOB_NAME/lastBuild/console"
else
    echo -e "${RED}[ERROR]${NC} 触发构建失败! HTTP 状态码: $HTTP_CODE"
    exit 1
fi
