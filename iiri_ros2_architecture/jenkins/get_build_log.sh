#!/bin/bash

# 获取 Jenkins 最新构建日志

JENKINS_URL="http://192.168.1.93:8080"
JENKINS_USER="admin"
JENKINS_PASSWORD="westlake"
JOB_NAME="iiri-layered-build-ci"

echo "=== 获取最新构建信息 ==="
BUILD_INFO=$(curl -s --user "$JENKINS_USER:$JENKINS_PASSWORD" \
    "$JENKINS_URL/job/$JOB_NAME/lastBuild/api/json")

BUILD_NUMBER=$(echo "$BUILD_INFO" | python3 -c "import sys, json; print(json.load(sys.stdin).get('number', 'N/A'))" 2>/dev/null || echo "N/A")
BUILD_RESULT=$(echo "$BUILD_INFO" | python3 -c "import sys, json; print(json.load(sys.stdin).get('result', 'UNKNOWN'))" 2>/dev/null || echo "UNKNOWN")

echo "构建编号: $BUILD_NUMBER"
echo "构建结果: $BUILD_RESULT"
echo ""

echo "=== 获取控制台输出（最后 500 行）==="
curl -s --user "$JENKINS_USER:$JENKINS_PASSWORD" \
    "$JENKINS_URL/job/$JOB_NAME/lastBuild/consoleText" | tail -500
