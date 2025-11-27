#!/bin/bash

#=============================================================================
# Jenkins 构建监控脚本
# 功能：实时监控最新 Jenkins 构建状态
# 用法：./watch_latest_build.sh
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
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# 清屏函数
clear_screen() {
    printf "\033c"
}

# 打印函数
print_header() {
    echo -e "${CYAN}========================================${NC}"
    echo -e "${CYAN}$1${NC}"
    echo -e "${CYAN}========================================${NC}"
}

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

# 获取构建状态函数
get_build_status() {
    local API_URL="${JENKINS_URL}/job/${JOB_NAME}/lastBuild/api/json"

    # 获取构建信息
    local RESPONSE=$(curl -s --user "${JENKINS_USER}:${JENKINS_TOKEN}" "$API_URL")

    if [ -z "$RESPONSE" ]; then
        echo "ERROR:无法连接到 Jenkins"
        return 1
    fi

    # 解析 JSON（使用 Python）
    python3 -c "
import sys, json
try:
    data = json.loads('''$RESPONSE''')

    # 提取信息
    number = data.get('number', 'N/A')
    result = data.get('result', 'BUILDING')
    building = data.get('building', False)
    duration = data.get('duration', 0)
    timestamp = data.get('timestamp', 0)
    url = data.get('url', '')

    # 获取参数
    actions = data.get('actions', [])
    params = {}
    for action in actions:
        if '_class' in action and 'ParametersAction' in action['_class']:
            for param in action.get('parameters', []):
                params[param['name']] = param['value']

    arch = params.get('ARCHITECTURE', 'N/A')
    mode = params.get('BUILD_MODE', 'N/A')

    # 计算持续时间
    if building:
        import time
        elapsed = int(time.time() * 1000) - timestamp
        duration_sec = elapsed // 1000
    else:
        duration_sec = duration // 1000

    duration_min = duration_sec // 60
    duration_sec_rem = duration_sec % 60

    # 打印结果
    print(f'BUILD_NUMBER={number}')
    print(f'BUILD_RESULT={result if result else \"BUILDING\"}')
    print(f'BUILD_BUILDING={str(building).lower()}')
    print(f'BUILD_DURATION={duration_min}m {duration_sec_rem}s')
    print(f'BUILD_ARCH={arch}')
    print(f'BUILD_MODE={mode}')
    print(f'BUILD_URL={url}')

except Exception as e:
    print(f'ERROR:{e}', file=sys.stderr)
    sys.exit(1)
" 2>&1
}

# 主循环
echo ""
print_header "Jenkins Build Monitor - qr_wl"
echo ""
print_info "Job: ${JOB_NAME}"
print_info "Jenkins: ${JENKINS_URL}"
echo ""
print_info "Press Ctrl+C to stop monitoring"
echo ""
sleep 2

LAST_BUILD_NUMBER=""

while true; do
    clear_screen

    print_header "Jenkins Build Monitor - qr_wl"
    echo ""
    print_info "Refresh Time: $(date '+%Y-%m-%d %H:%M:%S')"
    print_info "Job: ${JOB_NAME}"
    echo ""

    # 获取构建状态
    STATUS_OUTPUT=$(get_build_status)

    if [[ $STATUS_OUTPUT == ERROR:* ]]; then
        print_error "${STATUS_OUTPUT#ERROR:}"
        echo ""
        print_info "Retrying in 10 seconds..."
    else
        # 解析状态
        eval "$STATUS_OUTPUT"

        # 显示构建信息
        echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
        echo -e "📦 Build Number:    ${YELLOW}#${BUILD_NUMBER}${NC}"
        echo -e "🏗️  Architecture:    ${CYAN}${BUILD_ARCH}${NC}"
        echo -e "⚙️  Build Mode:      ${CYAN}${BUILD_MODE}${NC}"
        echo -e "⏱️  Duration:        ${BLUE}${BUILD_DURATION}${NC}"

        # 根据状态显示不同颜色
        case "$BUILD_RESULT" in
            "SUCCESS")
                echo -e "✅ Status:          ${GREEN}${BUILD_RESULT}${NC}"
                ;;
            "FAILURE")
                echo -e "❌ Status:          ${RED}${BUILD_RESULT}${NC}"
                ;;
            "BUILDING")
                echo -e "🔄 Status:          ${YELLOW}${BUILD_RESULT}...${NC}"
                ;;
            *)
                echo -e "⚪ Status:          ${YELLOW}${BUILD_RESULT}${NC}"
                ;;
        esac

        echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
        echo ""

        # 显示 URL
        print_info "🔗 Build URL: ${BUILD_URL}"
        echo ""

        # 新构建检测
        if [ "$BUILD_NUMBER" != "$LAST_BUILD_NUMBER" ] && [ -n "$LAST_BUILD_NUMBER" ]; then
            print_warning "🆕 New build detected: #${BUILD_NUMBER}"
            echo ""
        fi
        LAST_BUILD_NUMBER=$BUILD_NUMBER

        # 构建完成提示
        if [ "$BUILD_BUILDING" == "false" ]; then
            if [ "$BUILD_RESULT" == "SUCCESS" ]; then
                print_success "🎉 Build completed successfully!"
            elif [ "$BUILD_RESULT" == "FAILURE" ]; then
                print_error "💥 Build failed! Check logs for details."
            fi
            echo ""
        fi
    fi

    print_info "Next update in 10 seconds... (Press Ctrl+C to exit)"
    sleep 10
done
