#!/bin/bash

#=============================================================================
# System Bringup 分层测试脚本
# 在Docker容器内测试每一层的启动功能
# 作者: 唐文浩
# 日期: 2025-10-10
#=============================================================================

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 测试配置
TEST_LOG_DIR="test_logs_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$TEST_LOG_DIR"

# 输出函数
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[✓]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[✗]${NC} $1"
}

print_header() {
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}$1${NC}"
    echo -e "${GREEN}========================================${NC}"
}

# 检查ROS2环境
check_ros_env() {
    print_info "检查ROS2环境..."

    if ! command -v ros2 &> /dev/null; then
        print_error "ROS2环境未配置"
        echo "请先source ROS2环境："
        echo "  source /opt/ros/humble/setup.bash"
        echo "  source build_x86_shared/install/setup.bash"
        exit 1
    fi

    if ! ros2 pkg list | grep -q "system_bringup"; then
        print_error "system_bringup包未找到"
        echo "请先编译core_layer："
        echo "  ./build_layered.sh core_layer"
        exit 1
    fi

    print_success "ROS2环境检查通过"
}

# 测试单个层级
test_layer() {
    local LAYER_NAME=$1
    local LAUNCH_FILE=$2
    local TEST_DURATION=${3:-15}  # 默认测试15秒

    print_header "测试 $LAYER_NAME"
    print_info "启动文件: $LAUNCH_FILE"
    print_info "测试时长: ${TEST_DURATION}秒"

    local LOG_FILE="$TEST_LOG_DIR/${LAYER_NAME}.log"
    local NODES_FILE="$TEST_LOG_DIR/${LAYER_NAME}_nodes.txt"
    local TOPICS_FILE="$TEST_LOG_DIR/${LAYER_NAME}_topics.txt"
    local RESULT_FILE="$TEST_LOG_DIR/${LAYER_NAME}_result.txt"

    # 启动launch文件
    print_info "启动节点..."
    timeout ${TEST_DURATION} ros2 launch system_bringup "$LAUNCH_FILE" > "$LOG_FILE" 2>&1 &
    local LAUNCH_PID=$!

    # 等待节点启动
    sleep 5

    # 检查进程是否还在运行
    if ! ps -p $LAUNCH_PID > /dev/null 2>&1; then
        print_error "$LAYER_NAME 启动失败"
        echo "FAILED" > "$RESULT_FILE"
        echo "详细日志: $LOG_FILE"
        return 1
    fi

    print_success "节点启动成功"

    # 收集运行信息
    print_info "收集运行信息..."

    # 列出节点
    ros2 node list > "$NODES_FILE" 2>&1
    local NODE_COUNT=$(grep -v "^$" "$NODES_FILE" 2>/dev/null | wc -l)

    # 列出话题
    ros2 topic list > "$TOPICS_FILE" 2>&1
    local TOPIC_COUNT=$(grep -v "^$" "$TOPICS_FILE" 2>/dev/null | wc -l)

    print_info "检测到 $NODE_COUNT 个节点"
    print_info "检测到 $TOPIC_COUNT 个话题"

    # 等待测试完成
    print_info "等待测试完成..."
    sleep $((TEST_DURATION - 5))

    # 停止节点
    print_info "停止节点..."
    kill $LAUNCH_PID 2>/dev/null
    wait $LAUNCH_PID 2>/dev/null

    # 保存测试结果
    {
        echo "Layer: $LAYER_NAME"
        echo "Launch File: $LAUNCH_FILE"
        echo "Nodes: $NODE_COUNT"
        echo "Topics: $TOPIC_COUNT"
        echo "Status: PASSED"
    } > "$RESULT_FILE"

    print_success "$LAYER_NAME 测试完成"
    echo ""

    return 0
}

# 生成测试报告
generate_report() {
    local REPORT_FILE="$TEST_LOG_DIR/test_report.md"

    print_header "生成测试报告"

    {
        echo "# System Bringup 测试报告"
        echo ""
        echo "**测试时间**: $(date '+%Y-%m-%d %H:%M:%S')"
        echo ""
        echo "## 测试结果汇总"
        echo ""
        echo "| 层级 | 启动文件 | 节点数 | 话题数 | 状态 |"
        echo "|------|---------|--------|--------|------|"

        for result_file in "$TEST_LOG_DIR"/*_result.txt; do
            if [ -f "$result_file" ]; then
                local layer=$(grep "Layer:" "$result_file" | cut -d: -f2 | xargs)
                local launch=$(grep "Launch File:" "$result_file" | cut -d: -f2 | xargs)
                local nodes=$(grep "Nodes:" "$result_file" | cut -d: -f2 | xargs)
                local topics=$(grep "Topics:" "$result_file" | cut -d: -f2 | xargs)
                local status=$(grep "Status:" "$result_file" | cut -d: -f2 | xargs)

                local status_icon="✅"
                [ "$status" != "PASSED" ] && status_icon="❌"

                echo "| $layer | $launch | $nodes | $topics | $status_icon $status |"
            fi
        done

        echo ""
        echo "## 详细日志"
        echo ""
        echo "测试日志目录: \`$TEST_LOG_DIR\`"
        echo ""

        for log_file in "$TEST_LOG_DIR"/*.log; do
            if [ -f "$log_file" ]; then
                local basename=$(basename "$log_file" .log)
                echo "### $basename"
                echo ""
                echo "\`\`\`"
                tail -20 "$log_file"
                echo "\`\`\`"
                echo ""
            fi
        done

    } > "$REPORT_FILE"

    print_success "测试报告已生成: $REPORT_FILE"

    # 显示简要报告
    echo ""
    print_header "测试结果汇总"
    grep -A 20 "## 测试结果汇总" "$REPORT_FILE" | grep "|" || true
}

# 主函数
main() {
    print_header "System Bringup 分层测试"

    # 检查环境
    check_ros_env
    echo ""

    # 运行测试
    local test_duration=15

    # 分层测试
    print_info "开始分层测试..."
    echo ""

    test_layer "1_hardware" "1_hardware.launch.py" $test_duration
    test_layer "2_perception" "2_perception.launch.py" $test_duration
    test_layer "3_intelligence" "3_intelligence.launch.py" $test_duration
    test_layer "4_application" "4_application.launch.py" $test_duration

    # 平台测试
    print_info "开始平台测试..."
    echo ""

    test_layer "qr_debug" "qr_debug.launch.py" $test_duration
    test_layer "qr_raspi" "qr_raspi.launch.py" $test_duration
    test_layer "qr_orin" "qr_orin.launch.py" $test_duration
    test_layer "qr_arm" "qr_arm.launch.py" $test_duration

    # 生成报告
    echo ""
    generate_report

    # 完成
    echo ""
    print_header "测试完成"
    print_success "所有测试已完成"
    print_info "测试日志: $TEST_LOG_DIR"
}

# 运行主函数
main "$@"
