#!/bin/bash
# Coredump Analysis Tool for IIRI ROS2
# 使用 gdb 自动分析 coredump 文件
#
# 用法:
#   ./analyze_coredump.sh <core_file> <executable>
#   ./analyze_coredump.sh /var/coredumps/core-motion_control_node-12345-1234567890 /home/wl/autorun/iiri-ros/install/lib/motion_control/motion_control_node
#
# 作者: 唐文浩
# 日期: 2025-10-21

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 打印函数
print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 检查参数
if [ $# -lt 1 ]; then
    echo "用法: $0 <core_file> [executable]"
    echo ""
    echo "示例:"
    echo "  $0 /var/coredumps/core-motion_control_node-12345-1234567890"
    echo "  $0 core.12345 /home/wl/autorun/iiri-ros/install/lib/motion_control/motion_control_node"
    echo ""
    echo "如果未指定 executable，脚本将尝试从 core 文件中提取"
    exit 1
fi

CORE_FILE="$1"
EXECUTABLE="$2"

# 检查 core 文件是否存在
if [ ! -f "$CORE_FILE" ]; then
    print_error "Core 文件不存在: $CORE_FILE"
    exit 1
fi

print_info "分析 core 文件: $CORE_FILE"

# 如果未指定可执行文件，尝试从 core 文件提取
if [ -z "$EXECUTABLE" ]; then
    print_info "尝试从 core 文件中提取可执行文件路径..."
    EXECUTABLE=$(file "$CORE_FILE" | grep -oP "from '.*?'" | sed "s/from '//;s/'//" | awk '{print $1}' | head -1)

    if [ -z "$EXECUTABLE" ]; then
        print_error "无法从 core 文件中提取可执行文件路径"
        print_info "请手动指定可执行文件: $0 $CORE_FILE <executable>"
        exit 1
    fi

    print_info "检测到可执行文件: $EXECUTABLE"
fi

# 检查可执行文件是否存在
if [ ! -f "$EXECUTABLE" ]; then
    print_error "可执行文件不存在: $EXECUTABLE"
    exit 1
fi

# 检查 gdb 是否安装
if ! command -v gdb &> /dev/null; then
    print_error "gdb 未安装，请先安装: sudo apt-get install gdb"
    exit 1
fi

# 生成分析报告文件名
REPORT_FILE="${CORE_FILE}.analysis.txt"
print_info "生成分析报告: $REPORT_FILE"

# 创建 gdb 命令脚本
GDB_SCRIPT=$(mktemp)
cat > "$GDB_SCRIPT" << 'EOF'
# GDB 自动分析脚本
set pagination off
set print pretty on
set print array on
set print array-indexes on

echo ====================================\n
echo  Coredump Analysis Report\n
echo ====================================\n
echo \n

echo [1] Binary Information\n
echo ------------------------------------\n
info file
echo \n

echo [2] Signal Information\n
echo ------------------------------------\n
info program
info signals
echo \n

echo [3] Thread Information\n
echo ------------------------------------\n
info threads
echo \n

echo [4] Current Backtrace (详细)\n
echo ------------------------------------\n
bt full
echo \n

echo [5] All Threads Backtrace\n
echo ------------------------------------\n
thread apply all bt
echo \n

echo [6] Register Information\n
echo ------------------------------------\n
info registers
echo \n

echo [7] Local Variables\n
echo ------------------------------------\n
info locals
echo \n

echo [8] Function Arguments\n
echo ------------------------------------\n
info args
echo \n

echo [9] Shared Libraries\n
echo ------------------------------------\n
info sharedlibrary
echo \n

echo [10] Memory Mappings\n
echo ------------------------------------\n
info proc mappings
echo \n

echo ====================================\n
echo  Analysis Complete\n
echo ====================================\n

quit
EOF

# 执行 gdb 分析
print_info "开始 gdb 分析..."
gdb -batch -x "$GDB_SCRIPT" "$EXECUTABLE" "$CORE_FILE" > "$REPORT_FILE" 2>&1

# 清理临时文件
rm -f "$GDB_SCRIPT"

print_info "分析完成！"
print_info "报告已保存到: $REPORT_FILE"

# 显示关键信息摘要
echo ""
echo "========================================"
echo "  关键信息摘要"
echo "========================================"
echo ""

# 提取信号信息
SIGNAL_INFO=$(grep -A 5 "Signal Information" "$REPORT_FILE" | grep -i "signal\|stopped" | head -3)
if [ -n "$SIGNAL_INFO" ]; then
    print_info "崩溃信号:"
    echo "$SIGNAL_INFO"
    echo ""
fi

# 提取栈顶信息
print_info "栈顶帧 (Top 5 frames):"
grep -A 50 "Current Backtrace" "$REPORT_FILE" | grep "^#" | head -5
echo ""

print_info "完整分析报告请查看: $REPORT_FILE"
print_warn "可使用以下命令手动分析: gdb $EXECUTABLE $CORE_FILE"
