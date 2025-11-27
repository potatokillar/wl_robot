#!/bin/bash
# Show Recent Crashes - coredumpctl wrapper
# 显示最近的崩溃信息和 coredump 列表
#
# 用法:
#   ./show_crashes.sh                    # 显示所有崩溃
#   ./show_crashes.sh --since "1 day ago" # 显示最近1天的崩溃
#   ./show_crashes.sh --today            # 显示今天的崩溃
#   ./show_crashes.sh list              # 列表格式
#   ./show_crashes.sh info <PID>        # 查看特定崩溃详情
#
# 作者: 唐文浩
# 日期: 2025-10-21

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
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

print_title() {
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}  $1${NC}"
    echo -e "${BLUE}========================================${NC}"
}

# 检查 coredumpctl 是否可用
if ! command -v coredumpctl &> /dev/null; then
    print_warn "coredumpctl 未安装，使用传统方法查找 coredump"
    echo ""

    # 显示 /var/coredumps 目录
    if [ -d /var/coredumps ]; then
        print_title "Coredump 文件 (/var/coredumps)"
        ls -lht /var/coredumps/ 2>/dev/null || echo "  (目录为空)"
    fi

    # 显示 /var/crash 目录
    if [ -d /var/crash ]; then
        echo ""
        print_title "Crash 报告 (/var/crash)"
        ls -lht /var/crash/*.crash 2>/dev/null || echo "  (目录为空)"
    fi

    echo ""
    print_info "安装 systemd-coredump 以获得更好的崩溃管理:"
    echo "  sudo apt-get install systemd-coredump"
    exit 0
fi

# 解析命令行参数
SINCE_TIME=""
COMMAND="list"
TARGET_PID=""

while [[ $# -gt 0 ]]; do
    case $1 in
        --since)
            SINCE_TIME="--since=$2"
            shift 2
            ;;
        --today)
            SINCE_TIME="--since=today"
            shift
            ;;
        list)
            COMMAND="list"
            shift
            ;;
        info)
            COMMAND="info"
            TARGET_PID="$2"
            shift 2
            ;;
        dump)
            COMMAND="dump"
            TARGET_PID="$2"
            shift 2
            ;;
        --help|-h)
            echo "用法: $0 [选项] [命令]"
            echo ""
            echo "选项:"
            echo "  --since <时间>     仅显示指定时间之后的崩溃 (如 '1 day ago', '2 hours ago')"
            echo "  --today            仅显示今天的崩溃"
            echo ""
            echo "命令:"
            echo "  list              列表格式显示崩溃 (默认)"
            echo "  info <PID>        查看特定进程的崩溃详情"
            echo "  dump <PID>        提取特定进程的 coredump 文件"
            echo ""
            echo "示例:"
            echo "  $0                          # 显示所有崩溃"
            echo "  $0 --since '1 day ago'      # 显示最近1天"
            echo "  $0 --today                  # 显示今天"
            echo "  $0 info 12345               # 查看 PID 12345 的详情"
            echo "  $0 dump 12345               # 提取 PID 12345 的 core 文件"
            exit 0
            ;;
        *)
            print_error "未知参数: $1"
            echo "使用 --help 查看帮助"
            exit 1
            ;;
    esac
done

# 执行命令
case $COMMAND in
    list)
        print_title "Recent Crashes"
        echo ""

        # 显示崩溃列表
        if coredumpctl list $SINCE_TIME 2>/dev/null; then
            echo ""
            COUNT=$(coredumpctl list $SINCE_TIME 2>/dev/null | tail -n +2 | wc -l)
            if [ "$COUNT" -eq 0 ]; then
                print_info "没有找到崩溃记录"
            else
                print_info "共找到 $COUNT 个崩溃记录"
                echo ""
                print_info "使用以下命令查看详情:"
                echo "  $0 info <PID>       # 查看崩溃详情"
                echo "  $0 dump <PID>       # 提取 core 文件"
                echo "  coredumpctl gdb <PID> # 直接用 gdb 调试"
            fi
        else
            print_warn "没有找到崩溃记录"
        fi
        ;;

    info)
        if [ -z "$TARGET_PID" ]; then
            print_error "请指定 PID: $0 info <PID>"
            exit 1
        fi

        print_title "Crash Details for PID $TARGET_PID"
        echo ""
        coredumpctl info $TARGET_PID

        echo ""
        print_info "使用以下命令进一步分析:"
        echo "  coredumpctl gdb $TARGET_PID           # 使用 gdb 调试"
        echo "  coredumpctl dump $TARGET_PID -o core  # 提取 core 文件"
        ;;

    dump)
        if [ -z "$TARGET_PID" ]; then
            print_error "请指定 PID: $0 dump <PID>"
            exit 1
        fi

        OUTPUT_FILE="core-$TARGET_PID-$(date +%Y%m%d-%H%M%S)"
        print_info "提取 coredump 到: $OUTPUT_FILE"

        if coredumpctl dump $TARGET_PID -o "$OUTPUT_FILE"; then
            print_info "提取成功: $OUTPUT_FILE"
            ls -lh "$OUTPUT_FILE"

            echo ""
            print_info "可使用以下工具分析:"
            echo "  ./tools/analyze_coredump.sh $OUTPUT_FILE"
            echo "  gdb <executable> $OUTPUT_FILE"
        else
            print_error "提取失败"
            exit 1
        fi
        ;;

    *)
        print_error "未知命令: $COMMAND"
        exit 1
        ;;
esac
