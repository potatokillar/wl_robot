#!/bin/bash
# IIRI 服务版本切换工具
# 支持 iiri-ros 和 iiri-qr 两个服务的版本管理

set -e

AUTORUN_DIR="/home/wl/autorun"
ROS_SYMLINK="$AUTORUN_DIR/iiri-ros"
QR_SYMLINK="$AUTORUN_DIR/iiri-qr"

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

show_usage() {
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}IIRI 服务版本切换工具${NC}"
    echo -e "${GREEN}========================================${NC}"
    echo ""
    echo -e "${CYAN}用法:${NC}"
    echo "  $0 list [service]              列出所有已安装版本"
    echo "  $0 current [service]           显示当前版本"
    echo "  $0 ros <版本号>                切换 iiri-ros 到指定版本"
    echo "  $0 qr <版本号>                 切换 iiri-qr 到指定版本"
    echo "  $0 rollback <service>          回滚到上一个版本"
    echo ""
    echo -e "${CYAN}参数说明:${NC}"
    echo "  service: ros, qr, all (默认: all)"
    echo ""
    echo -e "${CYAN}示例:${NC}"
    echo "  $0 list                        # 查看所有服务的版本"
    echo "  $0 list ros                    # 只查看 ros 版本"
    echo "  $0 current                     # 查看当前版本"
    echo "  $0 ros v1.2.3                  # 切换 ros 到 v1.2.3"
    echo "  $0 qr v2.0.1                   # 切换 qr 到 v2.0.1"
    echo "  $0 rollback ros                # 回滚 ros 到上一版本"
    echo "  $0 rollback qr                 # 回滚 qr 到上一版本"
}

# 列出指定服务的版本
list_service_versions() {
    local service="$1"
    local service_name=""
    local symlink_path=""
    local pattern=""

    case "$service" in
        ros)
            service_name="IIRI-ROS"
            symlink_path="$ROS_SYMLINK"
            pattern="iiri-ros-*"
            ;;
        qr)
            service_name="IIRI-QR"
            symlink_path="$QR_SYMLINK"
            pattern="iiri-qr-*"
            ;;
        *)
            echo -e "${RED}错误：未知服务 '$service'${NC}"
            return 1
            ;;
    esac

    echo -e "${BLUE}━━━ $service_name 已安装版本 ━━━${NC}"
    echo ""

    local current_version=""
    if [ -L "$symlink_path" ]; then
        current_version=$(basename "$(readlink -f "$symlink_path")")
    fi

    local versions=$(ls -dt "$AUTORUN_DIR"/$pattern 2>/dev/null | grep -v "\.backup$" || true)

    if [ -z "$versions" ]; then
        echo "  (无已安装版本)"
        echo ""
        return
    fi

    local idx=1
    for version_path in $versions; do
        local version_name=$(basename "$version_path")
        local version_file="$version_path/VERSION.txt"
        local version_info=""
        local build_date=""

        if [ -f "$version_file" ]; then
            version_info=$(grep "^Version:" "$version_file" | cut -d: -f2 | xargs)
            build_date=$(grep "^Build Date:" "$version_file" | cut -d: -f2- | xargs)
        fi

        if [ "$version_name" == "$current_version" ]; then
            echo -e "  ${GREEN}[$idx] $version_name  ← 当前版本${NC}"
        else
            echo "  [$idx] $version_name"
        fi

        if [ -n "$version_info" ]; then
            echo "       版本: $version_info"
        fi
        if [ -n "$build_date" ]; then
            echo "       构建时间: $build_date"
        fi

        idx=$((idx + 1))
    done
    echo ""
}

# 列出所有版本
list_versions() {
    local service="${1:-all}"

    case "$service" in
        all)
            list_service_versions "ros"
            list_service_versions "qr"
            ;;
        ros|qr)
            list_service_versions "$service"
            ;;
        *)
            echo -e "${RED}错误：未知服务 '$service'${NC}"
            echo "支持的服务: ros, qr, all"
            return 1
            ;;
    esac
}

# 显示指定服务的当前版本
show_service_current() {
    local service="$1"
    local service_name=""
    local symlink_path=""
    local service_unit=""

    case "$service" in
        ros)
            service_name="IIRI-ROS"
            symlink_path="$ROS_SYMLINK"
            service_unit="iiri-ros.service"
            ;;
        qr)
            service_name="IIRI-QR"
            symlink_path="$QR_SYMLINK"
            service_unit="iiri-qr.service"
            ;;
        *)
            echo -e "${RED}错误：未知服务 '$service'${NC}"
            return 1
            ;;
    esac

    echo -e "${BLUE}━━━ $service_name 当前版本 ━━━${NC}"

    if [ ! -L "$symlink_path" ]; then
        echo -e "${YELLOW}未找到当前版本（符号链接不存在）${NC}"
        echo ""
        return
    fi

    local current=$(readlink -f "$symlink_path")
    local current_name=$(basename "$current")

    echo -e "${GREEN}版本:${NC} $current_name"
    echo -e "${BLUE}路径:${NC} $current"

    # 显示服务状态
    if systemctl is-active --quiet "$service_unit" 2>/dev/null; then
        echo -e "${GREEN}状态: ● 运行中${NC}"
    else
        echo -e "${YELLOW}状态: ○ 已停止${NC}"
    fi

    if [ -f "$current/VERSION.txt" ]; then
        echo ""
        cat "$current/VERSION.txt"
    fi
    echo ""
}

# 显示当前版本
show_current() {
    local service="${1:-all}"

    case "$service" in
        all)
            show_service_current "ros"
            show_service_current "qr"
            ;;
        ros|qr)
            show_service_current "$service"
            ;;
        *)
            echo -e "${RED}错误：未知服务 '$service'${NC}"
            echo "支持的服务: ros, qr, all"
            return 1
            ;;
    esac
}

# 查找匹配的版本路径
find_version_path() {
    local service="$1"
    local target="$2"
    local prefix=""

    case "$service" in
        ros)
            prefix="iiri-ros"
            ;;
        qr)
            prefix="iiri-qr"
            ;;
        *)
            return 1
            ;;
    esac

    # 1. 尝试完整路径
    if [ -d "$AUTORUN_DIR/$target" ]; then
        echo "$AUTORUN_DIR/$target"
        return 0
    fi

    # 2. 尝试添加前缀匹配（x86/arm）
    if [ -d "$AUTORUN_DIR/$prefix-x86-$target" ]; then
        echo "$AUTORUN_DIR/$prefix-x86-$target"
        return 0
    elif [ -d "$AUTORUN_DIR/$prefix-arm-$target" ]; then
        echo "$AUTORUN_DIR/$prefix-arm-$target"
        return 0
    fi

    # 3. 模糊匹配
    local matches=$(ls -d "$AUTORUN_DIR"/$prefix-*"$target"* 2>/dev/null || true)
    local match_count=$(echo "$matches" | grep -c . || echo "0")

    if [ "$match_count" -eq 1 ]; then
        echo "$matches"
        return 0
    elif [ "$match_count" -gt 1 ]; then
        echo -e "${RED}错误：找到多个匹配版本${NC}" >&2
        echo "" >&2
        echo "$matches" >&2
        echo "" >&2
        echo "请指定完整版本号" >&2
        return 1
    else
        echo -e "${RED}错误：未找到版本 '$target'${NC}" >&2
        echo "" >&2
        echo "使用 '$0 list $service' 查看可用版本" >&2
        return 1
    fi
}

# 切换服务版本
switch_service_version() {
    local service="$1"
    local target="$2"
    local service_name=""
    local symlink_path=""
    local service_unit=""

    case "$service" in
        ros)
            service_name="IIRI-ROS"
            symlink_path="$ROS_SYMLINK"
            service_unit="iiri-ros.service"
            ;;
        qr)
            service_name="IIRI-QR"
            symlink_path="$QR_SYMLINK"
            service_unit="iiri-qr.service"
            ;;
        *)
            echo -e "${RED}错误：未知服务 '$service'${NC}"
            return 1
            ;;
    esac

    # 查找目标版本
    local target_path=$(find_version_path "$service" "$target")
    if [ $? -ne 0 ]; then
        return 1
    fi

    local target_name=$(basename "$target_path")

    echo -e "${BLUE}━━━ 切换 $service_name 版本 ━━━${NC}"
    echo ""

    # 停止服务
    echo -e "${BLUE}>>> 停止服务: $service_unit${NC}"
    systemctl stop "$service_unit" 2>/dev/null || true
    sleep 1

    # 更新符号链接
    echo -e "${BLUE}>>> 切换到: $target_name${NC}"
    ln -snf "$target_path" "$symlink_path"

    # 启动服务
    echo -e "${BLUE}>>> 启动服务: $service_unit${NC}"
    systemctl start "$service_unit"

    # 等待服务启动
    sleep 2

    # 检查状态
    if systemctl is-active --quiet "$service_unit"; then
        echo ""
        echo -e "${GREEN}✅ $service_name 版本切换成功！${NC}"
        echo ""
    else
        echo ""
        echo -e "${RED}⚠️  $service_name 服务启动失败${NC}"
        echo ""
        echo "请检查日志: sudo journalctl -u $service_unit -n 50"
        return 1
    fi
}

# 回滚到上一版本
rollback_service() {
    local service="$1"
    local symlink_path=""
    local pattern=""

    case "$service" in
        ros)
            symlink_path="$ROS_SYMLINK"
            pattern="iiri-ros-*"
            ;;
        qr)
            symlink_path="$QR_SYMLINK"
            pattern="iiri-qr-*"
            ;;
        *)
            echo -e "${RED}错误：未知服务 '$service'${NC}"
            return 1
            ;;
    esac

    if [ ! -L "$symlink_path" ]; then
        echo -e "${RED}错误：找不到当前版本${NC}"
        return 1
    fi

    local current=$(readlink -f "$symlink_path")
    local versions=$(ls -dt "$AUTORUN_DIR"/$pattern 2>/dev/null | grep -v "\.backup$" || true)

    local previous=""
    local found_current=false

    for version in $versions; do
        if [ "$found_current" = true ]; then
            previous="$version"
            break
        fi

        if [ "$version" == "$current" ]; then
            found_current=true
        fi
    done

    if [ -z "$previous" ]; then
        echo -e "${YELLOW}没有可回滚的旧版本${NC}"
        return 1
    fi

    echo -e "${BLUE}回滚到: $(basename "$previous")${NC}"
    echo ""
    switch_service_version "$service" "$(basename "$previous")"
}

# 主函数
main() {
    # 检查 root 权限
    if [ "$EUID" -ne 0 ]; then
        echo -e "${RED}错误：需要 root 权限${NC}"
        echo "请使用: sudo $0 $@"
        exit 1
    fi

    local command="${1:-help}"

    case "$command" in
        list)
            list_versions "${2:-all}"
            ;;
        current)
            show_current "${2:-all}"
            ;;
        ros)
            if [ -z "$2" ]; then
                echo -e "${RED}错误：请指定版本号${NC}"
                echo "用法: $0 ros <版本号>"
                exit 1
            fi
            switch_service_version "ros" "$2"
            ;;
        qr)
            if [ -z "$2" ]; then
                echo -e "${RED}错误：请指定版本号${NC}"
                echo "用法: $0 qr <版本号>"
                exit 1
            fi
            switch_service_version "qr" "$2"
            ;;
        rollback)
            if [ -z "$2" ]; then
                echo -e "${RED}错误：请指定服务名称${NC}"
                echo "用法: $0 rollback <ros|qr>"
                exit 1
            fi
            rollback_service "$2"
            ;;
        help|--help|-h)
            show_usage
            ;;
        *)
            echo -e "${RED}错误：未知命令 '$command'${NC}"
            echo ""
            show_usage
            exit 1
            ;;
    esac
}

main "$@"
