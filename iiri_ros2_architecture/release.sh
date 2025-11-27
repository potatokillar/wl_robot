#!/bin/bash

# 发布脚本 - 用于创建和管理项目版本发布

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 打印带颜色的消息
print_msg() {
    echo -e "${GREEN}[RELEASE]${NC} $1"
}

print_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

# 显示帮助信息
show_help() {
    echo "用法: $0 [命令] [选项]"
    echo ""
    echo "命令:"
    echo "  create [版本号]     创建新版本发布"
    echo "  list               列出所有版本标签"
    echo "  status             显示各层当前版本状态"
    echo "  validate           验证发布准备状态"
    echo "  help               显示此帮助信息"
    echo ""
    echo "版本号格式: vX.Y.Z (例如: v1.0.0, v1.2.3)"
    echo ""
    echo "示例:"
    echo "  $0 validate                 # 验证发布准备状态"
    echo "  $0 create v1.0.1           # 创建v1.0.1版本"
    echo "  $0 list                    # 列出所有版本"
    echo "  $0 status                  # 显示当前状态"
}

# 层级列表
LAYERS=("core_layer" "hardware_layer" "perception_layer" "intelligence_layer" "application_layer")

# 验证版本号格式
validate_version() {
    local version=$1
    if [[ ! $version =~ ^v[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
        print_error "版本号格式错误，应该是 vX.Y.Z 格式，例如: v1.0.0"
        exit 1
    fi
}

# 检查是否在git仓库中
check_git_repo() {
    if ! git rev-parse --git-dir > /dev/null 2>&1; then
        print_error "当前目录不是git仓库"
        exit 1
    fi
}

# 检查工作区是否干净
check_clean_workdir() {
    if ! git diff-index --quiet HEAD --; then
        print_error "工作区有未提交的更改，请先提交所有更改"
        print_info "运行 'git status' 查看未提交的文件"
        exit 1
    fi
}

# 验证发布准备状态
validate_release() {
    print_msg "正在验证发布准备状态..."

    # 检查主工作区
    check_git_repo
    check_clean_workdir

    # 检查src目录是否存在
    if [ ! -d "src" ]; then
        print_error "src目录不存在，请先运行 './sync.sh import'"
        exit 1
    fi

    # 检查每个层的状态
    local all_clean=true
    for layer in "${LAYERS[@]}"; do
        local layer_path="src/$layer"
        if [ -d "$layer_path" ]; then
            print_info "检查 $layer..."
            cd "$layer_path"

            # 检查是否是git仓库
            if ! git rev-parse --git-dir > /dev/null 2>&1; then
                print_warn "$layer 不是git仓库"
                all_clean=false
            else
                # 检查是否有未提交的更改
                if ! git diff-index --quiet HEAD --; then
                    print_warn "$layer 有未提交的更改"
                    all_clean=false
                fi

                # 检查是否有未推送的提交
                local unpushed=$(git log @{u}.. --oneline 2>/dev/null | wc -l)
                if [ "$unpushed" -gt 0 ]; then
                    print_warn "$layer 有 $unpushed 个未推送的提交"
                    all_clean=false
                fi
            fi

            cd - > /dev/null
        else
            print_warn "层 $layer 不存在"
            all_clean=false
        fi
    done

    if [ "$all_clean" = true ]; then
        print_msg "所有层都已准备好发布！"
        return 0
    else
        print_error "部分层未准备好发布，请检查上述警告"
        return 1
    fi
}

# 显示各层状态
show_status() {
    print_msg "显示各层版本状态..."

    if [ ! -d "src" ]; then
        print_error "src目录不存在，请先运行 './sync.sh import'"
        exit 1
    fi

    for layer in "${LAYERS[@]}"; do
        local layer_path="src/$layer"
        if [ -d "$layer_path" ]; then
            cd "$layer_path"
            if git rev-parse --git-dir > /dev/null 2>&1; then
                local current_branch=$(git branch --show-current)
                local latest_tag=$(git describe --tags --abbrev=0 2>/dev/null || echo "无标签")
                local commit_hash=$(git rev-parse --short HEAD)

                print_info "$layer:"
                echo "  分支: $current_branch"
                echo "  最新标签: $latest_tag"
                echo "  提交: $commit_hash"
                echo ""
            else
                print_warn "$layer: 不是git仓库"
            fi
            cd - > /dev/null
        else
            print_warn "$layer: 目录不存在"
        fi
    done
}

# 列出所有版本标签
list_versions() {
    print_msg "列出主工作空间的所有版本标签..."

    check_git_repo

    local tags=$(git tag -l "v*" --sort=-version:refname)

    if [ -z "$tags" ]; then
        print_info "未找到版本标签"
    else
        print_info "已发布的版本:"
        echo "$tags"
    fi
}

# 创建发布版本
create_release() {
    local version=$1

    if [ -z "$version" ]; then
        print_error "请指定版本号"
        print_info "用法: $0 create v1.0.0"
        exit 1
    fi

    validate_version "$version"

    print_msg "正在创建版本 $version..."

    # 验证发布状态
    if ! validate_release; then
        print_error "发布验证失败，请先解决上述问题"
        exit 1
    fi

    # 确认发布
    print_warn "确认创建版本 $version 吗？(y/N)"
    read -r confirmation

    if [[ ! $confirmation =~ ^[Yy]$ ]]; then
        print_info "操作已取消"
        exit 0
    fi

    # 为每个层打标签
    print_msg "为各层创建标签..."
    for layer in "${LAYERS[@]}"; do
        local layer_path="src/$layer"
        if [ -d "$layer_path" ]; then
            cd "$layer_path"
            if git rev-parse --git-dir > /dev/null 2>&1; then
                print_info "为 $layer 创建标签 $version"
                git tag "$version"
                git push origin "$version"
            fi
            cd - > /dev/null
        fi
    done

    # 更新稳定版本配置
    print_msg "更新稳定版本配置..."

    # 备份当前.repos文件
    cp .repos .repos.backup

    # 更新.repos.stable文件
    if [ -f ".repos.stable" ]; then
        # 将.repos.stable中的所有version字段更新为新版本
        sed "s/version: v[0-9]\+\.[0-9]\+\.[0-9]\+/version: $version/g" .repos.stable > .repos.new
        mv .repos.new .repos.stable

        # 将稳定版本设置为当前版本
        cp .repos.stable .repos

        print_info "已更新稳定版本配置到 $version"
    else
        print_warn "未找到.repos.stable文件，跳过更新"
    fi

    # 提交主工作空间更改
    print_msg "提交主工作空间更改..."
    git add .repos .repos.stable
    git commit -m "release: $version

Update all layers to $version

📦 作者：唐文浩

Co-Authored-By: 唐文浩 <twh@example.com>"

    # 为主工作空间打标签
    git tag "$version"

    print_msg "版本 $version 创建完成！"
    print_info "推送到远程仓库请运行: git push origin main --tags"

    # 显示发布摘要
    print_info "发布摘要:"
    echo "  版本: $version"
    echo "  包含层: ${LAYERS[*]}"
    echo "  主工作空间提交: $(git rev-parse --short HEAD)"
}

# 主函数
main() {
    # 处理命令
    case ${1:-help} in
        validate)
            validate_release
            ;;
        status)
            show_status
            ;;
        list)
            list_versions
            ;;
        create)
            create_release "$2"
            ;;
        help|--help|-h)
            show_help
            ;;
        *)
            print_error "未知命令: $1"
            show_help
            exit 1
            ;;
    esac
}

# 运行主函数
main "$@"