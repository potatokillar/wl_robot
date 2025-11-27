#!/bin/bash

# 同步脚本 - 用于管理和同步所有层的代码

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 打印带颜色的消息
print_msg() {
    echo -e "${GREEN}[SYNC]${NC} $1"
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
    echo "  status          显示所有仓库状态"
    echo "  pull            拉取所有仓库更新"
    echo "  push [选项]     推送所有仓库改动"
    echo "  import          导入所有仓库（首次使用）"
    echo "  switch [配置]   切换配置文件"
    echo "  clean           清理所有仓库（谨慎使用）"
    echo "  help            显示此帮助信息"
    echo ""
    echo "配置选项（用于switch命令）:"
    echo "  main            使用主分支配置 (.repos)"
    echo "  devel           使用开发分支配置 (.repos.devel)"
    echo "  stable          使用稳定版本配置 (.repos.stable)"
    echo ""
    echo "push命令选项:"
    echo "  --update-main   自动更新主仓库指针（无需确认）"
    echo ""
    echo "示例:"
    echo "  $0 import                   # 首次导入所有仓库"
    echo "  $0 status                   # 检查所有仓库状态"
    echo "  $0 pull                     # 更新所有仓库"
    echo "  $0 push                     # 推送所有仓库（交互式）"
    echo "  $0 push --update-main       # 推送并自动更新主仓库"
    echo "  $0 switch devel             # 切换到开发分支"
    echo "  $0 switch stable            # 切换到稳定版本"
}

# 检查vcstool是否可用
check_vcstool() {
    if ! command -v vcs &> /dev/null; then
        print_error "vcstool未安装，请先安装: sudo apt install python3-vcstool"
        exit 1
    fi
}

# 检查.repos文件是否存在
check_repos_file() {
    if [ ! -f ".repos" ]; then
        print_error "未找到.repos配置文件"
        exit 1
    fi
}

# 显示仓库状态
show_status() {
    print_msg "正在检查所有仓库状态..."
    if [ -d "src" ]; then
        vcs status src
    else
        print_warn "src目录不存在，请先运行 'import' 命令"
    fi
}

# 拉取更新
pull_updates() {
    print_msg "正在拉取所有仓库更新..."
    if [ -d "src" ]; then
        vcs pull src
        print_msg "更新完成！"
    else
        print_warn "src目录不存在，请先运行 'import' 命令"
    fi
}

# 导入仓库
import_repos() {
    print_msg "正在导入所有仓库..."

    # 创建src目录
    mkdir -p src

    # 导入仓库
    vcs import src < .repos

    print_msg "导入完成！"
    print_info "提示：现在可以运行 './build.sh' 来构建项目"
}

# 切换配置
switch_config() {
    local config=$1

    case $config in
        main)
            if [ -f ".repos.main" ]; then
                cp .repos.main .repos
            else
                print_info "使用默认主配置"
            fi
            print_msg "已切换到主分支配置"
            ;;
        devel)
            if [ -f ".repos.devel" ]; then
                cp .repos.devel .repos
                print_msg "已切换到开发分支配置"
            else
                print_error "未找到开发分支配置文件 .repos.devel"
                exit 1
            fi
            ;;
        stable)
            if [ -f ".repos.stable" ]; then
                cp .repos.stable .repos
                print_msg "已切换到稳定版本配置"
            else
                print_error "未找到稳定版本配置文件 .repos.stable"
                exit 1
            fi
            ;;
        *)
            print_error "未知配置: $config"
            print_info "可用配置: main, devel, stable"
            exit 1
            ;;
    esac

    print_info "当前配置内容:"
    cat .repos

    print_warn "配置已更改，建议运行 'vcs pull src' 来更新代码"
}

# 推送所有子仓库改动
push_changes() {
    print_msg "正在推送所有子仓库改动..."

    if [ ! -d "src" ]; then
        print_warn "src目录不存在，请先运行 'import' 命令"
        return 1
    fi

    # 推送所有子仓库
    vcs push src/

    local push_result=$?

    if [ $push_result -eq 0 ]; then
        print_msg "子仓库推送完成！"
    else
        print_error "子仓库推送失败，请检查错误信息"
        return 1
    fi

    # 检查主仓库是否需要更新子模块指针
    print_info "检查主仓库子模块指针状态..."

    if ! git diff-index --quiet HEAD -- src/ 2>/dev/null; then
        print_warn "检测到子模块指针变化！"
        print_info ""
        print_info "子模块已推送，但主仓库的子模块指针需要更新"
        print_info ""

        # 显示变化的子模块
        git status -s src/ 2>/dev/null | while read line; do
            print_info "  $line"
        done

        print_info ""

        # 询问是否自动更新
        if [ "$1" == "--update-main" ]; then
            update_main_repo
        else
            print_info "请选择操作："
            print_info "  1. 自动更新主仓库指针并推送"
            print_info "  2. 手动更新（跳过）"
            print_info ""
            read -p "请输入选项 [1/2]: " choice

            case $choice in
                1)
                    update_main_repo
                    ;;
                2)
                    print_info "已跳过主仓库更新"
                    print_info "稍后可手动运行："
                    echo "  git add src/"
                    echo "  git commit -m 'chore: 更新子模块指针'"
                    echo "  git push"
                    ;;
                *)
                    print_warn "无效选项，已跳过"
                    ;;
            esac
        fi
    else
        print_msg "主仓库子模块指针无需更新"
    fi
}

# 更新主仓库子模块指针
update_main_repo() {
    print_msg "正在更新主仓库子模块指针..."

    # 添加所有子模块变化
    git add src/

    # 生成提交信息
    local changed_modules=$(git diff --cached --name-only src/ | sed 's|src/||g' | tr '\n' ', ' | sed 's/,$//')
    local commit_msg="chore: 更新子模块指针

更新的子模块: $changed_modules

📝 作者：唐文浩

Co-Authored-By: 唐文浩 <twh@westlake.edu.cn>"

    # 提交
    if git commit -m "$commit_msg"; then
        print_msg "主仓库提交成功"

        # 推送
        if git push; then
            print_msg "主仓库推送成功！"
            print_info ""
            print_info "✅ 所有改动已完整推送："
            print_info "   - 子仓库改动已推送"
            print_info "   - 主仓库子模块指针已更新"
        else
            print_error "主仓库推送失败，请检查网络或权限"
            return 1
        fi
    else
        print_error "主仓库提交失败"
        return 1
    fi
}

# 清理仓库
clean_repos() {
    print_warn "这将删除src目录下的所有代码，确认继续吗？(y/N)"
    read -r confirmation

    if [[ $confirmation =~ ^[Yy]$ ]]; then
        print_msg "正在清理仓库..."
        rm -rf src
        print_msg "清理完成！"
        print_info "重新导入代码请运行: $0 import"
    else
        print_info "操作已取消"
    fi
}

# 主函数
main() {
    # 检查vcstool
    check_vcstool

    # 处理命令
    case ${1:-help} in
        status)
            check_repos_file
            show_status
            ;;
        pull)
            check_repos_file
            pull_updates
            ;;
        push)
            check_repos_file
            push_changes "$2"
            ;;
        import)
            check_repos_file
            import_repos
            ;;
        switch)
            if [ -z "$2" ]; then
                print_error "switch命令需要指定配置类型"
                print_info "用法: $0 switch [main|devel|stable]"
                exit 1
            fi
            switch_config "$2"
            ;;
        clean)
            clean_repos
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