#!/bin/bash
# 版本发布脚本
# 用法:
#   ./release.sh           - 查看当前版本状态

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 显示当前状态
show_status() {
    local current_branch=$(git rev-parse --abbrev-ref HEAD)
    local commit_short=$(git rev-parse --short HEAD)

    # 获取远程最新tag
    echo -e "${YELLOW}正在获取远程最新tag...${NC}"
    git fetch --tags 2>/dev/null || true
    local latest_tag=$(git tag --sort=-version:refname | head -1 2>/dev/null || echo "无tag")

    echo -e "${BLUE}=== 版本状态 ===${NC}"
    echo -e "当前分支: ${YELLOW}$current_branch${NC}"
    echo -e "最新commit: ${YELLOW}$commit_short${NC}"
    echo -e "远程最新tag: ${YELLOW}$latest_tag${NC}"

    if [ "$current_branch" = "main" ]; then
        echo -e "版本模式: ${GREEN}发布模式 (使用远程最新tag: $latest_tag)${NC}"
    else
        local timestamp=$(date +"%Y%m%d_%H%M%S")
        echo -e "版本模式: ${GREEN}开发模式 (${current_branch}_${timestamp})${NC}"
    fi

    echo ""
    echo -e "${BLUE}版本规则：${NC}"
    echo -e "  • ${YELLOW}main分支${NC}: software = 远程最新git tag (如: v2.0.0)"
    echo -e "  • ${YELLOW}其他分支${NC}: software = 分支名_编译时间 (如: dev_20250925_093000)"
}

# 主逻辑
case "$1" in
    "")
        show_status
        ;;
    *)
        echo -e "${BLUE}基于分支的版本管理脚本${NC}"
        echo ""
        echo "用法:"
        echo -e "  ${YELLOW}./release.sh${NC}           - 查看当前版本状态"
        echo ""
        echo -e "${BLUE}版本管理规则：${NC}"
        echo -e "  • ${YELLOW}开发时${NC}: 在dev/feature分支开发，编译时software显示 '分支名_时间'"
        echo -e "  • ${YELLOW}发布时${NC}: 切换到main分支，编译时software显示远程最新tag版本"
        echo ""
        echo -e "${BLUE}工作流程：${NC}"
        echo -e "  1. 在dev分支开发: ${YELLOW}git checkout dev && ./build.sh${NC}"
        echo -e "  2. 开发完成后切换到main: ${YELLOW}git checkout main${NC}"
        echo -e "  3. 编译发布版本: ${YELLOW}./build.sh${NC} (自动使用远程最新tag)"
        echo -e "  4. 查看版本状态: ${YELLOW}./release.sh${NC}"
        echo ""
        echo -e "${BLUE}说明：${NC}"
        echo -e "  • tag由其他人员在远程仓库创建"
        echo -e "  • main分支编译时自动获取远程最新tag作为版本号"
        ;;
esac