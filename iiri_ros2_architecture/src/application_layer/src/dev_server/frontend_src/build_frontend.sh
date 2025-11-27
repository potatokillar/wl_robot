#!/bin/bash

##############################################################################
# 前端构建脚本
# 用途: 编译 Vue 3 前端为静态文件，并复制到 static 目录
# 使用: ./build_frontend.sh
##############################################################################

set -e  # 遇到错误立即退出

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 进入脚本所在目录
cd "$(dirname "$0")"

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  Dev Server 前端构建脚本${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

# 检查 Node.js 和 npm
if ! command -v node &> /dev/null; then
    echo -e "${RED}错误: 未安装 Node.js${NC}"
    echo "请访问 https://nodejs.org/ 安装 Node.js"
    exit 1
fi

if ! command -v npm &> /dev/null; then
    echo -e "${RED}错误: 未安装 npm${NC}"
    exit 1
fi

echo -e "${YELLOW}Node 版本:${NC} $(node --version)"
echo -e "${YELLOW}npm 版本:${NC} $(npm --version)"
echo ""

# 安装依赖（首次或 package.json 更新时）
if [ ! -d "node_modules" ] || [ "package.json" -nt "node_modules" ]; then
    echo -e "${YELLOW}[1/3] 安装前端依赖...${NC}"
    npm install
    echo -e "${GREEN}✓ 依赖安装完成${NC}"
    echo ""
else
    echo -e "${GREEN}[1/3] 依赖已是最新，跳过安装${NC}"
    echo ""
fi

# 构建生产版本
echo -e "${YELLOW}[2/3] 构建前端（生产模式）...${NC}"
npm run build

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ 前端构建完成${NC}"
    echo ""
else
    echo -e "${RED}✗ 前端构建失败${NC}"
    exit 1
fi

# 复制构建产物到 static 目录
echo -e "${YELLOW}[3/3] 复制构建文件到 static 目录...${NC}"

STATIC_DIR="../static"
DIST_DIR="dist"

# 验证 dist 目录存在
if [ ! -d "$DIST_DIR" ]; then
    echo -e "${RED}✗ dist 目录不存在${NC}"
    exit 1
fi

# 统计 dist 中的源文件
DIST_FILE_COUNT=$(find "$DIST_DIR" -type f | wc -l)
echo -e "${YELLOW}Source (dist): $DIST_FILE_COUNT 文件${NC}"

# 清空 static 目录（完全删除并重建）
if [ -d "$STATIC_DIR" ]; then
    echo -e "${YELLOW}清空 $STATIC_DIR 目录...${NC}"
    rm -rf "$STATIC_DIR"
    if [ $? -ne 0 ]; then
        echo -e "${RED}✗ 删除 static 目录失败${NC}"
        exit 1
    fi
fi

# 重新创建 static 目录
mkdir -p "$STATIC_DIR"
if [ $? -ne 0 ]; then
    echo -e "${RED}✗ 创建 static 目录失败${NC}"
    exit 1
fi

# 复制 dist 目录的所有内容（包括隐藏文件，但排除 .git）
echo -e "${YELLOW}复制文件中...${NC}"
cp -rv "$DIST_DIR"/* "$STATIC_DIR/" 2>&1 | head -20
CP_EXIT_CODE=$?

if [ $CP_EXIT_CODE -ne 0 ]; then
    echo -e "${RED}✗ 文件复制失败（exit code: $CP_EXIT_CODE）${NC}"
    exit 1
fi

echo ""
echo -e "${GREEN}✓ 文件复制完成${NC}"
echo ""

# 验证复制结果
STATIC_FILE_COUNT=$(find "$STATIC_DIR" -type f | wc -l)
STATIC_TOTAL_SIZE=$(du -sh "$STATIC_DIR" | cut -f1)

if [ "$STATIC_FILE_COUNT" -lt "$DIST_FILE_COUNT" ]; then
    echo -e "${RED}⚠ 警告：文件数量不匹配！${NC}"
    echo "  Expected: $DIST_FILE_COUNT files"
    echo "  Actual:   $STATIC_FILE_COUNT files"
    exit 1
fi

# 详细文件统计
echo -e "${GREEN}构建结果统计:${NC}"
echo "  - 源文件数量 (dist):   $DIST_FILE_COUNT"
echo "  - 目标文件数量 (static): $STATIC_FILE_COUNT"
echo "  - 总大小:           $STATIC_TOTAL_SIZE"
echo ""

# 列出关键文件
echo -e "${GREEN}关键文件验证:${NC}"
for file in "index.html" "assets/index-*.js" "assets/index-*.css" "favicon.ico"; do
    if compgen -G "$STATIC_DIR/$file" > /dev/null 2>&1; then
        FILE_LIST=$(find "$STATIC_DIR" -path "*$file" -type f 2>/dev/null)
        echo -e "${GREEN}  ✓${NC} $(basename "$FILE_LIST")"
    else
        echo -e "${RED}  ✗${NC} $file (未找到)"
    fi
done
echo ""

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  前端构建成功！${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo -e "${YELLOW}提示:${NC}"
echo "  - 静态文件位于: $STATIC_DIR"
echo "  - 继续运行分层编译脚本以部署到 ROS2"
echo ""

exit 0
