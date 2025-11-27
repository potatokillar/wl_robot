#!/bin/bash
# IIRI-QR 一键安装脚本

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# 获取脚本所在目录（版本特定目录）
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VERSION_DIR=$(basename "$SCRIPT_DIR")
TARGET_DIR="/home/wl/autorun"
SYMLINK_PATH="$TARGET_DIR/iiri-qr"

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}IIRI-QR 一键安装${NC}"
echo -e "${GREEN}========================================${NC}"
echo -e "${BLUE}版本:${NC} $VERSION_DIR"
echo ""

# 1. 确保在 autorun 目录中
if [[ "$SCRIPT_DIR" != "$TARGET_DIR"/* ]]; then
    echo -e "${RED}错误：请将部署包解压到 $TARGET_DIR 目录${NC}"
    echo ""
    echo -e "${YELLOW}正确步骤：${NC}"
    echo "  tar -xzf iiri-qr-*.tar.gz -C $TARGET_DIR"
    echo "  cd $TARGET_DIR/iiri-qr-*"
    echo "  sudo ./install.sh"
    exit 1
fi

# 2. 自动创建符号链接
echo -e "${BLUE}>>> 创建符号链接...${NC}"
ln -snf "$SCRIPT_DIR" "$SYMLINK_PATH"
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✅ 符号链接已创建: $SYMLINK_PATH -> $SCRIPT_DIR${NC}"
else
    echo -e "${RED}❌ 符号链接创建失败${NC}"
    exit 1
fi
echo ""

# 3. 调用部署脚本
echo -e "${BLUE}>>> 部署 systemd 服务...${NC}"
cd "$SYMLINK_PATH"
./deploy_qr_service.sh deploy

echo ""
echo -e "${GREEN}==========================================${NC}"
echo -e "${GREEN}✅ 安装完成！${NC}"
echo -e "${GREEN}==========================================${NC}"
echo ""
echo -e "${BLUE}当前版本:${NC} $VERSION_DIR"
echo -e "${BLUE}部署路径:${NC} $SYMLINK_PATH"
echo ""
echo -e "${YELLOW}常用命令:${NC}"
echo "  查看状态: sudo systemctl status iiri-qr.service"
echo "  查看日志: sudo journalctl -u iiri-qr.service -f"
echo ""
