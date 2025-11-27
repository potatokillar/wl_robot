#!/bin/bash

##############################################################################
# Dev Server + Frontend 完整部署脚本
# 用途: 构建前端、编译 ROS2 包、部署到目标设备
# 使用: ./deploy_dev_server_frontend.sh [x86|arm] [--no-deploy]
##############################################################################

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# 默认参数
PLATFORM="x86"
SKIP_DEPLOY=false
TARGET_HOST="192.168.1.54"
TARGET_USER="wl"
TARGET_PASS="123456"
TARGET_DIR="/home/wl/autorun/iiri-ros"

# 解析参数
while [[ $# -gt 0 ]]; do
  case $1 in
    x86|arm)
      PLATFORM="$1"
      shift
      ;;
    --no-deploy)
      SKIP_DEPLOY=true
      shift
      ;;
    --help|-h)
      echo "用法: $0 [x86|arm] [--no-deploy]"
      echo ""
      echo "选项:"
      echo "  x86           编译 x86 版本（默认）"
      echo "  arm           编译 ARM 版本"
      echo "  --no-deploy   仅构建，不部署"
      echo "  --help, -h    显示帮助信息"
      exit 0
      ;;
    *)
      echo -e "${RED}未知参数: $1${NC}"
      exit 1
      ;;
  esac
done

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  Dev Server + Frontend 部署脚本${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""
echo -e "${YELLOW}平台:${NC} $PLATFORM"
echo -e "${YELLOW}部署:${NC} $([ "$SKIP_DEPLOY" = true ] && echo "否" || echo "是")"
echo ""

# 步骤 1: 构建前端
echo -e "${GREEN}[1/4] 构建前端...${NC}"
cd src/application_layer/src/dev_server/frontend_src
if [ -f "build_frontend.sh" ]; then
    ./build_frontend.sh
else
    echo -e "${RED}错误: build_frontend.sh 不存在${NC}"
    exit 1
fi
cd - > /dev/null
echo ""

# 步骤 2: 使用分层编译脚本编译 application_layer
echo -e "${GREEN}[2/4] 编译 application_layer (${PLATFORM})...${NC}"
./build_layered.sh "$PLATFORM" application_layer

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ 编译成功${NC}"
    echo ""
else
    echo -e "${RED}✗ 编译失败${NC}"
    exit 1
fi

# 步骤 3: 复制静态文件到 install 目录
echo -e "${GREEN}[3/4] 复制静态文件到 install 目录...${NC}"
INSTALL_STATIC_DIR="install/application_layer/share/dev_server"
mkdir -p "$INSTALL_STATIC_DIR"

cp -r src/application_layer/src/dev_server/static "$INSTALL_STATIC_DIR/"

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ 静态文件复制完成${NC}"
    FILE_COUNT=$(find "$INSTALL_STATIC_DIR/static" -type f | wc -l)
    echo "  - 文件数量: $FILE_COUNT"
    echo ""
else
    echo -e "${RED}✗ 文件复制失败${NC}"
    exit 1
fi

# 步骤 4: 部署到目标设备
if [ "$SKIP_DEPLOY" = false ]; then
    if [ "$PLATFORM" = "arm" ]; then
        echo -e "${GREEN}[4/4] 部署到 Raspberry Pi ($TARGET_HOST)...${NC}"

        # 检查 sshpass
        if ! command -v sshpass &> /dev/null; then
            echo -e "${RED}错误: 未安装 sshpass${NC}"
            echo "安装命令: sudo apt-get install sshpass"
            exit 1
        fi

        # 复制 application_layer
        echo "  → 复制可执行文件和库..."
        sshpass -p "$TARGET_PASS" scp -r install/application_layer \
            "$TARGET_USER@$TARGET_HOST:$TARGET_DIR/install/"

        # 复制静态文件
        echo "  → 复制前端静态文件..."
        sshpass -p "$TARGET_PASS" scp -r "$INSTALL_STATIC_DIR/static" \
            "$TARGET_USER@$TARGET_HOST:$TARGET_DIR/install/application_layer/share/dev_server/"

        # 重启服务
        echo "  → 重启 iiri-ros 服务..."
        sshpass -p "$TARGET_PASS" ssh "$TARGET_USER@$TARGET_HOST" \
            "sudo systemctl restart iiri-ros.service"

        echo ""
        echo -e "${GREEN}========================================${NC}"
        echo -e "${GREEN}  部署完成！${NC}"
        echo -e "${GREEN}========================================${NC}"
        echo ""
        echo -e "${YELLOW}访问地址:${NC}"
        echo "  → http://$TARGET_HOST:8080"
        echo ""
        echo -e "${YELLOW}查看日志:${NC}"
        echo "  → ssh $TARGET_USER@$TARGET_HOST"
        echo "  → sudo journalctl -u iiri-ros.service -f"
        echo ""
    else
        echo -e "${YELLOW}[4/4] x86 平台跳过部署（本地测试）${NC}"
        echo ""
        echo -e "${GREEN}========================================${NC}"
        echo -e "${GREEN}  构建完成（本地测试）${NC}"
        echo -e "${GREEN}========================================${NC}"
        echo ""
        echo -e "${YELLOW}本地运行:${NC}"
        echo "  → cd install/application_layer/lib/application_layer"
        echo "  → ./dev_server"
        echo ""
        echo -e "${YELLOW}访问地址:${NC}"
        echo "  → http://localhost:8080"
        echo ""
    fi
else
    echo -e "${YELLOW}[4/4] 跳过部署步骤${NC}"
    echo ""
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}  构建完成！${NC}"
    echo -e "${GREEN}========================================${NC}"
    echo ""
fi

exit 0
