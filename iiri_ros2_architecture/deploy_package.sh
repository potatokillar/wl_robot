#!/bin/bash

#=============================================================================
# IIRI ROS2 Deployment Package Builder
# 功能：打包 iiri-ros 部署包，包含编译产物和启动脚本
# 用法：./deploy_package.sh [x86|arm] [output_dir]
# 作者：Claude Code
# 日期：2025-10-11
#=============================================================================

set -e  # 遇到错误立即退出

# Color definitions
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Print functions
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_header() {
    echo ""
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}$1${NC}"
    echo -e "${GREEN}========================================${NC}"
}

# 显示使用说明
show_usage() {
    echo "IIRI ROS2 Deployment Package Builder"
    echo ""
    echo "Usage: $0 [--no-checksum] [architecture] [output_dir]"
    echo ""
    echo "Options:"
    echo "  --no-checksum Skip SHA256 checksum generation (faster)"
    echo ""
    echo "Arguments:"
    echo "  architecture  Target architecture: x86 or arm (default: auto-detect)"
    echo "  output_dir    Output directory (default: deploy_packages)"
    echo ""
    echo "Examples:"
    echo "  $0                        # Auto-detect architecture with SHA256"
    echo "  $0 --no-checksum x86      # Build x86 package without checksum"
    echo "  $0 x86                    # Build x86 package with checksum"
    echo "  $0 arm custom_dir         # Build ARM package to custom directory"
    echo ""
}

# 参数处理
if [ "$1" == "help" ] || [ "$1" == "-h" ] || [ "$1" == "--help" ]; then
    show_usage
    exit 0
fi

# 检查 --no-checksum 参数
SKIP_CHECKSUM=false
if [ "$1" == "--no-checksum" ]; then
    SKIP_CHECKSUM=true
    shift  # 移除第一个参数
fi

# 自动检测架构
ARCH=$(uname -m)
if [ "$ARCH" == "x86_64" ]; then
    BUILD_ARCH="x86"
elif [ "$ARCH" == "aarch64" ]; then
    BUILD_ARCH="arm"
else
    BUILD_ARCH="x86"  # 默认
fi

# 从参数获取架构（如果提供）
if [ -n "$1" ]; then
    if [ "$1" == "x86" ] || [ "$1" == "arm" ]; then
        BUILD_ARCH="$1"
        shift
    else
        print_error "Unknown architecture: $1"
        show_usage
        exit 1
    fi
fi

# 输出目录
OUTPUT_DIR="${1:-deploy_packages}"

# 获取版本号
if git rev-parse --git-dir > /dev/null 2>&1; then
    VERSION=$(git describe --tags --always --dirty 2>/dev/null || echo "unknown")
else
    VERSION="unknown"
fi

# 获取时间戳
TIMESTAMP=$(date +%Y%m%d_%H%M%S)

# 包名
PACKAGE_NAME="iiri-ros-${BUILD_ARCH}-${VERSION}"
PACKAGE_DIR="${OUTPUT_DIR}/${PACKAGE_NAME}"

print_header "IIRI ROS2 Deployment Package Builder"
print_info "Architecture: ${BUILD_ARCH}"
print_info "Version: ${VERSION}"
print_info "Package name: ${PACKAGE_NAME}"
print_info "Output directory: ${OUTPUT_DIR}"
if [ "$SKIP_CHECKSUM" = true ]; then
    print_info "SHA256 checksum: Disabled (--no-checksum)"
else
    print_info "SHA256 checksum: Enabled"
fi

# 检查 iiri-ros 目录是否存在
if [ ! -d "iiri-ros" ]; then
    print_error "iiri-ros directory not found!"
    print_info "Please ensure you are running this script from the workspace root."
    exit 1
fi

# 检查 install 目录是否存在
if [ ! -d "iiri-ros/install" ]; then
    print_error "iiri-ros/install directory not found!"
    print_info "Please build the project first:"
    print_info "  ./build_layered.sh application_layer"
    exit 1
fi

# 创建输出目录
print_info "Creating package directory..."
rm -rf "$PACKAGE_DIR"
mkdir -p "$PACKAGE_DIR"

# 复制 install 目录
print_info "Copying install directory..."

# 优先使用 rsync（更高效），如果不可用则回退到 cp
if command -v rsync >/dev/null 2>&1; then
    print_info "Using rsync for efficient copying..."

    set +e  # 临时禁用立即退出，允许手动处理错误
    # 使用 -L 参数解引用符号链接，复制实际文件内容
    rsync -aL --exclude='*.pyc' --exclude='__pycache__' \
        iiri-ros/install/ "$PACKAGE_DIR/install/"

    rsync_exit_code=$?
    if [ $rsync_exit_code -ne 0 ] && [ $rsync_exit_code -ne 23 ]; then
        print_error "Failed to copy install directory with rsync (exit code: $rsync_exit_code)"
        exit 1
    elif [ $rsync_exit_code -eq 23 ]; then
        print_warning "Some file attributes couldn't be preserved (rsync code 23), but files were copied successfully"
    fi
    set -e  # 恢复立即退出模式
else
    print_warning "rsync not found, falling back to cp (slower)"
    mkdir -p "$PACKAGE_DIR/install"
    # 使用 -L 参数解引用符号链接，复制实际文件内容
    cp -rL iiri-ros/install/* "$PACKAGE_DIR/install/"

    cp_exit_code=$?
    if [ $cp_exit_code -ne 0 ]; then
        print_error "Failed to copy install directory with cp (exit code: $cp_exit_code)"
        exit 1
    fi

    # 手动清理 Python 缓存文件
    find "$PACKAGE_DIR/install" -type d -name '__pycache__' -exec rm -rf {} + 2>/dev/null || true
    find "$PACKAGE_DIR/install" -type f -name '*.pyc' -exec rm -f {} + 2>/dev/null || true
fi

# 复制启动脚本
print_info "Copying deployment scripts..."
cp iiri-ros/*.sh "$PACKAGE_DIR/" 2>/dev/null || true
cp iiri-ros/*.service "$PACKAGE_DIR/" 2>/dev/null || true
cp iiri-ros/iiri_env.sh "$PACKAGE_DIR/" 2>/dev/null || true
cp iiri-ros/README.md "$PACKAGE_DIR/" 2>/dev/null || true

# 生成 setup.bash
print_info "Generating setup.bash..."
cat > "$PACKAGE_DIR/setup.bash" << 'EOF'
#!/bin/bash
# IIRI ROS2 Environment Setup Script
# This script sets up the ROS2 environment for the deployed system

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Source ROS2 base environment
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
else
    echo "ERROR: ROS2 Humble not found at /opt/ros/humble/"
    echo "Please install ROS2 Humble first."
    return 1
fi

# Source the workspace
if [ -f "$SCRIPT_DIR/install/setup.bash" ]; then
    source "$SCRIPT_DIR/install/setup.bash"
    echo "✅ IIRI ROS2 environment loaded successfully"
    echo "   Workspace: $SCRIPT_DIR"
else
    echo "ERROR: Workspace setup file not found"
    return 1
fi
EOF

chmod +x "$PACKAGE_DIR/setup.bash"

# 生成 install.sh（一键安装脚本）
print_info "Generating install.sh..."
cat > "$PACKAGE_DIR/install.sh" << 'EOFINSTALL'
#!/bin/bash
# IIRI ROS2 一键安装脚本

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
SYMLINK_PATH="$TARGET_DIR/iiri-ros"

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}IIRI ROS2 一键安装${NC}"
echo -e "${GREEN}========================================${NC}"
echo -e "${BLUE}版本:${NC} $VERSION_DIR"
echo ""

# 1. 确保在 autorun 目录中
if [[ "$SCRIPT_DIR" != "$TARGET_DIR"/* ]]; then
    echo -e "${RED}错误：请将部署包解压到 $TARGET_DIR 目录${NC}"
    echo ""
    echo -e "${YELLOW}正确步骤：${NC}"
    echo "  tar -xzf iiri-ros-*.tar.gz -C $TARGET_DIR"
    echo "  cd $TARGET_DIR/iiri-ros-*"
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
./deploy_systemd_services.sh deploy

echo ""
echo -e "${GREEN}==========================================${NC}"
echo -e "${GREEN}✅ 安装完成！${NC}"
echo -e "${GREEN}==========================================${NC}"
echo ""
echo -e "${BLUE}当前版本:${NC} $VERSION_DIR"
echo -e "${BLUE}部署路径:${NC} $SYMLINK_PATH"
echo ""
echo -e "${YELLOW}常用命令:${NC}"
echo "  查看状态: sudo systemctl status iiri-ros.service"
echo "  查看日志: sudo journalctl -u iiri-ros.service -f"
echo ""
echo -e "${YELLOW}版本管理:${NC}"
echo "  列出版本: sudo /home/wl/autorun/switch-version.sh list"
echo "  切换版本: sudo /home/wl/autorun/switch-version.sh <版本号>"
echo "  回滚版本: sudo /home/wl/autorun/switch-version.sh rollback"
echo ""
EOFINSTALL
chmod +x "$PACKAGE_DIR/install.sh"

# 生成 switch-version.sh（版本切换工具）
print_info "Generating switch-version.sh..."
cat > "$PACKAGE_DIR/switch-version.sh" << 'EOFSWITCH'
#!/bin/bash
# IIRI ROS2 版本切换工具

set -e

AUTORUN_DIR="/home/wl/autorun"
SYMLINK_PATH="$AUTORUN_DIR/iiri-ros"

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

show_usage() {
    echo -e "${GREEN}IIRI ROS2 版本切换工具${NC}"
    echo ""
    echo "用法:"
    echo "  $0 list              列出所有已安装版本"
    echo "  $0 <版本号>          切换到指定版本"
    echo "  $0 rollback          回滚到上一个版本"
    echo "  $0 current           显示当前版本"
    echo ""
    echo "示例:"
    echo "  $0 list                        # 查看所有版本"
    echo "  $0 iiri-ros-x86-v1.2.3         # 切换到 v1.2.3"
    echo "  $0 v1.2.3                      # 简写版本号"
    echo "  $0 rollback                    # 回滚到上一版本"
}

# 获取所有版本
list_versions() {
    echo -e "${BLUE}已安装的版本:${NC}"
    echo ""

    local current_version=""
    if [ -L "$SYMLINK_PATH" ]; then
        current_version=$(basename "$(readlink -f "$SYMLINK_PATH")")
    fi

    local versions=$(ls -dt "$AUTORUN_DIR"/iiri-ros-* 2>/dev/null | grep -v "\.backup$" || true)

    if [ -z "$versions" ]; then
        echo "  (无已安装版本)"
        return
    fi

    local idx=1
    for version_path in $versions; do
        local version_name=$(basename "$version_path")
        local version_file="$version_path/VERSION.txt"
        local version_info=""

        if [ -f "$version_file" ]; then
            version_info=$(grep "^Version:" "$version_file" | cut -d: -f2 | xargs)
        fi

        if [ "$version_name" == "$current_version" ]; then
            echo -e "  ${GREEN}[$idx] $version_name  ← 当前版本${NC}"
        else
            echo "  [$idx] $version_name"
        fi

        if [ -n "$version_info" ]; then
            echo "       版本: $version_info"
        fi

        idx=$((idx + 1))
    done
}

# 显示当前版本
show_current() {
    if [ ! -L "$SYMLINK_PATH" ]; then
        echo -e "${YELLOW}未找到当前版本（符号链接不存在）${NC}"
        return 1
    fi

    local current=$(readlink -f "$SYMLINK_PATH")
    local current_name=$(basename "$current")

    echo -e "${GREEN}当前版本:${NC} $current_name"
    echo -e "${BLUE}路径:${NC} $current"

    if [ -f "$current/VERSION.txt" ]; then
        echo ""
        cat "$current/VERSION.txt" | head -12
    fi
}

# 切换版本
switch_version() {
    local target="$1"

    # 智能匹配版本号
    local target_path=""

    # 1. 尝试完整路径
    if [ -d "$AUTORUN_DIR/$target" ]; then
        target_path="$AUTORUN_DIR/$target"
    # 2. 尝试添加前缀匹配
    elif [ -d "$AUTORUN_DIR/iiri-ros-x86-$target" ]; then
        target_path="$AUTORUN_DIR/iiri-ros-x86-$target"
    elif [ -d "$AUTORUN_DIR/iiri-ros-arm-$target" ]; then
        target_path="$AUTORUN_DIR/iiri-ros-arm-$target"
    # 3. 模糊匹配
    else
        local matches=$(ls -d "$AUTORUN_DIR"/iiri-ros-*"$target"* 2>/dev/null || true)
        local match_count=$(echo "$matches" | grep -c . || echo "0")

        if [ "$match_count" -eq 1 ]; then
            target_path="$matches"
        elif [ "$match_count" -gt 1 ]; then
            echo -e "${RED}错误：找到多个匹配版本${NC}"
            echo ""
            echo "$matches"
            echo ""
            echo "请指定完整版本号"
            return 1
        else
            echo -e "${RED}错误：未找到版本 '$target'${NC}"
            echo ""
            echo "使用 '$0 list' 查看可用版本"
            return 1
        fi
    fi

    # 停止服务
    echo -e "${BLUE}>>> 停止服务...${NC}"
    systemctl stop iiri-ros.service 2>/dev/null || true

    # 更新符号链接
    echo -e "${BLUE}>>> 切换到: $(basename "$target_path")${NC}"
    ln -snf "$target_path" "$SYMLINK_PATH"

    # 启动服务
    echo -e "${BLUE}>>> 启动服务...${NC}"
    systemctl start iiri-ros.service

    # 等待服务启动
    sleep 2

    # 检查状态
    if systemctl is-active --quiet iiri-ros.service; then
        echo ""
        echo -e "${GREEN}==========================================${NC}"
        echo -e "${GREEN}✅ 版本切换成功！${NC}"
        echo -e "${GREEN}==========================================${NC}"
        echo ""
        show_current
    else
        echo ""
        echo -e "${RED}==========================================${NC}"
        echo -e "${RED}⚠️  服务启动失败${NC}"
        echo -e "${RED}==========================================${NC}"
        echo ""
        echo "请检查日志: sudo journalctl -u iiri-ros.service -n 50"
        return 1
    fi
}

# 回滚到上一版本
rollback() {
    if [ ! -L "$SYMLINK_PATH" ]; then
        echo -e "${RED}错误：找不到当前版本${NC}"
        return 1
    fi

    local current=$(readlink -f "$SYMLINK_PATH")
    local versions=$(ls -dt "$AUTORUN_DIR"/iiri-ros-* 2>/dev/null | grep -v "\.backup$" || true)

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
    switch_version "$(basename "$previous")"
}

# 主函数
main() {
    # 检查 root 权限
    if [ "$EUID" -ne 0 ]; then
        echo -e "${RED}错误：需要 root 权限${NC}"
        echo "请使用: sudo $0 $@"
        exit 1
    fi

    case "${1:-}" in
        list)
            list_versions
            echo ""
            show_current
            ;;
        current)
            show_current
            ;;
        rollback)
            rollback
            ;;
        help|--help|-h|"")
            show_usage
            ;;
        *)
            switch_version "$1"
            ;;
    esac
}

main "$@"
EOFSWITCH
chmod +x "$PACKAGE_DIR/switch-version.sh"

# 生成版本信息文件
print_info "Generating version info..."
cat > "$PACKAGE_DIR/VERSION.txt" << EOF
IIRI ROS2 Deployment Package
========================================
Architecture: ${BUILD_ARCH}
Version: ${VERSION}
Build Date: $(date '+%Y-%m-%d %H:%M:%S %Z')
Git Branch: $(git branch --show-current 2>/dev/null || echo "unknown")
Git Commit: $(git rev-parse HEAD 2>/dev/null || echo "unknown")
Build Host: $(hostname)
========================================

Package Contents:
- install/          ROS2 workspace install directory
- *.sh              Deployment and startup scripts
- *.service         Systemd service files
- setup.bash        Environment setup script
- VERSION.txt       This file

Deployment Instructions (一键安装):
1. Extract and install:
   tar -xzf ${PACKAGE_NAME}.tar.gz -C /home/wl/autorun/
   cd /home/wl/autorun/${PACKAGE_NAME}
   sudo ./install.sh

Version Management:
- List versions:    sudo /home/wl/autorun/switch-version.sh list
- Switch version:   sudo /home/wl/autorun/switch-version.sh <version>
- Rollback:         sudo /home/wl/autorun/switch-version.sh rollback

For more information, see DEPLOY.md
EOF

# 生成 README
print_info "Generating deployment README..."
cat > "$PACKAGE_DIR/DEPLOY.md" << 'EOF'
# IIRI ROS2 部署包使用说明

## 📦 包内容

此部署包包含完整的 IIRI ROS2 系统运行环境：

- **install/** - ROS2 编译产物目录
- **install.sh** - 一键安装脚本 ⭐ 新增
- **switch-version.sh** - 版本切换工具 ⭐ 新增
- **启动脚本** - start_ros2_iiri_start.sh, iiri_start.sh 等
- **systemd 服务** - iiri-qr.service, iiri-ros.service
- **部署脚本** - deploy_systemd_services.sh
- **环境配置** - setup.bash, iiri_env.sh

## 🚀 极简部署（一条命令）

### Systemd 服务部署（推荐）

```bash
# 解压并进入目录
tar -xzf iiri-ros-*.tar.gz -C /home/wl/autorun/ && cd /home/wl/autorun/iiri-ros-*

# 一键安装（自动创建符号链接并部署服务）
sudo ./install.sh
```

**就这么简单！✅** install.sh 会自动完成：
1. ✅ 验证部署位置
2. ✅ 创建符号链接到 /home/wl/autorun/iiri-ros
3. ✅ 安装并启动 systemd 服务
4. ✅ 显示服务状态和管理命令

### 手动启动（开发调试）

```bash
# 1. 加载环境
source setup.bash

# 2. 启动系统
ros2 launch system_bringup qr_raspi.launch.py
# 或根据平台选择：
# - qr_raspi.launch.py (树莓派)
# - qr_orin.launch.py (Orin)
# - qr_arm.launch.py (通用ARM)
# - qr_debug.launch.py (调试模式)
```

## 🔄 版本管理（一条命令）

### 列出所有已安装版本

```bash
sudo /home/wl/autorun/switch-version.sh list
```

输出示例：
```
可用版本:
  [当前] iiri-ros-x86-v1.2.3
         iiri-ros-x86-v1.2.2
         iiri-ros-x86-v1.2.1
```

### 切换到指定版本

```bash
# 使用完整版本号
sudo /home/wl/autorun/switch-version.sh v1.2.2

# 使用简短版本号
sudo /home/wl/autorun/switch-version.sh 1.2.2

# 使用 Git commit hash
sudo /home/wl/autorun/switch-version.sh c774b0c
```

**自动完成**：
1. ✅ 停止当前服务
2. ✅ 切换符号链接
3. ✅ 启动新版本服务
4. ✅ 验证服务状态

### 回滚到上一个版本

```bash
sudo /home/wl/autorun/switch-version.sh rollback
```

### 查看当前版本

```bash
sudo /home/wl/autorun/switch-version.sh current
```

## 📋 前置要求

- Ubuntu 22.04
- ROS2 Humble 已安装
- Python 3.10+
- 必要的系统依赖

## 🔧 常用命令

### 服务管理

```bash
# 启动服务
sudo systemctl start iiri-ros.service

# 停止服务
sudo systemctl stop iiri-ros.service

# 重启服务
sudo systemctl restart iiri-ros.service

# 查看状态
sudo systemctl status iiri-ros.service
```

### 日志查看

```bash
# 实时查看服务日志
sudo journalctl -u iiri-ros.service -f

# 查看启动脚本日志
cat /tmp/iiri_ros_startup_debug.log

# 查看历史日志
sudo journalctl -u iiri-ros.service -n 100
```

## 🗂️ 多版本管理

### 目录结构

```bash
/home/wl/autorun/
├── iiri-ros -> iiri-ros-x86-v1.2.3  # 符号链接（当前激活版本）
├── switch-version.sh                # 版本切换工具
├── iiri-ros-x86-v1.2.3/             # 最新版本
├── iiri-ros-x86-v1.2.2/             # 上一个版本
└── iiri-ros-x86-v1.2.1/             # 更早版本
```

### 清理旧版本

```bash
# 保留最近3个版本，删除更旧的
cd /home/wl/autorun
ls -dt iiri-ros-x86-* | tail -n +4 | xargs rm -rf

# 或手动删除指定版本
rm -rf /home/wl/autorun/iiri-ros-x86-v1.2.1
```

## 💡 技术说明

### 符号链接机制

- systemd 服务使用固定路径 `/home/wl/autorun/iiri-ros`
- 实际部署包含版本号（如 `iiri-ros-x86-v1.2.3`）
- 符号链接桥接固定路径和版本路径
- 支持多版本并存和快速切换

### 版本识别

switch-version.sh 支持智能匹配：
- **完整名称**: `iiri-ros-x86-v1.2.3`
- **版本号**: `v1.2.3` 或 `1.2.3`
- **Git hash**: `c774b0c` 或 `c774b0c-dirty`
- **模糊匹配**: 自动查找包含关键字的版本

## 📚 更多信息

详细文档请参阅 README.md

---

**作者**: 唐文浩
**更新日期**: 2025-10-11
EOF

# 设置正确的权限
print_info "Setting permissions..."
chmod +x "$PACKAGE_DIR"/*.sh 2>/dev/null || true

# 打包
print_info "Creating tar.gz archive..."
cd "$OUTPUT_DIR"
tar -czf "${PACKAGE_NAME}.tar.gz" "${PACKAGE_NAME}"

if [ $? -eq 0 ]; then
    PACKAGE_SIZE=$(du -h "${PACKAGE_NAME}.tar.gz" | cut -f1)
    print_success "Package created successfully!"
    print_info "Package: ${OUTPUT_DIR}/${PACKAGE_NAME}.tar.gz"
    print_info "Size: ${PACKAGE_SIZE}"
else
    print_error "Failed to create tar.gz archive"
    exit 1
fi

# 生成 SHA256 校验文件（可选）
if [ "$SKIP_CHECKSUM" = false ]; then
    print_info "Generating SHA256 checksum..."
    sha256sum "${PACKAGE_NAME}.tar.gz" > "${PACKAGE_NAME}.tar.gz.sha256"
    print_success "Checksum: ${PACKAGE_NAME}.tar.gz.sha256"
else
    print_info "Skipped SHA256 checksum generation (--no-checksum)"
fi

cd - > /dev/null

print_header "Package Build Complete"
echo ""
print_success "Deployment package ready:"
echo "  📦 Package: ${OUTPUT_DIR}/${PACKAGE_NAME}.tar.gz"
if [ "$SKIP_CHECKSUM" = false ]; then
    echo "  🔐 Checksum: ${OUTPUT_DIR}/${PACKAGE_NAME}.tar.gz.sha256"
fi
echo "  📏 Size: ${PACKAGE_SIZE}"
echo ""
print_info "To deploy this package:"
echo "  1. Copy to target system"
echo "  2. Extract: tar -xzf ${PACKAGE_NAME}.tar.gz -C /home/wl/autorun/"
echo "  3. Create symlink: ln -snf /home/wl/autorun/${PACKAGE_NAME} /home/wl/autorun/iiri-ros"
echo "  4. Deploy: cd /home/wl/autorun/iiri-ros && sudo ./deploy_systemd_services.sh deploy"
echo ""
