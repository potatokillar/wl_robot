#!/bin/bash
#=============================================================================
# qr_wl 项目打包脚本（简化版）
# 功能：打包 iiri-qr 目录为部署包
# 用法：./deploy_package_qr.sh [x86|arm] [output_dir] [--skip-build] [--no-checksum]
# 作者：唐文浩
# 日期：2025-10-15
#=============================================================================

set -e  # 遇到错误立即退出

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 打印函数
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
    echo "qr_wl Deployment Package Builder"
    echo ""
    echo "Usage: $0 [architecture] [output_dir] [options]"
    echo ""
    echo "Arguments:"
    echo "  architecture  Target architecture: x86 or arm (default: arm)"
    echo "  output_dir    Output directory (default: deploy_packages)"
    echo ""
    echo "Options:"
    echo "  --skip-build   Skip building, use existing binaries in iiri-qr/"
    echo "  --no-checksum  Skip SHA256 checksum generation"
    echo "  -h, --help     Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0                          # Build and package ARM version"
    echo "  $0 x86                      # Build and package x86 version"
    echo "  $0 arm --skip-build         # Package ARM without rebuilding"
    echo "  $0 arm custom_dir           # Package to custom directory"
    echo ""
}

# 默认参数
ARCH="arm"
OUTPUT_DIR="deploy_packages"
SKIP_BUILD=false
SKIP_CHECKSUM=false

# 参数解析
while [[ $# -gt 0 ]]; do
    case $1 in
        --skip-build)
            SKIP_BUILD=true
            shift
            ;;
        --no-checksum)
            SKIP_CHECKSUM=true
            shift
            ;;
        -h|--help)
            show_usage
            exit 0
            ;;
        x86|arm)
            ARCH=$1
            shift
            ;;
        --*)
            print_error "Unknown option: $1"
            show_usage
            exit 1
            ;;
        *)
            if [ -z "$OUTPUT_DIR" ] || [ "$OUTPUT_DIR" == "deploy_packages" ]; then
                OUTPUT_DIR=$1
            fi
            shift
            ;;
    esac
done

# 获取版本信息
if git rev-parse --git-dir > /dev/null 2>&1; then
    VERSION=$(git describe --tags --always --dirty 2>/dev/null || echo "unknown")
    GIT_COMMIT=$(git rev-parse HEAD 2>/dev/null || echo "unknown")
    GIT_BRANCH=$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo "unknown")
else
    VERSION="unknown"
    GIT_COMMIT="unknown"
    GIT_BRANCH="unknown"
fi

# 获取时间戳
TIMESTAMP=$(date +%Y%m%d_%H%M%S)

# 包名
PACKAGE_NAME="iiri-qr-${ARCH}-${VERSION}"
PACKAGE_TAR="${PACKAGE_NAME}.tar.gz"

print_header "qr_wl Deployment Package Builder"
print_info "Architecture: ${ARCH}"
print_info "Version: ${VERSION}"
print_info "Package name: ${PACKAGE_NAME}"
print_info "Output directory: ${OUTPUT_DIR}"
if [ "$SKIP_BUILD" = true ]; then
    print_info "Build: Skipped (--skip-build)"
else
    print_info "Build: Enabled"
fi
if [ "$SKIP_CHECKSUM" = true ]; then
    print_info "SHA256 checksum: Disabled (--no-checksum)"
else
    print_info "SHA256 checksum: Enabled"
fi

# 步骤1：编译（可选）
if [ "$SKIP_BUILD" != "true" ]; then
    print_info "Building ${ARCH} architecture..."
    if [ -f "./build_docker.sh" ]; then
        ./build_docker.sh ${ARCH}
    elif [ -f "./build.sh" ]; then
        print_warning "Using build.sh script"
        ./build.sh ${ARCH}
    else
        print_error "No build script found!"
        print_info "Expected: ./build_docker.sh or ./build.sh"
        exit 1
    fi
else
    print_info "Skipping build step (--skip-build)"
fi

# 步骤2：检查 iiri-qr 目录
if [ ! -d "iiri-qr" ]; then
    print_error "iiri-qr directory not found!"
    print_info "Please ensure the iiri-qr deployment template directory exists"
    exit 1
fi

# 检查 qr 可执行文件
if [ ! -f "iiri-qr/qr" ]; then
    print_error "qr executable not found in iiri-qr/ directory!"
    print_info "Please build the project first:"
    print_info "  ./build.sh $ARCH"
    print_info "The build process should copy qr binary to iiri-qr/"
    exit 1
fi

# 验证 qr 可执行文件是否可执行
if [ ! -x "iiri-qr/qr" ]; then
    print_warning "qr binary is not executable, fixing permissions..."
    chmod +x iiri-qr/qr
fi

# 步骤3：生成版本信息文件
print_info "Generating version information..."
cat > "iiri-qr/VERSION.txt" << EOF
Project: qr_wl
Version: ${VERSION}
Architecture: ${ARCH}
Build Date: $(date '+%Y-%m-%d %H:%M:%S')
Build Type: $([ -f "build/${ARCH}64/.build_type" ] && cat "build/${ARCH}64/.build_type" || echo "Release")
Git Commit: ${GIT_COMMIT}
Git Branch: ${GIT_BRANCH}
Package: ${PACKAGE_NAME}
EOF

print_success "Version file created: iiri-qr/VERSION.txt"

# 步骤4：创建输出目录
print_info "Creating output directory..."
mkdir -p "${OUTPUT_DIR}"

# 步骤5：创建压缩包（直接打包 iiri-qr 目录，重命名为带版本号的名称）
print_info "Creating deployment package..."

# 使用临时符号链接以重命名目录
ln -snf iiri-qr "${PACKAGE_NAME}"
tar -czf "${OUTPUT_DIR}/${PACKAGE_TAR}" \
    --exclude='*.pyc' \
    --exclude='__pycache__' \
    --exclude='.git' \
    --exclude='.gitignore' \
    -h "${PACKAGE_NAME}"
rm "${PACKAGE_NAME}"

print_success "Package created: ${OUTPUT_DIR}/${PACKAGE_TAR}"

# 步骤6：生成校验和
if [ "$SKIP_CHECKSUM" != "true" ]; then
    print_info "Generating SHA256 checksum..."
    cd "${OUTPUT_DIR}"
    sha256sum "${PACKAGE_TAR}" > "${PACKAGE_TAR}.sha256"
    cd - > /dev/null

    # 显示校验和
    echo ""
    echo "SHA256 Checksum:"
    cat "${OUTPUT_DIR}/${PACKAGE_TAR}.sha256"
    print_success "Checksum file: ${OUTPUT_DIR}/${PACKAGE_TAR}.sha256"
fi

# 计算包大小
PACKAGE_SIZE=$(du -h "${OUTPUT_DIR}/${PACKAGE_TAR}" | cut -f1)

# 步骤7：显示包内容预览
print_info "Package contents preview:"
tar -tzf "${OUTPUT_DIR}/${PACKAGE_TAR}" | head -20
echo "  ... ($(tar -tzf "${OUTPUT_DIR}/${PACKAGE_TAR}" | wc -l) files total)"
echo ""

print_header "Package Created Successfully!"
echo ""
echo "Package Information:"
echo "  Name: ${PACKAGE_NAME}"
echo "  File: ${OUTPUT_DIR}/${PACKAGE_TAR}"
echo "  Size: ${PACKAGE_SIZE}"
echo "  Architecture: ${ARCH}"
echo "  Version: ${VERSION}"
echo "  Git Commit: ${GIT_COMMIT:0:8}"
echo ""
if [ "$SKIP_CHECKSUM" != "true" ]; then
    echo "SHA256:"
    echo "  $(cat ${OUTPUT_DIR}/${PACKAGE_TAR}.sha256)"
    echo ""
fi
print_info "Next steps:"
echo ""
echo "1️⃣  Transfer to target machine:"
echo "   scp ${OUTPUT_DIR}/${PACKAGE_TAR} wl@192.168.1.54:/tmp/"
echo ""
echo "2️⃣  Extract and install on target:"
echo "   tar -xzf /tmp/${PACKAGE_TAR} -C /home/wl/autorun/"
echo "   cd /home/wl/autorun/${PACKAGE_NAME}"
echo "   sudo ./install.sh"
echo ""
echo "3️⃣  Or download from Jenkins:"
echo "   http://192.168.1.93:8080/job/qr-wl-build-ci/lastSuccessfulBuild/artifact/deploy_packages/${PACKAGE_TAR}"
echo ""
