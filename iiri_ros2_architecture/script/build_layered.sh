#!/bin/bash

ARCH=$(uname -m)

# 显示版本管理信息
echo "=== ROS2 Layered Architecture Build ==="

# 留核用于其他任务
# 在 CI 环境中使用更少的并行数，避免资源耗尽
if [ -n "$CI" ]; then
    get_ncpu=1  # CI 环境中只使用1个串行worker，避免内存耗尽
    echo "CI 环境：使用串行编译（1个worker）以节省内存"
else
    get_ncpu=$(expr $(nproc) / 2)  # 本地环境使用CPU核心数的一半
fi

# 根据架构设置build目录
# 使用传入的 TARGET_ARCH 环境变量（如果可用），否则从 uname 检测
if [ -n "$TARGET_ARCH" ]; then
    echo "使用指定的目标架构: $TARGET_ARCH"
    if [ "$TARGET_ARCH" == "x86" ]; then
        BUILD_BASE="build_x86"
        # x86 始终使用 Debug
        CMAKE_BUILD_TYPE="Debug"
    elif [ "$TARGET_ARCH" == "arm" ]; then
        BUILD_BASE="build_arm"
        # ARM 在 CI 环境下使用 Debug（降低 QEMU 负载），本地使用 RelWithDebInfo（优化+调试符号）
        if [ -n "$CI" ]; then
            CMAKE_BUILD_TYPE="Debug"
            echo "CI ARM 构建：使用 Debug 模式降低 QEMU 负载"
        else
            CMAKE_BUILD_TYPE="RelWithDebInfo"
            echo "ARM 构建：使用 RelWithDebInfo 模式（优化+调试符号，支持 coredump 分析）"
        fi
    fi
else
    # 兼容旧行为：从容器内架构推断
    if [ $ARCH == "x86_64" ]; then
        BUILD_BASE="build_x86"
        CMAKE_BUILD_TYPE="Debug"
    elif [ $ARCH == "aarch64" ]; then
        BUILD_BASE="build_arm"
        CMAKE_BUILD_TYPE="RelWithDebInfo"
        echo "ARM (aarch64) 构建：使用 RelWithDebInfo 模式（优化+调试符号）"
    fi
fi

# 定义编译层次和顺序 - 修正依赖关系
ALL_LAYERS=("core_layer" "hardware_layer" "perception_layer" "intelligence_layer" "application_layer")

# 处理命令行参数
CLEAN_BUILD=false
TARGET_LAYER=""
USE_CERES="OFF"
LAYERS=()

while [[ $# -gt 0 ]]; do
    case $1 in
        -c)
            CLEAN_BUILD=true
            shift
            ;;
        --ceres)
            USE_CERES="ON"
            shift
            ;;
        hardware_layer|core_layer|perception_layer|intelligence_layer|application_layer)
            TARGET_LAYER="$1"
            shift
            ;;
        help|--help|-h)
            echo "Usage: $0 [-c] [--ceres] [layer_name]"
            echo "  -c: Clean build cache"
            echo "  --ceres: Enable Ceres optimization (default: OFF)"
            echo "  layer_name: One of hardware_layer, core_layer, perception_layer, intelligence_layer, application_layer"
            exit 0
            ;;
        *)
            echo "Unknown argument: $1"
            echo "Usage: $0 [-c] [--ceres] [layer_name]"
            echo "  -c: Clean build cache"
            echo "  --ceres: Enable Ceres optimization (default: OFF)"
            echo "  layer_name: One of hardware_layer, core_layer, perception_layer, intelligence_layer, application_layer"
            exit 1
            ;;
    esac
done

# 确定要编译的层
if [ -n "$TARGET_LAYER" ]; then
    echo "Building single layer: $TARGET_LAYER"
    # 找到目标层及其所有依赖层
    for i in "${!ALL_LAYERS[@]}"; do
        LAYERS+=("${ALL_LAYERS[$i]}")
        if [ "${ALL_LAYERS[$i]}" == "$TARGET_LAYER" ]; then
            break
        fi
    done
else
    echo "Building all layers..."
    LAYERS=("${ALL_LAYERS[@]}")
fi

# 处理清理参数
if [ "$CLEAN_BUILD" = true ]; then
    if [ -n "$TARGET_LAYER" ]; then
        echo "Cleaning build cache for $TARGET_LAYER..."
        rm -rf ${BUILD_BASE}_${TARGET_LAYER}
    else
        echo "Cleaning build cache for all layers..."
        rm -rf ${BUILD_BASE}_*
    fi
fi

# 分层编译
for layer in "${LAYERS[@]}"; do
    echo ""
    echo "=== Building $layer ==="

    cd "src/$layer" || {
        echo "ERROR: Cannot find directory src/$layer"
        exit 1
    }

    # 设置 overlay 环境
    if [ "$layer" == "core_layer" ]; then
        # core layer 只需要 ROS2 基础环境
        echo "Setting up ROS2 base environment for $layer..."
        source /opt/ros/humble/setup.bash
    else
        # 其他层需要 overlay 环境
        echo "Setting up overlay environment for $layer..."
        source setup.bash
    fi

    # 构建当前层 - 使用完整共享架构
    # 根据层次决定是否需要 CERES 优化参数
    CMAKE_ARGS="-DCMAKE_BUILD_TYPE=$CMAKE_BUILD_TYPE"
    if [ "$layer" == "intelligence_layer" ]; then
        CMAKE_ARGS="$CMAKE_ARGS -DUSE_CERES_OPTIMIZATION=$USE_CERES"
    fi

    # CI ARM 构建：添加特殊优化以降低 QEMU 负载
    if [ -n "$CI" ] && [ "$TARGET_ARCH" == "arm" ]; then
        echo "CI ARM 构建：启用 QEMU 优化参数"
        # 禁用并行链接（一次只链接一个目标）
        CMAKE_ARGS="$CMAKE_ARGS -DCMAKE_BUILD_PARALLEL_LEVEL=1"
        # 减少链接器内存使用
        CMAKE_ARGS="$CMAKE_ARGS -DCMAKE_EXE_LINKER_FLAGS=-Wl,--no-keep-memory"
        CMAKE_ARGS="$CMAKE_ARGS -DCMAKE_SHARED_LINKER_FLAGS=-Wl,--no-keep-memory"
    fi

    # 执行 colcon build
    colcon --log-base ../../${BUILD_BASE}_shared/${layer}/log \
        build --symlink-install \
        --build-base ../../${BUILD_BASE}_shared/${layer}/build \
        --install-base ../../${BUILD_BASE}_shared/install \
        --cmake-args $CMAKE_ARGS \
        --parallel-workers "$get_ncpu"

    # 检查编译结果
    if [ $? -eq 0 ]; then
        echo "$layer build successful!"
    else
        echo "ERROR: $layer build failed!"
        cd ../..
        exit 1
    fi

    cd ../..
done

echo ""
echo "=== All layers built successfully! ==="

# 自动同步到 iiri-ros/install（用于部署）
if [ -d "${BUILD_BASE}_shared/install" ]; then
    echo ""
    echo "=== Syncing to iiri-ros/install for deployment ==="

    DEPLOY_INSTALL_DIR="iiri-ros/install"

    # 创建 iiri-ros 目录（如果不存在）
    mkdir -p "iiri-ros"

    # 使用 rsync 同步，保留权限和符号链接
    if command -v rsync >/dev/null 2>&1; then
        rsync -a --delete \
            ${BUILD_BASE}_shared/install/ \
            "$DEPLOY_INSTALL_DIR/"

        if [ $? -eq 0 ]; then
            echo "✅ Successfully synced install to $DEPLOY_INSTALL_DIR"
            echo "   You can now deploy using the iiri-ros/ directory"
        else
            echo "⚠️  Warning: Failed to sync to $DEPLOY_INSTALL_DIR"
        fi
    else
        # 如果没有 rsync，使用 cp 作为后备
        cp -af ${BUILD_BASE}_shared/install/* "$DEPLOY_INSTALL_DIR/" 2>/dev/null
        echo "✅ Synced install to $DEPLOY_INSTALL_DIR (using cp)"
    fi
fi

echo ""
echo "To use the built system, run:"
echo "cd src/application_layer && source setup.bash"