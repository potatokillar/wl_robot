#!/bin/bash
# Build update_manager in Docker container

set -e

ARCH=${1:-arm}

if [ "$ARCH" = "arm" ]; then
    DOCKER_IMG="192.168.1.93/iiri/build_arm_ros2:latest"
elif [ "$ARCH" = "x86" ]; then
    DOCKER_IMG="192.168.1.93/iiri/build_x86_ros2:v1.4.3"
else
    echo "Error: Invalid architecture: $ARCH"
    echo "Usage: $0 [arm|x86]"
    exit 1
fi

WORKSPACE=$(pwd)

echo "=== Building update_manager in Docker ==="
echo "Architecture: $ARCH"
echo "Docker image: $DOCKER_IMG"
echo ""

docker run --rm \
    -v "$WORKSPACE:/workspace" \
    -w /workspace \
    "$DOCKER_IMG" \
    bash -c "
        mkdir -p build && cd build
        cmake .. -DCMAKE_BUILD_TYPE=Release
        make -j\$(nproc)
        echo ''
        echo '✓ Build completed'
        ls -lh update_manager
    "

# Fix permissions
sudo chown -R $USER:$USER build/

echo ""
echo "✓ Build artifacts ready at: build/update_manager"
