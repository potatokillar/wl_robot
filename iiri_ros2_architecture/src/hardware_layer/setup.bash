#!/bin/bash
# Hardware Layer Setup Script

# Source ROS2 base installation
source /opt/ros/humble/setup.bash

# Determine architecture - try both host arch and common build archs
ARCH=$(uname -m)

# Always try both architectures to handle cross-compilation scenarios
if [ $ARCH == "x86_64" ]; then
    BUILD_SUFFIXES=("x86" "arm")  # x86 host: try x86 first, then arm
elif [ $ARCH == "aarch64" ]; then
    BUILD_SUFFIXES=("arm" "x86")  # ARM host: try arm first, then x86
else
    BUILD_SUFFIXES=("arm" "x86")  # Default fallback
fi

# Source shared install directory (contains all layers up to hardware_layer)
SOURCED=false
for BUILD_SUFFIX in "${BUILD_SUFFIXES[@]}"; do
    SHARED_SETUP_PATH="../../build_${BUILD_SUFFIX}_shared/install/setup.bash"
    if [ -f "$SHARED_SETUP_PATH" ]; then
        source "$SHARED_SETUP_PATH"
        SOURCED=true
        echo "Using shared install: build_${BUILD_SUFFIX}_shared"
        break
    fi
done

if [ "$SOURCED" = false ]; then
    echo "WARNING: No shared install setup found for any architecture"
fi

echo "Hardware Layer environment sourced (using shared install)"