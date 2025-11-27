#!/bin/bash
# Core Layer Setup Script

# Source ROS2 base installation only
source /opt/ros/humble/setup.bash

# Determine architecture
ARCH=$(uname -m)
if [ $ARCH == "x86_64" ]; then
    BUILD_SUFFIX="x86"
elif [ $ARCH == "aarch64" ]; then
    BUILD_SUFFIX="arm"
fi

# Source this layer's install
if [ -f "../build_${BUILD_SUFFIX}_core_layer/install/setup.bash" ]; then
    source ../build_${BUILD_SUFFIX}_core_layer/install/setup.bash
fi

echo "Core Layer environment sourced"