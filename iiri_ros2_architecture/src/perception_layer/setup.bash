#!/bin/bash
# Perception Layer Setup Script

# Source hardware layer (which includes core layer)
source ../hardware_layer/setup.bash

# Determine architecture
ARCH=$(uname -m)
if [ $ARCH == "x86_64" ]; then
    BUILD_SUFFIX="x86"
elif [ $ARCH == "aarch64" ]; then
    BUILD_SUFFIX="arm"
fi

# Source this layer's install
if [ -f "../../build_${BUILD_SUFFIX}_perception_layer/install/setup.bash" ]; then
    source ../../build_${BUILD_SUFFIX}_perception_layer/install/setup.bash
fi

echo "Perception Layer environment sourced"