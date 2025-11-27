#!/bin/bash
# Intelligence Layer Setup Script

# Source perception layer (which includes all lower layers)
source ../perception_layer/setup.bash

# Determine architecture
ARCH=$(uname -m)
if [ $ARCH == "x86_64" ]; then
    BUILD_SUFFIX="x86"
elif [ $ARCH == "aarch64" ]; then
    BUILD_SUFFIX="arm"
fi

# Source this layer's install
if [ -f "../../build_${BUILD_SUFFIX}_intelligence_layer/install/setup.bash" ]; then
    source ../../build_${BUILD_SUFFIX}_intelligence_layer/install/setup.bash
fi

echo "Intelligence Layer environment sourced"