#!/bin/bash
# Application Layer Setup Script

# Source intelligence layer (which includes all lower layers)
source ../intelligence_layer/setup.bash

# Determine architecture
ARCH=$(uname -m)
if [ $ARCH == "x86_64" ]; then
    BUILD_SUFFIX="x86"
elif [ $ARCH == "aarch64" ]; then
    BUILD_SUFFIX="arm"
fi

# Source this layer's install
if [ -f "../../build_${BUILD_SUFFIX}_application_layer/install/setup.bash" ]; then
    source ../../build_${BUILD_SUFFIX}_application_layer/install/setup.bash
fi

echo "Application Layer environment sourced"