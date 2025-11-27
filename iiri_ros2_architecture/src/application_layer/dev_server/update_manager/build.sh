#!/bin/bash
# Build update_manager locally (for testing)

set -e

cd "$(dirname "$0")"

echo "=== Building update_manager ==="

mkdir -p build
cd build

cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

echo ""
echo "✓ Build completed successfully"
ls -lh update_manager
