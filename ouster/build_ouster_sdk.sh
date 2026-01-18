#!/bin/bash
# Build script for Ouster SDK with all required dependencies

set -e  # Exit on error

cd "$(dirname "$0")"

# Activate virtual environment
source venv/bin/activate

# Set Eigen3 path
export CMAKE_PREFIX_PATH="$(pwd)/local_deps/eigen3:$CMAKE_PREFIX_PATH"

# Set C++17 standard (required for robin-map compatibility)
# Note: We've patched CMakeLists.txt files to default to C++17, but keeping this as backup
export OUSTER_SDK_CMAKE_ARGS="-DCMAKE_CXX_STANDARD=17 -DCMAKE_CXX_STANDARD_REQUIRED=OFF"

echo "Building Ouster SDK with:"
echo "  - CMAKE_PREFIX_PATH: $CMAKE_PREFIX_PATH"
echo "  - OUSTER_SDK_CMAKE_ARGS: $OUSTER_SDK_CMAKE_ARGS"
echo ""
echo "Clearing any existing CMake cache..."
# Clear pip build directories
rm -rf /tmp/pip-install-*/ouster-sdk_* 2>/dev/null || true
rm -rf /tmp/pip-build-*/ouster-sdk_* 2>/dev/null || true
# Also clear any local build directories
find /private/var/folders -name "*ouster-sdk*" -type d -maxdepth 3 2>/dev/null | xargs rm -rf 2>/dev/null || true
# Clear the local SDK build directory (where setup.py puts CMakeCache.txt)
rm -rf ouster-sdk/build 2>/dev/null || true
echo "Cache cleared."
echo ""

# Build from local source (so our C++17 patches are used)
echo "Starting build from local source (this may take 10-20 minutes)..."
cd ouster-sdk/python
pip install --no-cache-dir -e .

echo ""
echo "Build completed successfully!"
