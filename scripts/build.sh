#!/bin/bash
# RoboRacer WS - Build Script
source /opt/ros/jazzy/setup.bash
cd "$(dirname "$0")/.."
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
echo ""
echo "✅ Build complete! Source with: source install/setup.bash"
