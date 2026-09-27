#!/bin/bash

set -e

source /opt/ros/jazzy/setup.bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=== Building Livox ROS Driver 2 and workspace ==="
cd "$SCRIPT_DIR/src/livox_ros_driver2"
./build.sh jazzy

echo "=== Build complete ==="
