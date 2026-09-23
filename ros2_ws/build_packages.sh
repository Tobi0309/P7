#!/bin/bash

set -e

source /opt/ros/jazzy/setup.sh

echo "Building Livox ROS driver 2"
cd src/livox_ros_driver2
./build.sh jazzy

echo "Building FAST LIO"
cd src#!/bin/bash

set -e

source /opt/ros/jazzy/setup.bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=== Building Livox ROS Driver 2 ==="
cd "$SCRIPT_DIR/src/livox_ros_driver2"
./build.sh jazzy

echo "=== Building FAST-LIO ==="
cd "$SCRIPT_DIR"
colcon build --symlink-install --packages-select fast_lio

echo "=== Build complete ==="
