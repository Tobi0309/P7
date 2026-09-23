#!/bin/bash

set -e

source /opt/ros/jazzy/setup.sh

echo "Building Livox ROS driver 2"
cd src/livox_ros_driver2
./build.sh jazzy
