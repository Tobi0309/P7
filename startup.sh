#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

SESSION_NAME="sensors"

echo "Stopping existing sensor session..."

# Kill our existing screen session if it exists
screen -S "$SESSION_NAME" -X quit 2>/dev/null || true

echo "Stopping existing RealSense processes..."

# RealSense node
pkill -TERM -f "realsense2_camera_node" 2>/dev/null || true

# image_transport republish processes
pkill -TERM -f "image_transport.*republish" 2>/dev/null || true


# Give processes a moment to terminate
sleep 2


echo "Starting sensors..."

screen -dmS "$SESSION_NAME" bash -c "
    source /opt/ros/jazzy/setup.bash

    cd '$SCRIPT_DIR'

    echo 'Starting sensors.launch.py...'

    ros2 launch sensors.launch.py

    echo
    echo 'sensors.launch.py exited.'
    exec bash
"


echo "Sensors started in screen session: $SESSION_NAME"
echo
echo "Attach with:"
echo "    screen -r $SESSION_NAME"