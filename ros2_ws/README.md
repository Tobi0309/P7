# P7 Ros2 info

# Prerequisites
Make sure that submodules is working

### Building the ROS2-Workspace
1. Run the build_packages file
./build_packages

2. Source
source install/setup.bash

### Launching Livox_ros_driver2
ros2 launch livox_ros_driver2 msg_MID360s_launch.py

### Launching fast_lio
ros2 launch fast_lio mapping.launch.py