# P7 ROS2 info

The following guide describes how to build the ROS2 workspace and launch the Livox LiDAR driver and FAST-LIO.

## Prerequisites

Make sure that the required Git submodules are initialized and working correctly before building the workspace.

## Building the ROS2 Workspace

### 1. Build the ROS2 Workspace

Run the build script from the root of the ROS2 workspace:

```bash
./build_packages
```

### 2. Source the Workspace

After building, source the workspace:

```bash
source install/setup.bash
```

## Launching the Livox ROS Driver

To launch the Livox MID360s driver, run:

```bash
ros2 launch livox_ros_driver2 msg_MID360s_launch.py
```

## Launching FAST-LIO

To launch FAST-LIO, run:

```bash
ros2 launch fast_lio mapping.launch.py
```

## Optional TF Transform for RViz

When using FAST-LIO with RViz, a static transform can optionally be added to correctly display the point cloud with the desired orientation.

### FAST-LIO Transform

Run:

```bash
ros2 run tf2_ros static_transform_publisher \
    0 0 0 \
    0 3.14159 0 \
    up_frame camera_init
```

### RGB-D Camera Transform

If the RGB-D camera is being displayed in RViz, the following transform can optionally be used:

```bash
ros2 run tf2_ros static_transform_publisher \
    0 0 0 \
    0 3.14159 0 \
    up_frame rgbd_camera_color_optical_frame
```

These transforms are optional and are only required when needed for the correct frame orientation when visualizing the system in RViz.
