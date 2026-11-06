# frontier_segmentation (ROS 2 Jazzy)

C++ ROS 2 node that extracts 3D frontier voxels from `octomap_msgs/msg/Octomap`, clusters them with 26-neighbour connectivity, splits oversized/elongated/non-compact clusters, publishes a custom `FrontierArray`, and visualizes each segment as a differently coloured `CUBE_LIST` Marker in RViz2.

## Build

Set `octomap_topic` to the actual `octomap_msgs/msg/Octomap` topic from your mapper. The subscriber uses transient-local QoS; if your publisher is volatile, change the subscription QoS in the source to `rclcpp::QoS(1)`.

The node compares each cumulative/full map with the previously received map and updates only the affected frontier segments when voxel states change. It collects changed voxels and their 26-neighborhood, together with the voxels from affected segments, runs frontier detection and segmentation on that subset, and retains unaffected segments. The OctoMap frame must match `world_frame`; `sensor_frame`, `robot_frame`, and range parameters remain available for compatibility but are not used to decide which segments are updated.

### IntelliSense for C++ packages

The repository includes VS Code C/C++ settings that use the package's generated
compile database. Generate it from the ROS 2 workspace before opening the C++
source:

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select frontier_segmentation --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source install/setup.bash
```

VS Code will then use
`build/frontier_segmentation/compile_commands.json`, together with the ROS 2
and generated workspace headers, for IntelliSense. If a different ROS 2
distribution is installed, replace `jazzy` in the command and in
`.vscode/settings.json`.

## RViz2

Add a `MarkerArray` display for `/frontier_segments/markers`. Keep the OctoMap display enabled. Each segment has a deterministic colour; cubes are centered at frontier voxel centers.

Both output publishers use reliable, transient-local QoS, so RViz2 receives the latest frontier data when it subscribes after the node has already published it. The node expands pruned OctoMap leaves into native-resolution voxels before publishing, preventing oversized leaves from appearing as holes or off-grid cubes.

The latest marker array is also republished once per second as a compatibility fallback for RViz2 displays that do not replay transient-local `MarkerArray` data when toggled. A frontier component is discarded when every voxel has a known (free or occupied) voxel one to three cells below it; only unknown space counts as unsupported. `positive_z_is_up` controls which direction is considered below and defaults to `true`; set it to `false` when the positive Z axis points down.

Set `diagnostics: true` to log each approved segment's center and color, followed by every frontier voxel and its one-to-three-cell vertical checks.

## Sensor models

- `camera`: defaults to 90° × 64°, 0.4–6 m.
- `livox_mid360`: defaults to 360° azimuth, 59° elevation span, 0.2–40 m.
- `use_fov_extent_limits` optionally caps width/height thresholds using the FOV at `frontier_design_range_m`. It is off by default because FOV is an observability constraint, not a universal frontier-size limit.
- A 76° mounting pitch should be represented by the static TF from the robot/body frame to the LiDAR frame. Do not apply it a second time in segmentation. `lidar_axis_pitch_deg` is informational only; the actual mounting pitch must be present in the robot-to-sensor TF. The current segmentation does not ray-cast the sensor FOV.

## Algorithm notes

A frontier voxel is a known-free voxel with at least one unknown 26-neighbour. Connected frontier voxels are recursively split by their largest PCA axis if they exceed voxel count, horizontal width, world-Z height, maximum 3D extent, in-plane elongation, or projected convexity thresholds. `surface_area_proxy = voxel_count * resolution^2`; it is a ranking proxy, not an exact mesh surface area.

The node obtains the robot origin in the world frame from TF (`world_frame` ← `robot_frame`) at the map timestamp and applies the min/max range there. If TF is unavailable, that map is skipped because robot-centered range filtering cannot be performed.

## Commands

ros2 run octomap_server octomap_server_node --ros-args -p octomap_path:=/home/P7_drone_sit_awareness_ws/map.ot

LD_PRELOAD=/usr/lib/x86_64-linux-gnu/liboctomap.so ros2 run rviz2 rviz2

ros2 launch frontier_segmentation frontier_segmentation.launch.py