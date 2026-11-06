# frontier_viewpoint_optimizer

Incremental ROS 2 C++ frontier viewpoint optimizer.

## Required upstream message change

Append `bool recent` to `frontier_segmentation/msg/Frontier.msg`.

Semantics: `recent=true` means newly created or geometrically changed in the latest segmentation update; retained unchanged segments must publish `false`. The optimizer caches by `Frontier.id`, recomputes new/recent frontiers, and removes IDs no longer present. Therefore retained segment IDs must remain stable between publications.

## Inputs
- `frontier_segmentation/msg/FrontierArray`
- `octomap_msgs/msg/Octomap`: required for occupancy-aware collision and unknown-space estimates
- `nav_msgs/msg/Odometry`: current UAV position and conservative straight-line reachability test

FrontierArray alone cannot provide collision clearance or path reachability.

## Outputs
- `frontier_viewpoint_optimizer/msg/OptimizedFrontierArray`
- `visualization_msgs/msg/MarkerArray`: translucent grey triangular FOV shell per optimized frontier, or solid directional arrows when `visualize_markers_as_arrows` is enabled

## Algorithms
Candidate positions are sampled around frontier centroid and a subsample of frontier voxels, using radial offsets and altitude offsets. A candidate is rejected if its collision sphere intersects occupied or unknown OctoMap cells. Reachability is a swept-sphere straight-line check from odometry to candidate; this is a conservative line-of-sight proxy, not A*.

Yaw scoring is `w_area * observable_frontier_area + w_volume * expected_unknown_volume - altitude_bias - excessive_altitude_penalty`. The default area weight is intentionally larger. The volume term ray-samples the FOV, discounts known free space by only counting the first unknown interval, stops at occupied cells, and caps unknown depth. It is an approximate ranking metric, not exact volumetric integration.

Altitude bias is one-sided: a linear penalty applies above `altitude_reference_z`, followed by an additional quadratic penalty above `altitude_reference_z + altitude_excess_scale`. There is deliberately no low-altitude penalty; low viewpoints are instead discouraged by reduced visible frontier area and collision clearance.

## 360-degree LiDAR yaw caveat
With `horizontal_fov_deg: 360`, yaw cannot change the geometric visibility of an ideal omnidirectional LiDAR. The node therefore evaluates a single bearing yaw. Yaw sampling becomes meaningful only when the effective horizontal FOV is configured below 360 degrees (e.g. sensor/body occlusion or a directional sensor model).

The default vertical envelope interprets the previously described coverage as a 76-degree band from +46.5 to -29.5 degrees plus another 59 degrees downward, giving -88.5 to +46.5 degrees. Verify this interpretation and the sensor-frame mounting before using the FOV mesh/gain estimates.

## Build and run
```bash
sudo apt install ros-$ROS_DISTRO-octomap ros-$ROS_DISTRO-octomap-msgs nlohmann-json3-dev libeigen3-dev
cd ~/your_ws
colcon build --packages-up-to frontier_viewpoint_optimizer
source install/setup.bash
ros2 launch frontier_viewpoint_optimizer frontier_viewpoint_optimizer.launch.py
```
The upstream `frontier_segmentation` package must be built/sourced. `ros2top` registration and heartbeat are compiled in when `ros2top/ros2top.hpp` is discoverable by CMake.

## Limitations
- Not compiled against your exact workspace/package versions.
- Unknown-volume estimate is sampled and approximate; tune `new_volume_weight`.
- Unknown cells are treated as non-traversable for candidate collision clearance.
- Straight-line reachability is not a full 3D path planner.
- The FOV shell models the angular envelope, not Livox non-repetitive scan pattern, beam timing, or airframe occlusion.
