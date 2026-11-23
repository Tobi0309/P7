# frontier_viewpoint_optimizer — spatial incremental / no orphan retention

ROS 2 Jazzy package. Publishes one `OptimizedFrontier` per *currently listed* frontier that has a validated viewpoint. Preserves native 0.5 m OctoMap voxel-center sampling and six-connected BFS, refinement within a single cell, frontier-area-based scoring, and ros2top registration/heartbeat.

## Lifecycle and correctness

- Each received `FrontierArray` is an authoritative snapshot. An ID not present in it is erased immediately, including its cached result and pending work. An ID never gets published twice; no orphan retention or missing-ID grace period.
- An unchanged frontier remains in the cache and keeps its optimized viewpoint regardless of distance traveled by the UAV. Neither odometry changes nor unrelated map changes request resampling.
- Geometry revisions schedule expensive optimization only when the segment is marked `recent`, or map native-cell changes touch that segment. Removal-only changes (which segmentation currently marks `recent=false`) are caught using spatial differences.
- Changed map voxels only request cheap validation when they intersect the cached viewpoint cell or its watched observation-ray cells. Other viewpoints are untouched. Geometric reoptimization and validation run on separate threads.
- `reachable=0` after a *completed, untruncated* BFS freezes optimization of that frontier geometry revision. A new materially changed frontier geometry in an affected region can unfreeze it. Search time/node-limit failure is **not** proof of unreachability. The selector/DSP must still verify an actual navigation route.
- If optimization fails to improve a valid cached result, the old result stays available. If validation proves that its cell is occupied or all sampled observations are blocked, the cache is invalidated.
- Canceled IDs cannot resurrect a result even when an expensive optimization finishes late; results are committed only if the ID and monotonic geometry revision still match.

## Threads

1. **ROS callbacks / executor:** keeps the latest OctoMap input only; synchronizes frontier membership and odometry.
2. **Map worker:** decodes the newest map, computes the exact native-grid occupancy delta from the last processed map, and schedules only affected IDs. Coalesces intermediate map messages safely by comparing snapshots.
3. **Validation worker:** checks only the affected cached viewpoint and its rays, with no reachability BFS.
4. **Optimization worker pool:** `optimization_worker_count` (default 2), each running expensive candidate sampling and a shared local BFS per processed snapshot/robot cell. The work queue contains each frontier ID at most once.

## Build and run

```bash
colcon build --packages-select frontier_viewpoint_optimizer --symlink-install
source install/setup.bash
ros2 launch frontier_viewpoint_optimizer frontier_viewpoint_optimizer.launch.py
```

Requires `ros2top` and `nlohmann_json` unconditionally; **no** `FVO_WITH_ROS2TOP` preprocessor switch. C++ uses K&R/1TBS attached braces and the included `.clang-format` has `ColumnLimit: 0`.

## Upstream frontier_segmentation findings (uploaded reference)

- Existing segments **are** carried forward outside `dirty_keys` (`retained.push_back(voxel)`), but segments smaller than `min_frontier_voxels` are removed from the published set. Removed IDs must disappear from optimized output as well.
- New frontier voxels are discarded if their distance to the robot is beyond `max_range_`, whereas existing segments are retained. This is an intentional or accidental robot-dependent **discovery** filter, not a global frontier lifetime filter.
- `recent` is cleared each publication and removal-only changed segments are retained with `recent=false`. Therefore an optimizer cannot use `recent` as its only changed-frontier signal.
- `hasUnsupportedVoxel()` returns true if *any* voxel lacks known cells below, while the callers reject a component if it returns false. That test can delete an otherwise geometrically valid frontier after split/merge. Review the intended filtering semantics.
- `isFrontierVoxel()` in the uploaded source considers unknown neighbors in **the same layer or below** (`dz=-1..0`), not only XY (`dz=0`). This differs from the earlier strict-XY definition. The optimizer does not reimplement upstream frontier acceptance.
- If a previously seen frontier ID vanishes from the **complete** `FrontierArray`, no downstream optimizer can prove it should exist. Correct the segmentation ownership and removal rules if an ID vanishes outside dirty map cells.

## Physical assumptions and limitations

- The entire rotating/tilting UAV collision envelope **must fit inside a 0.5 m voxel** for the coarse BFS assumption to be collision-safe; check this before real flight.
- Voxel padding (`obstacle_padding_enabled`, `obstacle_padding`) rejects candidate viewpoints whose voxel centre is closer than the padding to an occupied voxel, and invalidates cached viewpoints that end up inside it. It uses DSP's padding rule; keep both values equal to DSP's, otherwise DSP treats the viewpoints as blocked.
- The FOV model assumes yaw-level LiDAR unless transforms are incorporated; it does not model roll/pitch changes.
- A bounded BFS can report zero reachable candidate centers because the frontier is outside its configured 25 m radius. If that is a concern, increase `reachability_max_distance` or let DSP handle long-distance route verification, but do not conflate local BFS with global reachability.
- Spatial deltas are computed from the decoded full OctoMap; this adds CPU and RAM per map update but runs asynchronously. If your Octomap is rolling or resets unexpectedly, dropped segment IDs should be diagnosed upstream.
- The changes are syntax/structure verified only; ROS 2 Jazzy compilation is not available in this environment.
