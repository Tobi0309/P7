# frontier_viewpoint_selection (ROS 2 Jazzy)

Committed frontier viewpoint selection based on the optimizer's information score and asynchronous DSP PathCost calls. The node ranks the optimized viewpoints, commits a goal, and sends that goal straight to ego-planner, which plans and follows the path by itself. DSP is only used to cost routes; it does not navigate. The node periodically revalidates the committed goal. It does not require the optimizer to include path penalties.

## Interfaces

| Role | Name / Type | Details |
| --- | --- | --- |
| Input | `/frontier_viewpoints` / `frontier_viewpoint_optimizer/msg/OptimizedFrontierArray` | Current optimized viewpoint set. |
| Input | `/Odometry` / `nav_msgs/msg/Odometry` | Robot position in the optimizer map frame (`odometry_topic`; P7 uses `/Odometry_base_link`). |
| Service client | `/dsp/path_cost` / `dsp_interfaces/srv/PathCost` | Asynchronous directed path cost and distance queries. |
| Service server | `/frontier_viewpoint_selection/finish` / `std_srvs/srv/Trigger` | Latches the terminal FINISHED state, sends a hold at the latest odometry pose to ego-planner, and stops dispatching goals until node restart. |
| Output | `/move_base_simple/goal` / `geometry_msgs/msg/PoseStamped` | Goals and holds for ego-planner (`goal_topic`). The orientation is the viewpoint yaw, which ego-planner turns to after it arrives. |
| Output | `~/state` / `std_msgs/msg/String` | SELECT_GOAL, NAVIGATING, REVALIDATE, REPLAN, GOAL_REACHED, FINISHED. |

## Goal hand-off to ego-planner

- Each committed goal is published once on `goal_topic`. ego-planner has no acknowledgement: it plans from the newest goal it receives, so a goal is not re-sent.
- ego-planner ignores `frame_id`. Viewpoints and odometry must be in ego-planner's map frame (`camera_init` in P7); the node refuses to plan when the viewpoint and odometry frames differ.
- `ego_planner_node` (default `/drone_0_ego_planner_node`) is looked up in the ROS graph about once per second. A subscriber count is not used because `pos_cmd_to_px4` also subscribes to the goal topic. While the node is absent, commands wait, no new goal is committed and the no-progress timeout is paused. When it is back, the active goal or hold is sent again. An empty name disables the check.
- A hold is a goal at the robot's current odometry pose (refreshed when it is published). It makes ego-planner plan a stop at that point.
- A committed viewpoint that moved more than 0.25 m (see re-association below) is sent again.

## Arrival and state machine

Arrival uses the 3D Euclidean odometry-to-committed-goal distance, and enters GOAL_REACHED immediately at `arrival_position_tolerance`. ego-planner moves a goal that lies inside its obstacle inflation (up to `fsm/goal_search_radius`) and clamps its height to the flyable band, so the tolerance must cover that. After `observation_settle_seconds`, the frontier is placed on cooldown and a new goal is selected.

A missing goal, a timeout with no odometry progress, or a confirmed improvement can transition to REPLAN. Before a missing goal counts, the selector looks for a viewpoint within `committed_goal_reassociation_distance` of the committed pose (segmentation gives a split or merged frontier a new ID) and continues to it.

REPLAN clears the committed goal and returns to SELECT_GOAL. ego-planner keeps flying toward the old goal until the next goal replaces it. A hold is sent only if no goal is committed within `replan_hold_delay_seconds` (0 sends it at once). FINISHED is terminal until process restart.

## Path-cost queries

One `/dsp/path_cost` request is in flight at a time. The queue is rebuilt from the current shortlist every tick, in priority order: robot to each candidate (and to the committed goal), committed goal to the next candidates, then pairwise edges between the `pairwise_cost_candidates`, shortest first. An empty path, or a cost at or above `max_valid_path_cost` (a route through an obstacle or its padding), marks the edge unreachable for `failed_path_retry_seconds`.

## Diagnostics

Every `ranking_period_seconds` the node logs its state, the viewpoint, cache and queue sizes, and run counters: goals reached, goals dropped (viewpoint disappeared / no progress / other), the distance flown (odometry steps of at least 0.25 m), and whether ego-planner runs.

## Service usage

```bash
ros2 service call /frontier_viewpoint_selection/finish std_srvs/srv/Trigger '{}'
ros2 topic echo /move_base_simple/goal
ros2 topic echo /frontier_viewpoint_selection/state
```

## Build and run

```bash
colcon build --packages-select frontier_viewpoint_selection --symlink-install
source install/setup.bash
ros2 launch frontier_viewpoint_selection frontier_viewpoint_selection.launch.py
```

The node requires `ros2top` and `nlohmann_json` unconditionally, without conditional compilation. Every package source uses 1TBS / attached braces and avoids forced line wrapping for individual statements.

This package is not a flight-certified controller. DSP path cost is queried only for ranking and is not a guarantee that ego-planner can fly the route: ego-planner follows a straight reference to the goal and avoids obstacles locally. The ordering is a greedy partial ordering of verified edges, not a globally optimal traveling-salesman tour. Frame IDs must match exactly; the node does not perform TF transforms.
