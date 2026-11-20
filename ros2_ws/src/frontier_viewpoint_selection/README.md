# frontier_viewpoint_selection (ROS 2 Jazzy)

Committed frontier viewpoint selection based on optimizer information score and asynchronous DSP PathCost calls. The node selects a best supported viewpoint, commits a goal, maintains an ordered `nav_msgs/Path` of candidate viewpoints, and periodically revalidates the committed goal. It does not require the optimizer to include path penalties.

## Interfaces

| Role | Name / Type | Details |
| --- | --- | --- |
| Input | `/frontier_viewpoints` / `frontier_viewpoint_optimizer/msg/OptimizedFrontierArray` | Current optimized viewpoint set. |
| Input | `/Odometry` / `nav_msgs/msg/Odometry` | Robot position in the optimizer map frame. |
| Service client | `/dsp/path_cost` / `dsp_interfaces/srv/PathCost` | Asynchronous directed path cost and distance queries. |
| Service server | `/dsp/planning_failure` / `std_srvs/srv/Trigger` | Called by DSP whenever it fails to plan to the current goal. While navigating, the committed goal is put on cooldown and the selector enters REPLAN (response `success=true`); otherwise the call is answered with `success=false`. |
| Service server | `/frontier_viewpoint_selection/finish` / `std_srvs/srv/Trigger` | Latches the terminal FINISHED state, sends a hold at the latest odometry pose to DSP's `set_goal`, and stops dispatching goals until node restart. |
| Service client | `/dsp/set_goal` / `dsp_interfaces/srv/SetGoal` | Request `geometry_msgs/PoseStamped goal`, response `bool success`, `string message`. Each goal or hold pose is re-sent every `goal_command_retry_seconds` until a response has `success=true`; a request with no answer is re-sent after `goal_command_timeout_seconds`. A newer command replaces one that has not been acknowledged yet. The service name is set by `dsp_set_goal_service`. |
| Output | `~/ordered_viewpoints` / `nav_msgs/msg/Path` | Ordered poses, first element is currently committed destination if present, followed by DSP-verified greedy future destinations. Not a navigable path trajectory. |
| Output | `~/state` / `std_msgs/msg/String` | SELECT_GOAL, NAVIGATING, REVALIDATE, REPLAN, GOAL_REACHED, FINISHED. |

DSP's `set_goal` is a desired-goal interface. A hold command replaces DSP's goal with the current odometry pose, so DSP drops its old path and ego-planner is sent the hold pose. **That does not by itself brake a trajectory that is already executing in ego-planner or PX4.** `success=true` only means DSP accepted the goal and stops the re-sending; DSP reports planning failures separately through `/dsp/planning_failure`, and arrival is still decided from odometry. The same pose (same header stamp) is sent on every retry.

## Arrival and state machine

Arrival uses the 3D Euclidean odometry-to-committed-goal distance, and enters GOAL_REACHED immediately at `arrival_position_tolerance` (default `0.5` m). After `observation_settle_seconds`, the frontier is placed on cooldown and a new goal is selected. A missing goal, timeout with no odometry progress, reported planning failure, or confirmed improvement can transition to REPLAN. REPLAN sends the current odometry pose to DSP's `set_goal`, clears the committed goal, then returns to SELECT_GOAL. FINISHED is terminal until process restart.

## Service usage

```bash
ros2 service call /frontier_viewpoint_selection/finish std_srvs/srv/Trigger '{}'
ros2 interface show dsp_interfaces/srv/SetGoal
ros2 service type /dsp/planning_failure
ros2 topic echo /frontier_viewpoint_selection_node/ordered_viewpoints
```

**Planning-failure protocol:** DSP is the client and pushes each failure by calling `/dsp/planning_failure`, which this node serves. `std_srvs/Trigger` has an empty request, so the failure text and the goal it belongs to are not carried; a failure is always applied to the currently committed goal.

## Build and run

```bash
colcon build --packages-select frontier_viewpoint_selection --symlink-install
source install/setup.bash
ros2 launch frontier_viewpoint_selection frontier_viewpoint_selection.launch.py
```

The node requires `ros2top` and `nlohmann_json` unconditionally, without conditional compilation. Every package source uses 1TBS / attached braces and avoids forced line wrapping for individual statements.

This package is not a flight-certified controller. DSP path cost is queried only for ranking and is not a complete guarantee that a downstream trajectory remains collision-free. The ordering is a greedy partial ordering of verified edges, not a globally optimal traveling-salesman tour. Frame IDs must match exactly; the node does not perform TF transforms.
