# frontier_viewpoint_selection (ROS 2 Jazzy)

Committed frontier viewpoint selection based on optimizer information score and asynchronous DSP PathCost calls. The node selects a best supported viewpoint, commits a goal, maintains an ordered `nav_msgs/Path` of candidate viewpoints, and periodically revalidates the committed goal. It does not require the optimizer to include path penalties.

## Interfaces

| Role | Name / Type | Details |
| --- | --- | --- |
| Input | `/frontier_viewpoints` / `frontier_viewpoint_optimizer/msg/OptimizedFrontierArray` | Current optimized viewpoint set. |
| Input | `/Odometry` / `nav_msgs/msg/Odometry` | Robot position in the optimizer map frame. |
| Service client | `/dsp/path_cost` / `dsp_interfaces/srv/PathCost` | Asynchronous directed path cost and distance queries. |
| Service client | `/dsp/planning_failure` / `std_srvs/srv/Trigger` | Polled only while navigating; success=true means a NEW planning failure; false means none. The server must clear/consume its flag after reporting it. This service is **not provided by the supplied dsp.cpp** and must be implemented by the navigation/DSP adapter. |
| Service server | `/frontier_viewpoint_selection/finish` / `std_srvs/srv/Trigger` | Latches the terminal FINISHED state, sends a hold at the latest odometry pose through `goal_command`, and stops dispatching goals until node restart. |
| Service client | `/frontier_viewpoint_selection/goal_command` / `frontier_viewpoint_selection/srv/GoalCommand` | Request `geometry_msgs/PoseStamped pose`, response `bool success`. Each goal or hold pose is re-sent every `goal_command_retry_seconds` until a response has `success=true`; a request with no answer is re-sent after `goal_command_timeout_seconds`. A newer command replaces one that has not been acknowledged yet. |
| Output | `~/ordered_viewpoints` / `nav_msgs/msg/Path` | Ordered poses, first element is currently committed destination if present, followed by DSP-verified greedy future destinations. Not a navigable path trajectory. |
| Output | `~/state` / `std_msgs/msg/String` | SELECT_GOAL, NAVIGATING, REVALIDATE, REPLAN, GOAL_REACHED, FINISHED. |

The goal command service is a desired-goal interface. **A current-pose (hold) command does not necessarily cancel a trajectory already executing in Nav2, DSP, or PX4.** The downstream navigation adapter that serves `goal_command` must treat it as preemption/hold and must implement any required path cancellation or braking safely. `success=true` only stops the re-sending; arrival is still decided from odometry. The same pose (same header stamp) is sent on every retry, so the server can recognise repeats.

## Arrival and state machine

Arrival uses the 3D Euclidean odometry-to-committed-goal distance, and enters GOAL_REACHED immediately at `arrival_position_tolerance` (default `0.5` m). After `observation_settle_seconds`, the frontier is placed on cooldown and a new goal is selected. A missing goal, timeout with no odometry progress, reported planning failure, or confirmed improvement can transition to REPLAN. REPLAN sends the current odometry pose through `goal_command`, clears the committed goal, then returns to SELECT_GOAL. FINISHED is terminal until process restart.

## Service usage

```bash
ros2 service call /frontier_viewpoint_selection/finish std_srvs/srv/Trigger '{}'
ros2 interface show frontier_viewpoint_selection/srv/GoalCommand
ros2 service type /dsp/planning_failure
ros2 topic echo /frontier_viewpoint_selection_node/ordered_viewpoints
```

**Planning-failure protocol:** A `Trigger` service cannot push notifications to a client. This node polls it every `planning_failure_poll_seconds` while navigating. The service's `success` field is interpreted as a failure-event flag (true means a failure happened, false means normal), *not* as an indication that the RPC was handled successfully. It must reset the event once reported or repeated replans will occur. If a different failure-reporting protocol is needed, use a dedicated service with explicit fields or a push-style event topic.

## Build and run

```bash
colcon build --packages-select frontier_viewpoint_selection --symlink-install
source install/setup.bash
ros2 launch frontier_viewpoint_selection frontier_viewpoint_selection.launch.py
```

The node requires `ros2top` and `nlohmann_json` unconditionally, without conditional compilation. Every package source uses 1TBS / attached braces and avoids forced line wrapping for individual statements.

This package is not a flight-certified controller. DSP path cost is queried only for ranking and is not a complete guarantee that a downstream trajectory remains collision-free. The ordering is a greedy partial ordering of verified edges, not a globally optimal traveling-salesman tour. Frame IDs must match exactly; the node does not perform TF transforms.
