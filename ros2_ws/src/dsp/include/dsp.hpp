#ifndef _DSL_GRID_3D_H_
#define _DSL_GRID_3D_H_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "dsp_interfaces/srv/path_cost.hpp"
#include "dsp_interfaces/srv/set_goal.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "dsl/gridpath.h"
#include "dsl/grid3d.h"
#include "dsl/grid3dconnectivity.h"

#include "dsp_grid_search.hpp"
#include "dsp_voxel_utils.hpp"

#include "octomap_msgs/msg/octomap.hpp"
#include "octomap_msgs/conversions.h"
#include "octomap/octomap.h"

#include "visualization_msgs/msg/marker.hpp"

#include <atomic>
#include <chrono>
#include <memory>
#include <algorithm>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <cmath>

namespace dsp
{

// Threading model (MultiThreadedExecutor, one mutually exclusive callback group per role):
//   goal    - dsp/set_goal: stores the goal and answers immediately; never touches DSL.
//   nav     - planner tick: plans the flown path with nav_search_.
//   publish - look-ahead goal publisher for ego-planner; only needs path_mutex_.
//   query   - /dsp/path_cost: answered with its own persistent search (query_search_).
//   map     - map subscription (stores the newest message) and the timer that applies it.
// DSL's Vertex/Edge ids come from a global non-atomic counter, so the two searches must never
// run at the same time: every DSL call holds dsl_mutex_. The planner and map updates take it
// with priority over path_cost queries (see lockDslWithPriority()).
class Dsp : public rclcpp::Node
{
public:
    Dsp();
    ~Dsp() override;

private:
    struct SearchResult
    {
        bool ok{false};
        std::string reason;
        std::vector<int> cells;                 // voxel indices from start to goal
        std::vector<Eigen::Vector3d> points;    // exact start, voxel centres, exact goal
        dsl::GridPath<3> grid_path;             // DSL path (grid units), for the spline
        double cost{0.0};
    };

    // Map input: callbacks only store the newest message; applyPendingMap() applies it.
    void octomap_data_callback(const octomap_msgs::msg::Octomap::SharedPtr msg);
    void occupancy_grid_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void applyPendingMap();
    void applyOctomap(const octomap::OcTree &tree);
    void applyOccupancyGrid(const nav_msgs::msg::OccupancyGrid &msg);
    void updateGDSP(const octomap::OcTree &tree);
    void buildGDSP(const octomap::OcTree &tree);
    void forEachLeafVoxel(const octomap::OcTree &tree,
                          const std::function<void(int, int, int, bool)> &fn) const;
    void buildGraph();
    void releaseSearchIfLarge(std::unique_ptr<DspGridSearch> &search, const char *name);

    // Occupancy state and hard obstacle padding
    int voxelIndex(int x, int y, int z) const;
    bool inBounds(int x, int y, int z) const;
    VoxelLayout layout() const;
    void resetOccupancy();
    void setVoxelState(int x, int y, int z, double state);
    void addPadding(int x, int y, int z, int delta);
    void buildPaddingOffsets();
    double effectivePadding() const;
    double effectiveCost(int idx) const;
    bool blocked(int idx) const;
    bool wall(int idx) const;
    void syncGraphCosts();

    // set starting point of planing
    void handleSetStart(const geometry_msgs::msg::Point::SharedPtr msg);
    void handleSetStartOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
    bool lookupRobotPosition(Eigen::Vector3d &wpos);

    void request_cost(const std::shared_ptr<dsp_interfaces::srv::PathCost::Request> req, std::shared_ptr<dsp_interfaces::srv::PathCost::Response> res);
    float path_distance(const nav_msgs::msg::Path &path);
    float point_distance(geometry_msgs::msg::Point p0, geometry_msgs::msg::Point p1);
    // Logs the failure and notifies /dsp/planning_failure (served by frontier_viewpoint_selection).
    void reportPlanningFailure(const std::string &message);

    // Searches a path with `search` (caller holds dsl_mutex_). A start or goal inside a blocked
    // voxel is moved to the nearest free voxel within endpoint_snap_radius first.
    SearchResult searchPath(DspGridSearch &search, const Eigen::Vector3d &start,
                            const Eigen::Vector3d &goal);
    bool worldToIndex(const Eigen::Vector3d &wpos, int &idx) const;
    Eigen::Vector3d voxelCenter(int idx) const;
    Eigen::Vector3d gridCenter(int idx) const;

    // Height map:
    void buildHeightMap();
    void buildHeightCostMap();

    // set ending point of planing
    void handleSetGoal(const std::shared_ptr<dsp_interfaces::srv::SetGoal::Request> req,
                       std::shared_ptr<dsp_interfaces::srv::SetGoal::Response> res);
    void planTick();
    void publishGoalAhead();
    geometry_msgs::msg::Quaternion pathHeading(size_t idx) const;
    nav_msgs::msg::Path pointsToPath(const std::vector<Eigen::Vector3d> &points) const;
    // Timers
    void goalPublishCallback();

    void publishOccupancyGrid();
    Eigen::Vector3d posRes(Eigen::Vector3d wpos) const;

    // Locks dsl_mutex_ ahead of any path_cost query that is waiting for it.
    std::unique_lock<std::mutex> lockDslWithPriority();

    rclcpp::CallbackGroup::SharedPtr goal_group_;
    rclcpp::CallbackGroup::SharedPtr nav_group_;
    rclcpp::CallbackGroup::SharedPtr publish_group_;
    rclcpp::CallbackGroup::SharedPtr query_group_;
    rclcpp::CallbackGroup::SharedPtr map_group_;
    rclcpp::CallbackGroup::SharedPtr misc_group_;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr occ_map_viz_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr splinepath_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr set_start_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr set_start_odom_sub_;
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr get_octomap_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr get_ocupancy_sub_;
    rclcpp::TimerBase::SharedPtr plan_timer_;
    rclcpp::TimerBase::SharedPtr goal_publish_timer;
    rclcpp::TimerBase::SharedPtr map_timer_;

    rclcpp::Service<dsp_interfaces::srv::PathCost>::SharedPtr cost_srv_;
    rclcpp::Service<dsp_interfaces::srv::SetGoal>::SharedPtr set_goal_srv_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr planning_failure_client_;
    rclcpp::TimerBase::SharedPtr ros2top_timer_;
    bool ros2top_registered_{false};

    // ---- Guarded by dsl_mutex_: map state, layout and both searches ----
    std::mutex dsl_mutex_;
    std::atomic<int> priority_waiters_{0};

    // Declared in destruction-safe order: searches reference the grid and the connectivity.
    std::unique_ptr<dsl::Grid3d> grid_;
    std::unique_ptr<dsl::Grid3dConnectivity> connectivity_;
    DspGridCost grid_cost_;
    std::unique_ptr<DspGridSearch> nav_search_;
    std::unique_ptr<DspGridSearch> query_search_;

    // Raw per-voxel state from the map: DSP_FREE, dsp_unknown_ or DSP_OCCUPIED.
    std::vector<double> occupancy_map;
    // Number of occupied voxels whose padding covers this voxel (> 0 => padded).
    std::vector<int32_t> padding_count_;
    // Cost currently set in the search graph for each voxel.
    std::vector<double> graph_cost_;
    // Voxel offsets (relative to an occupied voxel) that fall inside the padding.
    std::vector<Eigen::Vector3i> padding_offsets_;
    // Padding [m] the current padding counts were built with.
    double padding_m_ = -1.0;

    std::vector<int> floor_height_map;
    std::vector<double> height_cost_map;

    // launch params
    const int DSP_OCCUPIED = 2000000000;
    const double DSP_FREE = 1.0;
    int dsp_unknown_ = 10000;

    int length_voxel = 0;
    int width_voxel = 0;
    int height_voxel = 0;
    double res_octomap = 0.0;
    double spline_step_;
    Eigen::Vector3d pmin;
    std::atomic<bool> grid_built{false};

    // ---- Newest unapplied map message (map callback group only) ----
    std::mutex map_msg_mutex_;
    octomap_msgs::msg::Octomap::SharedPtr pending_octomap_;
    nav_msgs::msg::OccupancyGrid::SharedPtr pending_grid_;

    // ---- Guarded by goal_mutex_ ----
    std::mutex goal_mutex_;
    // Goal as requested (in odom_frame_id); its orientation is forwarded to ego-planner.
    geometry_msgs::msg::PoseStamped goal_pose_;
    Eigen::Vector3d goal_pos;
    bool has_goal_ = false;
    // Last start received on dsp/set_start; used when TF is unavailable.
    Eigen::Vector3d start_pos;
    // Incremented for every new goal; a plan is only stored if the goal did not change meanwhile.
    std::atomic<uint64_t> goal_generation_{0};
    std::atomic<bool> plan_requested_{false};
    std::chrono::steady_clock::time_point last_plan_time_{};

    // ---- Guarded by path_mutex_: the path being followed ----
    std::mutex path_mutex_;
    // Latest planned path: exact start, one pose per voxel, exact goal.
    nav_msgs::msg::Path path_msg_;
    geometry_msgs::msg::PoseStamped path_goal_pose_;
    uint64_t path_generation_ = 0;
    bool has_path_ = false;
    size_t path_progress_idx_ = 0;
    bool final_goal_sent_ = false;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
};

} // namespace

#endif
