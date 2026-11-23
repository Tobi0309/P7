#ifndef _DSL_GRID_3D_H_
#define _DSL_GRID_3D_H_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "dsp_interfaces/srv/path_cost.hpp"

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

// DSP is a path-cost server for frontier_viewpoint_selection: it keeps a voxel grid of the
// octomap and answers /dsp/path_cost with a D* Lite (DSL) search. It does not navigate; the
// selector sends its goals straight to ego-planner.
//
// Threading (MultiThreadedExecutor, one mutually exclusive callback group per role):
//   query - /dsp/path_cost, answered with one persistent, lazily expanded search.
//   map   - map subscription (stores the newest message) and the timer that applies it.
//   misc  - ros2top heartbeat.
// DSL's Vertex/Edge ids come from a global non-atomic counter, so every DSL call holds
// dsl_mutex_.
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
        double cost{0.0};
        bool unconnected{false};                // obstacles separate start and goal
        bool over_budget{false};                // search abandoned after path_cost_max_search_ms
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
    // Swaps in a fresh search once the current one has more than max_search_vertices vertices.
    // The old one is freed by freeReleasedSearches() on the misc thread: freeing tens of
    // thousands of vertices takes long enough to delay a path_cost answer.
    void releaseSearchIfLarge();
    void freeReleasedSearches();

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
    // Connected components of the voxels that are not blocked (recomputed when stale).
    const std::vector<int> &components();

    void request_cost(const std::shared_ptr<dsp_interfaces::srv::PathCost::Request> req, std::shared_ptr<dsp_interfaces::srv::PathCost::Response> res);
    float path_distance(const nav_msgs::msg::Path &path);
    float point_distance(geometry_msgs::msg::Point p0, geometry_msgs::msg::Point p1);
    // INFO summary (count, median and max time, "no path" answers) of the recent queries.
    void logQueryStats();

    // Searches a path (caller holds dsl_mutex_). A start or goal inside a blocked voxel is moved
    // to the nearest free voxel within endpoint_snap_radius first. Endpoints in different
    // connected components are answered without a search; a search that runs longer than
    // path_cost_max_search_ms is abandoned.
    SearchResult searchPath(const Eigen::Vector3d &start, const Eigen::Vector3d &goal);
    bool worldToIndex(const Eigen::Vector3d &wpos, int &idx) const;
    Eigen::Vector3d voxelCenter(int idx) const;
    Eigen::Vector3d gridCenter(int idx) const;

    // Height map:
    void buildHeightMap();
    void buildHeightCostMap();

    nav_msgs::msg::Path pointsToPath(const std::vector<Eigen::Vector3d> &points) const;
    void publishOccupancyGrid();
    Eigen::Vector3d posRes(Eigen::Vector3d wpos) const;

    rclcpp::CallbackGroup::SharedPtr query_group_;
    rclcpp::CallbackGroup::SharedPtr map_group_;
    rclcpp::CallbackGroup::SharedPtr misc_group_;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr occ_map_viz_pub_;
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr get_octomap_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr get_ocupancy_sub_;
    rclcpp::TimerBase::SharedPtr map_timer_;

    rclcpp::Service<dsp_interfaces::srv::PathCost>::SharedPtr cost_srv_;
    rclcpp::TimerBase::SharedPtr ros2top_timer_;
    rclcpp::TimerBase::SharedPtr stats_timer_;
    rclcpp::TimerBase::SharedPtr release_timer_;
    bool ros2top_registered_{false};

    // ---- Guarded by released_mutex_: searches waiting to be freed (they only own their own
    // vertices and edges, never the shared grid, so they can be freed without dsl_mutex_) ----
    std::mutex released_mutex_;
    std::vector<std::unique_ptr<DspGridSearch>> released_searches_;

    // ---- Guarded by stats_mutex_: path_cost statistics since the last summary ----
    std::mutex stats_mutex_;
    std::vector<double> query_ms_;
    int no_path_count_ = 0;
    int unconnected_count_ = 0;
    int over_budget_count_ = 0;

    // ---- Guarded by dsl_mutex_: map state, layout and the search ----
    std::mutex dsl_mutex_;

    // Declared in destruction-safe order: the search references the grid and the connectivity.
    std::unique_ptr<dsl::Grid3d> grid_;
    std::unique_ptr<dsl::Grid3dConnectivity> connectivity_;
    DspGridCost grid_cost_;
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
    // Component label per voxel (-1 = blocked); stale when components_dirty_ is set.
    std::vector<int> component_;
    bool components_dirty_ = true;

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
    Eigen::Vector3d pmin;
    std::atomic<bool> grid_built{false};

    // ---- Newest unapplied map message (map callback group only) ----
    std::mutex map_msg_mutex_;
    octomap_msgs::msg::Octomap::SharedPtr pending_octomap_;
    nav_msgs::msg::OccupancyGrid::SharedPtr pending_grid_;
};

} // namespace

#endif
