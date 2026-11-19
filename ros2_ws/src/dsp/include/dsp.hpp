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

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "dsl/gridsearch.h"
#include "dsl/gridcost.h"
#include "dsl/grid3d.h"
#include "dsl/grid3dconnectivity.h"

#include "octomap_msgs/msg/octomap.hpp"
#include "octomap_msgs/conversions.h"
#include "octomap/octomap.h"

#include "visualization_msgs/msg/marker.hpp"

#include <memory>
#include <algorithm>
#include <cstdint>
#include <functional>
#include <vector>
#include <Eigen/Dense>
#include <cmath>

namespace dsp
{

class Dsp : public rclcpp::Node
{
public:
    Dsp();
    ~Dsp() override;

private:
    // callbac for map
    // 2D
    void occupancy_grid_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    //3D
    void octomap_data_callback(const octomap_msgs::msg::Octomap::SharedPtr msg);
    void updateGDSP(const octomap::OcTree &tree);
    void buildGDSP(const octomap::OcTree &tree);
    void forEachLeafVoxel(const octomap::OcTree &tree,
                          const std::function<void(int, int, int, bool)> &fn) const;
    void buildGraph();

    // Occupancy state and hard obstacle padding
    int voxelIndex(int x, int y, int z) const;
    bool inBounds(int x, int y, int z) const;
    void resetOccupancy();
    void setVoxelState(int x, int y, int z, double state);
    void addPadding(int x, int y, int z, int delta);
    void buildPaddingOffsets();
    double effectiveCost(int idx) const;
    void syncGraphCosts();

    // set starting point of planing
    void handleSetStart(const geometry_msgs::msg::Point::SharedPtr msg);
    void handleSetStartOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
    void setStart(Eigen::Vector3d wpos);
    bool lookupRobotPosition(Eigen::Vector3d &wpos);

    void request_cost(const std::shared_ptr<dsp_interfaces::srv::PathCost::Request> req, std::shared_ptr<dsp_interfaces::srv::PathCost::Response> res);
    float path_distance(const nav_msgs::msg::Path path);
    float point_distance(geometry_msgs::msg::Point p0, geometry_msgs::msg::Point p1);
    bool setSG(Eigen::Vector3d grid_start, Eigen::Vector3d grid_goal);

    // Height map:
    void buildHeightMap();
    void buildHeightCostMap();


    // set ending point of planing
    void handleSetGoal(const std::shared_ptr<dsp_interfaces::srv::SetGoal::Request> req,
                       std::shared_ptr<dsp_interfaces::srv::SetGoal::Response> res);
    bool planPath();
    void publishGoalAhead();
    geometry_msgs::msg::Quaternion pathHeading(size_t idx) const;
    void publishAllPaths();
    nav_msgs::msg::Path dspPathToRosMsg(const dsl::GridPath<3>& dsp_path, bool isSplined);
    nav_msgs::msg::Path dspPathToRosMsg(const std::vector<Eigen::Vector3d>& dsp_path, bool isSplined);
    // Timers
    void pathUpdateCallback();
    void goalPublishCallback();

    void publishOccupancyGrid();
    Eigen::Vector3d posRes(Eigen::Vector3d wpos);

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr occ_map_viz_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr splinepath_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr set_start_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr set_start_odom_sub_;
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr get_octomap_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr get_ocupancy_sub_;
    rclcpp::TimerBase::SharedPtr path_update_timer;
    rclcpp::TimerBase::SharedPtr goal_publish_timer;

    rclcpp::Service<dsp_interfaces::srv::PathCost>::SharedPtr cost_srv_;
    rclcpp::Service<dsp_interfaces::srv::SetGoal>::SharedPtr set_goal_srv_;

    // dsp variables
    std::shared_ptr<dsl::Grid3d> grid_;
    dsl::GridCost<3> cost_;
    std::shared_ptr<dsl::Grid3dConnectivity> connectivity_;
    std::shared_ptr<dsl::GridSearch<3>> gdsl_;
    dsl::GridPath<3> path_, splinecells_;
    std::vector<Eigen::Vector3d> splinepath_;

    // Raw per-voxel state from the map: DSP_FREE, dsp_unknown_ or DSP_OCCUPIED.
    std::vector<double> occupancy_map;
    // Number of occupied voxels whose padding covers this voxel (> 0 => padded).
    std::vector<int32_t> padding_count_;
    // Cost currently set in the search graph for each voxel.
    std::vector<double> graph_cost_;
    // Voxel offsets (relative to an occupied voxel) that fall inside the padding.
    std::vector<Eigen::Vector3i> padding_offsets_;
    // obstacle_padding [m] the current padding counts were built with.
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
    double res_octomap;
    double spline_step_;

    Eigen::Vector3d start_pos;
    Eigen::Vector3d goal_pos;
    Eigen::Vector3d last_grid_start;
    Eigen::Vector3d last_grid_goal;
    bool has_last_grid_start = false;
    bool has_last_grid_goal = false;
    bool grid_built = false;
    Eigen::Vector3d pmin;

    // Goal as requested (in odom_frame_id); its orientation is forwarded to ego-planner.
    geometry_msgs::msg::PoseStamped goal_pose_;
    bool has_goal_ = false;
    // Latest planned (unsplined) path, one pose per voxel.
    nav_msgs::msg::Path path_msg_;
    bool has_path_ = false;
    size_t path_progress_idx_ = 0;
    bool final_goal_sent_ = false;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
};

} // namespace

#endif
