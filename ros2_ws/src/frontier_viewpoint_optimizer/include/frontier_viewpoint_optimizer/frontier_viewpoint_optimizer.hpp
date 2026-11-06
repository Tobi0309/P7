#pragma once
#include <rclcpp/rclcpp.hpp>
#include <frontier_segmentation/msg/frontier_array.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <frontier_viewpoint_optimizer/msg/optimized_frontier_array.hpp>
#include <octomap/OcTree.h>
#include <Eigen/Core>
#include <unordered_map>
#include <vector>
#include <memory>
#include <string>

namespace frontier_viewpoint_optimizer {
struct Candidate {
  Eigen::Vector3d p{Eigen::Vector3d::Zero()};
  double yaw{0.0}, area{0.0}, volume{0.0}, score{-1e30};
};
class FrontierViewpointOptimizer : public rclcpp::Node {
public:
  FrontierViewpointOptimizer();
  ~FrontierViewpointOptimizer() override;
private:
  using FrontierArray = frontier_segmentation::msg::FrontierArray;
  void frontierCb(const FrontierArray::SharedPtr);
  void mapCb(const octomap_msgs::msg::Octomap::SharedPtr);
  void odomCb(const nav_msgs::msg::Odometry::SharedPtr);
  void publishCb();
  void processPending();
  void heartbeatCb();
  Candidate optimize(const frontier_segmentation::msg::Frontier &);
  Candidate evaluate(const frontier_segmentation::msg::Frontier &, const Eigen::Vector3d &, double) const;
  std::vector<Eigen::Vector3d> samplePositions(const frontier_segmentation::msg::Frontier &) const;
  bool freeSphere(const Eigen::Vector3d &) const;
  bool reachable(const Eigen::Vector3d &, const Eigen::Vector3d &) const;
  bool inFov(const Eigen::Vector3d &, const Eigen::Vector3d &, double) const;
  double frontierArea(const frontier_segmentation::msg::Frontier &, const Eigen::Vector3d &, double) const;
  double unknownVolume(const Eigen::Vector3d &, double) const;
  void refine(const frontier_segmentation::msg::Frontier &, Candidate &) const;
  visualization_msgs::msg::MarkerArray makeMarkers() const;
  void registerTop();
  void unregisterTop();
  template<typename T> T param(const std::string & n, const T & d) { return declare_parameter<T>(n,d); }

  rclcpp::Subscription<FrontierArray>::SharedPtr frontier_sub_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<frontier_viewpoint_optimizer::msg::OptimizedFrontierArray>::SharedPtr result_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_, heartbeat_timer_;
  std::shared_ptr<octomap::OcTree> tree_;
  Eigen::Vector3d robot_{Eigen::Vector3d::Zero()}, sensor_offset_{Eigen::Vector3d::Zero()};
  bool have_odom_{false}, have_frontiers_{false}, ros2top_registered_{false};
  std::string frame_{"map"}, frame_override_;
  std::unordered_map<uint32_t, frontier_segmentation::msg::Frontier> frontiers_;
  std::unordered_map<uint32_t, Candidate> results_;
  std::vector<uint32_t> active_ids_;

  double hfov_, vmin_, vmax_, rmin_, rmax_, radial_step_, min_offset_, max_offset_;
  double collision_radius_, collision_step_, reach_step_, area_weight_, volume_weight_;
  double altitude_weight_, altitude_ref_, altitude_excess_, excess_weight_;
  double ray_step_, unknown_cap_, refine_xy_, refine_z_, refine_yaw_, marker_alpha_;
  int radial_layers_, azimuth_samples_, max_candidates_, yaw_samples_;
  int ray_azimuth_samples_, ray_elevation_samples_, refine_iterations_, marker_segments_;
  bool require_map_, enable_refine_, enable_diagnostics_, visualize_markers_as_arrows_;
  std::vector<double> altitude_offsets_;
  std::string frontier_topic_, map_topic_, odom_topic_, result_topic_, marker_topic_;
};
}
