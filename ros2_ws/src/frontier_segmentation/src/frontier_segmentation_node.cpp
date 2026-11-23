#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstddef>
#include <memory>
#include <limits>
#include <iterator>
#include <functional>
#include <numeric>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <unistd.h>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/string.hpp>
#include "frontier_segmentation/msg/frontier_array.hpp"
#include "frontier_segmentation/msg/frontier.hpp"
#include "ros2top/ros2top.hpp"

namespace
{
  struct Voxel
  {
    int x, y, z;
    double wx, wy, wz;
  };
  using Component = std::vector<Voxel>;

  int64_t key3(int x, int y, int z)
  {
    // Pack signed 21-bit voxel coordinates. Adequate for normal mapping extents.
    constexpr int64_t mask = (1LL << 21) - 1;
    return ((static_cast<int64_t>(x) & mask) << 42) |
           ((static_cast<int64_t>(y) & mask) << 21) |
           (static_cast<int64_t>(z) & mask);
  }

  std::vector<Eigen::Vector2d> convexHull(std::vector<Eigen::Vector2d> p)
  {
    std::sort(p.begin(), p.end(), [](const auto &a, const auto &b)
              { return a.x() == b.x() ? a.y() < b.y() : a.x() < b.x(); });
    p.erase(std::unique(p.begin(), p.end(), [](const auto &a, const auto &b)
                        { return (a - b).norm() < 1e-8; }),
            p.end());
    if (p.size() < 3)
      return p;
    std::vector<Eigen::Vector2d> h;
    for (const auto &v : p)
    {
      while (h.size() >= 2 && (h.back() - h[h.size() - 2]).x() * (v - h.back()).y() -
                                      (h.back() - h[h.size() - 2]).y() * (v - h.back()).x() <=
                                  0)
        h.pop_back();
      h.push_back(v);
    }
    const size_t lower = h.size();
    for (int i = static_cast<int>(p.size()) - 2; i >= 0; --i)
    {
      const auto &v = p[static_cast<size_t>(i)];
      while (h.size() > lower && (h.back() - h[h.size() - 2]).x() * (v - h.back()).y() -
                                         (h.back() - h[h.size() - 2]).y() * (v - h.back()).x() <=
                                     0)
        h.pop_back();
      h.push_back(v);
    }
    if (!h.empty())
      h.pop_back();
    return h;
  }

  double polygonArea(const std::vector<Eigen::Vector2d> &p)
  {
    if (p.size() < 3)
      return 0.0;
    double a = 0.0;
    for (size_t i = 0; i < p.size(); ++i)
    {
      const auto &u = p[i], &v = p[(i + 1) % p.size()];
      a += u.x() * v.y() - v.x() * u.y();
    }
    return std::abs(a) * 0.5;
  }
}

class FrontierSegmentationNode : public rclcpp::Node
{
public:
  FrontierSegmentationNode() : Node("frontier_segmentation")
  {
    input_topic_ = declare_parameter<std::string>("octomap_topic", "/octomap_full");
    sensor_model_ = declare_parameter<std::string>("sensor_model", "livox_mid360");
    sensor_frame_ = declare_parameter<std::string>("sensor_frame", "livox_frame");
    world_frame_ = declare_parameter<std::string>("world_frame", "camera_init");
    robot_frame_ = declare_parameter<std::string>("robot_frame", "body");
    occupied_threshold_ = declare_parameter<double>("occupied_threshold", 0.5);
    min_frontier_voxels_ = declare_parameter<int>("min_frontier_voxels", 8);
    max_frontier_voxels_ = declare_parameter<int>("max_frontier_voxels", 1200);
    max_width_ = declare_parameter<double>("max_width_m", 12.0);
    max_height_ = declare_parameter<double>("max_height_m", 8.0);
    max_extent_ = declare_parameter<double>("max_extent_m", 15.0);
    max_elongation_ = declare_parameter<double>("max_elongation_ratio", 4.0);
    min_convexity_ = declare_parameter<double>("min_convexity_ratio", 0.45);
    min_range_ = declare_parameter<double>("min_range_m", -1.0);
    max_range_ = declare_parameter<double>("max_range_m", -1.0);
    camera_hfov_ = declare_parameter<double>("camera_hfov_deg", 90.0);
    camera_vfov_ = declare_parameter<double>("camera_vfov_deg", 64.0);
    lidar_hfov_ = declare_parameter<double>("lidar_hfov_deg", 360.0);
    lidar_vfov_ = declare_parameter<double>("lidar_vfov_deg", 59.0);
    lidar_axis_pitch_ = declare_parameter<double>("lidar_axis_pitch_deg", 0.0);
    design_range_ = declare_parameter<double>("frontier_design_range_m", 10.0);
    use_fov_extent_limits_ = declare_parameter<bool>("use_fov_extent_limits", false);
    positive_z_is_up_ = declare_parameter<bool>("positive_z_is_up", true);
    viz_topic_ = declare_parameter<std::string>("marker_topic", "/frontier_segments/markers");
    frontier_topic_ = declare_parameter<std::string>("frontier_topic", "/frontier_segments");
    diagnostics_ = declare_parameter<bool>("diagnostics", false);

    // Default ranges follow the selected sensor. Override explicitly if needed.
    if (min_range_ < 0.0)
      min_range_ = (sensor_model_ == "camera") ? 0.4 : 0.2;
    if (max_range_ < 0.0)
      max_range_ = (sensor_model_ == "camera") ? 6.0 : 40.0;
    applySensorExtentDefaults();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    sub_ = create_subscription<octomap_msgs::msg::Octomap>(input_topic_, rclcpp::QoS(1).transient_local(), std::bind(&FrontierSegmentationNode::mapCallback, this, std::placeholders::_1));
    const auto output_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    frontier_pub_ = create_publisher<frontier_segmentation::msg::FrontierArray>(frontier_topic_, output_qos);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(viz_topic_, output_qos);

    // 1. Initialize and Register the Node with metadata
    nlohmann::json node_info;
    node_info["description"] = "Frontier Segmentation Process";
    node_info["node_type"] = "navigation_processor";
    
    // Pass the rclcpp Node pointer and the node name string
    bool success = ros2top::register_node(this->get_name(), node_info);

    if (success) {
        RCLCPP_INFO(get_logger(), "Successfully registered with ros2top!");
        
        // 2. Set up a periodic heartbeat timer (ros2top expects updates to stay active)
        ros2top_timer_ = create_wall_timer(
            std::chrono::seconds(1),
            [this]() {
                ros2top::heartbeat(this->get_name());
            }
        );
    } else {
        RCLCPP_WARN(get_logger(), "ros2top registration failed.");
    }

    marker_republish_timer_ = create_wall_timer(std::chrono::seconds(1),[this]()
        {
          if (last_marker_array_)
            marker_pub_->publish(*last_marker_array_);
        });
    RCLCPP_INFO(get_logger(), "Listening to %s; sensor_model=%s, min/max range=%.2f/%.2f m, positive_z_is_up=%s", input_topic_.c_str(), sensor_model_.c_str(), min_range_, max_range_, positive_z_is_up_ ? "true" : "false");
  }

  ~FrontierSegmentationNode() override
  {
    // Cleanly notify ros2top that this process is shutting down
    ros2top::unregister_node(this->get_name());
  }

private:
  void applySensorExtentDefaults()
  {
    if (!use_fov_extent_limits_)
      return;
    const double r = std::min(design_range_, max_range_);
    if (sensor_model_ == "camera")
    {
      max_width_ = std::min(max_width_, 2.0 * r * std::tan(camera_hfov_ * M_PI / 360.0));
      max_height_ = std::min(max_height_, 2.0 * r * std::tan(camera_vfov_ * M_PI / 360.0));
    }
    else
    {
      // Full 360-degree azimuth: diameter is bounded by 2r, not tan(FOV/2).
      max_width_ = std::min(max_width_, 2.0 * r);
      max_height_ = std::min(max_height_, 2.0 * r * std::tan(lidar_vfov_ * M_PI / 360.0));
    }
  }

  void mapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg)
  {
    std::unique_ptr<octomap::AbstractOcTree> abstract_tree(octomap_msgs::msgToMap(*msg));
    auto *tree = dynamic_cast<octomap::OcTree *>(abstract_tree.get());
    if (!tree)
    {
      RCLCPP_ERROR(get_logger(), "Input Octomap is not an octomap::OcTree (id=%s)", msg->id.c_str());
      return;
    }
    if (msg->header.frame_id != world_frame_)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "OctoMap frame '%s' does not match configured world frame '%s'. Skipping map",
                           msg->header.frame_id.c_str(), world_frame_.c_str());
      return;
    }

    const double res = tree->getResolution();
    // The robot position only filters new frontier voxels by sensor range, so the latest
    // transform is good enough when none exists at the map's stamp.
    geometry_msgs::msg::TransformStamped tf;
    bool have_tf = false;
    std::string tf_error;
    try
    {
      tf = tf_buffer_->lookupTransform(world_frame_, robot_frame_, msg->header.stamp,
                                       rclcpp::Duration::from_seconds(0.1));
      have_tf = true;
    }
    catch (const tf2::TransformException &e)
    {
      tf_error = e.what();
    }
    if (!have_tf)
    {
      try
      {
        tf = tf_buffer_->lookupTransform(world_frame_, robot_frame_, tf2::TimePointZero);
        have_tf = true;
      }
      catch (const tf2::TransformException &e)
      {
        tf_error = e.what();
      }
    }
    if (!have_tf)
    {
      // The first (latched) map usually arrives before the TF listener has received anything.
      if (std::chrono::steady_clock::now() - start_time_ < std::chrono::seconds(5))
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Waiting for TF '%s' -> '%s' before segmenting maps",
                             world_frame_.c_str(), robot_frame_.c_str());
      else
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Cannot transform robot frame '%s' into world frame '%s'. Skipping map: %s",
                             robot_frame_.c_str(), world_frame_.c_str(), tf_error.c_str());
      return;
    }
    const Eigen::Vector3d robot_origin(tf.transform.translation.x,
                                       tf.transform.translation.y,
                                       tf.transform.translation.z);

    // Expand the OctoMap into a consistent native-resolution voxel snapshot.
    std::unordered_map<int64_t, Voxel> current_voxels;
    std::unordered_map<int64_t, bool> current_occupancy;
    std::unordered_set<int64_t> known_voxels;
    for (auto it = tree->begin_leafs(tree->getTreeDepth()), end = tree->end_leafs(); it != end; ++it)
    {
      const bool occupied = it->getOccupancy() >= occupied_threshold_;
      const octomap::OcTreeKey base = it.getIndexKey();
      const int cells_per_axis = std::max(1, static_cast<int>(std::llround(it.getSize() / res)));
      for (int ox = 0; ox < cells_per_axis; ++ox)
        for (int oy = 0; oy < cells_per_axis; ++oy)
          for (int oz = 0; oz < cells_per_axis; ++oz)
          {
            const int ix = static_cast<int>(base[0]) + ox;
            const int iy = static_cast<int>(base[1]) + oy;
            const int iz = static_cast<int>(base[2]) + oz;
            if (ix < 0 || iy < 0 || iz < 0 ||
                ix > std::numeric_limits<octomap::key_type>::max() ||
                iy > std::numeric_limits<octomap::key_type>::max() ||
                iz > std::numeric_limits<octomap::key_type>::max())
              continue;

            const octomap::OcTreeKey key(static_cast<octomap::key_type>(ix),
                                         static_cast<octomap::key_type>(iy),
                                         static_cast<octomap::key_type>(iz));
            const auto point = tree->keyToCoord(key);
            const int64_t packed = key3(ix, iy, iz);
            current_voxels.emplace(packed, Voxel{ix, iy, iz, point.x(), point.y(), point.z()});
            current_occupancy.emplace(packed, occupied);
            known_voxels.insert(packed);
          }
    }

    const bool first_map = !have_previous_octomap_;
    const bool resolution_changed = first_map || std::abs(previous_resolution_ - res) > 1e-9;
    std::unordered_set<int64_t> changed_keys;

    if (resolution_changed)
    {
      for (const auto &[key, voxel] : current_voxels)
        changed_keys.insert(key);

      published_segments_.clear();
      published_segment_recent_.clear();
      published_segment_ids_.clear();
    }
    else
    {
      for (const auto &[key, occupied] : current_occupancy)
      {
        const auto old = previous_occupancy_.find(key);
        if (old == previous_occupancy_.end() || old->second != occupied)
          changed_keys.insert(key);
      }

      for (const auto &[key, occupied] : previous_occupancy_)
        if (current_occupancy.find(key) == current_occupancy.end())
          changed_keys.insert(key);
    }

    if (changed_keys.empty())
    {
      previous_voxels_ = std::move(current_voxels);
      previous_occupancy_ = std::move(current_occupancy);
      previous_resolution_ = res;
      have_previous_octomap_ = true;
      return;
    }

    // Frontier status depends on the 26-neighbourhood. A changed map voxel
    // therefore dirties itself and its immediate neighbours.
    std::unordered_set<int64_t> dirty_keys;
    for (const int64_t changed_key : changed_keys)
    {
      const auto now = current_voxels.find(changed_key);
      const auto old = previous_voxels_.find(changed_key);
      const Voxel *center = now != current_voxels.end()
                                ? &now->second
                                : (old != previous_voxels_.end() ? &old->second : nullptr);
      if (!center)
        continue;

      for (int dx = -1; dx <= 1; ++dx)
        for (int dy = -1; dy <= 1; ++dy)
          for (int dz = -1; dz <= 1; ++dz)
            dirty_keys.insert(key3(center->x + dx, center->y + dy, center->z + dz));
    }

    // `recent` describes a change in frontier membership that should trigger
    // viewpoint work. Clear the event flag for this publication; unchanged
    // frontier IDs and geometry remain persistent.
    published_segment_recent_.resize(published_segments_.size(), false);
    std::fill(published_segment_recent_.begin(), published_segment_recent_.end(), false);

    // Validate only existing frontier voxels in the dirty neighbourhood.
    // Invalid voxels are removed in-place; the remaining cluster keeps its ID.
    size_t removed_frontier_voxels = 0;
    std::vector<Component> retained_segments;
    std::vector<bool> retained_recent;
    std::vector<uint32_t> retained_ids;
    retained_segments.reserve(published_segments_.size());
    retained_recent.reserve(published_segments_.size());
    retained_ids.reserve(published_segments_.size());

    for (size_t i = 0; i < published_segments_.size(); ++i)
    {
      Component retained;
      retained.reserve(published_segments_[i].size());
      for (const auto &voxel : published_segments_[i])
      {
        const int64_t key = key3(voxel.x, voxel.y, voxel.z);
        if (dirty_keys.find(key) == dirty_keys.end() ||
            isFrontierVoxel(voxel, *tree, current_occupancy))
        {
          retained.push_back(voxel);
        }
        else
        {
          ++removed_frontier_voxels;
        }
      }

      // A frontier that falls below the configured minimum is no longer useful.
      if (retained.size() >= static_cast<size_t>(min_frontier_voxels_))
      {
        retained_segments.push_back(std::move(retained));
        retained_recent.push_back(false);
        retained_ids.push_back(published_segment_ids_[i]);
      }
    }

    published_segments_ = std::move(retained_segments);
    published_segment_recent_ = std::move(retained_recent);
    published_segment_ids_ = std::move(retained_ids);

    // Rebuild the voxel -> persistent frontier ID index after removals.
    auto buildOwnerMap = [this]()
    {
      std::unordered_map<int64_t, uint32_t> owners;
      size_t count = 0;
      for (const auto &segment : published_segments_)
        count += segment.size();
      owners.reserve(count * 2);
      for (size_t i = 0; i < published_segments_.size(); ++i)
        for (const auto &voxel : published_segments_[i])
          owners[key3(voxel.x, voxel.y, voxel.z)] = published_segment_ids_[i];
      return owners;
    };

    auto owner_map = buildOwnerMap();

    // Detect only frontier voxels in the dirty region. Existing frontier
    // voxels are already represented by owner_map and are not re-segmented.
    std::vector<Voxel> added_frontier_voxels;
    added_frontier_voxels.reserve(dirty_keys.size());
    size_t out_of_range_candidates = 0;
    for (const int64_t key : dirty_keys)
    {
      const auto voxel_it = current_voxels.find(key);
      if (voxel_it == current_voxels.end())
        continue;

      const Voxel &voxel = voxel_it->second;
      if (!isFrontierVoxel(voxel, *tree, current_occupancy))
        continue;

      if (owner_map.find(key) != owner_map.end())
        continue;

      const double range =
          (Eigen::Vector3d(voxel.wx, voxel.wy, voxel.wz) - robot_origin).norm();
      if (range < min_range_ || range > max_range_)
      {
        ++out_of_range_candidates;
        continue;
      }

      added_frontier_voxels.push_back(voxel);
    }

    // Connected components are formed only from genuinely new frontier voxels.
    // Keep even small components here: a small addition may bridge two existing
    // frontiers or extend a cluster that already exceeds the minimum size.
    auto added_components = connectedComponents(added_frontier_voxels, res, false);

    size_t created_segments = 0;
    size_t merged_frontier_count = 0;
    size_t split_frontier_count = 0;

    for (const auto &added_component : added_components)
    {
      if (added_component.empty())
        continue;

      // Find all existing clusters adjacent to this new component. A new
      // component touching multiple clusters merges those clusters.
      owner_map = buildOwnerMap();
      std::unordered_set<uint32_t> adjacent_ids;
      for (const auto &voxel : added_component)
      {
        for (int dx = -1; dx <= 1; ++dx)
          for (int dy = -1; dy <= 1; ++dy)
            for (int dz = -1; dz <= 1; ++dz)
            {
              if (dx == 0 && dy == 0 && dz == 0)
                continue;
              const auto owner = owner_map.find(key3(voxel.x + dx, voxel.y + dy, voxel.z + dz));
              if (owner != owner_map.end())
                adjacent_ids.insert(owner->second);
            }
      }

      if (adjacent_ids.empty())
      {
        // A genuinely new frontier. Top-of-map voxels have already been
        // excluded by isFrontierVoxel(); apply normal geometric splitting.
        std::vector<Component> new_segments;
        for (const auto &component : connectedComponents(added_component, res, false)) {
          if (!hasUnsupportedVoxel(component, known_voxels)) {
            continue;
          }
          splitComponent(component, res, 0, new_segments, known_voxels);
        }

        for (auto &segment : new_segments)
        {
          if (segment.size() < static_cast<size_t>(min_frontier_voxels_))
            continue;
          published_segments_.push_back(std::move(segment));
          published_segment_recent_.push_back(true);
          published_segment_ids_.push_back(next_frontier_id_++);
          ++created_segments;
        }
        continue;
      }

      // Select the persistent ID with the largest existing voxel overlap.
      // Since clusters are disjoint, this is the largest pre-merge cluster.
      uint32_t keeper_id = 0;
      size_t largest_cluster_size = 0;
      Eigen::Vector3d keeper_old_centroid = Eigen::Vector3d::Zero();
      std::vector<size_t> merge_indices;

      for (size_t i = 0; i < published_segment_ids_.size(); ++i)
      {
        if (adjacent_ids.find(published_segment_ids_[i]) == adjacent_ids.end())
          continue;

        merge_indices.push_back(i);
        if (published_segments_[i].size() > largest_cluster_size)
        {
          largest_cluster_size = published_segments_[i].size();
          keeper_id = published_segment_ids_[i];
          keeper_old_centroid = componentCentroid(published_segments_[i]);
        }
      }

      if (merge_indices.empty())
        continue;

      Component merged_voxels;
      merged_voxels.reserve(added_component.size());
      merged_voxels.insert(merged_voxels.end(), added_component.begin(), added_component.end());

      // Remove merged clusters from the persistent arrays, retaining their voxels
      // in a temporary component. Erase from highest index to lowest.
      std::sort(merge_indices.begin(), merge_indices.end(), std::greater<size_t>());
      for (const size_t index : merge_indices)
      {
        merged_voxels.insert(merged_voxels.end(),
                             published_segments_[index].begin(),
                             published_segments_[index].end());
        published_segments_.erase(published_segments_.begin() + static_cast<std::ptrdiff_t>(index));
        published_segment_recent_.erase(
            published_segment_recent_.begin() + static_cast<std::ptrdiff_t>(index));
        published_segment_ids_.erase(
            published_segment_ids_.begin() + static_cast<std::ptrdiff_t>(index));
      }

      if (merge_indices.size() > 1)
        merged_frontier_count += merge_indices.size() - 1;

      // Re-cluster after additions. This also resolves any disconnection that
      // accumulated during earlier removal-only updates.
      std::vector<Component> merged_outputs;
      for (const auto &connected : connectedComponents(merged_voxels, res, false)) {
        if (!hasUnsupportedVoxel(connected, known_voxels)) {
          continue;
        }
        splitComponent(connected, res, 0, merged_outputs, known_voxels);
      }
      if (merged_outputs.empty())
        continue;

      // The component closest to the old keeper centroid retains the keeper ID.
      size_t keeper_output = 0;
      double best_distance = std::numeric_limits<double>::infinity();
      for (size_t i = 0; i < merged_outputs.size(); ++i)
      {
        const double distance =
            (componentCentroid(merged_outputs[i]) - keeper_old_centroid).squaredNorm();
        if (distance < best_distance)
        {
          best_distance = distance;
          keeper_output = i;
        }
      }

      for (size_t i = 0; i < merged_outputs.size(); ++i)
      {
        if (merged_outputs[i].size() < static_cast<size_t>(min_frontier_voxels_))
          continue;
        if (!hasUnsupportedVoxel(merged_outputs[i], known_voxels))
          continue;

        const uint32_t id = (i == keeper_output) ? keeper_id : next_frontier_id_++;
        published_segments_.push_back(std::move(merged_outputs[i]));
        published_segment_recent_.push_back(true);
        published_segment_ids_.push_back(id);
        ++created_segments;
      }

      if (merged_outputs.size() > 1)
        ++split_frontier_count;
    }

    previous_voxels_ = std::move(current_voxels);
    previous_occupancy_ = std::move(current_occupancy);
    previous_resolution_ = res;
    have_previous_octomap_ = true;

    publishResults(*msg, res, published_segments_, published_segment_recent_, published_segment_ids_, known_voxels);

    if (diagnostics_)
      RCLCPP_INFO(get_logger(),
                  "Persistent update: %zu changed map voxels, %zu dirty voxels, "
                  "%zu removed frontier voxels, %zu added frontier voxels, "
                  "%zu out-of-range candidates, %zu created/updated segments, "
                  "%zu merged clusters, %zu split events, %zu total frontiers",
                  changed_keys.size(), dirty_keys.size(), removed_frontier_voxels,
                  added_frontier_voxels.size(), out_of_range_candidates,
                  created_segments, merged_frontier_count, split_frontier_count,
                  published_segments_.size());
  }


  bool isFrontierVoxel(const Voxel &voxel, const octomap::OcTree &tree,
                       const std::unordered_map<int64_t, bool> &occupancy) const
  {
    // A frontier voxel must itself be known and free.
    const int64_t own_key = key3(voxel.x, voxel.y, voxel.z);
    const auto own = occupancy.find(own_key);
    if (own == occupancy.end() || own->second)
      return false;

    // Only unknown neighbours in the same horizontal layer or below count.
    // This rejects frontiers caused exclusively by unknown space above 
    // the voxel, while preserving frontiers that can be explored in XY.
    for (int dx = -1; dx <= 1; ++dx)
      for (int dy = -1; dy <= 1; ++dy)
        for (int dz = -1; dz <= 0; ++dz)
        {
          if (dx == 0 && dy == 0 && dz == 0)
            continue;

          const int nx = voxel.x + dx;
          const int ny = voxel.y + dy;
          const int nz = voxel.z + dz;

          if (nx < 0 || ny < 0 || nz < 0 ||
              nx > std::numeric_limits<octomap::key_type>::max() ||
              ny > std::numeric_limits<octomap::key_type>::max() ||
              nz > std::numeric_limits<octomap::key_type>::max())
            return true;

          const octomap::OcTreeKey neighbour(static_cast<octomap::key_type>(nx),
                                            static_cast<octomap::key_type>(ny),
                                            static_cast<octomap::key_type>(nz));
          if (tree.search(neighbour) == nullptr)
            return true;
        }

    return false;
  }

  Eigen::Vector3d componentCentroid(const Component &component) const
  {
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    if (component.empty())
      return centroid;
    for (const auto &voxel : component)
      centroid += Eigen::Vector3d(voxel.wx, voxel.wy, voxel.wz);
    return centroid / static_cast<double>(component.size());
  }

  bool are26Adjacent(const Voxel &a, const Voxel &b, double res)
  {
    const double tolerance = res * 1e-6;
    const double dx = std::abs(a.wx - b.wx);
    const double dy = std::abs(a.wy - b.wy);
    const double dz = std::abs(a.wz - b.wz);
    return dx <= res + tolerance && dy <= res + tolerance && dz <= res + tolerance && (dx > tolerance || dy > tolerance || dz > tolerance);
  }

  std::vector<Component> connectedComponents(const std::vector<Voxel> &v, double res, bool enforce_min_size = true)
  {
    std::unordered_map<int64_t, size_t> lookup;
    lookup.reserve(v.size() * 2);
    for (size_t i = 0; i < v.size(); ++i)
      lookup[key3(v[i].x, v[i].y, v[i].z)] = i;
    std::vector<uint8_t> seen(v.size(), 0);
    std::vector<Component> out;
    for (size_t i = 0; i < v.size(); ++i)
    {
      if (seen[i])
        continue;
      Component c;
      std::vector<size_t> stack{i};
      seen[i] = 1;
      while (!stack.empty())
      {
        size_t j = stack.back();
        stack.pop_back();
        c.push_back(v[j]);
        for (int dx = -1; dx <= 1; ++dx)
          for (int dy = -1; dy <= 1; ++dy)
            for (int dz = -1; dz <= 1; ++dz)
            {
              if (!dx && !dy && !dz)
                continue;
              auto it = lookup.find(key3(v[j].x + dx, v[j].y + dy, v[j].z + dz));
              if (it != lookup.end() && are26Adjacent(v[j], v[it->second], res) && !seen[it->second])
              {
                seen[it->second] = 1;
                stack.push_back(it->second);
              }
            }
      }
      if (!enforce_min_size || static_cast<int>(c.size()) >= min_frontier_voxels_)
        out.push_back(std::move(c));
    }
    return out;
  }

  bool hasUnsupportedVoxel(const Component &frontier, const std::unordered_set<int64_t> &known_voxels)
  {
    for (const auto &v : frontier)
    {
      const int direction = positive_z_is_up_ ? -1 : 1;
      bool has_known_voxel_below = false;
      for (int distance = 1; distance <= 3; ++distance)
      {
        if (known_voxels.find(key3(v.x, v.y, v.z + direction * distance)) != known_voxels.end())
        {
          has_known_voxel_below = true;
          break;
        }
      }
      if (!has_known_voxel_below)
        return true;
    }
    return false;
  }
  struct Metrics
  {
    double width, height, max_extent, elongation, convexity;
    Eigen::Vector3d axis;
  };
  Metrics metrics(const Component &c, double res)
  {
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (const auto &v : c)
      mean += Eigen::Vector3d(v.wx, v.wy, v.wz);
    mean /= static_cast<double>(c.size());
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    double minx = 1e9, maxx = -1e9, miny = 1e9, maxy = -1e9, minz = 1e9, maxz = -1e9;
    for (const auto &v : c)
    {
      Eigen::Vector3d p(v.wx, v.wy, v.wz);
      cov += (p - mean) * (p - mean).transpose();
      minx = std::min(minx, v.wx);
      maxx = std::max(maxx, v.wx);
      miny = std::min(miny, v.wy);
      maxy = std::max(maxy, v.wy);
      minz = std::min(minz, v.wz);
      maxz = std::max(maxz, v.wz);
    }
    cov /= std::max<size_t>(1, c.size());
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
    Eigen::Vector3d eval = solver.eigenvalues().cwiseMax(0.0);
    Eigen::Vector3d axis = solver.eigenvectors().col(2).normalized();
    // Width is horizontal XY footprint; height is world-Z span. PCA ratio ignores the
    // smallest eigenvalue so a thin but broad frontier surface is not marked elongated.
    double e1 = std::sqrt(eval(2)), e2 = std::sqrt(eval(1));
    double elongation = e1 / std::max(e2, res * 0.5);
    Eigen::Vector3d u = solver.eigenvectors().col(2), w = solver.eigenvectors().col(1);
    std::vector<Eigen::Vector2d> proj;
    proj.reserve(c.size());
    for (const auto &v : c)
    {
      Eigen::Vector3d d(v.wx, v.wy, v.wz);
      d -= mean;
      proj.emplace_back(d.dot(u), d.dot(w));
    }
    auto hull = convexHull(proj);
    double hull_area = polygonArea(hull);
    double convexity = (hull_area > res * res) ? std::min(1.0, c.size() * res * res / hull_area) : 1.0;
    return {std::hypot(maxx - minx + res, maxy - miny + res), maxz - minz + res, std::max({maxx - minx + res, maxy - miny + res, maxz - minz + res}), elongation, convexity, axis};
  }

  void splitComponent(const Component &c, double res, int depth,
                      std::vector<Component> &out,
                      const std::unordered_set<int64_t> &known_voxels)
  {
    if (c.empty())
      return;
    const Metrics m = metrics(c, res);
    bool split = static_cast<int>(c.size()) > max_frontier_voxels_ || m.width > max_width_ || m.height > max_height_ || m.max_extent > max_extent_ || m.elongation > max_elongation_ || m.convexity < min_convexity_;
    if (!split || c.size() < static_cast<size_t>(2 * min_frontier_voxels_) || depth >= 20)
    {
      if (static_cast<int>(c.size()) >= min_frontier_voxels_)
        out.push_back(c);
      return;
    }
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (const auto &v : c)
      mean += Eigen::Vector3d(v.wx, v.wy, v.wz);
    mean /= static_cast<double>(c.size());
    Eigen::Vector3d axis = m.axis;
    std::vector<std::pair<double, Voxel>> ordered;
    ordered.reserve(c.size());
    for (const auto &v : c)
      ordered.push_back({(Eigen::Vector3d(v.wx, v.wy, v.wz) - mean).dot(axis), v});
    std::sort(ordered.begin(), ordered.end(), [](const auto &a, const auto &b)
              { return a.first < b.first; });
    Component left, right;
    const size_t mid = ordered.size() / 2;
    for (size_t i = 0; i < ordered.size(); ++i)
      (i < mid ? left : right).push_back(ordered[i].second);
    if (left.empty() || right.empty())
    {
      if (static_cast<int>(c.size()) >= min_frontier_voxels_)
        out.push_back(c);
      return;
    }
    // PCA splitting can place disconnected pieces in the same half. Re-cluster
    // each half before continuing so every output segment remains connected.
    for (const Component *part : {&left, &right})
    {
      const auto connected_parts = connectedComponents(*part, res);
      for (const auto &connected_part : connected_parts) {
        if (!hasUnsupportedVoxel(connected_part, known_voxels)) {
          continue;
        }
        splitComponent(connected_part, res, depth + 1, out, known_voxels);
      }
    }
  }

  void publishResults(const octomap_msgs::msg::Octomap &map, double res, const std::vector<Component> &segments, const std::vector<bool> &segment_recent, const std::vector<uint32_t> &segment_ids, const std::unordered_set<int64_t> &known_voxels)
  {
    frontier_segmentation::msg::FrontierArray arr;
    arr.header = map.header;
    arr.resolution = static_cast<float>(res);
    visualization_msgs::msg::MarkerArray ma;
    visualization_msgs::msg::Marker clear;
    clear.header = map.header;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    ma.markers.push_back(clear);
    // uint32_t id = 0;
    size_t segment_index = 0;
    for (const auto &c : segments)
    {
      const bool recent = segment_recent.at(segment_index++);
      const uint32_t id = segment_ids.at(segment_index - 1);
      if (c.size() < static_cast<size_t>(min_frontier_voxels_))
        continue;
      frontier_segmentation::msg::Frontier f;
      f.id = id;
      f.recent = recent;
      double sx = 0, sy = 0, sz = 0, minx = 1e9, miny = 1e9, minz = 1e9, maxx = -1e9, maxy = -1e9, maxz = -1e9;
      for (const auto &v : c)
      {
        geometry_msgs::msg::Point p;
        p.x = v.wx;
        p.y = v.wy;
        p.z = v.wz;
        f.voxels.push_back(p);
        sx += v.wx;
        sy += v.wy;
        sz += v.wz;
        minx = std::min(minx, v.wx);
        miny = std::min(miny, v.wy);
        minz = std::min(minz, v.wz);
        maxx = std::max(maxx, v.wx);
        maxy = std::max(maxy, v.wy);
        maxz = std::max(maxz, v.wz);
      }
      f.centroid.x = sx / c.size();
      f.centroid.y = sy / c.size();
      f.centroid.z = sz / c.size();
      f.min_bound.x = minx - res / 2;
      f.min_bound.y = miny - res / 2;
      f.min_bound.z = minz - res / 2;
      f.max_bound.x = maxx + res / 2;
      f.max_bound.y = maxy + res / 2;
      f.max_bound.z = maxz + res / 2;
      const Metrics m = metrics(c, res);
      f.surface_area_proxy = static_cast<float>(c.size() * res * res);
      f.width = m.width;
      f.height = m.height;
      f.elongation_ratio = m.elongation;
      f.convexity_ratio = m.convexity;
      arr.frontiers.push_back(f);

      visualization_msgs::msg::Marker marker;
      marker.header = map.header;
      marker.ns = "frontier_segments";
      marker.id = static_cast<int>(id);
      marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = res;
      marker.scale.y = res;
      marker.scale.z = res;
      const double color_position = static_cast<double>((id % 16) + 2) / 18.0;
      const double intensity = 1.0 - color_position;
      marker.color.r = intensity;
      marker.color.g = intensity;
      marker.color.b = intensity;
      marker.color.a = 1.0;
      if (diagnostics_)
      {
        RCLCPP_INFO(get_logger(),
                    "Approved frontier segment %u: center=(%.3f, %.3f, %.3f), color=(%.3f, %.3f, %.3f)",
                    id, f.centroid.x, f.centroid.y, f.centroid.z,
                    marker.color.r, marker.color.g, marker.color.b);
        const int direction = positive_z_is_up_ ? -1 : 1;
        for (const auto &v : c)
        {
          RCLCPP_INFO(get_logger(),
                      "  voxel grid=(%d, %d, %d), world=(%.3f, %.3f, %.3f), check=(%s, %s, %s)",
                      v.x, v.y, v.z, v.wx, v.wy, v.wz,
                      known_voxels.find(key3(v.x, v.y, v.z + direction * 1)) != known_voxels.end() ? "true" : "false",
                      known_voxels.find(key3(v.x, v.y, v.z + direction * 2)) != known_voxels.end() ? "true" : "false",
                      known_voxels.find(key3(v.x, v.y, v.z + direction * 3)) != known_voxels.end() ? "true" : "false");
        }
      }
      for (const auto &v : c)
      {
        geometry_msgs::msg::Point p;
        p.x = v.wx;
        p.y = v.wy;
        p.z = v.wz;
        marker.points.push_back(p);
      }
      ma.markers.push_back(marker);
      // ++id;
    }
    frontier_pub_->publish(arr);
    marker_pub_->publish(ma);
    last_marker_array_ = std::move(ma);
  }

  std::string input_topic_, sensor_model_, sensor_frame_, world_frame_, robot_frame_, viz_topic_, frontier_topic_;
  double occupied_threshold_, max_width_, max_height_, max_extent_, max_elongation_, min_convexity_;
  double min_range_, max_range_, camera_hfov_, camera_vfov_, lidar_hfov_, lidar_vfov_, lidar_axis_pitch_, design_range_;
  int min_frontier_voxels_, max_frontier_voxels_;
  bool use_fov_extent_limits_, positive_z_is_up_, diagnostics_;
  const std::chrono::steady_clock::time_point start_time_ = std::chrono::steady_clock::now();
  bool have_previous_octomap_ = false;
  double previous_resolution_ = 0.0;
  std::unordered_map<int64_t, Voxel> previous_voxels_;
  std::unordered_map<int64_t, bool> previous_occupancy_;
  std::vector<Component> published_segments_;
  std::vector<bool> published_segment_recent_;
  std::vector<uint32_t> published_segment_ids_;
  uint32_t next_frontier_id_ = 0;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr sub_;
  rclcpp::Publisher<frontier_segmentation::msg::FrontierArray>::SharedPtr frontier_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr marker_republish_timer_;
  std::optional<visualization_msgs::msg::MarkerArray> last_marker_array_;
  rclcpp::TimerBase::SharedPtr ros2top_timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr registration_pub_;
  rclcpp::TimerBase::SharedPtr registration_timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrontierSegmentationNode>());
  rclcpp::shutdown();
  return 0;
}