#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <limits>
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

    // // Create transient local QoS matching ros2top's expectations
    // auto ros2top_qos = rclcpp::QoS(rclcpp::KeepLast(10)).transient_local();
    
    // // 1. Assign to the CLASS MEMBER variable instead of a local variable
    // registration_pub_ = create_publisher<std_msgs::msg::String>("/_ros2top/register", ros2top_qos);

    // // 2. Use a small timer to fire the registration after Zenoh has fully discovered the network
    // registration_timer_ = create_wall_timer(
    //     std::chrono::seconds(1),
    //     [this]() {
    //         auto msg = std_msgs::msg::String();
    //         // Note: Updated the node name string to match your actual node name
    //         msg.data = "{\"node\": \"/frontier_segmentation\", \"pid\": " + std::to_string(getpid()) + "}";
    //         registration_pub_->publish(msg);
            
    //         // Optional: Cancel the timer after successful publication so it only runs once
    //         registration_timer_->cancel();
    //     }
    // );

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
    const double res = tree->getResolution();
    std::vector<Voxel> frontiers;
    std::unordered_set<int64_t> known_voxels;
    std::unordered_set<int64_t> unique;
    if (msg->header.frame_id != world_frame_)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "OctoMap frame '%s' does not match configured world frame '%s'; skipping map",
                           msg->header.frame_id.c_str(), world_frame_.c_str());
      return;
    }

    Eigen::Vector3d robot_origin(0.0, 0.0, 0.0);
    try
    {
      auto tf = tf_buffer_->lookupTransform(world_frame_, robot_frame_, msg->header.stamp, rclcpp::Duration::from_seconds(0.1));

      robot_origin = Eigen::Vector3d(tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z);
    }
    catch (const tf2::TransformException &e)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Cannot transform robot frame '%s' into world frame '%s'; skipping map: %s",
                           robot_frame_.c_str(), world_frame_.c_str(), e.what());
      return;
    }

    std::unordered_set<int64_t> current_known_voxels;
    bool new_voxel_in_range = false;
    const bool resolution_changed = have_previous_octomap_ && previous_resolution_ != res;

    // A frontier voxel is known-free and 26-neighbour-adjacent to at least one unknown cell.
    // Expand pruned leaves to native-resolution cells so every published cube has the same
    // size and is centered on the actual OctoMap voxel grid.
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

            octomap::OcTreeKey center(
                static_cast<octomap::key_type>(ix),
                static_cast<octomap::key_type>(iy),
                static_cast<octomap::key_type>(iz));
            const auto point = tree->keyToCoord(center);
            const double x = point.x(), y = point.y(), z = point.z();
            const Voxel voxel{ix, iy, iz, x, y, z};
            const int64_t voxel_key = key3(ix, iy, iz);
            current_known_voxels.insert(voxel_key);
            const double range = (Eigen::Vector3d(x, y, z) - robot_origin).norm();
            const bool in_range = range <= max_range_ && range >= min_range_;
            if (in_range && previous_known_voxels_.find(voxel_key) == previous_known_voxels_.end())
              new_voxel_in_range = true;
            known_voxels.insert(voxel_key);
            if (occupied)
              continue;
            if (!in_range)
              continue;

            bool unknown_neighbor = false;
            for (int dx = -1; dx <= 1 && !unknown_neighbor; ++dx)
              for (int dy = -1; dy <= 1 && !unknown_neighbor; ++dy)
                for (int dz = -1; dz <= 1 && !unknown_neighbor; ++dz)
                {
                  if (dx == 0 && dy == 0 && dz == 0)
                    continue;
                  const int nx = ix + dx, ny = iy + dy, nz = iz + dz;
                  if (nx < 0 || ny < 0 || nz < 0 ||
                      nx > std::numeric_limits<octomap::key_type>::max() ||
                      ny > std::numeric_limits<octomap::key_type>::max() ||
                      nz > std::numeric_limits<octomap::key_type>::max())
                  {
                    unknown_neighbor = true;
                    continue;
                  }
                  const octomap::OcTreeKey neighbor(
                      static_cast<octomap::key_type>(nx),
                      static_cast<octomap::key_type>(ny),
                      static_cast<octomap::key_type>(nz));
                  if (tree->search(neighbor) == nullptr)
                    unknown_neighbor = true;
                }
            if (unknown_neighbor && unique.insert(key3(ix, iy, iz)).second)
              frontiers.push_back(voxel);
          }
    }

    const bool should_update = !have_previous_octomap_ || resolution_changed || new_voxel_in_range;
    previous_known_voxels_ = std::move(current_known_voxels);
    previous_resolution_ = res;
    have_previous_octomap_ = true;
    if (!should_update)
      return;

    auto components = connectedComponents(frontiers, res);
    std::vector<Component> segmented;
    for (const auto &c : components)
    {
      if (!hasUnsupportedVoxel(c, known_voxels))
        continue;
      splitComponent(c, res, 0, segmented);
    }
    segmented.erase(std::remove_if(segmented.begin(), segmented.end(),
                       [this](const Component &c)
                       {
                         return c.size() < static_cast<size_t>(min_frontier_voxels_);
                       }),
        segmented.end());
    segmented.erase(std::remove_if(segmented.begin(), segmented.end(),
                       [this, &known_voxels](const Component &c)
                       {
                         return !hasUnsupportedVoxel(c, known_voxels);
                       }),
        segmented.end());
    publishResults(*msg, res, segmented, known_voxels);
    RCLCPP_INFO(get_logger(), "OctoMap: %zu frontier voxels, %zu connected components, %zu segments", frontiers.size(), components.size(), segmented.size());
  }

  bool are26Adjacent(const Voxel &a, const Voxel &b, double res)
  {
    const double tolerance = res * 1e-6;
    const double dx = std::abs(a.wx - b.wx);
    const double dy = std::abs(a.wy - b.wy);
    const double dz = std::abs(a.wz - b.wz);
    return dx <= res + tolerance && dy <= res + tolerance && dz <= res + tolerance && (dx > tolerance || dy > tolerance || dz > tolerance);
  }

  std::vector<Component> connectedComponents(const std::vector<Voxel> &v, double res)
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
      if (static_cast<int>(c.size()) >= min_frontier_voxels_)
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

  void splitComponent(const Component &c, double res, int depth, std::vector<Component> &out)
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
      for (const auto &connected_part : connected_parts)
        splitComponent(connected_part, res, depth + 1, out);
    }
  }

  void publishResults(const octomap_msgs::msg::Octomap &map, double res, const std::vector<Component> &segments,const std::unordered_set<int64_t> &known_voxels)
  {
    frontier_segmentation::msg::FrontierArray arr;
    arr.header = map.header;
    arr.resolution = static_cast<float>(res);
    visualization_msgs::msg::MarkerArray ma;
    visualization_msgs::msg::Marker clear;
    clear.header = map.header;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    ma.markers.push_back(clear);
    uint32_t id = 0;
    for (const auto &c : segments)
    {
      if (c.size() < static_cast<size_t>(min_frontier_voxels_))
        continue;
      frontier_segmentation::msg::Frontier f;
      f.id = id;
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
      // Use 20 deterministic colors along yellow -> orange -> red.
      const double color_position = static_cast<double>(id % 20) / 19.0;
      std::array<double, 3> rgb{1.0, 1.0 - color_position, 0.0};
      marker.color.r = rgb[0];
      marker.color.g = rgb[1];
      marker.color.b = rgb[2];
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
      ++id;
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
  bool have_previous_octomap_ = false;
  double previous_resolution_ = 0.0;
  std::unordered_set<int64_t> previous_known_voxels_;
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
