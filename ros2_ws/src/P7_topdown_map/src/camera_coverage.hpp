// Camera coverage: accumulates, per (x, y) cell in the world frame, how long the camera has
// observed it. Input is /cloud_registered_body_fov_filtered (LiDAR points inside the camera image,
// FAST-LIO `body` frame). Every scan adds its duration once to each cell it hits, so the value
// is "seconds in view", independent of how many points land in a cell.
//
// Output (latched, every publish_period): ~/coverage sensor_msgs/PointCloud2 with one point per
// observed cell: x, y (cell centre), z (highest observed point), intensity (seconds in view).
// ~/reset_coverage (std_srvs/Trigger) clears the accumulation.

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>

class CameraCoverage {
public:
    explicit CameraCoverage(rclcpp::Node &node) : node_(node) {
        enabled_ = node.declare_parameter<bool>("coverage.enabled", true);
        cloud_topic_ = node.declare_parameter<std::string>("coverage.cloud_topic", "/cloud_registered_body_fov_filtered");
        world_frame_ = node.declare_parameter<std::string>("coverage.world_frame", "camera_init");
        resolution_ = node.declare_parameter<double>("coverage.resolution", 0.2);
        publish_period_ = node.declare_parameter<double>("coverage.publish_period_seconds", 2.0);
        nominal_scan_ = node.declare_parameter<double>("coverage.nominal_scan_seconds", 0.1);
        max_scan_gap_ = node.declare_parameter<double>("coverage.max_scan_gap_seconds", 0.5);
        min_z_ = node.declare_parameter<double>("coverage.min_z", -1.0e9);
        max_z_ = node.declare_parameter<double>("coverage.max_z", 1.0e9);
        tf_timeout_ = node.declare_parameter<double>("coverage.tf_timeout_seconds", 0.05);
        if (!enabled_) {
            return;
        }
        if (resolution_ <= 0.0 || publish_period_ <= 0.0 || nominal_scan_ <= 0.0 || max_scan_gap_ <= 0.0) {
            throw std::invalid_argument("coverage.resolution, publish_period_seconds and scan times must be positive.");
        }

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node.get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, &node, true);
        publisher_ = node.create_publisher<sensor_msgs::msg::PointCloud2>(
            "~/coverage", rclcpp::QoS(1).reliable().transient_local());
        subscription_ = node.create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_topic_, rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) { cloudCb(*msg); });
        reset_service_ = node.create_service<std_srvs::srv::Trigger>(
            "~/reset_coverage",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                cells_.clear();
                last_stamp_ = -1.0;
                dirty_ = true;
                response->success = true;
                response->message = "Camera coverage cleared.";
                RCLCPP_INFO(node_.get_logger(), "Camera coverage cleared.");
            });
        timer_ = node.create_wall_timer(std::chrono::duration<double>(publish_period_), [this]() { publish(); });
        RCLCPP_INFO(node.get_logger(), "Camera coverage from %s in %s, %.2f m cells.", cloud_topic_.c_str(),
                    world_frame_.c_str(), resolution_);
    }

private:
    struct Cell {
        float seconds{0.0f};
        float top{-std::numeric_limits<float>::infinity()};
    };

    static int64_t key(int32_t ix, int32_t iy) {
        return (static_cast<int64_t>(ix) << 32) | static_cast<uint32_t>(iy);
    }

    void cloudCb(const sensor_msgs::msg::PointCloud2 &msg) {
        if (msg.width * msg.height == 0) {
            return;
        }
        tf2::Transform transform;
        try {
            const auto t = tf_buffer_->lookupTransform(world_frame_, msg.header.frame_id, msg.header.stamp,
                                                       rclcpp::Duration::from_seconds(tf_timeout_));
            const auto &r = t.transform.rotation;
            const auto &p = t.transform.translation;
            transform = tf2::Transform(tf2::Quaternion(r.x, r.y, r.z, r.w), tf2::Vector3(p.x, p.y, p.z));
        } catch (const tf2::TransformException &e) {
            RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000, "Coverage: no TF %s -> %s: %s",
                                 msg.header.frame_id.c_str(), world_frame_.c_str(), e.what());
            return;
        }

        // Duration this scan represents (cloud stamp difference, bounded).
        const double stamp = rclcpp::Time(msg.header.stamp).seconds();
        double dt = nominal_scan_;
        if (last_stamp_ >= 0.0) {
            const double diff = stamp - last_stamp_;
            dt = diff > 0.0 && diff <= max_scan_gap_ ? diff : nominal_scan_;
        }
        last_stamp_ = stamp;

        std::unordered_map<int64_t, float> hit;  // cell -> highest point in this scan
        sensor_msgs::PointCloud2ConstIterator<float> x(msg, "x"), y(msg, "y"), z(msg, "z");
        for (; x != x.end(); ++x, ++y, ++z) {
            if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {
                continue;
            }
            const tf2::Vector3 w = transform * tf2::Vector3(*x, *y, *z);
            if (w.z() < min_z_ || w.z() > max_z_) {
                continue;
            }
            const auto ix = static_cast<int32_t>(std::floor(w.x() / resolution_));
            const auto iy = static_cast<int32_t>(std::floor(w.y() / resolution_));
            auto [it, inserted] = hit.try_emplace(key(ix, iy), static_cast<float>(w.z()));
            if (!inserted) {
                it->second = std::max(it->second, static_cast<float>(w.z()));
            }
        }
        for (const auto &[k, top] : hit) {
            Cell &cell = cells_[k];
            cell.seconds += static_cast<float>(dt);
            cell.top = std::max(cell.top, top);
        }
        dirty_ = dirty_ || !hit.empty();
    }

    void publish() {
        if (!dirty_) {
            return;
        }
        dirty_ = false;
        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header.frame_id = world_frame_;
        cloud.header.stamp = node_.now();
        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
                                      sensor_msgs::msg::PointField::FLOAT32, "z", 1,
                                      sensor_msgs::msg::PointField::FLOAT32, "intensity", 1,
                                      sensor_msgs::msg::PointField::FLOAT32);
        modifier.resize(cells_.size());
        sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z"), v(cloud, "intensity");
        for (const auto &[k, cell] : cells_) {
            const auto ix = static_cast<int32_t>(k >> 32);
            const auto iy = static_cast<int32_t>(static_cast<uint32_t>(k & 0xffffffffu));
            *x = static_cast<float>((ix + 0.5) * resolution_);
            *y = static_cast<float>((iy + 0.5) * resolution_);
            *z = cell.top;
            *v = cell.seconds;
            ++x, ++y, ++z, ++v;
        }
        publisher_->publish(cloud);
    }

    rclcpp::Node &node_;
    bool enabled_{true};
    std::string cloud_topic_, world_frame_;
    double resolution_{0.2}, publish_period_{2.0}, nominal_scan_{0.1}, max_scan_gap_{0.5};
    double min_z_{-1.0e9}, max_z_{1.0e9}, tf_timeout_{0.05};
    double last_stamp_{-1.0};
    bool dirty_{false};
    std::unordered_map<int64_t, Cell> cells_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
    rclcpp::TimerBase::SharedPtr timer_;
};
