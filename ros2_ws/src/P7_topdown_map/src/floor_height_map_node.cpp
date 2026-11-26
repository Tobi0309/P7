// 2D floor height map from the full LiDAR OctoMap (/octomap_full, the map DSP plans on).
//
// Floor per (x, y) column as DSP::buildHeightMap computes it (floor_map.hpp); columns without a
// floor take the nearest column's floor.
//
// Output (latched, ~/floor_height_map remapped to /floor_height_map by default):
//   sensor_msgs/PointCloud2, organized: width = nx, height = ny, point index = y * nx + x,
//   one point per column at the cell centre, same frame as the OctoMap.
//   fields: x, y   cell centre [m]
//           z      top surface of the floor voxel [m] (NaN only if no floor was seen at all)
//           observed  1 = floor seen in this column, 0 = smoothed/filled from neighbours
//   Cell size = distance between neighbouring points (= OctoMap resolution).

#include "floor_map.hpp"

#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>

#include <octomap_msgs/msg/octomap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

class FloorHeightMap : public rclcpp::Node {
public:
    FloorHeightMap() : rclcpp::Node("floor_height_map") {
        octomap_topic_ = declare_parameter<std::string>("octomap_topic", "/octomap_full");
        output_topic_ = declare_parameter<std::string>("output_topic", "/floor_height_map");
        update_period_ = declare_parameter<double>("update_period_seconds", 2.0);
        height_kernel_size_ = declare_parameter<int>("height_kernel_size", 3);
        if (height_kernel_size_ % 2 == 0) {
            RCLCPP_WARN(get_logger(), "height_kernel_size should be odd. Incrementing by 1.");
            ++height_kernel_size_;
        }
        if (update_period_ <= 0.0 || height_kernel_size_ < 1) {
            throw std::invalid_argument("Need update_period_seconds > 0 and height_kernel_size >= 1.");
        }

        const auto latched = rclcpp::QoS(1).reliable().transient_local();
        publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, latched);
        // octomap_server publishes latched by default; also accept a volatile publisher.
        const auto store = [this](octomap_msgs::msg::Octomap::SharedPtr msg) { latest_ = std::move(msg); };
        octomap_sub_ = create_subscription<octomap_msgs::msg::Octomap>(octomap_topic_, latched, store);
        octomap_volatile_sub_ = create_subscription<octomap_msgs::msg::Octomap>(
            octomap_topic_, rclcpp::QoS(1).reliable(), store);
        timer_ = create_wall_timer(std::chrono::duration<double>(update_period_), [this]() { process(); });
        RCLCPP_INFO(get_logger(), "Floor height map %s -> %s every %.1f s (median kernel %d).",
                    octomap_topic_.c_str(), output_topic_.c_str(), update_period_, height_kernel_size_);
    }

private:
    void process() {
        if (!latest_) {
            return;
        }
        const auto msg = std::move(latest_);
        latest_.reset();
        const auto begin = std::chrono::steady_clock::now();
        std::unique_ptr<octomap::AbstractOcTree> abstract_tree(octomap_msgs::msgToMap(*msg));
        const auto *tree = dynamic_cast<octomap::OcTree *>(abstract_tree.get());
        if (!tree) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "Received octomap that is not an OcTree.");
            return;
        }
        if (!p7_floor::fromOcTree(*tree, grid_)) {
            return;
        }
        const std::vector<int> raw = p7_floor::rawFloor(grid_);
        const std::vector<int> floor =
            p7_floor::fillFloor(grid_, p7_floor::smoothFloor(grid_, raw, height_kernel_size_));

        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header = msg->header;
        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
                                      sensor_msgs::msg::PointField::FLOAT32, "z", 1,
                                      sensor_msgs::msg::PointField::FLOAT32, "observed", 1,
                                      sensor_msgs::msg::PointField::FLOAT32);
        modifier.resize(static_cast<size_t>(grid_.nx) * grid_.ny);
        cloud.width = grid_.nx;  // organized: one row per y
        cloud.height = grid_.ny;
        cloud.row_step = cloud.width * cloud.point_step;
        cloud.is_dense = false;
        sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z"), observed(cloud, "observed");
        int seen = 0;
        for (int iy = 0; iy < grid_.ny; ++iy) {
            for (int ix = 0; ix < grid_.nx; ++ix, ++x, ++y, ++z, ++observed) {
                const size_t c = grid_.cell(ix, iy);
                *x = static_cast<float>(grid_.min_x + (ix + 0.5) * grid_.res);
                *y = static_cast<float>(grid_.min_y + (iy + 0.5) * grid_.res);
                *z = floor[c] >= 0 ? static_cast<float>(grid_.floorTop(floor[c]))
                                   : std::numeric_limits<float>::quiet_NaN();
                *observed = raw[c] >= 0 ? 1.0f : 0.0f;
                seen += raw[c] >= 0;
            }
        }
        publisher_->publish(cloud);
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count();
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 30000,
                             "Floor map %dx%d cells (%.2f m), floor observed in %d, %.1f ms.", grid_.nx, grid_.ny,
                             grid_.res, seen, ms);
    }

    std::string octomap_topic_, output_topic_;
    double update_period_{2.0};
    int height_kernel_size_{3};
    p7_floor::VoxelGrid grid_;
    octomap_msgs::msg::Octomap::SharedPtr latest_;

    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_volatile_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FloorHeightMap>());
    rclcpp::shutdown();
    return 0;
}
