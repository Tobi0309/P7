// Projects the camera-filtered octomap (/octomap_binary_filtered) to a 2D top-down map.
//
// Every (x, y) column of the octomap grid is classified:
//   UNKNOWN      no known voxel in the column: the camera has not seen it
//   SEEN         known voxels, but no floor observed (e.g. only air or canopy seen)
//   FLOOR        floor observed (DSP method: lowest occupied voxel with a known-free voxel above)
//   LOW          occupied voxel above the floor, below tree_min_height (bushes, rocks)
//   CANOPY       occupied voxel only above tree_max_height, floor below not observed
//   TREE         occupied voxel between tree_min_height and tree_max_height above the floor (trunk)
// The floor height is median-smoothed with height_kernel_size exactly like DSP::buildHeightMap;
// columns still without a floor take the nearest floor height.
//
// Outputs (all latched):
//   ~/grid  nav_msgs/OccupancyGrid with the class values below (RViz shows it as a map)
//   ~/image sensor_msgs/Image rgb8, +x right / +y up, floor shaded by floor height
//   ~/trees visualization_msgs/MarkerArray, one cylinder per connected TREE cluster
// The same node also runs CameraCoverage (camera_coverage.hpp): ~/coverage, ~/reset_coverage.

#include "camera_coverage.hpp"
#include "floor_map.hpp"

#include <octomap/OcTree.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

enum Class : int8_t {
    UNKNOWN = -1,
    FLOOR = 0,
    SEEN = 10,
    CANOPY = 40,
    LOW = 70,
    TREE = 100,
};

using Rgb = std::array<uint8_t, 3>;
using p7_floor::V_OCCUPIED;
using p7_floor::V_UNKNOWN;

}  // namespace

class TopdownMap : public rclcpp::Node {
public:
    TopdownMap() : rclcpp::Node("topdown_map") {
        octomap_topic_ = declare_parameter<std::string>("octomap_topic", "/octomap_binary_filtered");
        update_period_ = declare_parameter<double>("update_period_seconds", 2.0);
        height_kernel_size_ = declare_parameter<int>("height_kernel_size", 3);
        tree_min_height_ = declare_parameter<double>("tree_min_height", 1.0);
        tree_max_height_ = declare_parameter<double>("tree_max_height", 4.0);
        min_tree_cells_ = declare_parameter<int>("min_tree_cells", 1);
        if (height_kernel_size_ % 2 == 0) {
            RCLCPP_WARN(get_logger(), "height_kernel_size should be odd. Incrementing by 1.");
            ++height_kernel_size_;
        }
        if (update_period_ <= 0.0 || tree_min_height_ < 0.0 || tree_max_height_ <= tree_min_height_) {
            throw std::invalid_argument("Need update_period_seconds > 0 and 0 <= tree_min_height < tree_max_height.");
        }

        const auto latched = rclcpp::QoS(1).reliable().transient_local();
        grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("~/grid", latched);
        image_pub_ = create_publisher<sensor_msgs::msg::Image>("~/image", latched);
        trees_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/trees", latched);
        // octomap_server publishes latched by default. A transient-local subscription also gets
        // the map published before this node started, but cannot match a volatile publisher, so
        // subscribe both ways; only the newest message is kept.
        const auto store = [this](octomap_msgs::msg::Octomap::SharedPtr msg) { latest_ = std::move(msg); };
        octomap_sub_ = create_subscription<octomap_msgs::msg::Octomap>(octomap_topic_, latched, store);
        octomap_volatile_sub_ = create_subscription<octomap_msgs::msg::Octomap>(
            octomap_topic_, rclcpp::QoS(1).reliable(), store);
        timer_ = create_wall_timer(std::chrono::duration<double>(update_period_), [this]() { process(); });
        RCLCPP_INFO(get_logger(), "Top-down map from %s every %.1f s, trees %.2f-%.2f m above floor.",
                    octomap_topic_.c_str(), update_period_, tree_min_height_, tree_max_height_);
        coverage_ = std::make_unique<CameraCoverage>(*this);
    }

private:
    void process() {
        if (!latest_) {
            return;
        }
        const auto msg = std::move(latest_);
        latest_.reset();
        std::unique_ptr<octomap::AbstractOcTree> abstract_tree(octomap_msgs::msgToMap(*msg));
        const auto *tree = dynamic_cast<octomap::OcTree *>(abstract_tree.get());
        if (!tree || tree->size() == 0) {
            return;
        }

        // Grid layout and floor exactly as DSP builds them (floor_map.hpp).
        if (!p7_floor::fromOcTree(*tree, g_)) {
            return;
        }

        const std::vector<int> raw_floor = p7_floor::rawFloor(g_);
        // Every column has a floor afterwards, unless no floor was seen at all (then -1).
        const std::vector<int> floor =
            p7_floor::fillFloor(g_, p7_floor::smoothFloor(g_, raw_floor, height_kernel_size_));
        const auto classes = classify(raw_floor, floor);

        publishGrid(*msg, classes);
        publishImage(*msg, classes, floor);
        publishTrees(*msg, classes, floor);
    }

    size_t voxel(int x, int y, int z) const { return g_.voxel(x, y, z); }
    size_t cell(int x, int y) const { return g_.cell(x, y); }

    std::vector<int8_t> classify(const std::vector<int> &raw_floor, const std::vector<int> &floor) const {
        std::vector<int8_t> classes(static_cast<size_t>(g_.nx) * g_.ny, UNKNOWN);
        for (int y = 0; y < g_.ny; ++y) {
            for (int x = 0; x < g_.nx; ++x) {
                const size_t c = cell(x, y);
                bool known = false;
                bool tree = false, low = false, canopy = false;
                const int f = floor[c];
                for (int z = 0; z < g_.nz; ++z) {
                    const uint8_t v = g_.voxels[voxel(x, y, z)];
                    known = known || v != V_UNKNOWN;
                    if (v != V_OCCUPIED || f < 0 || z <= f || z == raw_floor[c]) {
                        continue;
                    }
                    // Height of the voxel centre above the top of the floor voxel.
                    const double height = (z - f - 0.5) * g_.res;
                    if (height < tree_min_height_) {
                        low = true;
                    } else if (height <= tree_max_height_) {
                        tree = true;
                    } else {
                        canopy = true;
                    }
                }
                if (!known) {
                    continue;
                }
                // Observed floor wins over canopy: the ground under the crowns was seen by the camera.
                classes[c] = tree ? TREE : low ? LOW : raw_floor[c] >= 0 ? FLOOR : canopy ? CANOPY : SEEN;
            }
        }
        return classes;
    }

    void publishGrid(const octomap_msgs::msg::Octomap &msg, const std::vector<int8_t> &classes) {
        nav_msgs::msg::OccupancyGrid grid;
        grid.header = msg.header;
        grid.info.map_load_time = msg.header.stamp;
        grid.info.resolution = static_cast<float>(g_.res);
        grid.info.width = g_.nx;
        grid.info.height = g_.ny;
        grid.info.origin.position.x = g_.min_x;
        grid.info.origin.position.y = g_.min_y;
        grid.info.origin.position.z = 0.0;
        grid.info.origin.orientation.w = 1.0;
        grid.data.assign(classes.begin(), classes.end());
        grid_pub_->publish(grid);
    }

    void publishImage(const octomap_msgs::msg::Octomap &msg, const std::vector<int8_t> &classes,
                      const std::vector<int> &floor) {
        int lowest = std::numeric_limits<int>::max(), highest = std::numeric_limits<int>::min();
        for (size_t c = 0; c < classes.size(); ++c) {
            if (classes[c] == FLOOR && floor[c] >= 0) {
                lowest = std::min(lowest, floor[c]);
                highest = std::max(highest, floor[c]);
            }
        }
        sensor_msgs::msg::Image image;
        image.header = msg.header;
        image.width = g_.nx;
        image.height = g_.ny;
        image.encoding = "rgb8";
        image.step = 3 * g_.nx;
        image.data.resize(static_cast<size_t>(image.step) * g_.ny);
        for (int y = 0; y < g_.ny; ++y) {
            const int row = g_.ny - 1 - y;  // +y up in the image
            for (int x = 0; x < g_.nx; ++x) {
                const size_t c = cell(x, y);
                Rgb rgb = colour(classes[c]);
                if (classes[c] == FLOOR && highest > lowest) {
                    const int h = floor[c];
                    const double t = static_cast<double>(h - lowest) / (highest - lowest);
                    const double shade = 0.7 + 0.5 * t;  // higher ground is lighter
                    for (auto &channel : rgb) {
                        channel = static_cast<uint8_t>(std::min(255.0, channel * shade));
                    }
                }
                std::copy(rgb.begin(), rgb.end(), image.data.begin() + row * image.step + 3 * x);
            }
        }
        image_pub_->publish(image);
    }

    static Rgb colour(int8_t c) {
        switch (c) {
            case FLOOR: return {150, 190, 110};
            case SEEN: return {165, 175, 195};
            case CANOPY: return {70, 125, 75};
            case LOW: return {225, 185, 70};
            case TREE: return {140, 70, 35};
            default: return {45, 45, 50};
        }
    }

    void publishTrees(const octomap_msgs::msg::Octomap &msg, const std::vector<int8_t> &classes,
                      const std::vector<int> &floor) {
        visualization_msgs::msg::MarkerArray markers;
        visualization_msgs::msg::Marker clear;
        clear.header = msg.header;
        clear.action = visualization_msgs::msg::Marker::DELETEALL;
        markers.markers.push_back(clear);

        std::vector<uint8_t> visited(classes.size(), 0);
        std::vector<std::pair<int, int>> stack;
        int id = 0;
        for (int y = 0; y < g_.ny; ++y) {
            for (int x = 0; x < g_.nx; ++x) {
                if (classes[cell(x, y)] != TREE || visited[cell(x, y)]) {
                    continue;
                }
                // 8-connected cluster of trunk cells = one tree.
                double sum_x = 0.0, sum_y = 0.0, sum_floor = 0.0;
                int count = 0, floor_count = 0;
                stack.assign(1, {x, y});
                visited[cell(x, y)] = 1;
                while (!stack.empty()) {
                    const auto [cx, cy] = stack.back();
                    stack.pop_back();
                    sum_x += cx;
                    sum_y += cy;
                    ++count;
                    if (floor[cell(cx, cy)] >= 0) {
                        sum_floor += floor[cell(cx, cy)];
                        ++floor_count;
                    }
                    for (int dy = -1; dy <= 1; ++dy) {
                        for (int dx = -1; dx <= 1; ++dx) {
                            const int nxp = cx + dx, nyp = cy + dy;
                            if (nxp < 0 || nyp < 0 || nxp >= g_.nx || nyp >= g_.ny) {
                                continue;
                            }
                            const size_t n = cell(nxp, nyp);
                            if (classes[n] == TREE && !visited[n]) {
                                visited[n] = 1;
                                stack.push_back({nxp, nyp});
                            }
                        }
                    }
                }
                if (count < min_tree_cells_) {
                    continue;
                }
                const double floor_voxel = floor_count ? sum_floor / floor_count : 0.0;
                const double ground_z = g_.min_z + (floor_voxel + 1.0) * g_.res;
                const double radius = std::max(0.5 * g_.res, std::sqrt(count / M_PI) * g_.res);
                visualization_msgs::msg::Marker m;
                m.header = msg.header;
                m.ns = "trees";
                m.id = id++;
                m.type = visualization_msgs::msg::Marker::CYLINDER;
                m.action = visualization_msgs::msg::Marker::ADD;
                m.pose.position.x = g_.min_x + (sum_x / count + 0.5) * g_.res;
                m.pose.position.y = g_.min_y + (sum_y / count + 0.5) * g_.res;
                m.pose.position.z = ground_z + 0.5 * tree_max_height_;
                m.pose.orientation.w = 1.0;
                m.scale.x = m.scale.y = 2.0 * radius;
                m.scale.z = tree_max_height_;
                m.color.r = 0.55f;
                m.color.g = 0.27f;
                m.color.b = 0.14f;
                m.color.a = 0.8f;
                markers.markers.push_back(m);
            }
        }
        trees_pub_->publish(markers);
    }

    std::string octomap_topic_;
    double update_period_{2.0};
    int height_kernel_size_{3};
    double tree_min_height_{1.0};
    double tree_max_height_{4.0};
    int min_tree_cells_{1};

    p7_floor::VoxelGrid g_;
    octomap_msgs::msg::Octomap::SharedPtr latest_;

    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_volatile_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr trees_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<CameraCoverage> coverage_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TopdownMap>());
    rclcpp::shutdown();
    return 0;
}
