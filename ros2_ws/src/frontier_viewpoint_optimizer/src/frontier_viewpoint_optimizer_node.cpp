#include "frontier_viewpoint_optimizer/frontier_viewpoint_optimizer.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <octomap/AbstractOcTree.h>
#include <octomap_msgs/conversions.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <nlohmann/json.hpp>
#include <ros2top/ros2top.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <numeric>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <tuple>
#include <utility>

namespace frontier_viewpoint_optimizer {
namespace {

constexpr double PI = 3.14159265358979323846;
using Vector = Eigen::Vector3d;

double radians(double angle) {
    return angle * PI / 180.0;
}

double wrapped(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

Vector asVector(const geometry_msgs::msg::Point &point) {
    return Vector(point.x, point.y, point.z);
}

geometry_msgs::msg::Point asPoint(const Vector &vector) {
    geometry_msgs::msg::Point point;
    point.x = vector.x();
    point.y = vector.y();
    point.z = vector.z();
    return point;
}

double pointDistance(const geometry_msgs::msg::Point &a,
                     const geometry_msgs::msg::Point &b) {
    return (asVector(a) - asVector(b)).norm();
}

uint64_t octomapMessageHash(const octomap_msgs::msg::Octomap &message) {
    uint64_t hash = 14695981039346656037ULL;
    auto mix = [&](uint8_t byte) {
        hash ^= byte;
        hash *= 1099511628211ULL;
    };
    for (const auto value : message.data) {
        mix(static_cast<uint8_t>(value));
    }
    for (const unsigned char value : message.header.frame_id) {
        mix(value);
    }
    for (const unsigned char value : message.id) {
        mix(value);
    }
    const auto *bytes = reinterpret_cast<const unsigned char *>(
        &message.resolution);
    for (std::size_t i = 0; i < sizeof(message.resolution); ++i) {
        mix(bytes[i]);
    }
    mix(static_cast<uint8_t>(message.binary));
    return hash;
}

int boundedInt(int value, int lower, int upper) {
    return std::clamp(value, lower, upper);
}

bool sameKey(const octomap::OcTreeKey &a, const octomap::OcTreeKey &b) {
    return a.k[0] == b.k[0] && a.k[1] == b.k[1] && a.k[2] == b.k[2];
}

int manhattanKeyDistance(const octomap::OcTreeKey &a,
                         const octomap::OcTreeKey &b) {
    return std::abs(int(a.k[0]) - int(b.k[0])) +
           std::abs(int(a.k[1]) - int(b.k[1])) +
           std::abs(int(a.k[2]) - int(b.k[2]));
}

}  // namespace

std::size_t FrontierViewpointOptimizer::KeyHash::operator()(
    const Key &key) const noexcept {
    std::size_t value = 14695981039346656037ULL;
    for (int axis = 0; axis < 3; ++axis) {
        value ^= static_cast<uint64_t>(key.k[axis]);
        value *= 1099511628211ULL;
    }
    return value;
}

bool FrontierViewpointOptimizer::KeyEqual::operator()(
    const Key &a, const Key &b) const noexcept {
    return sameKey(a, b);
}

bool FrontierViewpointOptimizer::Candidate::valid() const noexcept {
    return std::isfinite(score) && score > -1.0e20;
}

FrontierViewpointOptimizer::FrontierViewpointOptimizer()
    : rclcpp::Node("frontier_viewpoint_optimizer_node") {
    frontier_topic_ = declare_parameter<std::string>("frontier_topic", "/frontier_segments");
    map_topic_ = declare_parameter<std::string>("octomap_topic", "/octomap_binary");
    odom_topic_ = declare_parameter<std::string>("odometry_topic", "/Odometry");
    result_topic_ = declare_parameter<std::string>("optimized_topic", "/frontier_viewpoints");
    marker_topic_ = declare_parameter<std::string>(
        "fov_marker_topic", "/frontier_viewpoints/fov_markers");

    expected_resolution_ = declare_parameter<double>("planning_voxel_size", 0.5);
    hfov_ = radians(declare_parameter<double>("horizontal_fov_deg", 360.0));
    vmin_ = radians(declare_parameter<double>("vertical_fov_min_deg", -88.5));
    vmax_ = radians(declare_parameter<double>("vertical_fov_max_deg", 46.5));
    min_range_ = declare_parameter<double>("min_range", 0.2);
    max_range_ = declare_parameter<double>("max_range", 40.0);
    sensor_offset_ = Vector(declare_parameter<double>("sensor_offset_x", 0.0), declare_parameter<double>("sensor_offset_y", 0.0), declare_parameter<double>("sensor_offset_z", 0.0));
    minimum_offset_ = declare_parameter<double>("candidate_minimum_offset", 0.5);
    maximum_offset_ = declare_parameter<double>("candidate_maximum_offset", 7.0);
    anchor_spacing_ = declare_parameter<double>("candidate_anchor_spacing", 1.0);
    vertical_spacing_ = declare_parameter<double>("candidate_vertical_spacing", 1.0);
    shell_step_ = declare_parameter<double>("candidate_shell_step", 0.75);
    min_candidates_ = declare_parameter<int>("candidate_minimum_count", 32);
    max_candidates_ = declare_parameter<int>("candidate_maximum_count", 160);
    max_candidate_pool_ = declare_parameter<int>("candidate_maximum_pool", 3000);
    max_visibility_samples_ = declare_parameter<int>("frontier_visibility_samples", 48);
    yaw_samples_ = declare_parameter<int>("yaw_samples", 8);
    reachable_evaluation_limit_ = declare_parameter<int>("reachable_evaluation_limit", 20);
    reachability_max_distance_ = declare_parameter<double>("reachability_max_distance", 25.0);
    reachability_max_nodes_ = declare_parameter<int>("reachability_max_nodes", 120000);
    reachability_max_milliseconds_ = declare_parameter<int>("reachability_max_build_ms", 1500);
    optimization_worker_count_ = declare_parameter<int>("optimization_worker_count", 2);
    area_weight_ = declare_parameter<double>("frontier_area_weight", 3.0);
    volume_weight_ = declare_parameter<double>("new_volume_weight", 0.7);
    volume_depth_cap_ = declare_parameter<double>("volume_score_depth_cap_m", 2.0);
    unknown_depth_cap_ = declare_parameter<double>("volume_unknown_depth_cap", 6.0);
    volume_ray_step_ = declare_parameter<double>("volume_ray_step", 0.5);
    altitude_weight_ = declare_parameter<double>("altitude_bias_weight", 0.25);
    altitude_reference_ = declare_parameter<double>("altitude_reference_z", 3.0);
    altitude_excess_ = declare_parameter<double>("altitude_excess_scale", 2.0);
    excessive_altitude_penalty_ = declare_parameter<double>("excessive_altitude_penalty_weight", 1.2);
    // Terrain-following height limits from the floor height map. Empty topic: off, and the
    // altitude penalty uses the absolute altitude_reference_z instead.
    floor_height_topic_ = declare_parameter<std::string>("floor_height_topic", "");
    min_height_above_ground_ = declare_parameter<double>("min_height_above_ground", 0.5);
    max_height_above_ground_ = declare_parameter<double>("max_height_above_ground", 5.0);
    preferred_height_above_ground_ = declare_parameter<double>("preferred_height_above_ground", 3.0);
    default_floor_height_ = declare_parameter<double>("default_floor_height", 0.0);
    refinement_enabled_ = declare_parameter<bool>("enable_candidate_refining", true);
    refinement_iterations_ = declare_parameter<int>("refinement_iterations", 2);
    refinement_half_extent_ = declare_parameter<double>("refinement_uav_half_extent", 0.18);
    refinement_margin_ = declare_parameter<double>("refinement_clearance_margin", 0.01);
    refinement_max_fraction_ = declare_parameter<double>("refinement_voxel_fraction", 0.42);
    refinement_yaw_step_ = radians(declare_parameter<double>("refinement_yaw_step_deg", 10.0));
    geometry_tolerance_ = declare_parameter<double>("geometry_change_tolerance", 0.65);
    cached_visibility_min_fraction_ = declare_parameter<double>("cached_visibility_min_fraction", 0.10);
    diagnostics_ = declare_parameter<bool>("enable_diagnostics", true);
    // On demand: idle until a FrontierArray arrives, then solve every frontier on that map and
    // publish the complete result once. Off: continuous incremental optimization.
    on_demand_ = declare_parameter<bool>("on_demand", false);
    draw_fov_markers_ = declare_parameter<bool>("draw_fov_markers", true);
    marker_alpha_ = declare_parameter<double>("fov_marker_alpha", 0.45);
    marker_segments_ = declare_parameter<int>("fov_azimuth_segments", 20);

    // Invalid configuration must be reported explicitly instead of silently changed.
    auto require = [this](bool condition, const std::string &message) {
        if (!condition) {
            throw std::invalid_argument(message);
        }
    };
    require(expected_resolution_ > 0.0, "planning_voxel_size must be positive");
    require(hfov_ > 0.0 && hfov_ <= 2.0 * PI, "horizontal_fov_deg must be in (0, 360]");
    require(vmin_ < vmax_, "vertical FOV minimum must be smaller than maximum");
    require(min_range_ >= 0.0 && max_range_ > min_range_, "Invalid sensor range");
    require(minimum_offset_ >= 0.0 && maximum_offset_ >= minimum_offset_, "Invalid candidate offsets");
    require(anchor_spacing_ > 0.0 && vertical_spacing_ > 0.0 && shell_step_ > 0.0, "Candidate spacing must be positive");
    require(min_candidates_ > 0 && max_candidates_ >= min_candidates_ && max_candidate_pool_ >= max_candidates_, "Invalid candidate count configuration");
    require(max_visibility_samples_ > 0 && yaw_samples_ > 0 && reachable_evaluation_limit_ > 0, "Sampling limits must be positive");
    require(reachability_max_distance_ > 0.0 && reachability_max_nodes_ > 0 && reachability_max_milliseconds_ > 0, "Invalid reachability limits");
    require(optimization_worker_count_ >= 1 && optimization_worker_count_ <= 8, "optimization_worker_count must be between 1 and 8");
    require(refinement_iterations_ >= 0 && refinement_max_fraction_ >= 0.0 && refinement_max_fraction_ < 0.5, "Invalid refinement parameters");
    require(refinement_half_extent_ >= 0.0 && refinement_margin_ >= 0.0, "Invalid refinement clearance");
    require(geometry_tolerance_ >= 0.0 && cached_visibility_min_fraction_ >= 0.0 && cached_visibility_min_fraction_ <= 1.0, "Invalid geometry tolerance / visibility threshold");
    require(marker_alpha_ >= 0.0 && marker_alpha_ <= 1.0 && marker_segments_ >= 8, "Invalid marker configuration");
    require(min_height_above_ground_ < preferred_height_above_ground_ &&
            preferred_height_above_ground_ < max_height_above_ground_,
            "Need min_height_above_ground < preferred_height_above_ground < max_height_above_ground");

    const auto transient_qos = rclcpp::QoS(1).reliable().transient_local();
    frontier_sub_ = create_subscription<FrontierArray>(
        frontier_topic_, transient_qos,
        std::bind(&FrontierViewpointOptimizer::frontierCb, this,
                  std::placeholders::_1));
    map_sub_ = create_subscription<octomap_msgs::msg::Octomap>(
        map_topic_, transient_qos,
        std::bind(&FrontierViewpointOptimizer::mapCb, this,
                  std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, rclcpp::QoS(10),
        std::bind(&FrontierViewpointOptimizer::odomCb, this,
                  std::placeholders::_1));
    if (heightBandEnabled()) {
        floor_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            floor_height_topic_, transient_qos,
            std::bind(&FrontierViewpointOptimizer::floorHeightCb, this,
                      std::placeholders::_1));
        RCLCPP_INFO(get_logger(),
            "Viewpoints limited to %.2f..%.2f m above the floor (%s), preferred %.2f m",
            min_height_above_ground_, max_height_above_ground_,
            floor_height_topic_.c_str(), preferred_height_above_ground_);
    }
    result_pub_ = create_publisher<msg::OptimizedFrontierArray>(
        result_topic_, transient_qos);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        marker_topic_, transient_qos);
    publish_timer_ = create_wall_timer(
        std::chrono::milliseconds(on_demand_ ? 100 : 500),
        std::bind(&FrontierViewpointOptimizer::publishCb, this));

    nlohmann::json metadata = {
        {"description", "Voxel-center 3D frontier viewpoint optimizer"},
        {"version", "0.4.0"},
        {"node_type", "exploration_viewpoint_optimizer"},
        {"topics_subscribed", frontier_topic_ + ", " + map_topic_ + ", " + odom_topic_},
        {"topics_published", result_topic_ + ", " + marker_topic_}
    };
    ros2top_registered_ = ros2top::register_node("/" + std::string(get_name()), metadata);
    heartbeat_timer_ = create_wall_timer(
        std::chrono::seconds(3),
        std::bind(&FrontierViewpointOptimizer::heartbeatCb, this));

    map_worker_ = std::thread(&FrontierViewpointOptimizer::mapWorkerLoop, this);
    validation_worker_ = std::thread(&FrontierViewpointOptimizer::validationWorkerLoop, this);
    for (int i = 0; i < optimization_worker_count_; ++i) {
        optimization_workers_.emplace_back(&FrontierViewpointOptimizer::workerLoop, this);
    }
    RCLCPP_INFO(get_logger(),
        "Optimizer started: native voxel centers (%.2f m), 6-connected OctoMap "
        "reachability; no DSP dependency", expected_resolution_);
    RCLCPP_WARN(get_logger(),
        "The coarse voxel search assumes the entire UAV collision envelope "
        "fits within one planning voxel. Verify this before flight.");
}

FrontierViewpointOptimizer::~FrontierViewpointOptimizer() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stopping_ = true;
    }
    cv_.notify_all();
    validation_cv_.notify_all();
    map_cv_.notify_all();
    if (map_worker_.joinable()) {
        map_worker_.join();
    }
    if (validation_worker_.joinable()) {
        validation_worker_.join();
    }
    for (auto &worker : optimization_workers_) {
        if (worker.joinable()) {
            worker.join();
        }
    }
    if (ros2top_registered_) {
        ros2top::unregister_node("/" + std::string(get_name()));
    }
}

void FrontierViewpointOptimizer::heartbeatCb() {
    if (ros2top_registered_) {
        ros2top::heartbeat("/" + std::string(get_name()));
    }
}

bool FrontierViewpointOptimizer::geometryChanged(
    const Frontier &a, const Frontier &b, double tolerance) {
    if (pointDistance(a.centroid, b.centroid) > tolerance ||
        pointDistance(a.min_bound, b.min_bound) > 1.5 * tolerance ||
        pointDistance(a.max_bound, b.max_bound) > 1.5 * tolerance ||
        std::abs(a.width - b.width) > tolerance ||
        std::abs(a.height - b.height) > tolerance) {
        return true;
    }
    const double count_a = static_cast<double>(a.voxels.size());
    const double count_b = static_cast<double>(b.voxels.size());
    if (std::abs(count_a - count_b) >
        std::max(8.0, 0.3 * std::max(count_a, count_b))) {
        return true;
    }
    return std::abs(a.surface_area_proxy - b.surface_area_proxy) >
           std::max(1.0, 0.3 * std::max(std::abs(a.surface_area_proxy),
                                       std::abs(b.surface_area_proxy)));
}

bool FrontierViewpointOptimizer::readyLocked() const {
    return map_ && have_odom_ && have_frontiers_ &&
           !map_frame_.empty() && map_frame_ == frontier_frame_ &&
           map_frame_ == odom_frame_ &&
           std::abs(map_->getResolution() - expected_resolution_) < 1.0e-5;
}

void FrontierViewpointOptimizer::enqueueLocked(uint32_t id) {
    const auto found = frontiers_.find(id);
    if (found != frontiers_.end() && !found->second.no_reachable && !found->second.attempted && pending_set_.insert(id).second) {
        pending_.push_back(id);
    }
}


void FrontierViewpointOptimizer::enqueueValidationLocked(uint32_t id) {
    const auto found = frontiers_.find(id);
    if (found != frontiers_.end() && found->second.result && found->second.result->valid() && validation_set_.insert(id).second) {
        validation_pending_.push_back(id);
        validation_cv_.notify_one();
    }
}





// Voxel-set overlap is only used to transfer a cached pose when an upstream
// segmentation ID changes. The transferred pose is NOT published until its
// new frontier visibility and current path have both been revalidated.




std::unordered_set<FrontierViewpointOptimizer::Key, FrontierViewpointOptimizer::KeyHash, FrontierViewpointOptimizer::KeyEqual>
FrontierViewpointOptimizer::observationKeys(const Map &map, const Frontier &frontier, const Candidate &candidate) const {
    std::unordered_set<Key, KeyHash, KeyEqual> watched;
    if (!map || !candidate.valid()) {
        return watched;
    }
    Key goal;
    if (toKey(map, candidate.position, goal)) {
        watched.insert(goal);
    }
    const auto samples = sampleFrontier(frontier);
    const std::size_t count = std::min<std::size_t>(12, samples.size());
    const Vector sensor = sensorOrigin(candidate.position, candidate.yaw);
    for (std::size_t index = 0; index < count; ++index) {
        const Vector &target = samples[index * samples.size() / count];
        if (!inFov(target, sensor, candidate.yaw)) {
            continue;
        }
        octomap::KeyRay ray;
        if (map->computeRayKeys(octomap::point3d(sensor.x(), sensor.y(), sensor.z()), octomap::point3d(target.x(), target.y(), target.z()), ray)) {
            for (const Key &key : ray) {
                watched.insert(key);
            }
        }
    }
    return watched;
}

bool FrontierViewpointOptimizer::frontierTouchesChanges(const Map &map, const Frontier &frontier, const std::unordered_set<Key, KeyHash, KeyEqual> &changed) const {
    if (!map || changed.empty()) {
        return false;
    }
    for (const auto &point : frontier.voxels) {
        Key key;
        if (!toKey(map, asVector(point), key)) {
            continue;
        }
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dz = -1; dz <= 1; ++dz) {
                    const int x = int(key.k[0]) + dx;
                    const int y = int(key.k[1]) + dy;
                    const int z = int(key.k[2]) + dz;
                    if (x < 0 || y < 0 || z < 0 || x > 65535 || y > 65535 || z > 65535) {
                        continue;
                    }
                    if (changed.count(Key(static_cast<octomap::key_type>(x), static_cast<octomap::key_type>(y), static_cast<octomap::key_type>(z)))) {
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

void FrontierViewpointOptimizer::compactQueuesLocked() {
    auto compact = [this](std::deque<uint32_t> &queue, const std::unordered_set<uint32_t> &active) {
        std::deque<uint32_t> filtered;
        for (uint32_t id : queue) {
            if (active.count(id) && frontiers_.count(id)) {
                filtered.push_back(id);
            }
        }
        queue.swap(filtered);
    };
    compact(pending_, pending_set_);
    compact(validation_pending_, validation_set_);
}

bool FrontierViewpointOptimizer::endpointOccupied(const Map &map, const Candidate &candidate) const {
    if (!map || !candidate.valid()) {
        return false;
    }
    // Outside the height band (the floor map may have changed): as unusable as occupied.
    if (!inHeightBand(candidate.position)) {
        return true;
    }
    const auto *node = map->search(candidate.position.x(), candidate.position.y(), candidate.position.z());
    return node && map->isNodeOccupied(node);
}

// Returns 1 if at least one sampled known-free voxel is still an XY frontier,
// 2 if the sampled frontier is observed but no longer borders unknown space,
// and 0 if there is insufficient map evidence to make either conclusion.



// Inspect only the existing viewpoint, its observation rays and their sampled
// target frontier cells. Changes elsewhere in the OctoMap do not revalidate it.
// Unknown/free map-window transitions are not proof of an invalid viewpoint.
// Reject a cached observation only when every tested in-FOV ray is
// *definitively blocked* by known occupied space. An unknown ray is not
// considered proof that an old viewpoint can no longer observe its frontier.
bool FrontierViewpointOptimizer::definitelyOccluded(const Map &map, const Frontier &frontier, const Candidate &candidate) const {
    if (!map || !candidate.valid()) {
        return false;
    }
    const auto samples = sampleFrontier(frontier);
    const std::size_t count = std::min<std::size_t>(12, samples.size());
    const Vector origin = sensorOrigin(candidate.position, candidate.yaw);
    std::size_t in_fov = 0;
    for (std::size_t i = 0; i < count; ++i) {
        const Vector &target = samples[i * samples.size() / count];
        if (!inFov(target, origin, candidate.yaw)) {
            continue;
        }
        ++in_fov;
        octomap::KeyRay ray;
        if (!map->computeRayKeys(octomap::point3d(origin.x(), origin.y(), origin.z()), octomap::point3d(target.x(), target.y(), target.z()), ray)) {
            return false;
        }
        Key target_key;
        if (!toKey(map, target, target_key)) {
            return false;
        }
        bool blocked = false;
        for (const Key &key : ray) {
            if (sameKey(key, target_key)) {
                continue;
            }
            const auto *node = map->search(key);
            if (node && map->isNodeOccupied(node)) {
                blocked = true;
                break;
            }
        }
        if (!blocked) {
            return false;
        }
    }
    return in_fov > 0;
}




void FrontierViewpointOptimizer::mapCb(octomap_msgs::msg::Octomap::SharedPtr message) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (on_demand_) {
            // Stored only; decoded when a batch needs it.
            latest_map_message_ = std::move(message);
            if (!startBatchMapLocked()) {
                return;
            }
        } else {
            pending_map_message_ = std::move(message);
        }
    }
    map_cv_.notify_one();
}

void FrontierViewpointOptimizer::mapWorkerLoop() {
    using Occupancy = std::unordered_map<Key, uint8_t, KeyHash, KeyEqual>;
    auto expand = [](const Map &map) {
        Occupancy occupancy;
        if (!map) {
            return occupancy;
        }
        const double resolution = map->getResolution();
        for (auto leaf = map->begin_leafs(map->getTreeDepth()); leaf != map->end_leafs(); ++leaf) {
            const auto base = leaf.getIndexKey();
            const int width = static_cast<int>(std::llround(leaf.getSize() / resolution));
            const uint8_t state = map->isNodeOccupied(*leaf) ? 2 : 1;
            for (int dx = 0; dx < width; ++dx) {
                for (int dy = 0; dy < width; ++dy) {
                    for (int dz = 0; dz < width; ++dz) {
                        const int x = int(base.k[0]) + dx;
                        const int y = int(base.k[1]) + dy;
                        const int z = int(base.k[2]) + dz;
                        if (x > 65535 || y > 65535 || z > 65535) {
                            continue;
                        }
                        occupancy[Key(static_cast<octomap::key_type>(x), static_cast<octomap::key_type>(y), static_cast<octomap::key_type>(z))] = state;
                    }
                }
            }
        }
        return occupancy;
    };
    Occupancy old_occupancy;
    while (true) {
        octomap_msgs::msg::Octomap::SharedPtr message;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            map_cv_.wait(lock, [this]() { return stopping_ || pending_map_message_ != nullptr; });
            if (stopping_) {
                return;
            }
            message = std::move(pending_map_message_);
        }
        const uint64_t hash = octomapMessageHash(*message);
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (have_map_hash_ && hash == map_hash_) {
                if (on_demand_) {
                    installBatchLocked();
                    cv_.notify_all();
                }
                continue;
            }
        }
        std::unique_ptr<octomap::AbstractOcTree> abstract(octomap_msgs::msgToMap(*message));
        if (!abstract) {
            RCLCPP_ERROR(get_logger(), "Failed to deserialize OctoMap");
            continue;
        }
        auto *tree = dynamic_cast<octomap::OcTree *>(abstract.get());
        if (!tree) {
            RCLCPP_ERROR(get_logger(), "OctoMap is not an octomap::OcTree");
            continue;
        }
        abstract.release();
        const Map snapshot(tree);
        if (std::abs(snapshot->getResolution() - expected_resolution_) > 1.0e-5) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "OctoMap resolution %.3f differs from planning voxel %.3f", snapshot->getResolution(), expected_resolution_);
            continue;
        }
        if (on_demand_) {
            std::lock_guard<std::mutex> lock(mutex_);
            map_ = snapshot;
            map_hash_ = hash;
            have_map_hash_ = true;
            map_frame_ = message->header.frame_id;
            ++map_sequence_;
            installBatchLocked();
            cv_.notify_all();
            continue;
        }
        auto next_occupancy = expand(snapshot);
        std::unordered_set<Key, KeyHash, KeyEqual> changed;
        if (old_occupancy.empty()) {
            for (const auto &cell : next_occupancy) {
                changed.insert(cell.first);
            }
        } else {
            for (const auto &cell : next_occupancy) {
                const auto previous = old_occupancy.find(cell.first);
                if (previous == old_occupancy.end() || previous->second != cell.second) {
                    changed.insert(cell.first);
                }
            }
            for (const auto &cell : old_occupancy) {
                if (!next_occupancy.count(cell.first)) {
                    changed.insert(cell.first);
                }
            }
        }
        const std::size_t changed_count = changed.size();
        old_occupancy = std::move(next_occupancy);
        std::size_t affected = 0;
        std::size_t geometry_affected = 0;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            const bool initial = !map_;
            map_ = snapshot;
            map_hash_ = hash;
            have_map_hash_ = true;
            map_frame_ = message->header.frame_id;
            ++map_sequence_;
            last_changed_cells_ = std::move(changed);
            if (!initial) {
                for (auto &pair : frontiers_) {
                    Entry &entry = pair.second;
                    bool watched_dirty = false;
                    for (const Key &key : entry.observation_keys) {
                        if (last_changed_cells_.count(key)) {
                            watched_dirty = true;
                            break;
                        }
                    }
                    if (watched_dirty) {
                        enqueueValidationLocked(pair.first);
                        ++affected;
                    }
                    if (entry.geometry_dirty && frontierTouchesChanges(snapshot, entry.frontier, last_changed_cells_)) {
                        entry.geometry_dirty = false;
                        entry.attempted = false;
                        entry.no_reachable = false;
                        enqueueLocked(pair.first);
                        ++geometry_affected;
                    }
                }
            }
        }
        cv_.notify_all();
        validation_cv_.notify_all();
        if (diagnostics_) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Map delta: %zu changed native cells, %zu affected views, %zu geometry updates; others untouched", changed_count, affected, geometry_affected);
        }
    }
}


void FrontierViewpointOptimizer::floorHeightCb(sensor_msgs::msg::PointCloud2::SharedPtr message) {
    if (message->width < 2 || message->height < 1) {
        return;
    }
    auto floor = std::make_shared<FloorMap>();
    floor->nx = static_cast<int>(message->width);
    floor->ny = static_cast<int>(message->height);
    floor->z.resize(static_cast<std::size_t>(floor->nx) * floor->ny);
    sensor_msgs::PointCloud2ConstIterator<float> x(*message, "x"), y(*message, "y"), z(*message, "z");
    double x_first = 0.0, y_first = 0.0, x_second = 0.0;
    for (std::size_t i = 0; i < floor->z.size(); ++i, ++x, ++y, ++z) {
        if (i == 0) {
            x_first = *x;
            y_first = *y;
        } else if (i == 1) {
            x_second = *x;
        }
        floor->z[i] = *z;
    }
    floor->res = x_second - x_first;
    if (!(floor->res > 1.0e-6)) {
        return;
    }
    floor->x0 = x_first - 0.5 * floor->res;
    floor->y0 = y_first - 0.5 * floor->res;
    const auto previous = std::atomic_load(&floor_);
    const bool changed = !previous || previous->nx != floor->nx || previous->ny != floor->ny ||
        std::abs(previous->x0 - floor->x0) > 1.0e-6 || std::abs(previous->y0 - floor->y0) > 1.0e-6 ||
        previous->z != floor->z;
    std::atomic_store(&floor_, std::shared_ptr<const FloorMap>(std::move(floor)));
    if (!changed) {
        return;
    }
    // The height band moved: recheck every cached viewpoint (out-of-band ones are re-solved).
    {
        std::lock_guard<std::mutex> lock(mutex_);
        for (const auto &pair : frontiers_) {
            if (pair.second.result) {
                enqueueValidationLocked(pair.first);
            }
        }
    }
    validation_cv_.notify_all();
}

double FrontierViewpointOptimizer::floorAt(double x, double y) const {
    const auto floor = std::atomic_load(&floor_);
    if (floor) {
        const int ix = static_cast<int>(std::floor((x - floor->x0) / floor->res));
        const int iy = static_cast<int>(std::floor((y - floor->y0) / floor->res));
        if (ix >= 0 && iy >= 0 && ix < floor->nx && iy < floor->ny) {
            const float z = floor->z[static_cast<std::size_t>(iy) * floor->nx + ix];
            if (std::isfinite(z)) {
                return z;
            }
        }
    }
    return default_floor_height_;
}

bool FrontierViewpointOptimizer::inHeightBand(const Vector &position) const {
    if (!heightBandEnabled()) {
        return true;
    }
    const double height = position.z() - floorAt(position.x(), position.y());
    return height >= min_height_above_ground_ && height <= max_height_above_ground_;
}

double FrontierViewpointOptimizer::altitudeAboveReference(const Vector &position) const {
    if (!heightBandEnabled()) {
        return position.z() - altitude_reference_;
    }
    return position.z() - floorAt(position.x(), position.y()) - preferred_height_above_ground_;
}

void FrontierViewpointOptimizer::odomCb(nav_msgs::msg::Odometry::SharedPtr message) {
    const Vector pose = asVector(message->pose.pose.position);
    if (!pose.allFinite()) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    robot_ = pose;
    have_odom_ = true;
    odom_frame_ = message->header.frame_id;
    if (on_demand_ && !batch_active_) {
        return;
    }
    cv_.notify_all();
    validation_cv_.notify_all();
}


// On demand: hands the stored map to the map worker once it is at least as new as the batch.
bool FrontierViewpointOptimizer::startBatchMapLocked() {
    if (!pending_batch_ || !latest_map_message_ ||
        rclcpp::Time(latest_map_message_->header.stamp) < rclcpp::Time(pending_batch_->header.stamp)) {
        return false;
    }
    pending_map_message_ = latest_map_message_;
    return true;
}

// On demand: replaces all frontiers by the pending batch and queues every one of them. Results of
// earlier batches are discarded, because robot position and map have changed.
void FrontierViewpointOptimizer::installBatchLocked() {
    if (!pending_batch_) {
        return;
    }
    frontiers_.clear();
    pending_.clear();
    pending_set_.clear();
    validation_pending_.clear();
    validation_set_.clear();
    have_frontiers_ = true;
    frontier_frame_ = pending_batch_->header.frame_id;
    ++frontier_sequence_;
    for (const Frontier &frontier : pending_batch_->frontiers) {
        Entry entry;
        entry.frontier = frontier;
        entry.revision = ++global_revision_;
        if (frontiers_.emplace(frontier.id, std::move(entry)).second) {
            enqueueLocked(frontier.id);
        }
    }
    pending_batch_.reset();
    batch_active_ = true;
    RCLCPP_INFO(get_logger(), "Batch started: %zu frontiers", frontiers_.size());
}

void FrontierViewpointOptimizer::frontierCb(FrontierArray::SharedPtr message) {
    if (on_demand_) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            pending_batch_ = std::move(message);
            if (!startBatchMapLocked()) {
                return;  // mapCb continues when the map arrives
            }
        }
        map_cv_.notify_one();
        return;
    }
    std::size_t added = 0;
    std::size_t updated = 0;
    std::size_t removed = 0;
    std::size_t reused = 0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        have_frontiers_ = true;
        frontier_frame_ = message->header.frame_id;
        ++frontier_sequence_;
        std::unordered_set<uint32_t> live;
        for (const Frontier &frontier : message->frontiers) {
            if (!live.insert(frontier.id).second) {
                RCLCPP_WARN(get_logger(), "Duplicate frontier ID %u ignored", frontier.id);
                continue;
            }
            auto found = frontiers_.find(frontier.id);
            if (found == frontiers_.end()) {
                Entry entry;
                entry.frontier = frontier;
                entry.revision = ++global_revision_;
                frontiers_.emplace(frontier.id, std::move(entry));
                enqueueLocked(frontier.id);
                ++added;
                continue;
            }
            Entry &entry = found->second;
            const bool changed = geometryChanged(entry.frontier, frontier, geometry_tolerance_);
            entry.frontier = frontier;
            if (!changed) {
                ++reused;
                continue;
            }
            entry.revision = ++global_revision_;
            const bool spatial_update = frontierTouchesChanges(map_, frontier, last_changed_cells_);
            if (frontier.recent || spatial_update) {
                entry.attempted = false;
                entry.no_reachable = false;
                entry.geometry_dirty = false;
                enqueueLocked(frontier.id);
                if (entry.result) {
                    enqueueValidationLocked(frontier.id);
                }
                ++updated;
            } else {
                // Preserve the cached viewpoint. A geometry revision alone
                // does not imply a new observation region or invalid pose.
                entry.geometry_dirty = true;
            }
        }
        for (auto it = frontiers_.begin(); it != frontiers_.end();) {
            if (!live.count(it->first)) {
                pending_set_.erase(it->first);
                validation_set_.erase(it->first);
                it = frontiers_.erase(it);
                ++removed;
            } else {
                ++it;
            }
        }
        compactQueuesLocked();
    }
    cv_.notify_all();
    validation_cv_.notify_all();
    if (diagnostics_) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Frontiers: current=%zu new=%zu updated=%zu unchanged=%zu removed=%zu;", message->frontiers.size(), added, updated, reused, removed);
    }
}


bool FrontierViewpointOptimizer::candidateStillUseful(
    const Map &map, const Frontier &frontier, const Candidate &candidate) const {
    if (!map || !candidate.valid() || !inHeightBand(candidate.position)) {
        return false;
    }
    const auto samples = sampleFrontier(frontier);
    const std::size_t total = std::min<std::size_t>(12, samples.size());
    if (total == 0) {
        return false;
    }
    const Vector sensor = sensorOrigin(candidate.position, candidate.yaw);
    std::size_t visible_count = 0;
    for (std::size_t index = 0; index < total; ++index) {
        const auto &point = samples[index * samples.size() / total];
        if (inFov(point, sensor, candidate.yaw) && visible(map, sensor, point)) {
            ++visible_count;
        }
    }
    return visible_count > 0 &&
        double(visible_count) / double(total) >= cached_visibility_min_fraction_;
}

bool FrontierViewpointOptimizer::toKey(
    const Map &map, const Vector &position, Key &key) const {
    if (!map || !position.allFinite()) {
        return false;
    }
    return map->coordToKeyChecked(
        octomap::point3d(position.x(), position.y(), position.z()), key);
}

Vector FrontierViewpointOptimizer::keyCenter(
    const Map &map, const Key &key) const {
    const octomap::point3d point = map->keyToCoord(key);
    return Vector(point.x(), point.y(), point.z());
}

bool FrontierViewpointOptimizer::cellFree(
    const Map &map, const Key &key) const {
    const auto *node = map->search(key);
    return node != nullptr && !map->isNodeOccupied(node);
}

// Conservative refinement: every native grid cell intersecting the swept
// axis-aligned UAV box must be observed free. Coarse planning relies on the
// separate assumption that the UAV is fully contained in a single 0.5 m cell.
bool FrontierViewpointOptimizer::boxFree(
    const Map &map, const Vector &lower, const Vector &upper) const {
    constexpr double EPS = 1.0e-5;
    Key first;
    Key last;
    if (!toKey(map, lower + Vector::Constant(EPS), first) ||
        !toKey(map, upper - Vector::Constant(EPS), last)) {
        return false;
    }
    for (uint32_t x = first.k[0]; x <= last.k[0]; ++x) {
        for (uint32_t y = first.k[1]; y <= last.k[1]; ++y) {
            for (uint32_t z = first.k[2]; z <= last.k[2]; ++z) {
                const Key key(static_cast<octomap::key_type>(x),
                              static_cast<octomap::key_type>(y),
                              static_cast<octomap::key_type>(z));
                if (!cellFree(map, key)) {
                    return false;
                }
            }
        }
    }
    return true;
}

bool FrontierViewpointOptimizer::validRefinementMotion(
    const Map &map, const Vector &start, const Vector &end,
    const Key &base_key) const {
    Key start_key;
    Key end_key;
    if (!toKey(map, start, start_key) || !toKey(map, end, end_key) ||
        !sameKey(start_key, base_key) || !sameKey(end_key, base_key)) {
        return false;
    }
    const double half = refinement_half_extent_ + refinement_margin_;
    const Vector lo = start.cwiseMin(end) - Vector::Constant(half);
    const Vector hi = start.cwiseMax(end) + Vector::Constant(half);
    return boxFree(map, lo, hi);
}

FrontierViewpointOptimizer::Navigation
FrontierViewpointOptimizer::buildNavigation(
    const Map &map, const Vector &robot) const {
    Navigation navigation;
    navigation.map = map;
    navigation.robot = robot;
    if (!toKey(map, robot, navigation.origin) ||
        !cellFree(map, navigation.origin)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
            "Robot odometry is not inside a known-free OctoMap voxel. "
            "Check frame alignment and map coverage.");
        return navigation;
    }

    const auto begin = SteadyClock::now();
    const Vector origin = keyCenter(map, navigation.origin);
    const double max_distance_squared =
        reachability_max_distance_ * reachability_max_distance_;
    const std::array<std::array<int, 3>, 6> neighbors{{
        {{1, 0, 0}}, {{-1, 0, 0}}, {{0, 1, 0}},
        {{0, -1, 0}}, {{0, 0, 1}}, {{0, 0, -1}}
    }};
    std::deque<Key> queue;
    navigation.parent.reserve(
        static_cast<std::size_t>(std::min(reachability_max_nodes_, 30000)));
    navigation.parent.emplace(navigation.origin, navigation.origin);
    queue.push_back(navigation.origin);
    navigation.valid = true;

    while (!queue.empty()) {
        if (static_cast<int>(navigation.parent.size()) >= reachability_max_nodes_ ||
            std::chrono::duration_cast<std::chrono::milliseconds>(
                SteadyClock::now() - begin).count() >=
                reachability_max_milliseconds_) {
            navigation.limited = true;
            break;
        }
        const Key current = queue.front();
        queue.pop_front();
        for (const auto &direction : neighbors) {
            std::array<int, 3> next_value{};
            bool valid_key = true;
            for (int axis = 0; axis < 3; ++axis) {
                next_value[axis] = int(current.k[axis]) + direction[axis];
                if (next_value[axis] < 0 ||
                    next_value[axis] >
                        int(std::numeric_limits<octomap::key_type>::max())) {
                    valid_key = false;
                    break;
                }
            }
            if (!valid_key) {
                continue;
            }
            const Key next(static_cast<octomap::key_type>(next_value[0]),
                           static_cast<octomap::key_type>(next_value[1]),
                           static_cast<octomap::key_type>(next_value[2]));
            if (navigation.parent.find(next) != navigation.parent.end()) {
                continue;
            }
            if ((keyCenter(map, next) - origin).squaredNorm() >
                max_distance_squared || !cellFree(map, next)) {
                continue;
            }
            // Paths stay inside the height band (as DSP's), except straight up/down in the
            // robot's own column so a robot outside the band can still get into it.
            if (!inHeightBand(keyCenter(map, next)) &&
                (next.k[0] != navigation.origin.k[0] || next.k[1] != navigation.origin.k[1])) {
                continue;
            }
            navigation.parent.emplace(next, current);
            queue.push_back(next);
        }
    }
    navigation.duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        SteadyClock::now() - begin).count();
    if (diagnostics_) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
            "Shared reachability graph: %zu free voxels, %lld ms%s",
            navigation.parent.size(),
            static_cast<long long>(navigation.duration_ms),
            navigation.limited ? " (search budget reached)" : "");
    }
    return navigation;
}

std::vector<FrontierViewpointOptimizer::Key>
FrontierViewpointOptimizer::pathKeys(
    const Navigation &navigation, const Key &goal) const {
    if (!navigation.valid ||
        navigation.parent.find(goal) == navigation.parent.end()) {
        return {};
    }
    std::vector<Key> path;
    Key current = goal;
    for (std::size_t attempt = 0;
         attempt <= navigation.parent.size(); ++attempt) {
        path.push_back(current);
        if (sameKey(current, navigation.origin)) {
            std::reverse(path.begin(), path.end());
            return path;
        }
        const auto parent = navigation.parent.find(current);
        if (parent == navigation.parent.end()) {
            break;
        }
        current = parent->second;
    }
    return {};
}

bool FrontierViewpointOptimizer::pathStillFree(
    const Map &map, const std::vector<Key> &path) const {
    if (path.empty()) {
        return false;
    }
    for (std::size_t i = 0; i < path.size(); ++i) {
        if (!cellFree(map, path[i])) {
            return false;
        }
        if (i > 0 && manhattanKeyDistance(path[i - 1], path[i]) != 1) {
            return false;
        }
    }
    return true;
}

// Frontier geometry determines the candidate grid density, angular coverage,
// altitude layers, number of perimeter shells, and the search budget.
FrontierViewpointOptimizer::SamplingPlan
FrontierViewpointOptimizer::planFor(
    const Frontier &frontier, double resolution) const {
    const Vector mn = asVector(frontier.min_bound);
    const Vector mx = asVector(frontier.max_bound);
    const Vector span = (mx - mn).cwiseAbs();
    const double long_xy = std::max({resolution, span.x(), span.y(),
                                    double(frontier.width)});
    const double short_xy = std::max(resolution, std::min(span.x(), span.y()));
    const double vertical = std::max({resolution, span.z(),
                                      double(frontier.height)});
    const double elongation = std::clamp(
        double(frontier.elongation_ratio), 1.0, 5.0);
    const double convexity = std::clamp(
        double(frontier.convexity_ratio), 0.0, 1.0);
    const double irregularity = 1.0 - convexity;

    SamplingPlan plan;
    plan.along = boundedInt(
        1 + int(std::ceil(long_xy / anchor_spacing_ * std::sqrt(elongation))),
        3, 24);
    plan.across = boundedInt(
        1 + int(std::ceil(short_xy / anchor_spacing_ + 4.0 * irregularity)),
        2, 12);
    plan.vertical = boundedInt(
        2 + int(std::ceil(vertical / vertical_spacing_)), 3, 10);
    plan.angular = boundedInt(
        8 + int(std::ceil(long_xy / (2.0 * resolution))) +
        int(std::ceil(3.0 * (elongation - 1.0) + 8.0 * irregularity)),
        8, 48);
    plan.shells = boundedInt(
        1 + int(std::floor((long_xy + vertical) / 5.0)), 1, 5);
    plan.offset = std::clamp(
        minimum_offset_ + 0.1 * vertical, minimum_offset_, maximum_offset_);
    plan.shell_step = std::max(resolution,
        shell_step_ + 0.05 * std::sqrt(long_xy * vertical));
    plan.budget = boundedInt(
        min_candidates_ + 2 * plan.along + 2 * plan.across +
        3 * plan.vertical + 5 * (plan.shells - 1),
        min_candidates_, max_candidates_);
    return plan;
}

std::vector<Vector> FrontierViewpointOptimizer::sampleFrontier(
    const Frontier &frontier) const {
    std::vector<Vector> samples;
    if (frontier.voxels.empty()) {
        samples.push_back(asVector(frontier.centroid));
        return samples;
    }
    std::vector<Vector> ordered;
    ordered.reserve(frontier.voxels.size());
    for (const auto &voxel : frontier.voxels) {
        ordered.push_back(asVector(voxel));
    }
    std::sort(ordered.begin(), ordered.end(),
              [](const Vector &a, const Vector &b) {
                  if (a.x() != b.x()) {
                      return a.x() < b.x();
                  }
                  if (a.y() != b.y()) {
                      return a.y() < b.y();
                  }
                  return a.z() < b.z();
              });
    const std::size_t count = std::min(
        ordered.size(), static_cast<std::size_t>(max_visibility_samples_));
    samples.reserve(count);
    for (std::size_t index = 0; index < count; ++index) {
        samples.push_back(ordered[(index * ordered.size() + count / 2) / count]);
    }
    return samples;
}

std::vector<FrontierViewpointOptimizer::Key>
FrontierViewpointOptimizer::sampleCandidateKeys(
    const Map &map, const Frontier &frontier, const Vector &robot,
    const SamplingPlan &plan) const {
    const Vector mn = asVector(frontier.min_bound);
    const Vector mx = asVector(frontier.max_bound);
    const Vector center = asVector(frontier.centroid);
    const Vector box_center = 0.5 * (mn + mx);
    std::unordered_set<Key, KeyHash, KeyEqual> unique;
    std::vector<Key> pool;
    pool.reserve(static_cast<std::size_t>(max_candidate_pool_));

    auto add = [&](const Vector &position) {
        if (pool.size() >= static_cast<std::size_t>(max_candidate_pool_)) {
            return;
        }
        Key key;
        if (toKey(map, position, key) && cellFree(map, key) &&
            inHeightBand(keyCenter(map, key)) &&
            unique.insert(key).second) {
            pool.push_back(key);
        }
    };

    // Preserve known-free fallback choices near the frontier boundary.
    // Their *count* is derived from the frontier geometry, not voxel-array size.
    add(robot);
    add(center);
    add(box_center);
    const auto boundary = sampleFrontier(frontier);
    const int fallback_count = boundedInt(
        plan.along * plan.vertical + plan.across, 8, 80);
    for (int index = 0; index < fallback_count && !boundary.empty(); ++index) {
        const Vector &voxel = boundary[
            static_cast<std::size_t>(index) * boundary.size() / fallback_count];
        add(voxel);
    }

    // The exterior AABB shells may lie in unexplored space. Probe a small
    // geometric neighborhood of boundary cells so that viewpoints on the
    // already observed side of a frontier are still found. All probes are
    // snapped to exact native cell centers and rejected unless known free.
    const std::array<std::array<int, 3>, 10> directions{{
        {{1, 0, 0}}, {{-1, 0, 0}}, {{0, 1, 0}}, {{0, -1, 0}},
        {{1, 1, 0}}, {{-1, 1, 0}}, {{1, -1, 0}}, {{-1, -1, 0}},
        {{0, 0, 1}}, {{0, 0, -1}}
    }};
    const int boundary_seeds = std::min(24, fallback_count);
    for (int index = 0; index < boundary_seeds && !boundary.empty(); ++index) {
        const Vector seed = boundary[
            static_cast<std::size_t>(index) * boundary.size() / boundary_seeds];
        for (int depth = 1; depth <= std::min(3, 1 + plan.shells); ++depth) {
            for (const auto &direction : directions) {
                add(seed + map->getResolution() * depth * Vector(
                    direction[0], direction[1], direction[2]));
            }
        }
    }

    // Height controls the number and vertical spread of candidate layers;
    // robot and centroid heights are added to avoid excluding free flight
    // corridors whose altitude differs from the frontier bounding box.
    const double height = std::max({map->getResolution(),
        double(frontier.height), std::abs(mx.z() - mn.z())});
    const double padding = std::clamp(0.5 + 0.25 * height, 0.5, 2.5);
    std::vector<double> altitudes;
    altitudes.reserve(static_cast<std::size_t>(plan.vertical + 2));
    for (int index = 0; index < plan.vertical; ++index) {
        const double factor = (index + 0.5) / plan.vertical;
        altitudes.push_back(mn.z() - padding +
            factor * (mx.z() - mn.z() + 2.0 * padding));
    }
    altitudes.push_back(center.z());
    altitudes.push_back(robot.z());
    if (heightBandEnabled()) {
        // Keep the layers inside the height band (e.g. canopy frontiers are viewed from the band
        // top) and always try the preferred height.
        const double floor = floorAt(box_center.x(), box_center.y());
        const double half = 0.5 * map->getResolution();
        const double lo = floor + min_height_above_ground_ + half;
        const double hi = floor + max_height_above_ground_ - half;
        if (lo <= hi) {
            for (double &z : altitudes) {
                z = std::clamp(z, lo, hi);
            }
        }
        altitudes.push_back(floor + preferred_height_above_ground_);
    }
    std::sort(altitudes.begin(), altitudes.end());
    altitudes.erase(std::unique(altitudes.begin(), altitudes.end(),
        [](double a, double b) { return std::abs(a - b) < 0.24; }),
        altitudes.end());

    // Samples the inside and extent of the AABB; irregular frontiers receive
    // extra transverse samples, while elongated frontiers get more samples
    // along their long axis.
    const bool major_x = (mx.x() - mn.x()) >= (mx.y() - mn.y());
    for (double z : altitudes) {
        for (int along = 0; along < plan.along; ++along) {
            const double a = (along + 0.5) / plan.along;
            for (int across = 0; across < plan.across; ++across) {
                const double b = (across + 0.5) / plan.across;
                const double x = major_x ? mn.x() + a * (mx.x() - mn.x())
                                         : mn.x() + b * (mx.x() - mn.x());
                const double y = major_x ? mn.y() + b * (mx.y() - mn.y())
                                         : mn.y() + a * (mx.y() - mn.y());
                add(Vector(x, y, z));
            }
        }
    }

    // Additional viewing shells, measured from the AABB perimeter rather
    // than from the centroid. The shell count / angle density / altitude
    // density are all functions of the Frontier.msg geometry.
    for (int shell = 0; shell < plan.shells; ++shell) {
        const double offset = std::min(
            maximum_offset_, plan.offset + shell * plan.shell_step);
        for (double z : altitudes) {
            for (int ia = 0; ia < plan.angular; ++ia) {
                const double angle = 2.0 * PI * ia / plan.angular;
                const double dx = std::cos(angle);
                const double dy = std::sin(angle);
                double exit_distance = std::numeric_limits<double>::infinity();
                if (std::abs(dx) > 1.0e-8) {
                    const double boundary_x = dx > 0.0 ? mx.x() : mn.x();
                    exit_distance = std::min(
                        exit_distance, (boundary_x - box_center.x()) / dx);
                }
                if (std::abs(dy) > 1.0e-8) {
                    const double boundary_y = dy > 0.0 ? mx.y() : mn.y();
                    exit_distance = std::min(
                        exit_distance, (boundary_y - box_center.y()) / dy);
                }
                if (!std::isfinite(exit_distance)) {
                    continue;
                }
                add(Vector(box_center.x() + dx * (exit_distance + offset),
                           box_center.y() + dy * (exit_distance + offset), z));
            }
        }
    }

    // Only native OctoMap cell centers are returned. Sort by cheap geometric
    // coverage, but limit repeated selection from the same XY angular sector.
    const auto frontier_samples = sampleFrontier(frontier);
    struct RankedKey {
        Key key;
        double score;
        int sector;
    };
    std::vector<RankedKey> ranked;
    ranked.reserve(pool.size());
    for (const Key &key : pool) {
        const Vector p = keyCenter(map, key);
        if ((p - center).norm() > max_range_ +
            0.5 * (mx - mn).norm()) {
            continue;
        }
        const double aimed = std::atan2(center.y() - p.y(), center.x() - p.x());
        const double score = preliminaryScore(frontier, frontier_samples, p, aimed);
        if (score <= -1.0e20) {
            continue;
        }
        const double angle = wrapped(std::atan2(p.y() - center.y(),
                                                p.x() - center.x()));
        const int sector = boundedInt(int(std::floor(
            (angle + PI) / (2.0 * PI) * 12.0)), 0, 11);
        ranked.push_back({key, score, sector});
    }
    std::sort(ranked.begin(), ranked.end(),
              [](const RankedKey &a, const RankedKey &b) {
                  return a.score > b.score;
              });
    std::vector<Key> selected;
    selected.reserve(std::min(ranked.size(),
        static_cast<std::size_t>(plan.budget)));
    std::array<int, 12> sector_used{};
    std::unordered_set<Key, KeyHash, KeyEqual> selected_set;
    const int quota = std::max(1, (plan.budget + 11) / 12);
    for (const auto &rank : ranked) {
        if (static_cast<int>(selected.size()) >= plan.budget) {
            break;
        }
        if (sector_used[rank.sector] >= quota) {
            continue;
        }
        if (selected_set.insert(rank.key).second) {
            selected.push_back(rank.key);
            ++sector_used[rank.sector];
        }
    }
    for (const auto &rank : ranked) {
        if (static_cast<int>(selected.size()) >= plan.budget) {
            break;
        }
        if (selected_set.insert(rank.key).second) {
            selected.push_back(rank.key);
        }
    }
    return selected;
}

Vector FrontierViewpointOptimizer::sensorOrigin(
    const Vector &position, double yaw) const {
    const double cs = std::cos(yaw);
    const double sn = std::sin(yaw);
    return position + Vector(
        cs * sensor_offset_.x() - sn * sensor_offset_.y(),
        sn * sensor_offset_.x() + cs * sensor_offset_.y(),
        sensor_offset_.z());
}

bool FrontierViewpointOptimizer::inFov(
    const Vector &target, const Vector &sensor, double yaw) const {
    const Vector delta = target - sensor;
    const double range = delta.norm();
    if (range < min_range_ || range > max_range_) {
        return false;
    }
    const double elevation = std::atan2(
        delta.z(), std::hypot(delta.x(), delta.y()));
    if (elevation < vmin_ || elevation > vmax_) {
        return false;
    }
    return hfov_ >= 2.0 * PI - 1.0e-6 ||
           std::abs(wrapped(std::atan2(delta.y(), delta.x()) - yaw)) <=
               0.5 * hfov_;
}

bool FrontierViewpointOptimizer::visible(
    const Map &map, const Vector &sensor, const Vector &target) const {
    octomap::KeyRay ray;
    if (!map->computeRayKeys(
            octomap::point3d(sensor.x(), sensor.y(), sensor.z()),
            octomap::point3d(target.x(), target.y(), target.z()), ray)) {
        return false;
    }
    Key goal;
    if (!toKey(map, target, goal)) {
        return false;
    }
    for (const auto &key : ray) {
        if (sameKey(key, goal)) {
            continue;
        }
        if (!cellFree(map, key)) {
            return false;
        }
    }
    return true;
}

double FrontierViewpointOptimizer::preliminaryScore(
    const Frontier &frontier, const std::vector<Vector> &samples,
    const Vector &position, double yaw) const {
    if (samples.empty()) {
        return -1.0e30;
    }
    const Vector sensor = sensorOrigin(position, yaw);
    int visible_in_fov = 0;
    for (const auto &sample : samples) {
        if (inFov(sample, sensor, yaw)) {
            ++visible_in_fov;
        }
    }
    if (visible_in_fov == 0) {
        return -1.0e30;
    }
    const double ratio = double(visible_in_fov) / samples.size();
    const double altitude = altitudeAboveReference(position);
    const double penalty = altitude_weight_ * std::max(0.0, altitude) +
        excessive_altitude_penalty_ * std::pow(
            std::max(0.0, altitude - altitude_excess_), 2.0);
    const double area_proxy = frontier.surface_area_proxy > 0.0f ?
        double(frontier.surface_area_proxy) :
        samples.size() * expected_resolution_ * expected_resolution_;
    return area_weight_ * area_proxy *
        ratio - penalty - 0.02 *
        (position - asVector(frontier.centroid)).norm();
}

FrontierViewpointOptimizer::Candidate FrontierViewpointOptimizer::evaluate(
    const Map &map, const Frontier &frontier,
    const std::vector<Vector> &samples, const Vector &position,
    double yaw, double path_length) const {
    Candidate result;
    result.position = position;
    result.yaw = wrapped(yaw);
    result.path_length = path_length;
    if (samples.empty()) {
        return result;
    }
    const Vector sensor = sensorOrigin(position, yaw);
    const double area_proxy = frontier.surface_area_proxy > 0.0f ?
        double(frontier.surface_area_proxy) :
        frontier.voxels.size() * expected_resolution_ * expected_resolution_;
    const double sample_area = std::max(
        expected_resolution_ * expected_resolution_, area_proxy) / samples.size();
    for (const auto &sample : samples) {
        if (!inFov(sample, sensor, yaw) || !visible(map, sensor, sample)) {
            continue;
        }
        result.area += sample_area;
        const Vector direction = (sample - sensor).normalized();
        const double remaining = std::max(
            0.0, max_range_ - (sample - sensor).norm());
        const double horizon = std::min(remaining, unknown_depth_cap_ + 2.0);
        bool inside_unknown = false;
        double unknown_depth = 0.0;
        for (double length = map->getResolution(); length <= horizon;
             length += volume_ray_step_) {
            const Vector point = sample + direction * length;
            const auto *node = map->search(point.x(), point.y(), point.z());
            if (node) {
                if (map->isNodeOccupied(node) || inside_unknown) {
                    break;
                }
                continue;
            }
            inside_unknown = true;
            unknown_depth += volume_ray_step_;
            if (unknown_depth >= unknown_depth_cap_) {
                break;
            }
        }
        result.volume += sample_area *
            std::min(unknown_depth, unknown_depth_cap_);
    }
    if (result.area <= 0.0) {
        return result;
    }
    const double altitude = altitudeAboveReference(position);
    const double penalty = altitude_weight_ * std::max(0.0, altitude) +
        excessive_altitude_penalty_ * std::pow(
            std::max(0.0, altitude - altitude_excess_), 2.0);
    result.score = area_weight_ * result.area +
        volume_weight_ * std::min(
            result.volume, result.area * volume_depth_cap_) -
        penalty;
    return result;
}

FrontierViewpointOptimizer::Candidate FrontierViewpointOptimizer::optimize(
    const Map &map, const Navigation &navigation,
    const Frontier &frontier, const Vector &robot, int &reachable_output) const {
    reachable_output = 0;
    Candidate best;
    const SamplingPlan plan = planFor(frontier, map->getResolution());
    const auto samples = sampleFrontier(frontier);
    const auto positions = sampleCandidateKeys(map, frontier, robot, plan);
    if (positions.empty()) {
        if (diagnostics_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                "Frontier %u: no known-free voxel-center candidates", frontier.id);
        }
        return best;
    }

    // Sort by approximate frontier coverage, then use the shared reachability
    // result. No new path search is performed for individual candidates.
    struct Ranked {
        Key key;
        double preliminary{0.0};
    };
    std::vector<Ranked> ranked;
    ranked.reserve(positions.size());
    const bool yaw_invariant = hfov_ >= 2.0 * PI - 1.0e-6 &&
        sensor_offset_.head<2>().norm() < 1.0e-6;
    const int yaw_count = yaw_invariant ? 1 : yaw_samples_;
    for (const Key &key : positions) {
        const Vector position = keyCenter(map, key);
        const double aimed = std::atan2(
            frontier.centroid.y - position.y(),
            frontier.centroid.x - position.x());
        double top_score = -1.0e30;
        for (int iyaw = 0; iyaw < yaw_count; ++iyaw) {
            const double yaw = yaw_count == 1 ? aimed :
                wrapped(aimed - PI + 2.0 * PI * iyaw / yaw_count);
            top_score = std::max(top_score,
                preliminaryScore(frontier, samples, position, yaw));
        }
        if (top_score > -1.0e20) {
            ranked.push_back({key, top_score});
        }
    }
    std::sort(ranked.begin(), ranked.end(),
              [](const Ranked &a, const Ranked &b) {
                  return a.preliminary > b.preliminary;
              });

    std::size_t reachable_count = 0;
    for (const Ranked &ranked_position : ranked) {
        const auto location = navigation.parent.find(ranked_position.key);
        if (location == navigation.parent.end()) {
            continue;
        }
        const std::vector<Key> path = pathKeys(
            navigation, ranked_position.key);
        if (path.empty()) {
            continue;
        }
        ++reachable_count;
        const Vector position = keyCenter(map, ranked_position.key);
        const double path_length =
            double(path.size() - 1) * map->getResolution();
        const double aimed = std::atan2(
            frontier.centroid.y - position.y(),
            frontier.centroid.x - position.x());
        for (int iyaw = 0; iyaw < yaw_count; ++iyaw) {
            const double yaw = yaw_count == 1 ? aimed :
                wrapped(aimed - PI + 2.0 * PI * iyaw / yaw_count);
            Candidate candidate = evaluate(
                map, frontier, samples, position, yaw, path_length);
            if (candidate.score > best.score) {
                candidate.path_keys = path;
                best = std::move(candidate);
            }
        }
        if (static_cast<int>(reachable_count) >= reachable_evaluation_limit_) {
            break;
        }
    }
    reachable_output = static_cast<int>(reachable_count);
    if (best.valid() && refinement_enabled_) {
        refine(map, frontier, samples, best);
    }
    if (diagnostics_) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1500,
            "Frontier %u: geometry [along=%d across=%d z=%d angles=%d "
            "shells=%d budget=%d], centers=%zu reachable=%zu score=%.2f%s",
            frontier.id, plan.along, plan.across, plan.vertical,
            plan.angular, plan.shells, plan.budget, positions.size(),
            reachable_count, best.score,
            navigation.limited ? " (reachability truncated)" : "");
    }
    return best;
}

void FrontierViewpointOptimizer::refine(
    const Map &map, const Frontier &frontier,
    const std::vector<Vector> &samples, Candidate &best) const {
    // Refinement never changes the initial discrete path: it remains inside
    // the selected native-resolution cell and checks the swept UAV box.
    Key base;
    if (!toKey(map, best.position, base)) {
        return;
    }
    const Vector base_center = keyCenter(map, base);
    const double radius = refinement_max_fraction_ * map->getResolution();
    const Vector offsets[] = {
        Vector(0.0, 0.0, 0.0),
        Vector(1.0, 0.0, 0.0), Vector(-1.0, 0.0, 0.0),
        Vector(0.0, 1.0, 0.0), Vector(0.0, -1.0, 0.0),
        Vector(0.0, 0.0, 1.0), Vector(0.0, 0.0, -1.0),
        Vector(1.0, 1.0, 0.0), Vector(-1.0, 1.0, 0.0),
        Vector(1.0, -1.0, 0.0), Vector(-1.0, -1.0, 0.0),
        Vector(1.0, 0.0, 1.0), Vector(-1.0, 0.0, 1.0),
        Vector(0.0, 1.0, -1.0), Vector(0.0, -1.0, -1.0)
    };
    double fraction = 1.0;
    double yaw_change = refinement_yaw_step_;
    const bool yaw_invariant = hfov_ >= 2.0 * PI - 1.0e-6 &&
        sensor_offset_.head<2>().norm() < 1.0e-6;
    for (int iteration = 0; iteration < refinement_iterations_; ++iteration) {
        Candidate improved = best;
        for (const auto &offset : offsets) {
            const Vector candidate_position = best.position +
                radius * fraction * offset;
            if ((candidate_position - base_center).cwiseAbs().maxCoeff() >=
                0.5 * map->getResolution() || !inHeightBand(candidate_position)) {
                continue;
            }
            if (!validRefinementMotion(
                    map, best.position, candidate_position, base)) {
                continue;
            }
            const int variants = yaw_invariant ? 1 : 3;
            for (int variant = 0; variant < variants; ++variant) {
                const double delta = variant == 0 ? 0.0 :
                    (variant == 1 ? yaw_change : -yaw_change);
                const double yaw = wrapped(best.yaw + delta);
                const double path_length = best.path_length +
                    (candidate_position - best.position).norm();
                Candidate candidate = evaluate(
                    map, frontier, samples, candidate_position, yaw,
                    path_length);
                if (candidate.score > improved.score) {
                    candidate.path_keys = best.path_keys;
                    improved = std::move(candidate);
                }
            }
        }
        best = std::move(improved);
        fraction *= 0.5;
        yaw_change *= 0.5;
    }
}

void FrontierViewpointOptimizer::validationWorkerLoop() {
    while (true) {
        Work task;
        Map snapshot;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            validation_cv_.wait(lock, [this]() { return stopping_ || !validation_pending_.empty(); });
            if (stopping_) {
                return;
            }
            if (!readyLocked()) {
                validation_cv_.wait_for(lock, std::chrono::milliseconds(100), [this]() { return stopping_ || readyLocked(); });
                continue;
            }
            bool found = false;
            while (!validation_pending_.empty()) {
                const uint32_t id = validation_pending_.front();
                validation_pending_.pop_front();
                if (!validation_set_.erase(id)) {
                    continue;
                }
                const auto entry = frontiers_.find(id);
                if (entry == frontiers_.end() || !entry->second.result || !entry->second.result->valid()) {
                    continue;
                }
                task = {id, entry->second.revision, entry->second.frontier, entry->second.result, false, true};
                snapshot = map_;
                found = true;
                break;
            }
            if (!found) {
                continue;
            }
        }
        const bool occupied = endpointOccupied(snapshot, *task.previous);
        const bool useful = !occupied && candidateStillUseful(snapshot, task.frontier, *task.previous);
        const bool blocked = !useful && definitelyOccluded(snapshot, task.frontier, *task.previous);
        const auto watched = observationKeys(snapshot, task.frontier, *task.previous);
        {
            std::lock_guard<std::mutex> lock(mutex_);
            const auto found = frontiers_.find(task.id);
            if (found == frontiers_.end() || !found->second.result || found->second.revision != task.revision || (found->second.result->position - task.previous->position).norm() > 1.0e-4) {
                continue;
            }
            Entry &entry = found->second;
            if (map_.get() != snapshot.get()) {
                enqueueValidationLocked(task.id);
                continue;
            }
            entry.last_validation = SteadyClock::now();
            if (occupied || blocked) {
                RCLCPP_WARN(get_logger(), "Viewpoint %u invalidated by %s", task.id, occupied ? "occupied destination" : "definitively blocked sightlines");
                entry.result.reset();
                entry.validated = false;
                entry.observation_keys.clear();
                entry.attempted = false;
                entry.no_reachable = false;
                enqueueLocked(task.id);
            } else {
                // Unknown/partial evidence is not proof of invalidity.
                // The last known-good viewpoint is preserved.
                entry.validated = true;
                entry.observation_keys = watched;
            }
        }
        cv_.notify_one();
    }
}

void FrontierViewpointOptimizer::workerLoop() {
    Navigation cached_navigation;
    bool working = false;  // this worker is counted in in_flight_
    while (true) {
        Work task;
        Map snapshot;
        Vector robot = Vector::Zero();
        {
            std::unique_lock<std::mutex> lock(mutex_);
            if (working) {
                // Every path out of the previous task ends up here.
                --in_flight_;
                working = false;
            }
            cv_.wait_for(lock, std::chrono::milliseconds(200), [this]() { return stopping_ || !pending_.empty(); });
            if (stopping_) {
                return;
            }
            if (!readyLocked()) {
                cv_.wait_for(lock, std::chrono::milliseconds(100), [this]() { return stopping_ || readyLocked(); });
                continue;
            }
            bool got_task = false;
            while (!pending_.empty()) {
                const uint32_t id = pending_.front();
                pending_.pop_front();
                if (!pending_set_.erase(id)) {
                    continue;
                }
                const auto found = frontiers_.find(id);
                if (found == frontiers_.end() || found->second.no_reachable || found->second.attempted) {
                    continue;
                }
                Entry &entry = found->second;
                entry.attempted = true;
                task = {id, entry.revision, entry.frontier, entry.result, entry.solved_revision == entry.revision, false};
                snapshot = map_;
                robot = robot_;
                got_task = true;
                ++in_flight_;
                working = true;
                break;
            }
            if (!got_task) {
                continue;
            }
        }
        Key robot_key;
        if (!toKey(snapshot, robot, robot_key) || !cellFree(snapshot, robot_key)) {
            std::lock_guard<std::mutex> lock(mutex_);
            auto found = frontiers_.find(task.id);
            if (found != frontiers_.end() && found->second.revision == task.revision) {
                found->second.attempted = false;
                // An unavailable starting position is not proof that the frontier is unreachable.
                // It will be retried on the next genuine frontier/map-region change.
            }
            continue;
        }
        if (cached_navigation.map.get() != snapshot.get() || !sameKey(cached_navigation.origin, robot_key)) {
            cached_navigation = buildNavigation(snapshot, robot);
        }
        int reachable_count = 0;
        Candidate best = optimize(snapshot, cached_navigation, task.frontier, robot, reachable_count);
        Map latest_map;
        Frontier latest_frontier;
        uint64_t latest_revision = 0;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            const auto entry = frontiers_.find(task.id);
            if (entry == frontiers_.end()) {
                continue;
            }
            latest_map = map_;
            latest_frontier = entry->second.frontier;
            latest_revision = entry->second.revision;
        }
        if (latest_revision != task.revision) {
            // The frontier callback owns scheduling for the new revision.
            // Never reset 'attempted' here: a newer worker may already be running.
            continue;
        }
        if (best.valid() && latest_map && latest_map.get() != snapshot.get()) {
            Key goal;
            if (!toKey(latest_map, best.position, goal) || !cellFree(latest_map, goal) || !pathStillFree(latest_map, best.path_keys) || !candidateStillUseful(latest_map, latest_frontier, best)) {
                best = Candidate{};
            }
        }
        const auto observed = best.valid() ? observationKeys(latest_map, latest_frontier, best) : std::unordered_set<Key, KeyHash, KeyEqual>{};
        {
            std::lock_guard<std::mutex> lock(mutex_);
            auto found = frontiers_.find(task.id);
            if (found == frontiers_.end() || found->second.revision != task.revision) {
                continue;
            }
            Entry &entry = found->second;
            entry.last_attempt = SteadyClock::now();
            if (best.valid()) {
                // Exactly one optimized viewpoint is stored for each live frontier ID.
                entry.result = std::move(best);
                entry.validated = true;
                entry.solved_revision = task.revision;
                entry.observation_keys = observed;
                entry.no_reachable = false;
                ++optimized_count_;
            } else {
                // Never erase a separately validated cached result on failure.
                // reachable=0 is a terminal outcome for this geometry revision.
                if (reachable_count == 0 && !cached_navigation.limited && cached_navigation.valid) {
                    entry.no_reachable = true;
                    RCLCPP_INFO(get_logger(), "Frontier %u: reachable=0, frozen until actual geometry update", task.id);
                } else if (!entry.result) {
                    ++missing_count_;
                }
            }
        }
    }
}


visualization_msgs::msg::MarkerArray
FrontierViewpointOptimizer::makeMarkers(
    const std::vector<std::pair<uint32_t, Candidate>> &results,
    const std::string &frame, const rclcpp::Time &stamp) const {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = frame;
    clear.header.stamp = stamp;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);

    for (const auto &item : results) {
        const uint32_t id = item.first;
        const Candidate &candidate = item.second;
        visualization_msgs::msg::Marker arrow;
        arrow.header.frame_id = frame;
        arrow.header.stamp = stamp;
        arrow.ns = "frontier_viewpoint_arrow";
        arrow.id = static_cast<int>(id);
        arrow.action = visualization_msgs::msg::Marker::ADD;
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.pose.position = asPoint(candidate.position);
        arrow.pose.orientation.z = std::sin(0.5 * candidate.yaw);
        arrow.pose.orientation.w = std::cos(0.5 * candidate.yaw);
        arrow.scale.x = 1.25;
        arrow.scale.y = 0.15;
        arrow.scale.z = 0.15;
        const double color_position = static_cast<double>((id % 16) + 2) / 18.0;
        const double intensity = 1.0 - color_position;
        arrow.color.r = intensity;
        arrow.color.g = intensity;
        arrow.color.b = intensity;
        arrow.color.a = 1.0f;
        markers.markers.push_back(arrow);

        if (!draw_fov_markers_) {
            continue;
        }
        visualization_msgs::msg::Marker fov;
        fov.header = arrow.header;
        fov.ns = "frontier_sensor_fov";
        fov.id = arrow.id;
        fov.action = visualization_msgs::msg::Marker::ADD;
        fov.type = visualization_msgs::msg::Marker::LINE_LIST;
        fov.pose.orientation.w = 1.0;
        fov.scale.x = 0.035;
        fov.color = arrow.color;
        fov.color.a = static_cast<float>(marker_alpha_);
        const Vector origin = sensorOrigin(candidate.position, candidate.yaw);
        auto tip = [&](double azimuth, double elevation) {
            return origin + max_range_ * Vector(
                std::cos(elevation) * std::cos(azimuth),
                std::cos(elevation) * std::sin(azimuth),
                std::sin(elevation));
        };
        auto line = [&](const Vector &a, const Vector &b) {
            fov.points.push_back(asPoint(a));
            fov.points.push_back(asPoint(b));
        };
        for (int segment = 0; segment < marker_segments_; ++segment) {
            const double az0 = candidate.yaw - 0.5 * hfov_ +
                hfov_ * segment / marker_segments_;
            const double az1 = candidate.yaw - 0.5 * hfov_ +
                hfov_ * (segment + 1) / marker_segments_;
            const Vector top0 = tip(az0, vmax_);
            const Vector top1 = tip(az1, vmax_);
            const Vector bottom0 = tip(az0, vmin_);
            const Vector bottom1 = tip(az1, vmin_);
            line(top0, top1);
            line(bottom0, bottom1);
            if (segment % std::max(1, marker_segments_ / 8) == 0) {
                line(top0, bottom0);
                line(origin, top0);
            }
        }
        markers.markers.push_back(std::move(fov));
    }
    return markers;
}

void FrontierViewpointOptimizer::publishCb() {
    std::vector<std::pair<uint32_t, Candidate>> results;
    std::string frame;
    std::size_t active = 0;
    std::size_t pending = 0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!readyLocked()) {
            return;
        }
        if (on_demand_) {
            if (!batch_active_ || pending_batch_ || !pending_set_.empty() || in_flight_ > 0) {
                return;
            }
            batch_active_ = false;
        }
        frame = map_frame_;
        active = frontiers_.size();
        pending = pending_set_.size();
        results.reserve(frontiers_.size());
        for (const auto &item : frontiers_) {
            const Entry &entry = item.second;
            if (entry.validated && entry.result && entry.result->valid()) {
                // A geometry revision queues improvement but DOES NOT hide
                // the last validated result while optimization is pending.
                results.emplace_back(item.first, *entry.result);
            }
        }
    }
    cv_.notify_one();
    const rclcpp::Time stamp = now();
    msg::OptimizedFrontierArray output;
    output.header.frame_id = frame;
    output.header.stamp = stamp;
    output.viewpoints.reserve(results.size());
    for (const auto &item : results) {
        const Candidate &candidate = item.second;
        msg::OptimizedFrontier viewpoint;
        viewpoint.id = item.first;
        viewpoint.viewpoint.header = output.header;
        viewpoint.viewpoint.pose.position = asPoint(candidate.position);
        viewpoint.viewpoint.pose.orientation.z = std::sin(candidate.yaw / 2.0);
        viewpoint.viewpoint.pose.orientation.w = std::cos(candidate.yaw / 2.0);
        viewpoint.observable_frontier_area = candidate.area;
        viewpoint.expected_new_volume = candidate.volume;
        viewpoint.information_gain = candidate.area + candidate.volume;
        viewpoint.objective_score = candidate.score;
        output.viewpoints.push_back(viewpoint);
    }
    result_pub_->publish(output);
    marker_pub_->publish(makeMarkers(results, frame, stamp));
    if (diagnostics_) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
            "Published %zu viewpoints, active frontiers=%zu pending=%zu",
            results.size(), active, pending);
    }
}

}  // namespace frontier_viewpoint_optimizer

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<
        frontier_viewpoint_optimizer::FrontierViewpointOptimizer>();
    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
    executor.remove_node(node);
    node.reset();
    rclcpp::shutdown();
    return 0;
}
