#pragma once

#include <Eigen/Core>
#include <frontier_segmentation/msg/frontier.hpp>
#include <frontier_segmentation/msg/frontier_array.hpp>
#include <frontier_viewpoint_optimizer/msg/optimized_frontier.hpp>
#include <frontier_viewpoint_optimizer/msg/optimized_frontier_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <octomap/OcTree.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace frontier_viewpoint_optimizer {

class FrontierViewpointOptimizer final : public rclcpp::Node {
public:
    FrontierViewpointOptimizer();
    ~FrontierViewpointOptimizer() override;

private:
    using Vector = Eigen::Vector3d;
    using Map = std::shared_ptr<const octomap::OcTree>;
    using Key = octomap::OcTreeKey;
    using Frontier = frontier_segmentation::msg::Frontier;
    using FrontierArray = frontier_segmentation::msg::FrontierArray;
    using SteadyClock = std::chrono::steady_clock;

    struct KeyHash {
        std::size_t operator()(const Key &key) const noexcept;
    };

    struct KeyEqual {
        bool operator()(const Key &a, const Key &b) const noexcept;
    };

    struct SamplingPlan {
        int along{1};
        int across{1};
        int vertical{1};
        int angular{8};
        int shells{1};
        int budget{48};
        double offset{0.5};
        double shell_step{0.5};
    };

    struct Candidate {
        Vector position{0.0, 0.0, 0.0};
        double yaw{0.0};
        double area{0.0};
        double volume{0.0};
        double score{-1.0e30};
        double path_length{0.0};
        std::vector<Key> path_keys;
        bool valid() const noexcept;
    };

    struct Entry {
        Frontier frontier;
        std::optional<Candidate> result;
        uint64_t revision{1};
        uint64_t solved_revision{0};
        bool validated{false};
        bool attempted{false};
        bool no_reachable{false};
        bool geometry_dirty{false};
        std::unordered_set<Key, KeyHash, KeyEqual> observation_keys;
        SteadyClock::time_point last_attempt{};
        SteadyClock::time_point last_validation{};
    };

    struct Work {
        uint32_t id{0};
        uint64_t revision{0};
        Frontier frontier;
        std::optional<Candidate> previous;
        bool previous_matches_revision{false};
        bool validate_only{false};
    };

    struct Navigation {
        Map map;
        Key origin{};
        Vector robot{0.0, 0.0, 0.0};
        std::unordered_map<Key, Key, KeyHash, KeyEqual> parent;
        bool valid{false};
        bool limited{false};
        int64_t duration_ms{0};
    };

    void mapCb(octomap_msgs::msg::Octomap::SharedPtr message);
    // Floor height map (P7_topdown_map floor_height_map_node, built from /octomap_full). This
    // node plans on the camera-filtered OctoMap; both use the same 0.5 m grid in the same frame,
    // and the floor is looked up by world (x, y), so the two maps need not match cell by cell.
    void floorHeightCb(sensor_msgs::msg::PointCloud2::SharedPtr message);
    bool heightBandEnabled() const { return !floor_height_topic_.empty(); }
    // Floor [m] at (x, y): floor map value, else default_floor_height (take-off level).
    double floorAt(double x, double y) const;
    // Hard limits: position within [floor + min_height_above_ground, floor + max_height_above_ground].
    bool inHeightBand(const Vector &position) const;
    // Height above the preferred flight height, used by the altitude penalty.
    double altitudeAboveReference(const Vector &position) const;
    void frontierCb(FrontierArray::SharedPtr message);
    void odomCb(nav_msgs::msg::Odometry::SharedPtr message);
    void publishCb();
    void heartbeatCb();
    void workerLoop();
    void mapWorkerLoop();
    void validationWorkerLoop();
    std::unordered_set<Key, KeyHash, KeyEqual> observationKeys(const Map &map, const Frontier &frontier, const Candidate &candidate) const;
    bool frontierTouchesChanges(const Map &map, const Frontier &frontier, const std::unordered_set<Key, KeyHash, KeyEqual> &changed) const;
    void compactQueuesLocked();

    bool startBatchMapLocked();
    void installBatchLocked();
    bool readyLocked() const;
    void enqueueLocked(uint32_t id);
    void enqueueValidationLocked(uint32_t id);
    bool candidateStillUseful(const Map &map, const Frontier &frontier, const Candidate &candidate) const;


    bool endpointOccupied(const Map &map, const Candidate &candidate) const;
    bool definitelyOccluded(const Map &map, const Frontier &frontier, const Candidate &candidate) const;


    static bool geometryChanged(const Frontier &a, const Frontier &b, double tolerance);

    bool toKey(const Map &map, const Vector &point, Key &key) const;
    Vector keyCenter(const Map &map, const Key &key) const;
    bool cellFree(const Map &map, const Key &key) const;
    bool boxFree(const Map &map, const Vector &lower, const Vector &upper) const;
    bool validRefinementMotion(const Map &map, const Vector &start,
                               const Vector &end, const Key &base_key) const;
    Navigation buildNavigation(const Map &map, const Vector &robot) const;
    std::vector<Key> pathKeys(const Navigation &navigation, const Key &goal) const;
    bool pathStillFree(const Map &map, const std::vector<Key> &path) const;

    SamplingPlan planFor(const Frontier &frontier, double resolution) const;
    std::vector<Vector> sampleFrontier(const Frontier &frontier) const;
    std::vector<Key> sampleCandidateKeys(const Map &map,
                                         const Frontier &frontier,
                                         const Vector &robot,
                                         const SamplingPlan &plan) const;
    Vector sensorOrigin(const Vector &point, double yaw) const;
    bool inFov(const Vector &point, const Vector &sensor, double yaw) const;
    bool visible(const Map &map, const Vector &sensor, const Vector &target) const;
    double preliminaryScore(const Frontier &frontier,
                            const std::vector<Vector> &samples,
                            const Vector &position, double yaw) const;
    Candidate evaluate(const Map &map, const Frontier &frontier,
                       const std::vector<Vector> &samples,
                       const Vector &position, double yaw,
                       double path_length) const;
    Candidate optimize(const Map &map, const Navigation &navigation, const Frontier &frontier, const Vector &robot, int &reachable_count) const;
    void refine(const Map &map, const Frontier &frontier,
                const std::vector<Vector> &samples, Candidate &best) const;
    visualization_msgs::msg::MarkerArray makeMarkers(
        const std::vector<std::pair<uint32_t, Candidate>> &results,
        const std::string &frame, const rclcpp::Time &stamp) const;

    rclcpp::Subscription<FrontierArray>::SharedPtr frontier_sub_;
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr floor_sub_;
    struct FloorMap {
        double x0{0.0}, y0{0.0}, res{0.5};  // lower corner of cell (0, 0)
        int nx{0}, ny{0};
        std::vector<float> z;
    };
    // Replaced as a whole and read by the worker threads: use std::atomic_load / atomic_store.
    std::shared_ptr<const FloorMap> floor_;
    std::string floor_height_topic_;
    double min_height_above_ground_{0.5};
    double max_height_above_ground_{5.0};
    double preferred_height_above_ground_{3.0};
    double default_floor_height_{0.0};
    rclcpp::Publisher<frontier_viewpoint_optimizer::msg::OptimizedFrontierArray>::SharedPtr result_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;

    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::vector<std::thread> optimization_workers_;
    std::thread validation_worker_;
    std::thread map_worker_;
    std::condition_variable validation_cv_;
    std::condition_variable map_cv_;
    octomap_msgs::msg::Octomap::SharedPtr pending_map_message_;
    // On-demand batches
    bool on_demand_{false};
    bool batch_active_{false};
    int in_flight_{0};
    octomap_msgs::msg::Octomap::SharedPtr latest_map_message_;
    FrontierArray::SharedPtr pending_batch_;
    std::unordered_set<Key, KeyHash, KeyEqual> last_changed_cells_;
    uint64_t global_revision_{0};
    int optimization_worker_count_{2};
    bool stopping_{false};
    bool have_odom_{false};
    bool have_frontiers_{false};
    Map map_;
    Vector robot_{0.0, 0.0, 0.0};
    std::string map_frame_;
    std::string frontier_frame_;
    std::string odom_frame_;
    std::map<uint32_t, Entry> frontiers_;
    std::deque<uint32_t> pending_;
    std::unordered_set<uint32_t> pending_set_;
    std::deque<uint32_t> validation_pending_;
    std::unordered_set<uint32_t> validation_set_;
    uint64_t map_sequence_{0};
    uint64_t map_hash_{0};
    bool have_map_hash_{false};
    uint64_t frontier_sequence_{0};
    uint64_t optimized_count_{0};
    uint64_t missing_count_{0};
    bool ros2top_registered_{false};

    std::string frontier_topic_;
    std::string map_topic_;
    std::string odom_topic_;
    std::string result_topic_;
    std::string marker_topic_;

    double expected_resolution_;
    double hfov_;
    double vmin_;
    double vmax_;
    double min_range_;
    double max_range_;
    Vector sensor_offset_{0.0, 0.0, 0.0};

    double minimum_offset_;
    double maximum_offset_;
    double anchor_spacing_;
    double vertical_spacing_;
    double shell_step_;
    int min_candidates_;
    int max_candidates_;
    int max_candidate_pool_;
    int max_visibility_samples_;
    int yaw_samples_;
    int reachable_evaluation_limit_;

    double reachability_max_distance_;
    int reachability_max_nodes_;
    int reachability_max_milliseconds_;

    double area_weight_;
    double volume_weight_;
    double volume_depth_cap_;
    double unknown_depth_cap_;
    double volume_ray_step_;
    double altitude_weight_;
    double altitude_reference_;
    double altitude_excess_;
    double excessive_altitude_penalty_;

    bool refinement_enabled_;
    int refinement_iterations_;
    double refinement_half_extent_;
    double refinement_margin_;
    double refinement_max_fraction_;
    double refinement_yaw_step_;

    double geometry_tolerance_;
    double cached_visibility_min_fraction_;

    bool diagnostics_;
    bool draw_fov_markers_;
    double marker_alpha_;
    int marker_segments_;
};

}  // namespace frontier_viewpoint_optimizer
