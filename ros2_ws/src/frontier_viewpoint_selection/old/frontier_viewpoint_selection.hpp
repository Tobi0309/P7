#pragma once

#include <dsp_interfaces/srv/path_cost.hpp>
#include <frontier_viewpoint_optimizer/msg/optimized_frontier_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <Eigen/Core>

#include <array>
#include <chrono>
#include <cstdint>
#include <deque>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

namespace frontier_viewpoint_selection {

class FrontierViewpointSelection final : public rclcpp::Node {
public:
    FrontierViewpointSelection();
    ~FrontierViewpointSelection() override;

private:
    using Clock = std::chrono::steady_clock;
    using Vector = Eigen::Vector3d;
    using View = frontier_viewpoint_optimizer::msg::OptimizedFrontier;
    using Array = frontier_viewpoint_optimizer::msg::OptimizedFrontierArray;
    using PathCost = dsp_interfaces::srv::PathCost;
    using Trigger = std_srvs::srv::Trigger;
    using PairKey = std::array<int64_t, 6>;

    enum class State { SELECT_GOAL, NAVIGATING, REVALIDATE, REPLAN, GOAL_REACHED, FINISHED };

    struct SavedView {
        View data;
        Clock::time_point updated{};
    };

    struct CostRecord {
        bool valid{false};
        double cost{0.0};
        double distance{0.0};
        Clock::time_point updated{};
    };

    struct EdgeRequest {
        PairKey key{};
        Vector from{0.0, 0.0, 0.0};
        Vector to{0.0, 0.0, 0.0};
    };

    struct Ranked {
        uint32_t id{0};
        double utility{-1.0e30};
    };

    void viewpointsCb(Array::SharedPtr msg);
    void odomCb(nav_msgs::msg::Odometry::SharedPtr msg);
    void finishCb(const std::shared_ptr<Trigger::Request> request, std::shared_ptr<Trigger::Response> response);
    void planningFailureCb(const std::shared_ptr<Trigger::Request> request, std::shared_ptr<Trigger::Response> response);
    void tick();
    void heartbeatCb();
    void registerTop();
    void unregisterTop();

    void requestNextCost();
    void costResponse(uint64_t token, PairKey key, rclcpp::Client<PathCost>::SharedFuture future);
    void queueCost(const Vector &from, const Vector &to);
    std::optional<CostRecord> getCost(const Vector &from, const Vector &to) const;
    PairKey makeKey(const Vector &from, const Vector &to) const;

    std::vector<uint32_t> shortlist() const;
    std::vector<Ranked> rankFrom(const Vector &start, const std::vector<uint32_t> &ids) const;
    std::vector<uint32_t> planOrder() const;
    bool canConsider(uint32_t id) const;
    void updateCostAnchor();
    void refreshQueries();
    void publishStatus();
    void logSelectedOrder() const;
    void publishHoldPose();
    void startGoal(uint32_t id);
    void abandonGoal(const char *reason);
    void completeGoal();
    bool currentGoalAvailable() const;
    bool arrived() const;
    double currentUtility() const;
    void processStateMachine();
    std::string stateName() const;
    static Vector point(const geometry_msgs::msg::Point &p);
    static geometry_msgs::msg::Point pointMsg(const Vector &p);

    rclcpp::Subscription<Array>::SharedPtr viewpoints_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Client<PathCost>::SharedPtr cost_client_;
    rclcpp::Service<Trigger>::SharedPtr planning_failure_service_;
    rclcpp::Service<Trigger>::SharedPtr finish_service_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr command_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;

    std::unordered_map<uint32_t, SavedView> views_;
    std::map<PairKey, CostRecord> costs_;
    std::map<PairKey, Clock::time_point> failed_until_;
    std::deque<EdgeRequest> queue_;
    std::set<PairKey> queued_;
    std::unordered_map<uint32_t, Clock::time_point> visited_until_;

    std::string frame_;
    std::string odom_frame_;
    Vector robot_{0.0, 0.0, 0.0};
    Vector cost_anchor_{0.0, 0.0, 0.0};
    geometry_msgs::msg::Quaternion robot_orientation_;
    bool have_odom_{false};
    bool have_anchor_{false};
    bool have_views_{false};
    Clock::time_point last_view_message_{};

    State state_{State::SELECT_GOAL};
    std::optional<uint32_t> goal_id_;
    geometry_msgs::msg::PoseStamped goal_pose_;
    Clock::time_point goal_started_{};
    Clock::time_point missing_since_{};
    Clock::time_point reached_since_{};
    Clock::time_point last_progress_time_{};
    double closest_goal_distance_{std::numeric_limits<double>::infinity()};
    Clock::time_point last_ranking_{};
    Clock::time_point last_review_{};
    std::optional<uint32_t> better_goal_id_;
    int better_goal_hits_{0};
    std::vector<uint32_t> order_;
    std::string replan_reason_{"navigation goal invalidated"};
    uint64_t goal_generation_{0};

    bool cost_inflight_{false};
    uint64_t cost_request_token_{0};
    int64_t cost_request_id_{0};
    EdgeRequest cost_inflight_edge_;
    Clock::time_point cost_request_started_{};
    Clock::time_point last_cost_request_sent_{};


    bool ros2top_registered_{false};
    std::string viewpoints_topic_;
    std::string odom_topic_;
    std::string cost_service_;
    std::string planning_failure_service_name_;
    std::string finish_service_name_;
    std::string command_topic_;
    double information_weight_{1.0};
    double path_cost_weight_{0.03};
    double path_distance_weight_{0.8};
    double future_reward_weight_{0.25};
    double ranking_period_{3.0};
    double cache_ttl_{12.0};
    double failure_retry_{10.0};
    double service_timeout_{4.0};
    double service_interval_{0.15};
    double cost_origin_distance_{1.5};
    double key_quantization_{0.5};
    double missing_grace_{3.0};
    double no_progress_timeout_{50.0};
    double progress_increment_{0.5};
    double arrival_tolerance_{0.5};
    double observation_dwell_{2.0};
    double visited_cooldown_{30.0};
    double improvement_hysteresis_{0.25};
    double min_commit_{5.0};
    int shortlist_size_{8};
    int lookahead_size_{4};
    int switch_confirmations_{3};
    int min_cost_coverage_{3};
    bool utility_preemption_{false};
    bool diagnostics_{true};
};

}  // namespace frontier_viewpoint_selection
