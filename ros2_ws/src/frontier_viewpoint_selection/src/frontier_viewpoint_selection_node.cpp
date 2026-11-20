#include "frontier_viewpoint_selection/frontier_viewpoint_selection.hpp"

#include <ros2top/ros2top.hpp>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <functional>
#include <iomanip>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace frontier_viewpoint_selection {
namespace {

bool validPose(const geometry_msgs::msg::PoseStamped &pose) {
    const auto &p = pose.pose.position;
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}

}  // namespace

FrontierViewpointSelection::Vector FrontierViewpointSelection::point(const geometry_msgs::msg::Point &p) {
    return Vector(p.x, p.y, p.z);
}

geometry_msgs::msg::Point FrontierViewpointSelection::pointMsg(const Vector &p) {
    geometry_msgs::msg::Point result;
    result.x = p.x();
    result.y = p.y();
    result.z = p.z();
    return result;
}

FrontierViewpointSelection::FrontierViewpointSelection() : rclcpp::Node("frontier_viewpoint_selection_node") {
    viewpoints_topic_ = declare_parameter<std::string>("optimized_topic", "/frontier_viewpoints");
    odom_topic_ = declare_parameter<std::string>("odometry_topic", "/Odometry");
    cost_service_ = declare_parameter<std::string>("dsp_cost_service", "/dsp/path_cost");
    planning_failure_service_name_ = declare_parameter<std::string>("planning_failure_service", "/dsp/planning_failure");
    finish_service_name_ = declare_parameter<std::string>("finish_service", "/frontier_viewpoint_selection/finish");
    command_service_ = declare_parameter<std::string>("dsp_set_goal_service", "/dsp/set_goal");
    command_retry_ = declare_parameter<double>("goal_command_retry_seconds", 0.5);
    command_timeout_ = declare_parameter<double>("goal_command_timeout_seconds", 2.0);
    information_weight_ = declare_parameter<double>("information_weight", 1.0);
    path_cost_weight_ = declare_parameter<double>("path_cost_weight", 0.03);
    path_distance_weight_ = declare_parameter<double>("path_distance_weight", 0.8);
    gaussian_weight_ = declare_parameter<double>("gaussian_weight", 15.0);
    gaussian_sigma_xy_ = declare_parameter<double>("gaussian_sigma_xy", 8.0);
    gaussian_sigma_z_ = declare_parameter<double>("gaussian_sigma_z", 3.0);
    gaussian_mean_mode_ = declare_parameter<std::string>("gaussian_mean_mode", "viewpoint_mean");
    gaussian_fixed_mean_ = Vector(declare_parameter<double>("gaussian_fixed_mean_x", 0.0), declare_parameter<double>("gaussian_fixed_mean_y", 0.0), declare_parameter<double>("gaussian_fixed_mean_z", 0.0));
    route_visit_discount_ = declare_parameter<double>("route_visit_discount", 0.95);
    route_initial_wait_ = declare_parameter<double>("route_initial_pairwise_wait_seconds", 3.0);
    route_required_pairwise_edges_ = declare_parameter<int>("route_initial_required_pairwise_edges", 1);
    route_nearby_reserve_ = declare_parameter<int>("route_nearby_candidate_reserve", 2);
    route_corridor_reserve_ = declare_parameter<int>("route_corridor_candidate_reserve", 2);
    route_corridor_width_ = declare_parameter<double>("route_corridor_width_m", 3.0);
    ranking_period_ = declare_parameter<double>("ranking_period_seconds", 3.0);
    cache_ttl_ = declare_parameter<double>("path_cost_cache_ttl_seconds", 12.0);
    pairwise_cache_ttl_ = declare_parameter<double>("pairwise_cost_cache_ttl_seconds", 90.0);
    failure_retry_ = declare_parameter<double>("failed_path_retry_seconds", 10.0);
    service_timeout_ = declare_parameter<double>("dsp_service_timeout_seconds", 4.0);
    service_interval_ = declare_parameter<double>("dsp_request_interval_seconds", 0.15);
    cost_origin_distance_ = declare_parameter<double>("cost_origin_requery_distance", 1.5);
    key_quantization_ = declare_parameter<double>("cost_key_quantization", 0.5);
    missing_grace_ = declare_parameter<double>("missing_goal_grace_seconds", 3.0);
    no_progress_timeout_ = declare_parameter<double>("navigation_no_progress_timeout_seconds", 50.0);
    progress_increment_ = declare_parameter<double>("navigation_progress_increment", 0.5);
    arrival_tolerance_ = declare_parameter<double>("arrival_position_tolerance", 0.5);
    observation_dwell_ = declare_parameter<double>("observation_settle_seconds", 2.0);
    visited_cooldown_ = declare_parameter<double>("visited_frontier_cooldown_seconds", 30.0);
    improvement_hysteresis_ = declare_parameter<double>("switch_utility_margin", 0.25);
    min_commit_ = declare_parameter<double>("minimum_goal_commit_seconds", 5.0);
    shortlist_size_ = declare_parameter<int>("max_rank_candidates", 8);
    lookahead_size_ = declare_parameter<int>("pairwise_cost_candidates", 4);
    switch_confirmations_ = declare_parameter<int>("switch_confirmation_cycles", 3);
    min_cost_coverage_ = declare_parameter<int>("minimum_cost_coverage", 3);
    utility_preemption_ = declare_parameter<bool>("enable_utility_preemption", false);
    diagnostics_ = declare_parameter<bool>("enable_diagnostics", true);

    if (shortlist_size_ <= 0 || shortlist_size_ > 12 || lookahead_size_ < 0 || switch_confirmations_ <= 0 || min_cost_coverage_ <= 0) {
        throw std::invalid_argument("max_rank_candidates must be in [1, 12]; other counts must be positive; pairwise_cost_candidates may be zero.");
    }
    if (ranking_period_ <= 0.0 || cache_ttl_ <= 0.0 || pairwise_cache_ttl_ <= 0.0 || failure_retry_ <= 0.0 || service_timeout_ <= 0.0 || service_interval_ < 0.0) {
        throw std::invalid_argument("Invalid DSP cost/ranking timing parameter.");
    }
    if (cost_origin_distance_ <= 0.0 || key_quantization_ <= 0.0) {
        throw std::invalid_argument("Invalid DSP cost-key parameter.");
    }
    if (command_retry_ <= 0.0 || command_timeout_ <= 0.0) {
        throw std::invalid_argument("goal_command_retry_seconds and goal_command_timeout_seconds must be positive.");
    }
    if (arrival_tolerance_ <= 0.0 || observation_dwell_ < 0.0 || no_progress_timeout_ <= 0.0 || progress_increment_ <= 0.0) {
        throw std::invalid_argument("Invalid navigation/arrival parameter.");
    }
    if (missing_grace_ < 0.0 || visited_cooldown_ < 0.0 || min_commit_ < 0.0 || improvement_hysteresis_ < 0.0) {
        throw std::invalid_argument("Invalid goal lifecycle parameter.");
    }
    if (information_weight_ < 0.0 || path_cost_weight_ < 0.0 || path_distance_weight_ < 0.0 || gaussian_weight_ < 0.0) {
        throw std::invalid_argument("Utility weights must be nonnegative.");
    }
    if (!std::isfinite(gaussian_weight_) || !std::isfinite(gaussian_sigma_xy_) || !std::isfinite(gaussian_sigma_z_) || gaussian_sigma_xy_ <= 0.0 || gaussian_sigma_z_ < 0.0 || !gaussian_fixed_mean_.allFinite()) {
        throw std::invalid_argument("Gaussian parameters must be finite; gaussian_sigma_xy must be positive and gaussian_sigma_z must be nonnegative.");
    }
    if (gaussian_mean_mode_ != "viewpoint_mean" && gaussian_mean_mode_ != "robot" && gaussian_mean_mode_ != "fixed") {
        throw std::invalid_argument("gaussian_mean_mode must be viewpoint_mean, robot, or fixed.");
    }
    if (route_visit_discount_ <= 0.0 || route_visit_discount_ > 1.0 || route_initial_wait_ < 0.0 || route_required_pairwise_edges_ < 0 || route_nearby_reserve_ < 0 || route_corridor_reserve_ < 0 || route_corridor_width_ < 0.0) {
        throw std::invalid_argument("Invalid route-search parameters.");
    }

    robot_orientation_.w = 1.0;
    const auto latched = rclcpp::QoS(1).reliable().transient_local();
    viewpoints_sub_ = create_subscription<Array>(viewpoints_topic_, latched, std::bind(&FrontierViewpointSelection::viewpointsCb, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(odom_topic_, rclcpp::QoS(10), std::bind(&FrontierViewpointSelection::odomCb, this, std::placeholders::_1));

    cost_client_ = create_client<PathCost>(cost_service_);

    planning_failure_service_ = create_service<Trigger>(planning_failure_service_name_, std::bind(&FrontierViewpointSelection::planningFailureCb, this, std::placeholders::_1, std::placeholders::_2));

    finish_service_ = create_service<Trigger>(finish_service_name_, std::bind(&FrontierViewpointSelection::finishCb, this, std::placeholders::_1, std::placeholders::_2));

    command_client_ = create_client<SetGoal>(command_service_);
    state_pub_ = create_publisher<std_msgs::msg::String>("~/state", rclcpp::QoS(10));
    timer_ = create_wall_timer(std::chrono::milliseconds(200), std::bind(&FrontierViewpointSelection::tick, this));
    registerTop();
    heartbeat_timer_ = create_wall_timer(std::chrono::seconds(3), std::bind(&FrontierViewpointSelection::heartbeatCb, this));
    RCLCPP_INFO(get_logger(), "Viewpoint selector started: DSP pairwise route optimization, Gaussian mean=%s weight=%.2f, odometry arrival %.2f m, finish service %s, DSP set_goal service %s (retry %.2f s)", gaussian_mean_mode_.c_str(), gaussian_weight_, arrival_tolerance_, finish_service_name_.c_str(), command_service_.c_str(), command_retry_);
}

FrontierViewpointSelection::~FrontierViewpointSelection() {
    if (cost_inflight_) {
        cost_client_->remove_pending_request(cost_request_id_);
    }
    if (command_inflight_) {
        command_client_->remove_pending_request(command_request_id_);
    }
    unregisterTop();
}

void FrontierViewpointSelection::registerTop() {
    const nlohmann::json metadata = {
        {"description", "Committed frontier viewpoint selection with Gaussian spatial reward and DSP directed route optimization"},
        {"version", "0.3.0"},
        {"node_type", "exploration_viewpoint_selector"},
        {"topics_subscribed", viewpoints_topic_ + ", " + odom_topic_},
        {"topics_published", "~/state"},
        {"services_provided", finish_service_name_ + ", " + planning_failure_service_name_},
        {"services_consumed", cost_service_ + ", " + command_service_}
    };
    ros2top_registered_ = ros2top::register_node("/" + std::string(get_name()), metadata);
    if (!ros2top_registered_) {
        RCLCPP_WARN(get_logger(), "ros2top registration failed.");
    }
}

void FrontierViewpointSelection::unregisterTop() {
    if (ros2top_registered_) {
        ros2top::unregister_node("/" + std::string(get_name()));
    }
}

void FrontierViewpointSelection::heartbeatCb() {
    if (ros2top_registered_) {
        ros2top::heartbeat("/" + std::string(get_name()));
    }
}

void FrontierViewpointSelection::viewpointsCb(Array::SharedPtr msg) {
    if (state_ == State::FINISHED) {
        return;
    }
    frame_ = msg->header.frame_id;
    const auto current = Clock::now();
    std::unordered_map<uint32_t, SavedView> next;
    next.reserve(msg->viewpoints.size());
    for (const auto &view : msg->viewpoints) {
        if (!validPose(view.viewpoint) || !std::isfinite(view.objective_score)) {
            continue;
        }
        next.emplace(view.id, SavedView{view, current});
    }
    views_ = std::move(next);
    have_views_ = true;
    last_view_message_ = current;
}

void FrontierViewpointSelection::odomCb(nav_msgs::msg::Odometry::SharedPtr msg) {
    const Vector position = point(msg->pose.pose.position);
    if (!position.allFinite()) {
        return;
    }
    robot_ = position;
    robot_orientation_ = msg->pose.pose.orientation;
    const double norm_sq = robot_orientation_.x * robot_orientation_.x + robot_orientation_.y * robot_orientation_.y + robot_orientation_.z * robot_orientation_.z + robot_orientation_.w * robot_orientation_.w;
    if (!std::isfinite(norm_sq) || norm_sq < 1e-8) {
        robot_orientation_ = geometry_msgs::msg::Quaternion();
        robot_orientation_.w = 1.0;
    }
    odom_frame_ = msg->header.frame_id;
    have_odom_ = true;
}

void FrontierViewpointSelection::finishCb(const std::shared_ptr<Trigger::Request> /*request*/, std::shared_ptr<Trigger::Response> response) {
    response->success = true;
    response->message = "Selector is FINISHED; no further exploration goals will be selected.";
    if (state_ == State::FINISHED) {
        return;
    }
    state_ = State::FINISHED;
    ++goal_generation_;
    goal_id_.reset();
    order_.clear();
    queue_.clear();
    queued_.clear();
    ++cost_request_token_;
    if (cost_inflight_) {
        cost_client_->remove_pending_request(cost_request_id_);
        cost_inflight_ = false;
    }
    sendHoldPose();
    publishStatus();
    RCLCPP_INFO(get_logger(), "Selector entered terminal FINISHED state.");
}

void FrontierViewpointSelection::planningFailureCb(const std::shared_ptr<Trigger::Request> /*request*/, std::shared_ptr<Trigger::Response> response) {
    // DSP pushes failures by calling this service. std_srvs/Trigger has an empty request,
    // so the originating DSP error text and goal ID cannot be carried by this protocol.
    if (state_ == State::FINISHED) {
        response->success = false;
        response->message = "Selector is FINISHED; planning failure ignored.";
        return;
    }
    if (state_ == State::REPLAN) {
        response->success = true;
        response->message = "Replanning is already pending.";
        return;
    }
    if ((state_ != State::NAVIGATING && state_ != State::REVALIDATE) || !goal_id_) {
        response->success = false;
        response->message = "No active navigation goal to replan.";
        return;
    }
    if (arrived()) {
        reached_since_ = Clock::now();
        state_ = State::GOAL_REACHED;
        response->success = true;
        response->message = "Goal already reached; no replanning required.";
        return;
    }

    const uint32_t failed_id = *goal_id_;
    visited_until_[failed_id] = Clock::now() + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(visited_cooldown_));
    replan_reason_ = "DSP reported failure to plan a route to the committed viewpoint";
    state_ = State::REPLAN;
    response->success = true;
    response->message = "Planning failure acknowledged; selector entering REPLAN.";
    RCLCPP_WARN(get_logger(), "DSP planning failure received for frontier %u; replanning initiated.", failed_id);
}

void FrontierViewpointSelection::sendHoldPose() {
    if (!have_odom_ || odom_frame_.empty()) {
        // Still withdraw an unacknowledged goal: the selector no longer wants it flown.
        cancelCommand();
        RCLCPP_WARN(get_logger(), "Cannot send hold command without valid odometry.");
        return;
    }
    geometry_msgs::msg::PoseStamped hold;
    hold.header.frame_id = odom_frame_;
    hold.header.stamp = now();
    hold.pose.position = pointMsg(robot_);
    hold.pose.orientation = robot_orientation_;
    sendCommand(hold);
}

void FrontierViewpointSelection::sendCommand(const geometry_msgs::msg::PoseStamped &pose) {
    // The newest command replaces any older one that has not been acknowledged yet.
    cancelCommand();
    pending_command_ = pose;
    command_attempts_ = 0;
    command_last_sent_ = Clock::time_point{};
    serviceCommand();
}

void FrontierViewpointSelection::cancelCommand() {
    if (command_inflight_) {
        command_client_->remove_pending_request(command_request_id_);
        command_inflight_ = false;
    }
    ++command_request_token_;
    pending_command_.reset();
}

void FrontierViewpointSelection::serviceCommand() {
    if (!pending_command_) {
        return;
    }
    const auto current = Clock::now();
    const double since_sent = std::chrono::duration<double>(current - command_last_sent_).count();
    if (command_inflight_) {
        if (since_sent < command_timeout_) {
            return;
        }
        // No answer: drop this request and re-send the same pose.
        command_client_->remove_pending_request(command_request_id_);
        ++command_request_token_;
        command_inflight_ = false;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "DSP set_goal service %s did not answer within %.2f s; re-sending.", command_service_.c_str(), command_timeout_);
    } else if (since_sent < command_retry_) {
        return;
    }
    if (!command_client_->service_is_ready()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for DSP set_goal service %s.", command_service_.c_str());
        return;
    }
    auto request = std::make_shared<SetGoal::Request>();
    request->goal = *pending_command_;
    const uint64_t token = ++command_request_token_;
    command_inflight_ = true;
    command_last_sent_ = current;
    ++command_attempts_;
    const auto pending = command_client_->async_send_request(request, [this, token](rclcpp::Client<SetGoal>::SharedFuture future) { commandResponse(token, future); });
    command_request_id_ = pending.request_id;
}

void FrontierViewpointSelection::commandResponse(uint64_t token, rclcpp::Client<SetGoal>::SharedFuture future) {
    if (!command_inflight_ || token != command_request_token_) {
        return;  // Superseded, cancelled or timed out.
    }
    command_inflight_ = false;
    bool success = false;
    std::string message;
    try {
        const auto response = future.get();
        success = response && response->success;
        if (response) {
            message = response->message;
        }
    } catch (const std::exception &error) {
        RCLCPP_WARN(get_logger(), "DSP set_goal service exception: %s", error.what());
    }
    const auto &p = pending_command_->pose.position;
    if (success) {
        // success=true only means DSP accepted the goal; DSP reports later planning failures
        // separately through the planning failure service.
        RCLCPP_INFO(get_logger(), "Goal (%.2f, %.2f, %.2f) accepted by DSP after %u attempt(s): %s", p.x, p.y, p.z, command_attempts_, message.c_str());
        pending_command_.reset();
        return;
    }
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Goal (%.2f, %.2f, %.2f) not accepted by DSP yet (attempt %u): %s; re-sending every %.2f s.", p.x, p.y, p.z, command_attempts_, message.c_str(), command_retry_);
}

FrontierViewpointSelection::PairKey FrontierViewpointSelection::makeKey(const Vector &from, const Vector &to) const {
    PairKey key{};
    for (int axis = 0; axis < 3; ++axis) {
        key[axis] = static_cast<int64_t>(std::llround(from[axis] / key_quantization_));
        key[axis + 3] = static_cast<int64_t>(std::llround(to[axis] / key_quantization_));
    }
    return key;
}

std::optional<FrontierViewpointSelection::CostRecord> FrontierViewpointSelection::getCost(const Vector &from, const Vector &to) const {
    if ((from - to).norm() <= arrival_tolerance_) {
        CostRecord zero;
        zero.valid = true;
        zero.updated = Clock::now();
        return zero;
    }
    const auto it = costs_.find(makeKey(from, to));
    const bool from_anchor = have_anchor_ && makeKey(from, from) == makeKey(cost_anchor_, cost_anchor_);
    const double ttl = from_anchor ? cache_ttl_ : pairwise_cache_ttl_;
    if (it == costs_.end() || std::chrono::duration<double>(Clock::now() - it->second.updated).count() > ttl) {
        return std::nullopt;
    }
    return it->second;
}

void FrontierViewpointSelection::queueCost(const Vector &from, const Vector &to) {
    if ((from - to).norm() <= arrival_tolerance_) {
        return;
    }
    const PairKey key = makeKey(from, to);
    if (getCost(from, to) || queued_.count(key) || (cost_inflight_ && cost_inflight_edge_.key == key)) {
        return;
    }
    const auto failure = failed_until_.find(key);
    if (failure != failed_until_.end() && Clock::now() < failure->second) {
        return;
    }
    queued_.insert(key);
    queue_.push_back({key, from, to});
}

void FrontierViewpointSelection::costResponse(uint64_t token, PairKey key, rclcpp::Client<PathCost>::SharedFuture future) {
    if (state_ == State::FINISHED || !cost_inflight_ || token != cost_request_token_ || cost_inflight_edge_.key != key) {
        return;
    }
    CostRecord record;
    record.updated = Clock::now();
    try {
        const auto response = future.get();
        record.valid = response && response->path.poses.size() >= 2 && std::isfinite(response->cost) && std::isfinite(response->distance) && response->cost >= 0.0 && response->distance > 0.0;
        if (record.valid) {
            record.cost = response->cost;
            record.distance = response->distance;
        }
    } catch (const std::exception &error) {
        RCLCPP_WARN(get_logger(), "DSP PathCost exception: %s", error.what());
    }
    if (record.valid) {
        costs_[key] = record;
        failed_until_.erase(key);
    } else {
        costs_.erase(key);
        failed_until_[key] = Clock::now() + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(failure_retry_));
    }
    cost_inflight_ = false;
}

void FrontierViewpointSelection::requestNextCost() {
    const auto current = Clock::now();
    if (cost_inflight_) {
        if (std::chrono::duration<double>(current - cost_request_started_).count() >= service_timeout_) {
            cost_client_->remove_pending_request(cost_request_id_);
            failed_until_[cost_inflight_edge_.key] = current + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(failure_retry_));
            ++cost_request_token_;
            cost_inflight_ = false;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "DSP path-cost request timed out.");
        }
        return;
    }
    if (!cost_client_->service_is_ready()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for DSP path-cost service %s.", cost_service_.c_str());
        return;
    }
    if (std::chrono::duration<double>(current - last_cost_request_sent_).count() < service_interval_) {
        return;
    }
    const auto relevant_ids = shortlist();
    const auto endpointIsRelevant = [&](const Vector &p) {
        if (makeKey(p, p) == makeKey(cost_anchor_, cost_anchor_)) {
            return true;
        }
        if (goal_id_ && makeKey(p, p) == makeKey(point(goal_pose_.pose.position), point(goal_pose_.pose.position))) {
            return true;
        }
        for (uint32_t id : relevant_ids) {
            const Vector destination = point(views_.at(id).data.viewpoint.pose.position);
            if (makeKey(p, p) == makeKey(destination, destination)) {
                return true;
            }
        }
        return false;
    };
    while (!queue_.empty()) {
        EdgeRequest edge = queue_.front();
        queue_.pop_front();
        queued_.erase(edge.key);
        if (!endpointIsRelevant(edge.from) || !endpointIsRelevant(edge.to)) {
            continue;
        }
        const auto failed = failed_until_.find(edge.key);
        if (getCost(edge.from, edge.to) || (failed != failed_until_.end() && current < failed->second)) {
            continue;
        }
        auto request = std::make_shared<PathCost::Request>();
        request->start = pointMsg(edge.from);
        request->stop = pointMsg(edge.to);
        const uint64_t token = ++cost_request_token_;
        cost_inflight_edge_ = edge;
        cost_inflight_ = true;
        cost_request_started_ = current;
        last_cost_request_sent_ = current;
        const auto pending = cost_client_->async_send_request(request, [this, token, key = edge.key](rclcpp::Client<PathCost>::SharedFuture future) { costResponse(token, key, future); });
        cost_request_id_ = pending.request_id;
        return;
    }
}

bool FrontierViewpointSelection::canConsider(uint32_t id) const {
    if (views_.find(id) == views_.end()) {
        return false;
    }
    const auto cooldown = visited_until_.find(id);
    return cooldown == visited_until_.end() || Clock::now() >= cooldown->second;
}

FrontierViewpointSelection::Vector FrontierViewpointSelection::gaussianMean() const {
    if (gaussian_mean_mode_ == "robot") {
        return robot_;
    }
    if (gaussian_mean_mode_ == "fixed") {
        return gaussian_fixed_mean_;
    }
    Vector mean = Vector::Zero();
    std::size_t count = 0;
    for (const auto &entry : views_) {
        if (canConsider(entry.first)) {
            mean += point(entry.second.data.viewpoint.pose.position);
            ++count;
        }
    }
    if (count == 0) {
        return robot_;
    }
    return mean / static_cast<double>(count);
}

double FrontierViewpointSelection::gaussianBonus(uint32_t id) const {
    if (gaussian_weight_ == 0.0 || views_.count(id) == 0) {
        return 0.0;
    }
    const Vector delta = point(views_.at(id).data.viewpoint.pose.position) - gaussianMean();
    const double horizontal = (delta.x() * delta.x() + delta.y() * delta.y()) / (gaussian_sigma_xy_ * gaussian_sigma_xy_);
    const double vertical = gaussian_sigma_z_ == 0.0 ? 0.0 : (delta.z() * delta.z()) / (gaussian_sigma_z_ * gaussian_sigma_z_);
    return gaussian_weight_ * std::exp(-0.5 * (horizontal + vertical));
}

double FrontierViewpointSelection::viewpointReward(uint32_t id) const {
    const auto item = views_.find(id);
    if (item == views_.end()) {
        return -1.0e30;
    }
    return information_weight_ * item->second.data.objective_score + gaussianBonus(id);
}

std::vector<uint32_t> FrontierViewpointSelection::shortlist() const {
    std::vector<uint32_t> available;
    available.reserve(views_.size());
    for (const auto &entry : views_) {
        if (canConsider(entry.first)) {
            available.push_back(entry.first);
        }
    }
    std::sort(available.begin(), available.end(), [this](uint32_t a, uint32_t b) { return viewpointReward(a) > viewpointReward(b); });
    if (static_cast<int>(available.size()) <= shortlist_size_) {
        return available;
    }

    // Reserve slots for nearby and on-the-way candidates. An exclusively top-score
    // shortlist can exclude worthwhile intermediate stops before DSP sees them.
    const int nearby_slots = std::min(route_nearby_reserve_, shortlist_size_ - 1);
    const int corridor_slots = std::min(route_corridor_reserve_, shortlist_size_ - 1 - nearby_slots);
    const int core_slots = shortlist_size_ - nearby_slots - corridor_slots;
    std::vector<uint32_t> selected;
    selected.reserve(static_cast<std::size_t>(shortlist_size_));
    const auto addUnique = [&](uint32_t id) {
        if (selected.size() < static_cast<std::size_t>(shortlist_size_) && std::find(selected.begin(), selected.end(), id) == selected.end()) {
            selected.push_back(id);
            return true;
        }
        return false;
    };
    for (int i = 0; i < core_slots; ++i) {
        addUnique(available[static_cast<std::size_t>(i)]);
    }

    if (have_odom_ && corridor_slots > 0) {
        struct Corridor { uint32_t id; double merit; };
        std::vector<Corridor> corridor;
        for (uint32_t id : available) {
            if (std::find(selected.begin(), selected.end(), id) != selected.end()) {
                continue;
            }
            const Vector target = point(views_.at(id).data.viewpoint.pose.position);
            double nearest_lateral = std::numeric_limits<double>::infinity();
            for (std::size_t leader = 0; leader < std::min<std::size_t>(2, available.size()); ++leader) {
                const Vector major = point(views_.at(available[leader]).data.viewpoint.pose.position) - robot_;
                const double norm_sq = major.squaredNorm();
                if (norm_sq <= 1.0e-6) {
                    continue;
                }
                const double t = (target - robot_).dot(major) / norm_sq;
                if (t <= 0.05 || t >= 0.98) {
                    continue;
                }
                const double lateral = (target - (robot_ + t * major)).norm();
                nearest_lateral = std::min(nearest_lateral, lateral);
            }
            if (nearest_lateral <= route_corridor_width_) {
                corridor.push_back({id, viewpointReward(id) - path_distance_weight_ * nearest_lateral});
            }
        }
        std::sort(corridor.begin(), corridor.end(), [](const Corridor &a, const Corridor &b) { return a.merit > b.merit; });
        int added = 0;
        for (const auto &entry : corridor) {
            if (added >= corridor_slots) {
                break;
            }
            if (addUnique(entry.id)) {
                ++added;
            }
        }
    }

    if (have_odom_ && nearby_slots > 0) {
        std::vector<uint32_t> by_distance = available;
        std::sort(by_distance.begin(), by_distance.end(), [this](uint32_t a, uint32_t b) {
            const double da = (point(views_.at(a).data.viewpoint.pose.position) - robot_).norm();
            const double db = (point(views_.at(b).data.viewpoint.pose.position) - robot_).norm();
            return da < db;
        });
        int added = 0;
        for (uint32_t id : by_distance) {
            if (added >= nearby_slots) {
                break;
            }
            if (addUnique(id)) {
                ++added;
            }
        }
    }
    for (uint32_t id : available) {
        addUnique(id);
    }
    return selected;
}

std::vector<FrontierViewpointSelection::Ranked> FrontierViewpointSelection::rankFrom(const Vector &start, const std::vector<uint32_t> &ids) const {
    std::vector<Ranked> ranked;
    for (uint32_t id : ids) {
        if (!canConsider(id)) {
            continue;
        }
        const auto cost = getCost(start, point(views_.at(id).data.viewpoint.pose.position));
        if (!cost || !cost->valid) {
            continue;
        }
        const double utility = viewpointReward(id) - path_cost_weight_ * cost->cost - path_distance_weight_ * cost->distance;
        ranked.push_back({id, utility});
    }
    std::sort(ranked.begin(), ranked.end(), [](const Ranked &a, const Ranked &b) { return a.utility > b.utility; });
    return ranked;
}

FrontierViewpointSelection::RouteSolution FrontierViewpointSelection::optimizeRoute(const Vector &start, const std::vector<uint32_t> &ids) const {
    RouteSolution solution;
    const std::size_t n = ids.size();
    if (n == 0) {
        return solution;
    }

    // Dynamic programming over verified, directed DSP edges. The objective is
    // sum_k discount^k * (information + spatial Gaussian - edge travel penalties).
    // This searches the best verified subset and order, not only the next hop.
    struct Cell {
        double utility{-std::numeric_limits<double>::infinity()};
        int previous{-1};
    };
    const std::size_t masks = std::size_t{1} << n;
    std::vector<Cell> dp(masks * n);
    const auto at = [n, &dp](std::size_t mask, std::size_t last) -> Cell & { return dp[mask * n + last]; };
    std::vector<Vector> poses;
    std::vector<double> rewards;
    poses.reserve(n);
    rewards.reserve(n);
    for (uint32_t id : ids) {
        poses.push_back(point(views_.at(id).data.viewpoint.pose.position));
        rewards.push_back(viewpointReward(id));
    }
    for (std::size_t next = 0; next < n; ++next) {
        const auto edge = getCost(start, poses[next]);
        if (edge && edge->valid) {
            at(std::size_t{1} << next, next).utility = rewards[next] - path_cost_weight_ * edge->cost - path_distance_weight_ * edge->distance;
        }
    }

    std::size_t best_mask = 0;
    int best_last = -1;
    for (std::size_t mask = 1; mask < masks; ++mask) {
        const unsigned visited_count = static_cast<unsigned>(__builtin_popcountll(static_cast<unsigned long long>(mask)));
        const double multiplier = std::pow(route_visit_discount_, static_cast<double>(visited_count));
        for (std::size_t last = 0; last < n; ++last) {
            if ((mask & (std::size_t{1} << last)) == 0) {
                continue;
            }
            const Cell current = at(mask, last);
            if (!std::isfinite(current.utility)) {
                continue;
            }
            if (current.utility > solution.utility) {
                solution.utility = current.utility;
                best_mask = mask;
                best_last = static_cast<int>(last);
            }
            for (std::size_t next = 0; next < n; ++next) {
                if ((mask & (std::size_t{1} << next)) != 0) {
                    continue;
                }
                const auto edge = getCost(poses[last], poses[next]);
                if (!edge || !edge->valid) {
                    continue;
                }
                const double added = multiplier * (rewards[next] - path_cost_weight_ * edge->cost - path_distance_weight_ * edge->distance);
                Cell &destination = at(mask | (std::size_t{1} << next), next);
                if (current.utility + added > destination.utility) {
                    destination.utility = current.utility + added;
                    destination.previous = static_cast<int>(last);
                }
            }
        }
    }
    while (best_last >= 0) {
        solution.ids.push_back(ids[static_cast<std::size_t>(best_last)]);
        const int previous = at(best_mask, static_cast<std::size_t>(best_last)).previous;
        best_mask &= ~(std::size_t{1} << static_cast<std::size_t>(best_last));
        best_last = previous;
    }
    std::reverse(solution.ids.begin(), solution.ids.end());
    return solution;
}

std::vector<uint32_t> FrontierViewpointSelection::planOrder(bool include_committed) const {
    std::vector<uint32_t> order;
    if (!have_anchor_) {
        return order;
    }
    Vector from = cost_anchor_;
    std::vector<uint32_t> ids = shortlist();
    if (include_committed && goal_id_) {
        order.push_back(*goal_id_);
        from = point(goal_pose_.pose.position);
        ids.erase(std::remove(ids.begin(), ids.end(), *goal_id_), ids.end());
    }
    RouteSolution suffix = optimizeRoute(from, ids);
    order.insert(order.end(), suffix.ids.begin(), suffix.ids.end());
    return order;
}

int FrontierViewpointSelection::verifiedPairwiseEdges(const std::vector<uint32_t> &ids) const {
    const std::size_t count = std::min(ids.size(), static_cast<std::size_t>(lookahead_size_));
    int verified = 0;
    for (std::size_t i = 0; i < count; ++i) {
        for (std::size_t j = 0; j < count; ++j) {
            if (i == j) {
                continue;
            }
            const auto cost = getCost(point(views_.at(ids[i]).data.viewpoint.pose.position), point(views_.at(ids[j]).data.viewpoint.pose.position));
            if (cost && cost->valid) {
                ++verified;
            }
        }
    }
    return verified;
}

void FrontierViewpointSelection::updateCostAnchor() {
    if (!have_anchor_ || (robot_ - cost_anchor_).norm() > cost_origin_distance_) {
        cost_anchor_ = robot_;
        have_anchor_ = true;
        queue_.clear();
        queued_.clear();
        last_ranking_ = Clock::time_point{};
    }
}

void FrontierViewpointSelection::refreshQueries() {
    if (!have_odom_ || !have_views_ || frame_.empty() || frame_ != odom_frame_) {
        return;
    }
    const auto ids = shortlist();
    for (uint32_t id : ids) {
        queueCost(cost_anchor_, point(views_.at(id).data.viewpoint.pose.position));
    }
    if (goal_id_) {
        queueCost(cost_anchor_, point(goal_pose_.pose.position));
    }
    if (lookahead_size_ <= 0) {
        return;
    }
    const std::size_t count = std::min(ids.size(), static_cast<std::size_t>(lookahead_size_));
    struct DirectedPair { Vector from; Vector to; double distance; };
    std::vector<DirectedPair> pairs;
    pairs.reserve(count * (count - (count > 0 ? 1U : 0U)));
    for (std::size_t i = 0; i < count; ++i) {
        for (std::size_t j = 0; j < count; ++j) {
            if (i == j) {
                continue;
            }
            const Vector from = point(views_.at(ids[i]).data.viewpoint.pose.position);
            const Vector to = point(views_.at(ids[j]).data.viewpoint.pose.position);
            pairs.push_back({from, to, (from - to).norm()});
        }
    }
    std::sort(pairs.begin(), pairs.end(), [](const DirectedPair &a, const DirectedPair &b) { return a.distance < b.distance; });
    for (const auto &pair : pairs) {
        queueCost(pair.from, pair.to);
    }
    if (goal_id_) {
        for (std::size_t j = 0; j < count; ++j) {
            if (*goal_id_ != ids[j]) {
                queueCost(point(goal_pose_.pose.position), point(views_.at(ids[j]).data.viewpoint.pose.position));
            }
        }
    }
}

bool FrontierViewpointSelection::currentGoalAvailable() const {
    return goal_id_ && views_.count(*goal_id_) != 0;
}

bool FrontierViewpointSelection::arrived() const {
    return goal_id_ && have_odom_ && (robot_ - point(goal_pose_.pose.position)).norm() <= arrival_tolerance_;
}

double FrontierViewpointSelection::currentUtility() const {
    if (!goal_id_ || views_.find(*goal_id_) == views_.end()) {
        return -1.0e30;
    }
    const double information = viewpointReward(*goal_id_);
    const auto cost = getCost(cost_anchor_, point(goal_pose_.pose.position));
    if (!cost || !cost->valid) {
        return information;
    }
    return information - path_cost_weight_ * cost->cost - path_distance_weight_ * cost->distance;
}

void FrontierViewpointSelection::startGoal(uint32_t id) {
    const auto view = views_.find(id);
    if (view == views_.end()) {
        return;
    }
    goal_id_ = id;
    goal_pose_ = view->second.data.viewpoint;
    goal_pose_.header.frame_id = frame_;
    goal_pose_.header.stamp = now();
    ++goal_generation_;
    goal_started_ = Clock::now();
    last_progress_time_ = goal_started_;
    closest_goal_distance_ = (robot_ - point(goal_pose_.pose.position)).norm();
    missing_since_ = Clock::time_point{};
    better_goal_id_.reset();
    better_goal_hits_ = 0;
    last_review_ = goal_started_;
    select_ready_since_ = Clock::time_point{};
    state_ = State::NAVIGATING;
    sendCommand(goal_pose_);
    order_ = planOrder();
    RCLCPP_INFO(get_logger(), "Committed frontier %u at (%.2f, %.2f, %.2f).", id, goal_pose_.pose.position.x, goal_pose_.pose.position.y, goal_pose_.pose.position.z);
    logSelectedOrder();
}

void FrontierViewpointSelection::abandonGoal(const char *reason) {
    if (goal_id_) {
        RCLCPP_WARN(get_logger(), "Replanning from frontier %u: %s", *goal_id_, reason);
    }
    // Replaces the old cancel topic: command the robot's current odometry pose.
    sendHoldPose();
    goal_id_.reset();
    ++goal_generation_;
    missing_since_ = Clock::time_point{};
    better_goal_id_.reset();
    better_goal_hits_ = 0;
    last_review_ = Clock::now();
    select_ready_since_ = Clock::time_point{};
}

void FrontierViewpointSelection::completeGoal() {
    if (!goal_id_) {
        return;
    }
    const uint32_t completed_id = *goal_id_;
    visited_until_[completed_id] = Clock::now() + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(visited_cooldown_));
    goal_id_.reset();
    ++goal_generation_;
    select_ready_since_ = Clock::time_point{};
    RCLCPP_INFO(get_logger(), "Reached frontier %u; observation dwell completed.", completed_id);
}

std::string FrontierViewpointSelection::stateName() const {
    switch (state_) {
        case State::SELECT_GOAL: return "SELECT_GOAL";
        case State::NAVIGATING: return "NAVIGATING";
        case State::REVALIDATE: return "REVALIDATE";
        case State::REPLAN: return "REPLAN";
        case State::GOAL_REACHED: return "GOAL_REACHED";
        case State::FINISHED: return "FINISHED";
    }
    return "UNKNOWN";
}

void FrontierViewpointSelection::processStateMachine() {
    const auto current = Clock::now();
    if (state_ == State::FINISHED) {
        return;
    }
    if (state_ == State::SELECT_GOAL) {
        const auto candidates = shortlist();
        if (candidates.empty()) {
            select_ready_since_ = Clock::time_point{};
            return;
        }
        const int required = std::min(min_cost_coverage_, static_cast<int>(candidates.size()));
        int checked = 0;
        for (uint32_t id : candidates) {
            const Vector destination = point(views_.at(id).data.viewpoint.pose.position);
            const PairKey key = makeKey(cost_anchor_, destination);
            if (getCost(cost_anchor_, destination) || (failed_until_.count(key) && current < failed_until_.at(key))) {
                ++checked;
            }
        }
        if (checked < required) {
            select_ready_since_ = Clock::time_point{};
            return;
        }
        if (select_ready_since_ == Clock::time_point{}) {
            select_ready_since_ = current;
        }
        if (candidates.size() > 1 && lookahead_size_ > 1 && verifiedPairwiseEdges(candidates) < route_required_pairwise_edges_ && std::chrono::duration<double>(current - select_ready_since_).count() < route_initial_wait_) {
            return;
        }
        const auto order = planOrder(false);
        if (!order.empty()) {
            startGoal(order.front());
        }
        return;
    }
    if (state_ == State::NAVIGATING) {
        // Completion is determined by odometry, not a succeeded subscription.
        if (arrived()) {
            reached_since_ = current;
            state_ = State::GOAL_REACHED;
            RCLCPP_INFO(get_logger(), "Reached goal position within %.2f m.", arrival_tolerance_);
            return;
        }
        const double distance = (robot_ - point(goal_pose_.pose.position)).norm();
        if (distance < closest_goal_distance_ - progress_increment_) {
            closest_goal_distance_ = distance;
            last_progress_time_ = current;
        }
        if (std::chrono::duration<double>(current - last_progress_time_).count() > no_progress_timeout_) {
            if (goal_id_) {
                visited_until_[*goal_id_] = current + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(visited_cooldown_));
            }
            replan_reason_ = "no odometry progress";
            state_ = State::REPLAN;
            return;
        }
        if (std::chrono::duration<double>(current - last_review_).count() >= ranking_period_) {
            state_ = State::REVALIDATE;
        }
        return;
    }
    if (state_ == State::REVALIDATE) {
        if (arrived()) {
            reached_since_ = current;
            state_ = State::GOAL_REACHED;
            return;
        }
        if (!currentGoalAvailable()) {
            if (missing_since_ == Clock::time_point{}) {
                missing_since_ = current;
            }
            if (std::chrono::duration<double>(current - missing_since_).count() >= missing_grace_) {
                replan_reason_ = "committed viewpoint disappeared from latest optimizer output";
                state_ = State::REPLAN;
            } else {
                state_ = State::NAVIGATING;
            }
            last_review_ = current;
            return;
        }
        missing_since_ = Clock::time_point{};
        if (utility_preemption_ && std::chrono::duration<double>(current - goal_started_).count() >= min_commit_) {
            const auto order = planOrder(false);
            if (!order.empty() && order.front() != *goal_id_) {
                const auto ranked = rankFrom(cost_anchor_, {order.front()});
                if (ranked.empty()) {
                    last_review_ = current;
                    state_ = State::NAVIGATING;
                    return;
                }
                const double incumbent = currentUtility();
                const double margin = improvement_hysteresis_ * std::max(1.0, std::abs(incumbent));
                if (ranked.front().utility > incumbent + margin) {
                    if (better_goal_id_ == ranked.front().id) {
                        ++better_goal_hits_;
                    } else {
                        better_goal_id_ = ranked.front().id;
                        better_goal_hits_ = 1;
                    }
                    if (better_goal_hits_ >= switch_confirmations_) {
                        replan_reason_ = "higher-utility frontier consistently outranks committed goal";
                        state_ = State::REPLAN;
                        return;
                    }
                } else {
                    better_goal_id_.reset();
                    better_goal_hits_ = 0;
                }
            } else {
                better_goal_id_.reset();
                better_goal_hits_ = 0;
            }
        }
        last_review_ = current;
        state_ = State::NAVIGATING;
        return;
    }
    if (state_ == State::REPLAN) {
        abandonGoal(replan_reason_.c_str());
        state_ = State::SELECT_GOAL;
        return;
    }
    if (state_ == State::GOAL_REACHED) {
        if (std::chrono::duration<double>(current - reached_since_).count() >= observation_dwell_) {
            completeGoal();
            state_ = State::SELECT_GOAL;
        }
    }
}

void FrontierViewpointSelection::logSelectedOrder() const {
    std::ostringstream verified;
    verified << std::fixed << std::setprecision(2);
    bool first = true;
    Vector previous = cost_anchor_;
    for (const uint32_t id : order_) {
        const auto view = views_.find(id);
        if (view == views_.end() && (!goal_id_ || *goal_id_ != id)) {
            continue;
        }
        const auto &position = goal_id_ && *goal_id_ == id ? goal_pose_.pose.position : view->second.data.viewpoint.pose.position;
        if (!first) {
            verified << " -> ";
        }
        const Vector destination = point(position);
        const auto edge = getCost(previous, destination);
        verified << "#" << id << " (" << position.x << ", " << position.y << ", " << position.z << ")";
        if (edge && edge->valid) {
            verified << " {DSP leg=" << edge->distance << "m, cost=" << edge->cost << "}";
        } else if (first && goal_id_ && id == *goal_id_) {
            verified << " {committed; current DSP leg unavailable}";
        }
        if (views_.count(id) != 0) {
            verified << " {Reward: Information=" << viewpointReward(id) - gaussianBonus(id) << ", Gaussian=" << gaussianBonus(id) << "}";
        }
        previous = destination;
        first = false;
    }
    if (first) {
        verified << "none";
    }
    std::ostringstream pending;
    std::size_t remaining = 0;
    for (const uint32_t id : shortlist()) {
        if (std::find(order_.begin(), order_.end(), id) == order_.end()) {
            if (remaining++ > 0) {
                pending << ", ";
            }
            pending << "#" << id;
        }
    }
    if (remaining == 0) {
        pending << "none";
    }
    const Vector mean = gaussianMean();
    RCLCPP_INFO(get_logger(), "Visit order selected (DSP-directed route search, spatial Gaussian mean [%.2f, %.2f, %.2f]): %s | Not yet included (unverified or lower utility): %s", mean.x(), mean.y(), mean.z(), verified.str().c_str(), pending.str().c_str());
}

void FrontierViewpointSelection::publishStatus() {
    std_msgs::msg::String state_msg;
    state_msg.data = stateName();
    state_pub_->publish(state_msg);
}

void FrontierViewpointSelection::tick() {
    // Before every early return: an unacknowledged command (including the FINISHED hold) keeps being re-sent.
    serviceCommand();
    if (state_ == State::FINISHED) {
        publishStatus();
        return;
    }
    if (!have_odom_ || !have_views_) {
        publishStatus();
        return;
    }
    if (frame_.empty() || frame_ != odom_frame_) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Frame mismatch: viewpoints='%s' odometry='%s'. Provide poses in one world frame.", frame_.c_str(), odom_frame_.c_str());
        publishStatus();
        return;
    }
    updateCostAnchor();
    refreshQueries();
    requestNextCost();
    processStateMachine();
    const auto current = Clock::now();
    if (std::chrono::duration<double>(current - last_ranking_).count() >= ranking_period_) {
        order_ = planOrder();
        last_ranking_ = current;
        for (auto it = costs_.begin(); it != costs_.end();) {
            const auto anchor_key = makeKey(cost_anchor_, cost_anchor_);
            const bool from_anchor = have_anchor_ && it->first[0] == anchor_key[0] && it->first[1] == anchor_key[1] && it->first[2] == anchor_key[2];
            const double ttl = from_anchor ? cache_ttl_ : pairwise_cache_ttl_;
            if (std::chrono::duration<double>(current - it->second.updated).count() > 2.0 * ttl) {
                it = costs_.erase(it);
            } else {
                ++it;
            }
        }
        for (auto it = failed_until_.begin(); it != failed_until_.end();) {
            if (it->second < current) {
                it = failed_until_.erase(it);
            } else {
                ++it;
            }
        }
        if (diagnostics_) {
            const std::string id_text = goal_id_ ? std::to_string(*goal_id_) : "none";
            RCLCPP_INFO(get_logger(), "Selector state=%s viewpoints=%zu ordered_poses=%zu cached_edges=%zu queued_edges=%zu goal=%s", stateName().c_str(), views_.size(), order_.size(), costs_.size(), queue_.size(), id_text.c_str());
        }
    }
    publishStatus();
}

}  // namespace frontier_viewpoint_selection

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<frontier_viewpoint_selection::FrontierViewpointSelection>());
    rclcpp::shutdown();
    return 0;
}
