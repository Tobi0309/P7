#include "frontier_viewpoint_selection/frontier_viewpoint_selection.hpp"

#include <ros2top/ros2top.hpp>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <functional>
#include <iomanip>
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
    command_topic_ = declare_parameter<std::string>("goal_command_topic", "/frontier_viewpoint_selection/goal_command");
    information_weight_ = declare_parameter<double>("information_weight", 1.0);
    path_cost_weight_ = declare_parameter<double>("path_cost_weight", 0.03);
    path_distance_weight_ = declare_parameter<double>("path_distance_weight", 0.8);
    future_reward_weight_ = declare_parameter<double>("future_reward_weight", 0.25);
    ranking_period_ = declare_parameter<double>("ranking_period_seconds", 3.0);
    cache_ttl_ = declare_parameter<double>("path_cost_cache_ttl_seconds", 12.0);
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

    if (shortlist_size_ <= 0 || lookahead_size_ < 0 || switch_confirmations_ <= 0 || min_cost_coverage_ <= 0) {
        throw std::invalid_argument("Candidate counts and confirmation cycles must be positive; pairwise candidates may be zero.");
    }
    if (ranking_period_ <= 0.0 || cache_ttl_ <= 0.0 || failure_retry_ <= 0.0 || service_timeout_ <= 0.0 || service_interval_ < 0.0) {
        throw std::invalid_argument("Invalid DSP cost/ranking timing parameter.");
    }
    if (cost_origin_distance_ <= 0.0 || key_quantization_ <= 0.0) {
        throw std::invalid_argument("Invalid DSP cost-key parameter.");
    }
    if (arrival_tolerance_ <= 0.0 || observation_dwell_ < 0.0 || no_progress_timeout_ <= 0.0 || progress_increment_ <= 0.0) {
        throw std::invalid_argument("Invalid navigation/arrival parameter.");
    }
    if (missing_grace_ < 0.0 || visited_cooldown_ < 0.0 || min_commit_ < 0.0 || improvement_hysteresis_ < 0.0) {
        throw std::invalid_argument("Invalid goal lifecycle parameter.");
    }
    if (information_weight_ < 0.0 || path_cost_weight_ < 0.0 || path_distance_weight_ < 0.0 || future_reward_weight_ < 0.0) {
        throw std::invalid_argument("Utility weights must be nonnegative.");
    }

    robot_orientation_.w = 1.0;
    const auto latched = rclcpp::QoS(1).reliable().transient_local();
    viewpoints_sub_ = create_subscription<Array>(viewpoints_topic_, latched, std::bind(&FrontierViewpointSelection::viewpointsCb, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(odom_topic_, rclcpp::QoS(10), std::bind(&FrontierViewpointSelection::odomCb, this, std::placeholders::_1));
    cost_client_ = create_client<PathCost>(cost_service_);
    planning_failure_service_ = create_service<Trigger>(planning_failure_service_name_, std::bind(&FrontierViewpointSelection::planningFailureCb, this, std::placeholders::_1, std::placeholders::_2));
    finish_service_ = create_service<Trigger>(finish_service_name_, std::bind(&FrontierViewpointSelection::finishCb, this, std::placeholders::_1, std::placeholders::_2));
    command_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(command_topic_, rclcpp::QoS(10).reliable());
    state_pub_ = create_publisher<std_msgs::msg::String>("~/state", rclcpp::QoS(10));
    timer_ = create_wall_timer(std::chrono::milliseconds(200), std::bind(&FrontierViewpointSelection::tick, this));
    registerTop();
    heartbeat_timer_ = create_wall_timer(std::chrono::seconds(3), std::bind(&FrontierViewpointSelection::heartbeatCb, this));
    RCLCPP_INFO(get_logger(), "Viewpoint selector started: DSP path-cost ordering, odometry arrival %.2f m, finish service %s", arrival_tolerance_, finish_service_name_.c_str());
}

FrontierViewpointSelection::~FrontierViewpointSelection() {
    if (cost_inflight_) {
        cost_client_->remove_pending_request(cost_request_id_);
    }
    unregisterTop();
}

void FrontierViewpointSelection::registerTop() {
    const nlohmann::json metadata = {
        {"description", "Committed frontier viewpoint selection with DSP path-cost ordering"},
        {"version", "0.2.1"},
        {"node_type", "exploration_viewpoint_selector"},
        {"topics_subscribed", viewpoints_topic_ + ", " + odom_topic_},
        {"topics_published", command_topic_ + ", ~/state"},
        {"services_provided", finish_service_name_ + ", " + planning_failure_service_name_},
        {"services_consumed", cost_service_}
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
    publishHoldPose();
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

void FrontierViewpointSelection::publishHoldPose() {
    if (!have_odom_ || odom_frame_.empty()) {
        RCLCPP_WARN(get_logger(), "Cannot publish hold command without valid odometry.");
        return;
    }
    geometry_msgs::msg::PoseStamped hold;
    hold.header.frame_id = odom_frame_;
    hold.header.stamp = now();
    hold.pose.position = pointMsg(robot_);
    hold.pose.orientation = robot_orientation_;
    command_pub_->publish(hold);
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
    if (it == costs_.end() || std::chrono::duration<double>(Clock::now() - it->second.updated).count() > cache_ttl_) {
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
    costs_[key] = record;
    if (!record.valid) {
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

std::vector<uint32_t> FrontierViewpointSelection::shortlist() const {
    std::vector<uint32_t> ids;
    ids.reserve(views_.size());
    for (const auto &entry : views_) {
        if (canConsider(entry.first)) {
            ids.push_back(entry.first);
        }
    }
    std::sort(ids.begin(), ids.end(), [this](uint32_t lhs, uint32_t rhs) { return views_.at(lhs).data.objective_score > views_.at(rhs).data.objective_score; });
    if (static_cast<int>(ids.size()) > shortlist_size_) {
        ids.resize(static_cast<std::size_t>(shortlist_size_));
    }
    return ids;
}

std::vector<FrontierViewpointSelection::Ranked> FrontierViewpointSelection::rankFrom(const Vector &start, const std::vector<uint32_t> &ids) const {
    std::vector<Ranked> ranked;
    for (uint32_t id : ids) {
        if (!canConsider(id)) {
            continue;
        }
        const auto &view = views_.at(id).data;
        const auto cost = getCost(start, point(view.viewpoint.pose.position));
        if (!cost || !cost->valid) {
            continue;
        }
        double utility = information_weight_ * view.objective_score - path_cost_weight_ * cost->cost - path_distance_weight_ * cost->distance;
        if (future_reward_weight_ > 0.0) {
            double best_onward = 0.0;
            const Vector from = point(view.viewpoint.pose.position);
            for (uint32_t next_id : ids) {
                if (next_id == id || !canConsider(next_id)) {
                    continue;
                }
                const auto &next = views_.at(next_id).data;
                const auto onward = getCost(from, point(next.viewpoint.pose.position));
                if (!onward || !onward->valid) {
                    continue;
                }
                const double value = information_weight_ * next.objective_score - path_cost_weight_ * onward->cost - path_distance_weight_ * onward->distance;
                best_onward = std::max(best_onward, value);
            }
            utility += future_reward_weight_ * best_onward;
        }
        ranked.push_back({id, utility});
    }
    std::sort(ranked.begin(), ranked.end(), [](const Ranked &lhs, const Ranked &rhs) { return lhs.utility > rhs.utility; });
    return ranked;
}

std::vector<uint32_t> FrontierViewpointSelection::planOrder() const {
    const auto ids = shortlist();
    std::vector<uint32_t> order;
    if (!have_anchor_) {
        return order;
    }
    Vector from = cost_anchor_;
    std::set<uint32_t> seen;
    if (goal_id_) {
        order.push_back(*goal_id_);
        seen.insert(*goal_id_);
        from = point(goal_pose_.pose.position);
    }
    while (order.size() < ids.size() + (goal_id_ ? 1U : 0U)) {
        std::vector<uint32_t> remaining;
        for (uint32_t id : ids) {
            if (!seen.count(id)) {
                remaining.push_back(id);
            }
        }
        if (remaining.empty()) {
            break;
        }
        const auto ranked = rankFrom(from, remaining);
        if (ranked.empty()) {
            break;
        }
        order.push_back(ranked.front().id);
        seen.insert(ranked.front().id);
        from = point(views_.at(ranked.front().id).data.viewpoint.pose.position);
    }
    return order;
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
    for (std::size_t i = 0; i < count; ++i) {
        for (std::size_t j = 0; j < count; ++j) {
            if (i != j) {
                queueCost(point(views_.at(ids[i]).data.viewpoint.pose.position), point(views_.at(ids[j]).data.viewpoint.pose.position));
            }
        }
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
    const double information = information_weight_ * views_.at(*goal_id_).data.objective_score;
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
    state_ = State::NAVIGATING;
    command_pub_->publish(goal_pose_);
    order_ = planOrder();
    RCLCPP_INFO(get_logger(), "Committed frontier %u at (%.2f, %.2f, %.2f).", id, goal_pose_.pose.position.x, goal_pose_.pose.position.y, goal_pose_.pose.position.z);
    logSelectedOrder();
}

void FrontierViewpointSelection::abandonGoal(const char *reason) {
    if (goal_id_) {
        RCLCPP_WARN(get_logger(), "Replanning from frontier %u: %s", *goal_id_, reason);
    }
    // Replaces the old cancel topic: command the robot's current odometry pose.
    publishHoldPose();
    goal_id_.reset();
    ++goal_generation_;
    missing_since_ = Clock::time_point{};
    better_goal_id_.reset();
    better_goal_hits_ = 0;
    last_review_ = Clock::now();
}

void FrontierViewpointSelection::completeGoal() {
    if (!goal_id_) {
        return;
    }
    const uint32_t completed_id = *goal_id_;
    visited_until_[completed_id] = Clock::now() + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(visited_cooldown_));
    goal_id_.reset();
    ++goal_generation_;
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
            return;
        }
        const auto ranked = rankFrom(cost_anchor_, candidates);
        if (!ranked.empty()) {
            startGoal(ranked.front().id);
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
            const auto ranked = rankFrom(cost_anchor_, shortlist());
            if (!ranked.empty() && ranked.front().id != *goal_id_) {
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
    for (const uint32_t id : order_) {
        const auto view = views_.find(id);
        if (view == views_.end() && (!goal_id_ || *goal_id_ != id)) {
            continue;
        }
        const auto &position = goal_id_ && *goal_id_ == id ? goal_pose_.pose.position : view->second.data.viewpoint.pose.position;
        if (!first) {
            verified << " -> ";
        }
        verified << "#" << id << " (" << position.x << ", " << position.y << ", " << position.z << ")";
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
    RCLCPP_INFO(get_logger(), "Visit order selected (DSP-verified): %s | Pending/unverified candidates: %s", verified.str().c_str(), pending.str().c_str());
}

void FrontierViewpointSelection::publishStatus() {
    std_msgs::msg::String state_msg;
    state_msg.data = stateName();
    state_pub_->publish(state_msg);
}

void FrontierViewpointSelection::tick() {
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
            if (std::chrono::duration<double>(current - it->second.updated).count() > 2.0 * cache_ttl_) {
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
