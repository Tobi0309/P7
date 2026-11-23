#include "dsp.hpp"

#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <nlohmann/json.hpp>
#include <ros2top/ros2top.hpp>

#include <limits>
#include <thread>


namespace dsp
{

namespace
{

bool samePose(const geometry_msgs::msg::Pose &a, const geometry_msgs::msg::Pose &b)
{
    const Eigen::Vector3d dp(a.position.x - b.position.x, a.position.y - b.position.y,
                             a.position.z - b.position.z);
    const double dq = std::abs(a.orientation.x - b.orientation.x) +
                      std::abs(a.orientation.y - b.orientation.y) +
                      std::abs(a.orientation.z - b.orientation.z) +
                      std::abs(a.orientation.w - b.orientation.w);
    return dp.norm() < 1e-3 && dq < 1e-4;
}

double millisecondsSince(std::chrono::steady_clock::time_point start)
{
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
}

}  // namespace

Dsp::Dsp() : Node("dsp")
{
    this->declare_parameter("map_topic", "octomap_full");
    this->declare_parameter("spline_step", 0.01);
    this->declare_parameter("lower_thresh", 59);
    this->declare_parameter("upper_thresh", 60);
    // Voxel padding: every voxel whose centre is closer than obstacle_padding [m] to the surface
    // of an occupied voxel is blocked. obstacle_padding_enabled=false turns padding off. Changing
    // either at runtime reallocates the grid on the next map update.
    this->declare_parameter("obstacle_padding_enabled", true);
    this->declare_parameter("obstacle_padding", 0.5);
    // Rate [Hz] of periodic replanning towards the current goal. <= 0 plans only on new goals.
    this->declare_parameter("plan_rate", 0.2);
    // Rate [Hz] of publishing the look-ahead goal on /move_base_simple/goal. <= 0 disables it.
    this->declare_parameter("goal_publish_rate", 1.0);
    // Rate [Hz] at which the newest map message is applied. <= 0 applies every message.
    this->declare_parameter("map_update_rate", 2.0);
    // Slack [m] around the octree bounds when the voxel grid is (re)allocated, so the grid and
    // the search graphs on it are not rebuilt every time the map grows a little.
    this->declare_parameter("map_bounds_margin_xy", 10.0);
    this->declare_parameter("map_bounds_margin_z", 3.0);
    // A start or goal inside a blocked voxel (e.g. the drone on the ground, inside the floor's
    // padding) is moved to the nearest free voxel within this radius [m].
    this->declare_parameter("endpoint_snap_radius", 1.0);
    // Each search (navigation and path_cost) releases its lazily built graph above this many
    // vertices, to bound memory (roughly 12-25 KB per vertex). <= 0 disables the limit.
    this->declare_parameter("max_search_vertices", 50000);
    this->declare_parameter("use_odometry", false);
    this->declare_parameter("use_3d", true);
    this->declare_parameter("odom_topic", "odometry/imu");
    this->declare_parameter("odom_frame_id", "map");
    this->declare_parameter("base_link_frame_id", "base_link");
    this->declare_parameter("DSP_UNKNOWN", 10000);
    this->declare_parameter("update_rate", 1);
    this->declare_parameter("debug", false);
    this->declare_parameter("height_kernel_size", 3);
    this->declare_parameter("optimal_voxel_height", 6);
    this->declare_parameter("optimal_voxel_height_cost", 10);
    // Number of path voxels ahead of the drone's closest path point to send as goal.
    this->declare_parameter("goal_voxels_ahead", 12);

    spline_step_ = this->get_parameter("spline_step").as_double();
    dsp_unknown_ = this->get_parameter("DSP_UNKNOWN").as_int();

    const auto exclusive = rclcpp::CallbackGroupType::MutuallyExclusive;
    goal_group_ = this->create_callback_group(exclusive);
    nav_group_ = this->create_callback_group(exclusive);
    publish_group_ = this->create_callback_group(exclusive);
    query_group_ = this->create_callback_group(exclusive);
    map_group_ = this->create_callback_group(exclusive);
    misc_group_ = this->create_callback_group(exclusive);

    occ_map_viz_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("dsp/occupancy_map",1);

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("dsp/path", 1);
    splinepath_pub_ =  this->create_publisher<nav_msgs::msg::Path>("dsp/spline_path", 1);
    goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/move_base_simple/goal", 1);

    rclcpp::SubscriptionOptions misc_options;
    misc_options.callback_group = misc_group_;
    rclcpp::SubscriptionOptions map_options;
    map_options.callback_group = map_group_;

    if(this->get_parameter("use_odometry").as_bool())
    {
        set_start_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            this->get_parameter("odom_topic").as_string(), 1,
            std::bind(&Dsp::handleSetStartOdom, this, std::placeholders::_1), misc_options);
    }
    else
    {
        set_start_sub_ = this->create_subscription<geometry_msgs::msg::Point>("dsp/set_start", 1,
            std::bind(&Dsp::handleSetStart, this, std::placeholders::_1), misc_options);
    }

    if(this->get_parameter("use_3d").as_bool()){
        get_octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
            this->get_parameter("map_topic").as_string(), 1,
            std::bind(&Dsp::octomap_data_callback, this, std::placeholders::_1), map_options);
    }
    else {
        get_ocupancy_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            this->get_parameter("map_topic").as_string(), 1,
            std::bind(&Dsp::occupancy_grid_callback, this, std::placeholders::_1), map_options);
        height_voxel = 1;
    }

    // Answered immediately: the goal is stored and the planner (nav group) picks it up.
    set_goal_srv_ = this->create_service<dsp_interfaces::srv::SetGoal>("dsp/set_goal",
        std::bind(&Dsp::handleSetGoal, this, std::placeholders::_1, std::placeholders::_2),
        rclcpp::ServicesQoS(), goal_group_);

    cost_srv_ = this->create_service<dsp_interfaces::srv::PathCost>("/dsp/path_cost",
        std::bind(&Dsp::request_cost, this, std::placeholders::_1, std::placeholders::_2),
        rclcpp::ServicesQoS(), query_group_);

    // Planning failures are pushed to this receiver (frontier_viewpoint_selection serves it).
    planning_failure_client_ = this->create_client<std_srvs::srv::Trigger>(
        "/dsp/planning_failure", rclcpp::ServicesQoS(), misc_group_);

    // The planner checks for new goals and periodic replans at 20 Hz.
    plan_timer_ = this->create_wall_timer(std::chrono::milliseconds(50),
        std::bind(&Dsp::planTick, this), nav_group_);

    const double goal_publish_rate = this->get_parameter("goal_publish_rate").as_double();
    if(goal_publish_rate > 0.0)
        goal_publish_timer = this->create_wall_timer(std::chrono::duration<double>(1.0 / goal_publish_rate),
            std::bind(&Dsp::goalPublishCallback, this), publish_group_);

    const double map_update_rate = this->get_parameter("map_update_rate").as_double();
    if(map_update_rate > 0.0)
        map_timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / map_update_rate),
            std::bind(&Dsp::applyPendingMap, this), map_group_);

    nlohmann::json node_info = {
        {"description", "D* Lite 3D path planner (lazy DSL graphs, multithreaded)"},
        {"version", "0.3.0"},
        {"node_type", "path_planner"},
        {"topics_published", "dsp/path, dsp/spline_path, dsp/occupancy_map, /move_base_simple/goal"},
        {"services", "/dsp/path_cost, /dsp/set_goal"},
        {"services_consumed", "/dsp/planning_failure"}
    };
    ros2top_registered_ = ros2top::register_node("/" + std::string(get_name()), node_info);
    if (!ros2top_registered_) {
        RCLCPP_WARN(get_logger(), "ros2top registration failed.");
    }
    ros2top_timer_ = create_wall_timer(std::chrono::seconds(3), [this]() {
        if (ros2top_registered_) {
            ros2top::heartbeat("/" + std::string(get_name()));
        }
    }, misc_group_);

    start_pos << 0,0,0;
    goal_pos << 0,0,0;
    pmin << 0,0,0;

    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    RCLCPP_INFO(get_logger(), "DSP started: map %s applied at %.1f Hz, voxel padding %s (%.2f m), "
        "set_goal answers immediately, path_cost uses its own persistent search",
        this->get_parameter("map_topic").as_string().c_str(), map_update_rate,
        this->get_parameter("obstacle_padding_enabled").as_bool() ? "on" : "off",
        this->get_parameter("obstacle_padding").as_double());
}

Dsp::~Dsp()
{
    if (ros2top_registered_) {
        ros2top::unregister_node("/" + std::string(get_name()));
    }
}

std::unique_lock<std::mutex> Dsp::lockDslWithPriority()
{
    priority_waiters_.fetch_add(1);
    std::unique_lock<std::mutex> lock(dsl_mutex_);
    priority_waiters_.fetch_sub(1);
    return lock;
}

// ---------------------------------------------------------------------------
// Occupancy state and hard obstacle padding
//
// occupancy_map holds the raw map state per voxel (DSP_FREE, dsp_unknown_ or DSP_OCCUPIED).
// padding_count_ counts, per voxel, how many occupied voxels have it inside their padding,
// so obstacles can be added and removed incrementally. The cost given to the search graph
// (effectiveCost) is DSP_OCCUPIED for occupied *and* padded voxels.
// All of it is guarded by dsl_mutex_.
// ---------------------------------------------------------------------------

int Dsp::voxelIndex(int x, int y, int z) const
{
    return x + y * length_voxel + z * length_voxel * width_voxel;
}

bool Dsp::inBounds(int x, int y, int z) const
{
    return x >= 0 && x < length_voxel && y >= 0 && y < width_voxel && z >= 0 && z < height_voxel;
}

VoxelLayout Dsp::layout() const
{
    return VoxelLayout{length_voxel, width_voxel, height_voxel};
}

void Dsp::resetOccupancy()
{
    const size_t size = static_cast<size_t>(length_voxel) * width_voxel * height_voxel;
    occupancy_map.assign(size, dsp_unknown_);
    padding_count_.assign(size, 0);
}

double Dsp::effectivePadding() const
{
    if (!this->get_parameter("obstacle_padding_enabled").as_bool()) {
        return 0.0;
    }
    return std::max(0.0, this->get_parameter("obstacle_padding").as_double());
}

void Dsp::buildPaddingOffsets()
{
    padding_offsets_.clear();
    if (padding_m_ <= 0.0 || res_octomap <= 0.0) {
        return;
    }
    padding_offsets_ = paddingOffsets(padding_m_ / res_octomap, height_voxel > 1);
}

void Dsp::addPadding(int x, int y, int z, int delta)
{
    for (const auto &o : padding_offsets_) {
        const int nx = x + o.x();
        const int ny = y + o.y();
        const int nz = z + o.z();
        if (inBounds(nx, ny, nz)) {
            padding_count_[voxelIndex(nx, ny, nz)] += delta;
        }
    }
}

void Dsp::setVoxelState(int x, int y, int z, double state)
{
    const int idx = voxelIndex(x, y, z);
    const double old_state = occupancy_map[idx];
    if (old_state == state) {
        return;
    }
    occupancy_map[idx] = state;

    const bool was_occupied = (old_state == DSP_OCCUPIED);
    const bool is_occupied = (state == DSP_OCCUPIED);
    if (was_occupied != is_occupied) {
        addPadding(x, y, z, is_occupied ? 1 : -1);
    }
}

double Dsp::effectiveCost(int idx) const
{
    const double raw = occupancy_map[idx];
    if (raw == DSP_OCCUPIED || padding_count_[idx] > 0) {
        return DSP_OCCUPIED;
    }
    if (raw >= dsp_unknown_) {
        return raw;
    }
    double cost = raw;
    if (height_cost_map.size() == occupancy_map.size()) {
        cost += height_cost_map[idx];
    }
    return std::min(cost, static_cast<double>(dsp_unknown_ - 1));
}

bool Dsp::blocked(int idx) const
{
    return graph_cost_[idx] >= DSP_OCCUPIED;
}

bool Dsp::wall(int idx) const
{
    return occupancy_map[idx] == DSP_OCCUPIED;
}

// Push every voxel whose effective cost changed into the grid and both search graphs.
void Dsp::syncGraphCosts()
{
    if (!grid_) {
        return;
    }
    const int size = length_voxel * width_voxel * height_voxel;
    for (int idx = 0; idx < size; idx++) {
        const double cost = effectiveCost(idx);
        if (cost == graph_cost_[idx]) {
            continue;
        }
        graph_cost_[idx] = cost;
        if (dsl::Cell<3> *cell = grid_->cells[idx]) {
            cell->cost = cost;
        }
        nav_search_->SyncCellCost(idx, cost);
        query_search_->SyncCellCost(idx, cost);
    }
}

// ---------------------------------------------------------------------------
// Map input. The subscription callbacks only keep the newest message; applyPendingMap()
// (map_update_rate timer, or directly when the rate is <= 0) decodes it and applies it.
// ---------------------------------------------------------------------------

void Dsp::occupancy_grid_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
    {
        std::lock_guard<std::mutex> lock(map_msg_mutex_);
        pending_grid_ = msg;
    }
    if (!map_timer_) {
        applyPendingMap();
    }
}

void Dsp::octomap_data_callback(const octomap_msgs::msg::Octomap::SharedPtr msg) {
    {
        std::lock_guard<std::mutex> lock(map_msg_mutex_);
        pending_octomap_ = msg;
    }
    if (!map_timer_) {
        applyPendingMap();
    }
}

void Dsp::applyPendingMap()
{
    octomap_msgs::msg::Octomap::SharedPtr octomap_msg;
    nav_msgs::msg::OccupancyGrid::SharedPtr grid_msg;
    {
        std::lock_guard<std::mutex> lock(map_msg_mutex_);
        octomap_msg.swap(pending_octomap_);
        grid_msg.swap(pending_grid_);
    }

    if (octomap_msg) {
        // Decoding is the expensive part and does not touch the planner state, so it runs
        // without dsl_mutex_.
        std::unique_ptr<octomap::AbstractOcTree> abstract_tree(octomap_msgs::msgToMap(*octomap_msg));
        const octomap::OcTree *tree = dynamic_cast<octomap::OcTree*>(abstract_tree.get());
        if (!tree) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Received octomap that is not an OcTree, ignoring");
        } else if (tree->size() > 0) {
            auto lock = lockDslWithPriority();
            applyOctomap(*tree);
        }
    }
    if (grid_msg) {
        auto lock = lockDslWithPriority();
        applyOccupancyGrid(*grid_msg);
    }
}

// 2D occupancyGrid (caller holds dsl_mutex_)
void Dsp::applyOccupancyGrid(const nav_msgs::msg::OccupancyGrid &msg){
    dsp_unknown_ =this->get_parameter("DSP_UNKNOWN").as_int();
    const int upper_thresh_ = this->get_parameter("upper_thresh").as_int();
    const int lower_thresh_ = this->get_parameter("lower_thresh").as_int();
    const double padding = effectivePadding();

    const int nx = msg.info.width;
    const int ny = msg.info.height;
    const double res = msg.info.resolution;
    const Eigen::Vector3d new_min(msg.info.origin.position.x, msg.info.origin.position.y, 0.0);
    if (nx <= 0 || ny <= 0 || res <= 0.0 || msg.data.size() != static_cast<size_t>(nx) * ny) {
        RCLCPP_WARN(this->get_logger(), "Ignoring malformed occupancy grid");
        return;
    }

    const bool same_layout = grid_built
        && nx == length_voxel && ny == width_voxel
        && std::abs(res - res_octomap) < 1e-9
        && (new_min - pmin).cwiseAbs().maxCoeff() < 0.5 * res
        && std::abs(padding - padding_m_) < 1e-9;

    if (!same_layout) {
        res_octomap = res;
        pmin = new_min;
        length_voxel = nx;
        width_voxel = ny;
        height_voxel = 1;
        padding_m_ = padding;
        floor_height_map.clear();
        height_cost_map.clear();
        buildPaddingOffsets();
        resetOccupancy();
    }

    for (int i = 0; i < nx * ny; i++) {
        const int x = i % nx;
        const int y = i / nx;
        const int value = msg.data[i];
        if (value > upper_thresh_) {
            setVoxelState(x, y, 0, DSP_OCCUPIED);
        }
        else if (value == -1) {
            setVoxelState(x, y, 0, dsp_unknown_);
        }
        else if (value < lower_thresh_) {
            setVoxelState(x, y, 0, DSP_FREE);
        }
        // values between lower and upper threshold keep their previous state
    }

    if (same_layout) {
        syncGraphCosts();
    } else {
        graph_cost_.resize(occupancy_map.size());
        for (size_t i = 0; i < occupancy_map.size(); i++) {
            graph_cost_[i] = effectiveCost(i);
        }
        buildGraph();
        grid_built = true;
        plan_requested_ = true;
    }
}

// octomap 3D (caller holds dsl_mutex_)
void Dsp::applyOctomap(const octomap::OcTree &tree) {
    dsp_unknown_ = this->get_parameter("DSP_UNKNOWN").as_int();
    const double padding = effectivePadding();
    const double res = tree.getResolution();
    Eigen::Vector3d tree_min, tree_max;
    tree.getMetricMin(tree_min(0), tree_min(1), tree_min(2));
    tree.getMetricMax(tree_max(0), tree_max(1), tree_max(2));
    if (!(res > 0.0) || !tree_min.allFinite() || !tree_max.allFinite() ||
        (tree_max - tree_min).minCoeff() <= 0.0) {
        return;
    }

    // The octree only grows: keep the current grid (and the search graphs on it) while the
    // map still fits inside it.
    const double eps = 0.25 * res;
    const Eigen::Vector3d grid_max =
        pmin + res_octomap * Eigen::Vector3d(length_voxel, width_voxel, height_voxel);
    const bool fits = grid_built && height_voxel > 1
        && std::abs(res - res_octomap) < 1e-9
        && std::abs(padding - padding_m_) < 1e-9
        && (tree_min - pmin).minCoeff() > -eps
        && (grid_max - tree_max).minCoeff() > -eps;
    if (fits) {
        updateGDSP(tree);
        return;
    }

    // (Re)allocate the grid around the octree with some slack. Octree leaves start at integer
    // multiples of the resolution, so whole-voxel margins keep the grid aligned with them.
    const int margin_xy = std::max(0, static_cast<int>(std::ceil(
        this->get_parameter("map_bounds_margin_xy").as_double() / res)));
    const int margin_z = std::max(0, static_cast<int>(std::ceil(
        this->get_parameter("map_bounds_margin_z").as_double() / res)));
    res_octomap = res;
    pmin = tree_min - res * Eigen::Vector3d(margin_xy, margin_xy, margin_z);
    length_voxel = static_cast<int>(std::lround((tree_max(0) - tree_min(0)) / res)) + 2 * margin_xy;
    width_voxel = static_cast<int>(std::lround((tree_max(1) - tree_min(1)) / res)) + 2 * margin_xy;
    height_voxel = static_cast<int>(std::lround((tree_max(2) - tree_min(2)) / res)) + 2 * margin_z;
    padding_m_ = padding;
    buildGDSP(tree);
    grid_built = true;
    // Replan the current goal on the new grid.
    plan_requested_ = true;
}

// Calls fn(x, y, z, occupied) for every grid voxel covered by an octomap leaf. Leaves above
// the finest resolution cover n^3 voxels.
void Dsp::forEachLeafVoxel(const octomap::OcTree &tree,
                           const std::function<void(int, int, int, bool)> &fn) const
{
    for (auto it = tree.begin_leafs(), end = tree.end_leafs(); it != end; ++it)
    {
        const bool occupied = tree.isNodeOccupied(*it);
        const double size = it.getSize();
        const int n = std::max(1, static_cast<int>(std::lround(size / res_octomap)));
        const int x0 = static_cast<int>(std::lround((it.getX() - 0.5 * size - pmin(0)) / res_octomap));
        const int y0 = static_cast<int>(std::lround((it.getY() - 0.5 * size - pmin(1)) / res_octomap));
        const int z0 = static_cast<int>(std::lround((it.getZ() - 0.5 * size - pmin(2)) / res_octomap));

        for (int k = 0; k < n; k++) {
            for (int j = 0; j < n; j++) {
                for (int i = 0; i < n; i++) {
                    if (inBounds(x0 + i, y0 + j, z0 + k)) {
                        fn(x0 + i, y0 + j, z0 + k, occupied);
                    }
                }
            }
        }
    }
}

void Dsp::buildGDSP(const octomap::OcTree &tree)
{
    buildPaddingOffsets();
    resetOccupancy();

    forEachLeafVoxel(tree, [this](int x, int y, int z, bool occupied) {
        setVoxelState(x, y, z, occupied ? DSP_OCCUPIED : DSP_FREE);
    });

    // Height penalties are computed from the raw map (padding does not move the floor).
    buildHeightMap();
    buildHeightCostMap();

    graph_cost_.resize(occupancy_map.size());
    for (size_t i = 0; i < occupancy_map.size(); i++) {
        graph_cost_[i] = effectiveCost(i);
    }
    buildGraph();
}


void Dsp::buildHeightMap() {
    int HEIGHT_KERNEL_SIZE = this->get_parameter("height_kernel_size").as_int();

    if (HEIGHT_KERNEL_SIZE % 2 == 0) {
        RCLCPP_WARN(this->get_logger(), "Height kernel size should be odd. Incrementing by 1.");
        HEIGHT_KERNEL_SIZE += 1;
    }

    floor_height_map.assign(length_voxel * width_voxel, -1); // -1 marks general value.
    for (int y = 0; y < width_voxel; y++) {
        for (int x = 0; x < length_voxel; x++) {
            for (int z = 0; z < height_voxel-1; z++) {
                int idx = voxelIndex(x, y, z);
                int idx_above = voxelIndex(x, y, z + 1);

                bool occupied = (occupancy_map[idx] == DSP_OCCUPIED);
                bool free_above = (occupancy_map[idx_above] < dsp_unknown_);

                if (occupied && free_above) {
                    floor_height_map[x + y * length_voxel] = z;
                    break;
                }
            }
        }
    }

    int padding = (HEIGHT_KERNEL_SIZE-1) / 2;

    std::vector<int> smoothed_floor_height_map = floor_height_map;

    for (int y = padding; y < width_voxel - padding; y++) {
        for (int x = padding; x < length_voxel - padding; x++) {
            std::vector<int> heights;

            for (int dy = -padding; dy <= padding; dy++) {
                for (int dx = -padding; dx <= padding; dx++) {
                    int neighbor_idx = (x + dx) + (y + dy) * length_voxel;
                    if (floor_height_map[neighbor_idx] >= 0) {
                        heights.push_back(floor_height_map[neighbor_idx]);
                    }
                }
            }

            if (!heights.empty()) {
                size_t mid = heights.size() / 2;

                std::nth_element(
                    heights.begin(),
                    heights.begin() + mid,
                    heights.end());

                smoothed_floor_height_map[x + y * length_voxel] = heights[mid];
            }
        }
    }
    floor_height_map.swap(smoothed_floor_height_map);
}

void Dsp::buildHeightCostMap() {
    int OPTIMAL_VOXEL_HEIGHT = this->get_parameter("optimal_voxel_height").as_int();
    int OPTIMAL_VOXEL_HEIGHT_COST = this->get_parameter("optimal_voxel_height_cost").as_int();

    height_cost_map.assign(length_voxel * width_voxel * height_voxel, 0.0);

    for (int x = 0; x < length_voxel; x++) {
        for (int y = 0; y < width_voxel; y++) {
            for (int z = 0; z < height_voxel; z++) {
                int idx = voxelIndex(x, y, z);
                int floor_height = floor_height_map[x + y * length_voxel];
                if (floor_height >= 0) {
                    int cost = std::abs(z - (floor_height+OPTIMAL_VOXEL_HEIGHT)) * OPTIMAL_VOXEL_HEIGHT_COST;
                    height_cost_map[idx] = cost;
                } else {
                    height_cost_map[idx] = dsp_unknown_;
                }
            }
        }
    }
}


// (Re)creates the DSL grid and both searches from graph_cost_. The searches are lazy: no
// vertex or edge exists until a search reaches it, so this is cheap even for large grids.
// Every cell is kept (maxCost above DSP_OCCUPIED) so the graph stays connected; DSL's Plan()
// has no "no path" result. Paths through blocked voxels are rejected in searchPath().
void Dsp::buildGraph(){
    const auto start = std::chrono::steady_clock::now();
    nav_search_.reset();
    query_search_.reset();
    connectivity_.reset();
    grid_.reset(new dsl::Grid3d(length_voxel, width_voxel, height_voxel,
        graph_cost_.data(),
        1, 1, 1, 1, DSP_OCCUPIED + 1));
    connectivity_.reset(new dsl::Grid3dConnectivity(*grid_));
    nav_search_ = std::make_unique<DspGridSearch>(*grid_, *connectivity_, grid_cost_);
    query_search_ = std::make_unique<DspGridSearch>(*grid_, *connectivity_, grid_cost_);
    RCLCPP_INFO(this->get_logger(),
        "Voxel grid allocated: %d x %d x %d voxels of %.2f m, padding %.2f m (%zu offsets), in %.1f ms; "
        "search graphs are expanded lazily",
        length_voxel, width_voxel, height_voxel, res_octomap, padding_m_, padding_offsets_.size(),
        millisecondsSince(start));
}

// A search keeps every vertex it has expanded; release it once it gets too large.
void Dsp::releaseSearchIfLarge(std::unique_ptr<DspGridSearch> &search, const char *name)
{
    const int64_t limit = this->get_parameter("max_search_vertices").as_int();
    if (!search || limit <= 0 || search->Vertices() <= limit) {
        return;
    }
    RCLCPP_INFO(this->get_logger(), "%s search graph reached %d vertices (limit %ld); releasing it",
        name, search->Vertices(), static_cast<long>(limit));
    search = std::make_unique<DspGridSearch>(*grid_, *connectivity_, grid_cost_);
}

void Dsp::updateGDSP(const octomap::OcTree &tree)
{
    forEachLeafVoxel(tree, [this](int x, int y, int z, bool occupied) {
        setVoxelState(x, y, z, occupied ? DSP_OCCUPIED : DSP_FREE);
    });

    // Recompute height penalties and push all changed costs to the grid and both searches.
    buildHeightMap();
    buildHeightCostMap();
    syncGraphCosts();
}

void Dsp::handleSetStartOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    Eigen::Vector3d wpos(msg->pose.pose.position.x , msg->pose.pose.position.y, msg->pose.pose.position.z);
    //setStart(wpos);
}

void Dsp::handleSetStart(const geometry_msgs::msg::Point::SharedPtr msg)
{
    Eigen::Vector3d wpos(msg->x, msg->y, msg->z);
    if(!this->get_parameter("use_3d").as_bool()){
        wpos[2] = 0;
    }
    std::lock_guard<std::mutex> lock(goal_mutex_);
    start_pos = wpos;
}

// Current drone position in odom_frame_id from TF.
bool Dsp::lookupRobotPosition(Eigen::Vector3d &wpos)
{
    const std::string odf = this->get_parameter("odom_frame_id").as_string();
    const std::string blf = this->get_parameter("base_link_frame_id").as_string();
    geometry_msgs::msg::TransformStamped t;
    try{
        t = tf_buffer->lookupTransform(odf, blf, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "Could not transform %s to %s: %s",
            odf.c_str(), blf.c_str(), ex.what());
        return false;
    }

    wpos << t.transform.translation.x,
            t.transform.translation.y,
            t.transform.translation.z;
    return true;
}

// ---------------------------------------------------------------------------
// Searching (caller holds dsl_mutex_)
// ---------------------------------------------------------------------------

bool Dsp::worldToIndex(const Eigen::Vector3d &wpos, int &idx) const
{
    if (!grid_built || res_octomap <= 0.0) {
        return false;
    }
    const Eigen::Vector3d g = posRes(wpos);
    if (!g.allFinite()) {
        return false;
    }
    if (g.x() < 0.0 || g.x() >= length_voxel || g.y() < 0.0 || g.y() >= width_voxel) {
        return false;
    }
    int z = 0;
    if (height_voxel > 1) {
        if (g.z() < 0.0 || g.z() >= height_voxel) {
            return false;
        }
        z = static_cast<int>(std::floor(g.z()));
    }
    idx = voxelIndex(static_cast<int>(std::floor(g.x())), static_cast<int>(std::floor(g.y())), z);
    return true;
}

// Centre of a voxel in DSL grid units (Grid3d is built with unit cell size).
Eigen::Vector3d Dsp::gridCenter(int idx) const
{
    return layout().coords(idx).cast<double>() + Eigen::Vector3d::Constant(0.5);
}

// Centre of a voxel in odom_frame_id.
Eigen::Vector3d Dsp::voxelCenter(int idx) const
{
    return pmin + res_octomap * gridCenter(idx);
}

Dsp::SearchResult Dsp::searchPath(DspGridSearch &search, const Eigen::Vector3d &start,
                                  const Eigen::Vector3d &goal)
{
    SearchResult result;
    int start_idx = -1;
    int goal_idx = -1;
    if (!worldToIndex(start, start_idx)) {
        result.reason = "The planning start is outside the DSP map.";
        return result;
    }
    if (!worldToIndex(goal, goal_idx)) {
        result.reason = "The goal is outside the DSP map.";
        return result;
    }

    const VoxelLayout grid_layout = layout();
    const auto is_blocked = [this](int idx) { return blocked(idx); };
    const auto is_wall = [this](int idx) { return wall(idx); };
    const double snap_radius =
        std::max(0.0, this->get_parameter("endpoint_snap_radius").as_double()) / res_octomap;
    const int start_cell = nearestFree(grid_layout, start_idx, snap_radius, is_blocked, is_wall);
    if (start_cell < 0) {
        result.reason = "The planning start is inside an obstacle or its padding, with no free voxel "
                        "within endpoint_snap_radius.";
        return result;
    }
    const int goal_cell = nearestFree(grid_layout, goal_idx, snap_radius, is_blocked, is_wall);
    if (goal_cell < 0) {
        result.reason = "The goal is inside an obstacle or its padding, with no free voxel within "
                        "endpoint_snap_radius.";
        return result;
    }

    if (start_cell == goal_cell) {
        result.cells.push_back(start_cell);
    } else {
        if (!search.SetStart(gridCenter(start_cell)) || !search.SetGoal(gridCenter(goal_cell))) {
            result.reason = "DSP failed to set the planning start or goal.";
            return result;
        }
        search.Plan(result.grid_path);
        for (const auto &cell : result.grid_path.cells) {
            result.cells.push_back(grid_->Id(cell.c));
        }
        if (result.cells.size() < 2) {
            result.reason = "DSP was unable to find a path to the requested goal.";
            return result;
        }
    }

    // Blocked voxels stay in the graph at DSP_OCCUPIED cost, so DSL always returns some path;
    // a path that needs one is a path that does not exist.
    if (crossesBlocked(result.cells, is_blocked)) {
        result.reason = "Every route to the goal passes through an obstacle or its padding.";
        return result;
    }

    result.cost = pathCost(grid_layout, result.cells, graph_cost_);
    result.points.push_back(start);
    for (size_t i = 0; i < result.cells.size(); i++) {
        // The exact start and goal replace the centre of their own voxel.
        const bool own_start_voxel = (i == 0 && start_cell == start_idx);
        const bool own_goal_voxel = (i + 1 == result.cells.size() && goal_cell == goal_idx);
        if (!own_start_voxel && !own_goal_voxel) {
            result.points.push_back(voxelCenter(result.cells[i]));
        }
    }
    result.points.push_back(goal);
    result.ok = true;
    return result;
}

// ---------------------------------------------------------------------------
// path_cost service (query group): its own persistent search, so a query never rebuilds a
// graph and never disturbs the path being flown.
// ---------------------------------------------------------------------------

void Dsp::request_cost(const std::shared_ptr<dsp_interfaces::srv::PathCost::Request> req, std::shared_ptr<dsp_interfaces::srv::PathCost::Response> res){
    // An empty path with negative cost/distance means "no path".
    res->cost = -1.0;
    res->distance = -1.0;
    if (!grid_built) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "path_cost requested before a map was received");
        return;
    }

    Eigen::Vector3d start(req->start.x, req->start.y, req->start.z);
    Eigen::Vector3d stop(req->stop.x, req->stop.y, req->stop.z);
    if (!this->get_parameter("use_3d").as_bool()) {
        start.z() = 0.0;
        stop.z() = 0.0;
    }

    const auto started = std::chrono::steady_clock::now();
    // The planner and map updates go first: a query waits for them, never the reverse.
    while (priority_waiters_.load() > 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    SearchResult result;
    int vertices = 0;
    {
        std::lock_guard<std::mutex> lock(dsl_mutex_);
        if (!query_search_) {
            return;
        }
        result = searchPath(*query_search_, start, stop);
        releaseSearchIfLarge(query_search_, "path_cost");
        vertices = query_search_->Vertices();
    }
    if (!result.ok) {
        RCLCPP_DEBUG(this->get_logger(), "path_cost (%.2f, %.2f, %.2f) -> (%.2f, %.2f, %.2f): %s",
            start.x(), start.y(), start.z(), stop.x(), stop.y(), stop.z(), result.reason.c_str());
        return;
    }

    res->path = pointsToPath(result.points);
    res->cost = result.cost;
    res->distance = path_distance(res->path);
    RCLCPP_DEBUG(this->get_logger(), "path_cost answered in %.1f ms (%zu voxels, %d graph vertices)",
        millisecondsSince(started), result.cells.size(), vertices);
}

float Dsp::path_distance(const nav_msgs::msg::Path &path)
{
    float distance = 0.0;
    for(unsigned long int i = 0; i + 1 < path.poses.size(); i++)
    {
        distance += point_distance(path.poses[i].pose.position, path.poses[i+1].pose.position);
    }
    return distance;
}

float Dsp::point_distance(geometry_msgs::msg::Point p0,  geometry_msgs::msg::Point p1)
{
    return sqrt(pow(p0.x - p1.x, 2) + pow(p0.y - p1.y, 2) + pow(p0.z - p1.z, 2));
}

// ---------------------------------------------------------------------------
// set_goal service (goal group): answered as soon as the goal is stored. Planning happens in
// planTick(); failures are reported on /dsp/planning_failure.
// ---------------------------------------------------------------------------

void Dsp::handleSetGoal(const std::shared_ptr<dsp_interfaces::srv::SetGoal::Request> req,
                        std::shared_ptr<dsp_interfaces::srv::SetGoal::Response> res)
{
    const std::string odom_frame = this->get_parameter("odom_frame_id").as_string();
    geometry_msgs::msg::PoseStamped goal = req->goal;

    if (goal.header.frame_id.empty()) {
        goal.header.frame_id = odom_frame;
    }
    else if (goal.header.frame_id != odom_frame) {
        try {
            const geometry_msgs::msg::TransformStamped t =
                tf_buffer->lookupTransform(odom_frame, goal.header.frame_id, tf2::TimePointZero);
            geometry_msgs::msg::PoseStamped transformed;
            tf2::doTransform(goal, transformed, t);
            goal = transformed;
        } catch (const tf2::TransformException &ex) {
            res->success = false;
            res->message = "Could not transform goal from '" + goal.header.frame_id +
                           "' to '" + odom_frame + "': " + ex.what();
            RCLCPP_WARN(this->get_logger(), "%s", res->message.c_str());
            return;
        }
    }

    Eigen::Vector3d position(goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
    if(!this->get_parameter("use_3d").as_bool()){
        position[2] = 0;
    }
    if (!position.allFinite()) {
        res->success = false;
        res->message = "Goal position is not finite";
        return;
    }

    bool unchanged = false;
    {
        std::lock_guard<std::mutex> lock(goal_mutex_);
        unchanged = has_goal_ && samePose(goal.pose, goal_pose_.pose);
        if (!unchanged) {
            goal_pose_ = goal;
            goal_pos = position;
            has_goal_ = true;
            goal_generation_.fetch_add(1);
        }
    }

    if (unchanged) {
        // A re-sent goal keeps the current path (and the drone's progress along it).
        bool have_path = false;
        {
            std::lock_guard<std::mutex> lock(path_mutex_);
            have_path = has_path_;
        }
        if (!have_path) {
            plan_requested_ = true;
        }
        res->success = true;
        res->message = have_path ? "Goal unchanged; keeping the current path"
                                 : "Goal unchanged; planning started";
        return;
    }

    {
        // Drop the path to the previous goal so it is never followed towards the new one.
        std::lock_guard<std::mutex> lock(path_mutex_);
        has_path_ = false;
        path_msg_.poses.clear();
        path_progress_idx_ = 0;
        final_goal_sent_ = false;
    }
    plan_requested_ = true;

    RCLCPP_INFO(this->get_logger(), "New goal (%.2f, %.2f, %.2f) in %s",
        position.x(), position.y(), position.z(), odom_frame.c_str());

    res->success = true;
    res->message = grid_built ? "Goal accepted; planning started"
                              : "Goal accepted; waiting for the first map before planning";
}

// ---------------------------------------------------------------------------
// Planner (nav group, 20 Hz): plans when a new goal arrived or the grid was rebuilt, and
// otherwise every 1 / plan_rate seconds towards the current goal.
// ---------------------------------------------------------------------------

void Dsp::planTick()
{
    const auto now_steady = std::chrono::steady_clock::now();
    if (!plan_requested_.exchange(false)) {
        const double plan_rate = this->get_parameter("plan_rate").as_double();
        if (plan_rate <= 0.0 ||
            now_steady - last_plan_time_ < std::chrono::duration<double>(1.0 / plan_rate)) {
            return;
        }
    }

    geometry_msgs::msg::PoseStamped goal;
    Eigen::Vector3d goal_position;
    Eigen::Vector3d fallback_start;
    uint64_t generation = 0;
    {
        std::lock_guard<std::mutex> lock(goal_mutex_);
        if (!has_goal_) {
            return;
        }
        goal = goal_pose_;
        goal_position = goal_pos;
        fallback_start = start_pos;
        generation = goal_generation_.load();
    }
    if (!grid_built) {
        plan_requested_ = true;  // plan as soon as the first map has been applied
        return;
    }
    last_plan_time_ = now_steady;

    // Falls back to the last start from dsp/set_start when TF is unavailable.
    Eigen::Vector3d robot;
    if (!lookupRobotPosition(robot)) {
        robot = fallback_start;
    }
    if (!this->get_parameter("use_3d").as_bool()) {
        robot[2] = 0.0;
    }

    const auto started = std::chrono::steady_clock::now();
    SearchResult result;
    std::vector<Eigen::Vector3d> spline;
    {
        auto lock = lockDslWithPriority();
        if (!nav_search_) {
            plan_requested_ = true;
            return;
        }
        result = searchPath(*nav_search_, robot, goal_position);
        // Short paths are published unsplined; DSL's spline needs more than 3 cells.
        if (result.ok && result.grid_path.cells.size() > 3) {
            std::vector<Eigen::Vector3d> grid_points;
            nav_search_->SplinePath(result.grid_path, grid_points, spline_step_);
            spline.reserve(grid_points.size());
            for (const auto &p : grid_points) {
                spline.push_back(pmin + res_octomap * p);
            }
        }
        releaseSearchIfLarge(nav_search_, "Navigation");
    }

    if (goal_generation_.load() != generation) {
        return;  // a newer goal arrived while planning; its own plan is already requested
    }
    if (!result.ok) {
        reportPlanningFailure(result.reason);
        return;
    }

    const nav_msgs::msg::Path path = pointsToPath(result.points);
    bool first_path_for_goal = false;
    {
        std::lock_guard<std::mutex> lock(path_mutex_);
        if (goal_generation_.load() != generation) {
            return;
        }
        first_path_for_goal = !has_path_ || path_generation_ != generation;
        path_msg_ = path;
        path_goal_pose_ = goal;
        path_generation_ = generation;
        path_progress_idx_ = 0;
        has_path_ = true;
        if (first_path_for_goal) {
            final_goal_sent_ = false;
        }
    }
    path_pub_->publish(path);
    splinepath_pub_->publish(spline.empty() ? path : pointsToPath(spline));
    RCLCPP_DEBUG(this->get_logger(), "Planned %zu poses (cost %.1f) in %.1f ms",
        path.poses.size(), result.cost, millisecondsSince(started));

    // Send the first look-ahead goal of a new goal right away instead of waiting for the timer.
    if (first_path_for_goal) {
        publishGoalAhead();
    }
}

void Dsp::reportPlanningFailure(const std::string &message)
{
    RCLCPP_WARN(this->get_logger(), "%s", message.c_str());
    if (!planning_failure_client_->service_is_ready()) {
        RCLCPP_WARN(
            this->get_logger(),
            "Planning failure receiver is unavailable; failure was not acknowledged.");
        return;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    planning_failure_client_->async_send_request(
        request,
        [this, message](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
            const auto response = future.get();
            if (!response->success) {
                RCLCPP_WARN(
                    this->get_logger(),
                    "Planning failure receiver rejected '%s': %s",
                    message.c_str(), response->message.c_str());
                return;
            }
            RCLCPP_INFO(
                this->get_logger(),
                "Planning failure receiver acknowledged '%s': %s",
                message.c_str(), response->message.c_str());
        });
}

// transfomre a pose betewn IRL cordinates and map cordinate
Eigen::Vector3d Dsp::posRes(Eigen::Vector3d wpos) const
{
    for (int i = 0; i < 3; i++)
    {
        wpos(i) = (wpos(i) - pmin(i)) / res_octomap;
    }
    return wpos;
}

void Dsp::goalPublishCallback(){
    publishGoalAhead();
}

// Publishes the path pose goal_voxels_ahead voxels ahead of the drone's closest point on the
// latest path. Once that runs past the end of the path, the requested goal itself (exact
// position + orientation) is published, once, instead of re-sending it every tick.
void Dsp::publishGoalAhead()
{
    Eigen::Vector3d robot_pos;
    if (!lookupRobotPosition(robot_pos)) {
        std::lock_guard<std::mutex> lock(goal_mutex_);
        robot_pos = start_pos;
    }
    if (!this->get_parameter("use_3d").as_bool()) {
        robot_pos[2] = 0.0;
    }
    const int64_t voxels_ahead = std::max<int64_t>(0, this->get_parameter("goal_voxels_ahead").as_int());

    geometry_msgs::msg::PoseStamped goal_msg;
    goal_msg.header.frame_id = this->get_parameter("odom_frame_id").as_string();
    goal_msg.header.stamp = this->now();
    {
        std::lock_guard<std::mutex> lock(path_mutex_);
        if (!has_path_ || path_msg_.poses.empty()) {
            return;
        }
        const auto &poses = path_msg_.poses;

        // Closest path pose, never moving backwards along the current path.
        size_t nearest = std::min(path_progress_idx_, poses.size() - 1);
        double best = std::numeric_limits<double>::infinity();
        for (size_t i = nearest; i < poses.size(); i++) {
            const auto &p = poses[i].pose.position;
            const double d = (Eigen::Vector3d(p.x, p.y, p.z) - robot_pos).squaredNorm();
            if (d < best) {
                best = d;
                nearest = i;
            }
        }
        path_progress_idx_ = nearest;

        const size_t target = nearest + static_cast<size_t>(voxels_ahead);
        if (target >= poses.size() - 1) {
            if (final_goal_sent_) {
                return;
            }
            goal_msg.pose = path_goal_pose_.pose;
            final_goal_sent_ = true;
        } else {
            // Intermediate goals face along the path; only the final goal carries the requested
            // orientation (ego-planner turns to a goal's yaw once it arrives there).
            goal_msg.pose.position = poses[target].pose.position;
            goal_msg.pose.orientation = pathHeading(target);
            final_goal_sent_ = false;
        }
    }
    goal_pub_->publish(goal_msg);
}

// Horizontal heading of the latest path at pose idx (direction to the next pose that is not
// straight above/below it). Falls back to the goal orientation if the rest of the path is
// vertical. Caller holds path_mutex_.
geometry_msgs::msg::Quaternion Dsp::pathHeading(size_t idx) const
{
    const auto &poses = path_msg_.poses;
    const auto &p0 = poses[idx].pose.position;
    for (size_t j = idx + 1; j < poses.size(); j++) {
        const double dx = poses[j].pose.position.x - p0.x;
        const double dy = poses[j].pose.position.y - p0.y;
        if (std::hypot(dx, dy) > 1e-6) {
            const double yaw = std::atan2(dy, dx);
            geometry_msgs::msg::Quaternion q;
            q.z = std::sin(0.5 * yaw);
            q.w = std::cos(0.5 * yaw);
            return q;
        }
    }
    return path_goal_pose_.pose.orientation;
}

nav_msgs::msg::Path Dsp::pointsToPath(const std::vector<Eigen::Vector3d> &points) const
{
    nav_msgs::msg::Path msg;
    msg.header.frame_id = this->get_parameter("odom_frame_id").as_string();
    msg.header.stamp = this->now();
    msg.poses.resize(points.size());
    for (size_t i = 0; i < points.size(); i++)
    {
        msg.poses[i].header = msg.header;
        msg.poses[i].pose.position.x = points[i].x();
        msg.poses[i].pose.position.y = points[i].y();
        msg.poses[i].pose.position.z = points[i].z();
        msg.poses[i].pose.orientation.w = 1.0;
    }
    return msg;
}


// publishing map usedfule when debuging (shows occupied + padded voxels)
void Dsp::publishOccupancyGrid()
{
    std::lock_guard<std::mutex> lock(dsl_mutex_);
    if (!grid_built) {
        return;
    }

    visualization_msgs::msg::Marker occmap_viz;

    std::vector<geometry_msgs::msg::Point> marker_pos;

    for(int z = 0; z < height_voxel; z++)
    {
        for(int y = 0; y < width_voxel; y++)
        {
            for(int x = 0; x < length_voxel; x++)
            {
                if(graph_cost_[voxelIndex(x, y, z)] == DSP_OCCUPIED)
                {
                    geometry_msgs::msg::Point pt;
                    pt.x = (x * res_octomap) + pmin(0);
                    pt.y = (y * res_octomap) + pmin(1);
                    pt.z = (z * res_octomap) + pmin(2);
                    marker_pos.push_back(pt);
                }
            }
        }
    }

    occmap_viz.header.frame_id = this->get_parameter("odom_frame_id").as_string();
    occmap_viz.header.stamp = rclcpp::Time();
    occmap_viz.ns = "dsp";
    occmap_viz.id = 1;
    occmap_viz.type = visualization_msgs::msg::Marker::CUBE_LIST;
    occmap_viz.action = visualization_msgs::msg::Marker::ADD;
    occmap_viz.pose.position.x = 0.5 * res_octomap;
    occmap_viz.pose.position.y = 0.5 * res_octomap;
    occmap_viz.pose.position.z = 0.5 * res_octomap;
    occmap_viz.pose.orientation.x = 0.0;
    occmap_viz.pose.orientation.y = 0.0;
    occmap_viz.pose.orientation.z = 0.0;
    occmap_viz.pose.orientation.w = 1.0;
    occmap_viz.scale.x = 1.0 * res_octomap;
    occmap_viz.scale.y = 1.0 * res_octomap;
    occmap_viz.scale.z = 1.0 * res_octomap;
    occmap_viz.color.a = 0.5;
    occmap_viz.color.r = 1.0;
    occmap_viz.color.g = 0.0;
    occmap_viz.color.b = 0.0;
    occmap_viz.points = marker_pos;
    occ_map_viz_pub_->publish(occmap_viz);

}

} // namespace

int main (int argc, char * argv[])
{
  rclcpp::init (argc, argv);
  auto node = std::make_shared<dsp::Dsp>();
  // One thread per callback group: set_goal and the look-ahead goals never wait for a
  // path_cost query or a map update.
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 6);
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  node.reset();
  rclcpp::shutdown();


  return 0;
}
