#include "dsp.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <nlohmann/json.hpp>
#include <ros2top/ros2top.hpp>

#include <limits>


namespace dsp
{

Dsp::Dsp() : Node("dsp")
{
    this->declare_parameter("map_topic", "octomap_full");
    this->declare_parameter("spline_step", 0.01);
    this->declare_parameter("lower_thresh", 59);
    this->declare_parameter("upper_thresh", 60);
    // Hard clearance [m]: every voxel whose centre is closer than this to the surface of an
    // occupied voxel is treated as occupied. Changing it at runtime rebuilds the graph on the
    // next map message.
    this->declare_parameter("obstacle_padding", 0.5);
    // Rate [Hz] of the path (re)planning timer. <= 0 disables it (plans only on new goals).
    this->declare_parameter("plan_rate", 0.2);
    // Rate [Hz] of publishing the look-ahead goal on /move_base_simple/goal. <= 0 disables it.
    this->declare_parameter("goal_publish_rate", 1.0);
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


    occ_map_viz_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("dsp/occupancy_map",1);

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("dsp/path", 1);
    splinepath_pub_ =  this->create_publisher<nav_msgs::msg::Path>("dsp/spline_path", 1);
    goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/move_base_simple/goal", 1);

    if(this->get_parameter("use_odometry").as_bool())
    {
        set_start_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            this->get_parameter("odom_topic").as_string(), 1,
            std::bind(&Dsp::handleSetStartOdom, this, std::placeholders::_1));
    }
    else
    {
        set_start_sub_ = this->create_subscription<geometry_msgs::msg::Point>("dsp/set_start", 1,
            std::bind(&Dsp::handleSetStart, this, std::placeholders::_1));
    }

    if(this->get_parameter("use_3d").as_bool()){
        get_octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
            this->get_parameter("map_topic").as_string(), 1,
            std::bind(&Dsp::octomap_data_callback, this, std::placeholders::_1));
    }
    else {
        get_ocupancy_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(this->get_parameter("map_topic").as_string(), 1,
            std::bind(&Dsp::occupancy_grid_callback, this, std::placeholders::_1));
        height_voxel = 1;
    }

    set_goal_srv_ = this->create_service<dsp_interfaces::srv::SetGoal>("dsp/set_goal",
        std::bind(&Dsp::handleSetGoal, this, std::placeholders::_1, std::placeholders::_2));

    const double plan_rate = this->get_parameter("plan_rate").as_double();
    if(plan_rate > 0.0)
        path_update_timer = rclcpp::create_timer(this, this->get_clock(), rclcpp::Duration::from_seconds(1.0 / plan_rate),
            std::bind(&Dsp::pathUpdateCallback, this));

    const double goal_publish_rate = this->get_parameter("goal_publish_rate").as_double();
    if(goal_publish_rate > 0.0)
        goal_publish_timer = rclcpp::create_timer(this, this->get_clock(), rclcpp::Duration::from_seconds(1.0 / goal_publish_rate),
            std::bind(&Dsp::goalPublishCallback, this));

    cost_srv_ = this->create_service<dsp_interfaces::srv::PathCost>("/dsp/path_cost",
    std::bind(&Dsp::request_cost, this, std::placeholders::_1, std::placeholders::_2));

    // Planning failures are pushed to this receiver (frontier_viewpoint_selection serves it).
    planning_failure_client_ = this->create_client<std_srvs::srv::Trigger>("/dsp/planning_failure");

    nlohmann::json node_info = {
        {"description", "D* Lite 3D path planner"},
        {"version", "0.2.0"},
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
    });

    start_pos << 0,0,0;
    goal_pos << 0,0,0;
    pmin << 0,0,0;

    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
}

Dsp::~Dsp()
{
    if (ros2top_registered_) {
        ros2top::unregister_node("/" + std::string(get_name()));
    }
}

// ---------------------------------------------------------------------------
// Occupancy state and hard obstacle padding
//
// occupancy_map holds the raw map state per voxel (DSP_FREE, dsp_unknown_ or DSP_OCCUPIED).
// padding_count_ counts, per voxel, how many occupied voxels have it inside their padding,
// so obstacles can be added and removed incrementally. The cost given to the search graph
// (effectiveCost) is DSP_OCCUPIED for occupied *and* padded voxels.
// ---------------------------------------------------------------------------

int Dsp::voxelIndex(int x, int y, int z) const
{
    return x + y * length_voxel + z * length_voxel * width_voxel;
}

bool Dsp::inBounds(int x, int y, int z) const
{
    return x >= 0 && x < length_voxel && y >= 0 && y < width_voxel && z >= 0 && z < height_voxel;
}

void Dsp::resetOccupancy()
{
    const size_t size = static_cast<size_t>(length_voxel) * width_voxel * height_voxel;
    occupancy_map.assign(size, dsp_unknown_);
    padding_count_.assign(size, 0);
}

void Dsp::buildPaddingOffsets()
{
    padding_offsets_.clear();
    if (padding_m_ <= 0.0 || res_octomap <= 0.0) {
        return;
    }

    // A voxel at offset d (in voxels) from an occupied voxel has its centre at distance
    // res * sqrt(sum(max(|d_i| - 0.5, 0)^2)) from that voxel's surface.
    const double r = padding_m_ / res_octomap;
    const double r2 = r * r - 1e-9;
    const int reach = static_cast<int>(std::ceil(r + 0.5));
    const int reach_z = (height_voxel > 1) ? reach : 0;

    for (int dz = -reach_z; dz <= reach_z; dz++) {
        for (int dy = -reach; dy <= reach; dy++) {
            for (int dx = -reach; dx <= reach; dx++) {
                if (dx == 0 && dy == 0 && dz == 0) {
                    continue;
                }
                const double gx = std::max(std::abs(dx) - 0.5, 0.0);
                const double gy = std::max(std::abs(dy) - 0.5, 0.0);
                const double gz = std::max(std::abs(dz) - 0.5, 0.0);
                if (gx * gx + gy * gy + gz * gz < r2) {
                    padding_offsets_.emplace_back(dx, dy, dz);
                }
            }
        }
    }
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

// Push every voxel whose effective cost changed into the search graph.
void Dsp::syncGraphCosts()
{
    if (!gdsl_) {
        return;
    }
    for (int z = 0; z < height_voxel; z++) {
        for (int y = 0; y < width_voxel; y++) {
            for (int x = 0; x < length_voxel; x++) {
                const int idx = voxelIndex(x, y, z);
                const double cost = effectiveCost(idx);
                if (cost != graph_cost_[idx]) {
                    // Use the voxel centre so DSL's floor() lands in the right cell.
                    gdsl_->SetCost(Eigen::Vector3d(x + 0.5, y + 0.5, z + 0.5), cost);
                    graph_cost_[idx] = cost;
                }
            }
        }
    }
}

// 2D occupancyGrid callback
void Dsp::occupancy_grid_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
    std::lock_guard<std::mutex> lock(planning_mutex_);

    dsp_unknown_ =this->get_parameter("DSP_UNKNOWN").as_int();
    const int upper_thresh_ = this->get_parameter("upper_thresh").as_int();
    const int lower_thresh_ = this->get_parameter("lower_thresh").as_int();
    const double padding = this->get_parameter("obstacle_padding").as_double();

    const int nx = msg->info.width;
    const int ny = msg->info.height;
    const double res = msg->info.resolution;
    const Eigen::Vector3d new_min(msg->info.origin.position.x, msg->info.origin.position.y, 0.0);
    if (nx <= 0 || ny <= 0 || res <= 0.0 || msg->data.size() != static_cast<size_t>(nx) * ny) {
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
        const int value = msg->data[i];
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
    }
}

// octomap 3D callback
void Dsp::octomap_data_callback(const octomap_msgs::msg::Octomap::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(planning_mutex_);

    std::unique_ptr<octomap::AbstractOcTree> abstract_tree(octomap_msgs::msgToMap(*msg));
    const octomap::OcTree *tree = dynamic_cast<octomap::OcTree*>(abstract_tree.get());
    if (!tree) {
        RCLCPP_WARN(this->get_logger(), "Received octomap that is not an OcTree, ignoring");
        return;
    }
    if (tree->size() == 0) {
        return;
    }

    dsp_unknown_ = this->get_parameter("DSP_UNKNOWN").as_int();
    const double padding = this->get_parameter("obstacle_padding").as_double();
    const double res = tree->getResolution();
    double size_x, size_y, size_z;
    tree->getMetricSize(size_x, size_y, size_z);
    Eigen::Vector3d new_min;
    tree->getMetricMin(new_min(0), new_min(1), new_min(2));

    const int nx = static_cast<int>(std::lround(size_x / res));
    const int ny = static_cast<int>(std::lround(size_y / res));
    const int nz = static_cast<int>(std::lround(size_z / res));
    if (nx <= 0 || ny <= 0 || nz <= 0) {
        return;
    }

    const bool same_layout = grid_built
        && nx == length_voxel && ny == width_voxel && nz == height_voxel
        && std::abs(res - res_octomap) < 1e-9
        && (new_min - pmin).cwiseAbs().maxCoeff() < 0.5 * res
        && std::abs(padding - padding_m_) < 1e-9;

    if (same_layout)
    {
        updateGDSP(*tree);
    }
    else
    {
        res_octomap = res;
        pmin = new_min;
        length_voxel = nx;
        width_voxel = ny;
        height_voxel = nz;
        padding_m_ = padding;
        buildGDSP(*tree);
        grid_built = true;
    }
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


// Builds the search graph from graph_cost_ (must already hold the effective costs).
void Dsp::buildGraph(){
    RCLCPP_INFO(this->get_logger(), "Building search graph (%d x %d x %d voxels, %zu padding offsets)...",
        length_voxel, width_voxel, height_voxel, padding_offsets_.size());
    gdsl_.reset();
    connectivity_.reset();
    grid_.reset(new dsl::Grid3d(length_voxel, width_voxel, height_voxel,
        graph_cost_.data(),
        1, 1, 1, 1, DSP_OCCUPIED + 1));
    connectivity_.reset(new dsl::Grid3dConnectivity(*grid_));
    gdsl_.reset(new dsl::GridSearch<3>(*grid_, *connectivity_, cost_, true));
    RCLCPP_INFO(this->get_logger(), "Graph built");
}

void Dsp::updateGDSP(const octomap::OcTree &tree)
{
    forEachLeafVoxel(tree, [this](int x, int y, int z, bool occupied) {
        setVoxelState(x, y, z, occupied ? DSP_OCCUPIED : DSP_FREE);
    });

    // Recompute height penalties and push all changed costs to the current planner graph.
    buildHeightMap();
    buildHeightCostMap();
    syncGraphCosts();
}

void Dsp::handleSetStartOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    Eigen::Vector3d wpos(msg->pose.pose.position.x , msg->pose.pose.position.y, msg->pose.pose.position.z);
    //setStart(wpos);
}

void Dsp::request_cost(const std::shared_ptr<dsp_interfaces::srv::PathCost::Request> req, std::shared_ptr<dsp_interfaces::srv::PathCost::Response> res){
    std::lock_guard<std::mutex> lock(planning_mutex_);

    if (!grid_built || !gdsl_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "path_cost requested before a map was received");
        return;
    }

    // Snap both ends to their voxel centres so DSL's floor() lands in the right cell.
    Eigen::Vector3d grid_start = posRes(Eigen::Vector3d(req->start.x, req->start.y, req->start.z));

    Eigen::Vector3d grid_goal = posRes(Eigen::Vector3d(req->stop.x, req->stop.y, req->stop.z));

    for (int axis = 0; axis < 3; ++axis) {
        grid_start(axis) = std::floor(grid_start(axis)) + 0.5;
        grid_goal(axis) = std::floor(grid_goal(axis)) + 0.5;
    }

    auto valid_grid_point = [this](const Eigen::Vector3d &point) {
        return point(0) >= 0.0 && point(0) < length_voxel &&
            point(1) >= 0.0 && point(1) < width_voxel &&
            point(2) >= 0.0 && point(2) < height_voxel;
    };
    if (!valid_grid_point(grid_start) || !valid_grid_point(grid_goal) ||
        grid_start == grid_goal) {
        return;
    }

    // A separate search over the shared grid, so cost queries never disturb the incremental
    // D* Lite state of gdsl_ (the path being flown). SetCost on gdsl_ writes through to grid_,
    // so this search sees the current map.
    dsl::GridSearch<3> query_search(*grid_, *connectivity_, cost_, true);
    if (!query_search.SetStart(grid_start) || !query_search.SetGoal(grid_goal)) {
        return;
    }

    dsl::GridPath<3> path;
    query_search.Plan(path);
    res->cost = path.cost;
    nav_msgs::msg::Path ros_path = dspPathToRosMsg(path,false);
    res->path = ros_path;
    res->distance = path_distance(ros_path);

    return;
}

float Dsp::path_distance(const nav_msgs::msg::Path path)
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

void Dsp::handleSetStart(const geometry_msgs::msg::Point::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(planning_mutex_);
    Eigen::Vector3d wpos(msg->x, msg->y, msg->z);
    setStart(wpos);
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

void Dsp::setStart(Eigen::Vector3d wpos)
{
    if(!this->get_parameter("use_3d").as_bool()){
        wpos[2] = 0;
    }
    start_pos = wpos;
}

void Dsp::handleSetGoal(const std::shared_ptr<dsp_interfaces::srv::SetGoal::Request> req,
                        std::shared_ptr<dsp_interfaces::srv::SetGoal::Response> res)
{
    std::lock_guard<std::mutex> lock(planning_mutex_);

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

    goal_pose_ = goal;
    goal_pos << goal.pose.position.x, goal.pose.position.y, goal.pose.position.z;
    if(!this->get_parameter("use_3d").as_bool()){
        goal_pos[2] = 0;
    }
    has_goal_ = true;

    // Drop the path to the previous goal so it is never followed towards the new one.
    has_path_ = false;
    path_msg_.poses.clear();
    path_progress_idx_ = 0;
    final_goal_sent_ = false;

    RCLCPP_INFO(this->get_logger(), "New goal (%.2f, %.2f, %.2f) in %s",
        goal_pos.x(), goal_pos.y(), goal_pos.z(), odom_frame.c_str());

    res->success = true;
    if (!grid_built) {
        res->message = "Goal accepted; waiting for the first map before planning";
        return;
    }

    // Plan and send the first look-ahead goal right away instead of waiting for the timers,
    // then restart the timers so their next tick is a full period from now.
    if (planPath()) {
        publishGoalAhead();
        res->message = "Goal accepted; path planned with " + std::to_string(path_msg_.poses.size()) + " poses";
    } else {
        res->message = "Goal accepted, but no path could be planned yet; retrying on the plan timer";
    }
    if (path_update_timer) {
        path_update_timer->reset();
    }
    if (goal_publish_timer) {
        goal_publish_timer->reset();
    }
}

// Plans from the drone's current position to the goal and stores/publishes the path.
bool Dsp::planPath(){
    if(!grid_built || !gdsl_ || !has_goal_){
        return false;
    }

    // Falls back to the last known start (e.g. from dsp/set_start) when TF is unavailable.
    Eigen::Vector3d robot_pos;
    if (lookupRobotPosition(robot_pos)) {
        setStart(robot_pos);
    }

    const Eigen::Vector3d grid_start = posRes(start_pos);
    const Eigen::Vector3d grid_goal = posRes(goal_pos);
    if (std::floor(grid_start(0)) == std::floor(grid_goal(0))
        && std::floor(grid_start(1)) == std::floor(grid_goal(1))
        && std::floor(grid_start(2)) == std::floor(grid_goal(2)))
    {
        // Already in the goal voxel: the remaining path is just the goal itself.
        path_msg_ = nav_msgs::msg::Path();
        path_msg_.header.frame_id = goal_pose_.header.frame_id;
        path_msg_.header.stamp = this->now();
        path_msg_.poses.push_back(goal_pose_);
        path_progress_idx_ = 0;
        has_path_ = true;
        path_pub_->publish(path_msg_);
        splinepath_pub_->publish(path_msg_);
        return true;
    }

    if(!setSG(start_pos, goal_pos)){
        return false;
    }

    dsl::GridPath<3> new_path;
    if (!gdsl_->Plan(new_path) || new_path.cells.empty()) {
        reportPlanningFailure("DSP was unable to find a path to the requested goal.");
        return false;
    }

    path_ = new_path;
    // Short paths are published unsplined (see publishAllPaths); DSL needs >= 3 points.
    if (path_.cells.size() > 3) {
        gdsl_->SplinePath(path_, splinepath_, spline_step_);
    }
    path_msg_ = dspPathToRosMsg(path_, false);
    path_msg_.header.stamp = this->now();
    path_progress_idx_ = 0;
    has_path_ = true;

    publishAllPaths();
    return true;
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

bool Dsp::setSG(Eigen::Vector3d start, Eigen::Vector3d goal){

    Eigen::Vector3d grid_start = posRes(start);
    Eigen::Vector3d grid_goal = posRes(goal);
    auto valid_grid_point = [this](const Eigen::Vector3d &point) {
        return point(0) >= 0.0 && point(0) < length_voxel &&
            point(1) >= 0.0 && point(1) < width_voxel &&
            point(2) >= 0.0 && point(2) < height_voxel;
    };

    if (!gdsl_) {
        reportPlanningFailure("DSP planner is not initialized.");
        return false;
    }
    if (!valid_grid_point(grid_start) || !valid_grid_point(grid_goal)) {
        reportPlanningFailure("Planning request is outside the DSP map.");
        return false;
    }

    if((int) grid_start(0) == (int) grid_goal(0)
        and (int) grid_start(1) == (int) grid_goal(1)
        and (int) grid_start(2) == (int) grid_goal(2))
    {
        reportPlanningFailure("Start and goal poses are in the same grid cell.");
        return false;
    }
    if (!gdsl_->SetStart(grid_start))
    {
        reportPlanningFailure("DSP failed to set the planning start.");
        return false;
    }
    if (!gdsl_->SetGoal(grid_goal))
    {
        reportPlanningFailure("DSP failed to set the planning goal.");
        return false;
    }
    return true;
}
// transfomre a pose betewn IRL cordinates and map cordinate
Eigen::Vector3d Dsp::posRes(Eigen::Vector3d wpos)
{
    for (int i = 0; i < 3; i++)
    {
        wpos(i) = (wpos(i) - pmin(i)) / res_octomap;
    }
    return wpos;
}

void Dsp::pathUpdateCallback(){
    std::lock_guard<std::mutex> lock(planning_mutex_);
    planPath();
}

void Dsp::goalPublishCallback(){
    std::lock_guard<std::mutex> lock(planning_mutex_);
    publishGoalAhead();
}

// Publishes the path pose goal_voxels_ahead voxels ahead of the drone's closest point on the
// latest path. Once that runs past the end of the path, the requested goal itself (exact
// position + orientation) is published, once, instead of re-sending it every tick.
void Dsp::publishGoalAhead()
{
    if (!has_goal_ || !has_path_ || path_msg_.poses.empty()) {
        return;
    }
    const auto &poses = path_msg_.poses;

    Eigen::Vector3d robot_pos;
    if (!lookupRobotPosition(robot_pos)) {
        robot_pos = start_pos;
    }
    if (!this->get_parameter("use_3d").as_bool()) {
        robot_pos[2] = 0.0;
    }

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

    const int64_t voxels_ahead = std::max<int64_t>(0, this->get_parameter("goal_voxels_ahead").as_int());
    const size_t target = nearest + static_cast<size_t>(voxels_ahead);

    geometry_msgs::msg::PoseStamped goal_msg;
    goal_msg.header.frame_id = this->get_parameter("odom_frame_id").as_string();
    goal_msg.header.stamp = this->now();

    if (target >= poses.size() - 1) {
        if (final_goal_sent_) {
            return;
        }
        goal_msg.pose = goal_pose_.pose;
        final_goal_sent_ = true;
    } else {
        // Intermediate goals face along the path; only the final goal carries the requested
        // orientation (ego-planner turns to a goal's yaw once it arrives there).
        goal_msg.pose.position = poses[target].pose.position;
        goal_msg.pose.orientation = pathHeading(target);
        final_goal_sent_ = false;
    }
    goal_pub_->publish(goal_msg);
}

// Horizontal heading of the latest path at pose idx (direction to the next pose that is not
// straight above/below it). Falls back to the goal orientation if the rest of the path is vertical.
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
    return goal_pose_.pose.orientation;
}

void Dsp::publishAllPaths()
{
    path_pub_->publish(path_msg_);

    if (path_.cells.size() <= 3){
        splinepath_pub_->publish(path_msg_);
    } else {
        splinepath_pub_->publish(dspPathToRosMsg(splinepath_, true));
    }
}

// transfom paht to ros paht
nav_msgs::msg::Path Dsp::dspPathToRosMsg(const dsl::GridPath<3> &dsp_path, bool isSplined)
{
  std::vector<Eigen::Vector3d> path;
  for(unsigned long int i = 0; i < dsp_path.cells.size(); i++)
  {
    path.push_back(dsp_path.cells[i].c * res_octomap);
  }
  return dspPathToRosMsg(path, isSplined);
}

nav_msgs::msg::Path Dsp::dspPathToRosMsg(const std::vector<Eigen::Vector3d> &dsp_path, bool isSplined)
{
  nav_msgs::msg::Path msg;

  msg.header.frame_id = this->get_parameter("odom_frame_id").as_string();
  msg.poses.resize(dsp_path.size());
  for(unsigned long int i = 0; i < dsp_path.size(); i++)
  {
    if(isSplined){
        msg.poses[i].pose.position.x = dsp_path[i][0] * res_octomap + pmin(0);
        msg.poses[i].pose.position.y = dsp_path[i][1] * res_octomap + pmin(1);
        msg.poses[i].pose.position.z = dsp_path[i][2] * res_octomap + pmin(2);
    }
    else{
        msg.poses[i].pose.position.x = dsp_path[i][0] + pmin(0);
        msg.poses[i].pose.position.y = dsp_path[i][1] + pmin(1);
        msg.poses[i].pose.position.z = dsp_path[i][2] + pmin(2);
    }
  }

  return msg;
}


// publishing map usedfule when debuging (shows occupied + padded voxels)
void Dsp::publishOccupancyGrid()
{
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
  rclcpp::spin( std::make_shared<dsp::Dsp>() );
  rclcpp::shutdown();


  return 0;
}
