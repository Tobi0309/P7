#include "dsp.hpp"

#include "rclcpp/executors/multi_threaded_executor.hpp"

#include <nlohmann/json.hpp>
#include <ros2top/ros2top.hpp>

#include <limits>
#include <optional>


namespace dsp
{

namespace
{

double millisecondsSince(std::chrono::steady_clock::time_point start)
{
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
}

}  // namespace

Dsp::Dsp() : Node("dsp")
{
    this->declare_parameter("map_topic", "octomap_full");
    this->declare_parameter("lower_thresh", 59);
    this->declare_parameter("upper_thresh", 60);
    // Voxel padding: every voxel whose centre is closer than obstacle_padding [m] to the surface
    // of an occupied voxel is blocked. obstacle_padding_enabled=false turns padding off. Changing
    // either at runtime reallocates the grid on the next map update.
    this->declare_parameter("obstacle_padding_enabled", true);
    this->declare_parameter("obstacle_padding", 0.5);
    // Rate [Hz] at which the newest map message is applied. <= 0 applies every message.
    this->declare_parameter("map_update_rate", 2.0);
    // Slack [m] around the octree bounds when the voxel grid is (re)allocated, so the grid and
    // the search graph on it are not rebuilt every time the map grows a little.
    this->declare_parameter("map_bounds_margin_xy", 10.0);
    this->declare_parameter("map_bounds_margin_z", 3.0);
    // A start or goal inside a blocked voxel (e.g. the drone on the ground, inside the floor's
    // padding) is moved to the nearest free voxel within this radius [m].
    this->declare_parameter("endpoint_snap_radius", 1.0);
    // The search releases its lazily built graph above this many vertices, to bound memory
    // (roughly 12-25 KB per vertex). <= 0 disables the limit.
    this->declare_parameter("max_search_vertices", 50000);
    // A path_cost search that runs longer than this [ms] is abandoned and answered with
    // "no path". <= 0 disables the limit.
    this->declare_parameter("path_cost_max_search_ms", 1000);
    this->declare_parameter("use_3d", true);
    // Frame of the map and of the paths returned by path_cost.
    this->declare_parameter("odom_frame_id", "map");
    this->declare_parameter("DSP_UNKNOWN", 10000);
    this->declare_parameter("height_kernel_size", 3);
    this->declare_parameter("optimal_voxel_height", 6);
    this->declare_parameter("optimal_voxel_height_cost", 10);

    dsp_unknown_ = this->get_parameter("DSP_UNKNOWN").as_int();

    const auto exclusive = rclcpp::CallbackGroupType::MutuallyExclusive;
    query_group_ = this->create_callback_group(exclusive);
    map_group_ = this->create_callback_group(exclusive);
    misc_group_ = this->create_callback_group(exclusive);

    occ_map_viz_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("dsp/occupancy_map",1);

    rclcpp::SubscriptionOptions map_options;
    map_options.callback_group = map_group_;

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

    cost_srv_ = this->create_service<dsp_interfaces::srv::PathCost>("/dsp/path_cost",
        std::bind(&Dsp::request_cost, this, std::placeholders::_1, std::placeholders::_2),
        rclcpp::ServicesQoS(), query_group_);

    const double map_update_rate = this->get_parameter("map_update_rate").as_double();
    if(map_update_rate > 0.0)
        map_timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / map_update_rate),
            std::bind(&Dsp::applyPendingMap, this), map_group_);

    nlohmann::json node_info = {
        {"description", "D* Lite 3D path-cost server for the frontier selector (lazy DSL graph)"},
        {"version", "0.4.0"},
        {"node_type", "path_cost_server"},
        {"topics_published", "dsp/occupancy_map"},
        {"services", "/dsp/path_cost"}
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

    // Latency summary of the path_cost queries since the last summary.
    stats_timer_ = create_wall_timer(std::chrono::seconds(30), [this]() { logQueryStats(); }, misc_group_);
    release_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { freeReleasedSearches(); }, misc_group_);

    pmin << 0,0,0;

    RCLCPP_INFO(get_logger(), "DSP started: path_cost server only (no navigation). Map %s applied at "
        "%.1f Hz, voxel padding %s (%.2f m), unknown voxel cost %d, search budget %ld ms",
        this->get_parameter("map_topic").as_string().c_str(), map_update_rate,
        this->get_parameter("obstacle_padding_enabled").as_bool() ? "on" : "off",
        this->get_parameter("obstacle_padding").as_double(), dsp_unknown_,
        static_cast<long>(this->get_parameter("path_cost_max_search_ms").as_int()));
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

// Push every voxel whose effective cost changed into the grid and the search graph.
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
        if ((graph_cost_[idx] >= DSP_OCCUPIED) != (cost >= DSP_OCCUPIED)) {
            components_dirty_ = true;
        }
        graph_cost_[idx] = cost;
        if (dsl::Cell<3> *cell = grid_->cells[idx]) {
            cell->cost = cost;
        }
        query_search_->SyncCellCost(idx, cost);
    }
}

const std::vector<int> &Dsp::components()
{
    if (components_dirty_ || component_.size() != graph_cost_.size()) {
        const auto started = std::chrono::steady_clock::now();
        component_ = labelComponents(layout(), [this](int idx) { return blocked(idx); });
        components_dirty_ = false;
        RCLCPP_DEBUG(this->get_logger(), "Connectivity labels rebuilt in %.1f ms",
            millisecondsSince(started));
    }
    return component_;
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
        // Decoding is the expensive part and does not touch the search state, so it runs
        // without dsl_mutex_.
        std::unique_ptr<octomap::AbstractOcTree> abstract_tree(octomap_msgs::msgToMap(*octomap_msg));
        const octomap::OcTree *tree = dynamic_cast<octomap::OcTree*>(abstract_tree.get());
        if (!tree) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Received octomap that is not an OcTree, ignoring");
        } else if (tree->size() > 0) {
            std::lock_guard<std::mutex> lock(dsl_mutex_);
            applyOctomap(*tree);
        }
    }
    if (grid_msg) {
        std::lock_guard<std::mutex> lock(dsl_mutex_);
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

    // The octree only grows: keep the current grid (and the search graph on it) while the
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


// (Re)creates the DSL grid and the search from graph_cost_. The search is lazy: no vertex or
// edge exists until a search reaches it, so this is cheap even for large grids. Every cell is
// kept (maxCost above DSP_OCCUPIED) so the graph stays connected; DSL's Plan() has no "no path"
// result. Unconnected endpoints are caught by the component labels before a search, and paths
// through blocked voxels are rejected in searchPath().
void Dsp::buildGraph(){
    const auto start = std::chrono::steady_clock::now();
    query_search_.reset();
    connectivity_.reset();
    grid_.reset(new dsl::Grid3d(length_voxel, width_voxel, height_voxel,
        graph_cost_.data(),
        1, 1, 1, 1, DSP_OCCUPIED + 1));
    connectivity_.reset(new dsl::Grid3dConnectivity(*grid_));
    query_search_ = std::make_unique<DspGridSearch>(*grid_, *connectivity_, grid_cost_);
    components_dirty_ = true;
    RCLCPP_INFO(this->get_logger(),
        "Voxel grid allocated: %d x %d x %d voxels of %.2f m, padding %.2f m (%zu offsets), in %.1f ms; "
        "the search graph is expanded lazily",
        length_voxel, width_voxel, height_voxel, res_octomap, padding_m_, padding_offsets_.size(),
        millisecondsSince(start));
}

// The search keeps every vertex it has expanded; release it once it gets too large
// (caller holds dsl_mutex_).
void Dsp::releaseSearchIfLarge()
{
    const int64_t limit = this->get_parameter("max_search_vertices").as_int();
    if (!query_search_ || limit <= 0 || query_search_->Vertices() <= limit) {
        return;
    }
    RCLCPP_INFO(this->get_logger(), "path_cost search graph reached %d vertices (limit %ld); releasing it",
        query_search_->Vertices(), static_cast<long>(limit));
    std::lock_guard<std::mutex> lock(released_mutex_);
    released_searches_.push_back(std::move(query_search_));
    query_search_ = std::make_unique<DspGridSearch>(*grid_, *connectivity_, grid_cost_);
}

void Dsp::freeReleasedSearches()
{
    std::vector<std::unique_ptr<DspGridSearch>> released;
    {
        std::lock_guard<std::mutex> lock(released_mutex_);
        released.swap(released_searches_);
    }
    if (released.empty()) {
        return;
    }
    const auto started = std::chrono::steady_clock::now();
    released.clear();
    RCLCPP_DEBUG(this->get_logger(), "Freed a released search graph in %.0f ms", millisecondsSince(started));
}

void Dsp::updateGDSP(const octomap::OcTree &tree)
{
    forEachLeafVoxel(tree, [this](int x, int y, int z, bool occupied) {
        setVoxelState(x, y, z, occupied ? DSP_OCCUPIED : DSP_FREE);
    });

    // Recompute height penalties and push all changed costs to the grid and the search.
    buildHeightMap();
    buildHeightCostMap();
    syncGraphCosts();
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

Dsp::SearchResult Dsp::searchPath(const Eigen::Vector3d &start, const Eigen::Vector3d &goal)
{
    SearchResult result;
    if (!query_search_) {
        result.reason = "The search graph does not exist yet.";
        return result;
    }
    int start_idx = -1;
    int goal_idx = -1;
    if (!worldToIndex(start, start_idx)) {
        result.reason = "The start is outside the DSP map.";
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
        result.reason = "The start is inside an obstacle or its padding, with no free voxel "
                        "within endpoint_snap_radius.";
        return result;
    }
    const int goal_cell = nearestFree(grid_layout, goal_idx, snap_radius, is_blocked, is_wall);
    if (goal_cell < 0) {
        result.reason = "The goal is inside an obstacle or its padding, with no free voxel within "
                        "endpoint_snap_radius.";
        return result;
    }

    // Blocked voxels stay in the graph at DSP_OCCUPIED cost, so a search between unconnected
    // voxels would expand everything it can reach before it crossed one. Answer it here.
    const std::vector<int> &labels = components();
    if (labels[start_cell] != labels[goal_cell]) {
        result.reason = "Obstacles separate the start from the goal.";
        result.unconnected = true;
        return result;
    }

    if (start_cell == goal_cell) {
        result.cells.push_back(start_cell);
    } else {
        if (!query_search_->SetStart(gridCenter(start_cell)) || !query_search_->SetGoal(gridCenter(goal_cell))) {
            result.reason = "DSP failed to set the start or goal.";
            return result;
        }
        const int64_t budget_ms = this->get_parameter("path_cost_max_search_ms").as_int();
        query_search_->SetDeadline(budget_ms > 0
            ? std::optional<std::chrono::steady_clock::time_point>(
                  std::chrono::steady_clock::now() + std::chrono::milliseconds(budget_ms))
            : std::nullopt);
        dsl::GridPath<3> grid_path;
        try {
            query_search_->Plan(grid_path);
        } catch (const SearchBudgetExceeded &) {
            query_search_->SetDeadline(std::nullopt);
            result.reason = "The search took longer than path_cost_max_search_ms.";
            result.over_budget = true;
            return result;
        }
        query_search_->SetDeadline(std::nullopt);
        for (const auto &cell : grid_path.cells) {
            result.cells.push_back(grid_->Id(cell.c));
        }
        if (result.cells.size() < 2) {
            result.reason = "DSP was unable to find a path to the requested goal.";
            return result;
        }
    }

    // Safety net: a path that needs a blocked voxel is a path that does not exist.
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
// path_cost service (query group): one persistent search, so a query never rebuilds a graph.
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
    SearchResult result;
    int vertices = 0;
    {
        std::lock_guard<std::mutex> lock(dsl_mutex_);
        result = searchPath(start, stop);
        releaseSearchIfLarge();
        vertices = query_search_ ? query_search_->Vertices() : 0;
    }
    const double elapsed_ms = millisecondsSince(started);
    {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        query_ms_.push_back(elapsed_ms);
        if (!result.ok) {
            ++no_path_count_;
        }
        if (result.unconnected) {
            ++unconnected_count_;
        }
        if (result.over_budget) {
            ++over_budget_count_;
        }
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
        elapsed_ms, result.cells.size(), vertices);
}

void Dsp::logQueryStats()
{
    std::vector<double> times;
    int no_path = 0;
    int unconnected = 0;
    int over_budget = 0;
    {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        times.swap(query_ms_);
        std::swap(no_path, no_path_count_);
        std::swap(unconnected, unconnected_count_);
        std::swap(over_budget, over_budget_count_);
    }
    if (times.empty()) {
        return;
    }
    std::sort(times.begin(), times.end());
    RCLCPP_INFO(this->get_logger(),
        "path_cost in the last 30 s: %zu queries, median %.1f ms, max %.1f ms; no path %d "
        "(%d separated by obstacles, %d over the search budget)",
        times.size(), times[times.size() / 2], times.back(), no_path, unconnected, over_budget);
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

// transfomre a pose betewn IRL cordinates and map cordinate
Eigen::Vector3d Dsp::posRes(Eigen::Vector3d wpos) const
{
    for (int i = 0; i < 3; i++)
    {
        wpos(i) = (wpos(i) - pmin(i)) / res_octomap;
    }
    return wpos;
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
  // One thread per callback group: a map update never waits for the ros2top heartbeat, and a
  // path_cost query only waits for a map update that holds the search state.
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  node.reset();
  rclcpp::shutdown();


  return 0;
}
