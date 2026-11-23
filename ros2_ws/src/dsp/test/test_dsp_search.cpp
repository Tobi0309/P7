#include <gtest/gtest.h>

#include "dsl/grid3d.h"
#include "dsl/grid3dconnectivity.h"
#include "dsp_grid_search.hpp"
#include "dsp_voxel_utils.hpp"

#include <algorithm>
#include <initializer_list>
#include <memory>
#include <vector>

namespace
{

constexpr double kOccupied = 2.0e9;  // DSP_OCCUPIED

// A small DSP-style world: a cost array, the DSL grid built on it, and lazy searches.
struct World
{
    dsp::VoxelLayout layout;
    std::vector<double> cost;
    std::unique_ptr<dsl::Grid3d> grid;
    std::unique_ptr<dsl::Grid3dConnectivity> connectivity;
    dsp::DspGridCost grid_cost;

    World(int nx, int ny, int nz) : layout{nx, ny, nz}, cost(static_cast<size_t>(nx) * ny * nz, 1.0)
    {
        // Same construction as Dsp::buildGraph(): every cell exists, blocked cells are expensive.
        grid = std::make_unique<dsl::Grid3d>(nx, ny, nz, cost.data(), 1, 1, 1, 1, kOccupied + 1);
        connectivity = std::make_unique<dsl::Grid3dConnectivity>(*grid);
    }

    std::unique_ptr<dsp::DspGridSearch> search() const
    {
        return std::make_unique<dsp::DspGridSearch>(*grid, *connectivity, grid_cost);
    }

    int idx(int x, int y, int z) const
    {
        return layout.index(Eigen::Vector3i(x, y, z));
    }

    // What Dsp::syncGraphCosts() does for one voxel.
    void setCost(int id, double value, std::initializer_list<dsp::DspGridSearch *> searches)
    {
        cost[id] = value;
        grid->cells[id]->cost = value;
        for (auto *search : searches) {
            search->SyncCellCost(id, value);
        }
    }

    std::vector<int> plan(dsp::DspGridSearch &search, int from, int to) const
    {
        const auto center = [this](int id) {
            return Eigen::Vector3d(layout.coords(id).cast<double>() + Eigen::Vector3d::Constant(0.5));
        };
        EXPECT_TRUE(search.SetStart(center(from)));
        EXPECT_TRUE(search.SetGoal(center(to)));
        dsl::GridPath<3> path;
        search.Plan(path);
        std::vector<int> cells;
        for (const auto &cell : path.cells) {
            cells.push_back(grid->Id(cell.c));
        }
        return cells;
    }

    bool blocked(int id) const
    {
        return cost[id] >= kOccupied;
    }
};

}  // namespace

TEST(DspGridSearch, PlansStraightLineInFreeSpace)
{
    World world(10, 9, 9);
    auto search = world.search();
    const auto cells = world.plan(*search, world.idx(0, 4, 4), world.idx(9, 4, 4));
    ASSERT_EQ(cells.size(), 10u);
    EXPECT_EQ(cells.front(), world.idx(0, 4, 4));
    EXPECT_EQ(cells.back(), world.idx(9, 4, 4));
    // 9 unit steps between free voxels, each (1 + (1 + 1) / 2) * 1 = 2.
    EXPECT_DOUBLE_EQ(dsp::pathCost(world.layout, cells, world.cost), 18.0);
    // Lazy: only the voxels around the line became graph vertices, not the whole 810-voxel grid.
    EXPECT_LT(search->Vertices(), 300);
}

TEST(DspGridSearch, SyncOnUnexpandedCellIsSafe)
{
    World world(10, 3, 3);
    auto search = world.search();
    ASSERT_FALSE(search->HasVertex(world.idx(5, 1, 1)));
    // DSL's own SetCost() dereferences a null vertex here.
    world.setCost(world.idx(5, 1, 1), kOccupied, {search.get()});
    const auto cells = world.plan(*search, world.idx(0, 1, 1), world.idx(9, 1, 1));
    ASSERT_FALSE(cells.empty());
    EXPECT_FALSE(dsp::crossesBlocked(cells, [&](int id) { return world.blocked(id); }));
}

TEST(DspGridSearch, ReplanAvoidsNewWall)
{
    World world(10, 5, 3);
    auto search = world.search();
    const int start = world.idx(0, 2, 1);
    const int goal = world.idx(9, 2, 1);
    ASSERT_FALSE(world.plan(*search, start, goal).empty());

    // Wall across x = 5 with a single gap at (5, 4, 1).
    for (int y = 0; y < 5; y++) {
        for (int z = 0; z < 3; z++) {
            if (!(y == 4 && z == 1)) {
                world.setCost(world.idx(5, y, z), kOccupied, {search.get()});
            }
        }
    }
    const auto cells = world.plan(*search, start, goal);
    ASSERT_FALSE(cells.empty());
    EXPECT_FALSE(dsp::crossesBlocked(cells, [&](int id) { return world.blocked(id); }));
    EXPECT_NE(std::find(cells.begin(), cells.end(), world.idx(5, 4, 1)), cells.end());
}

TEST(DspGridSearch, SharedGridUpdatesBothSearches)
{
    World world(10, 5, 3);
    auto nav = world.search();
    auto query = world.search();
    const int start = world.idx(0, 2, 1);
    const int goal = world.idx(9, 2, 1);
    ASSERT_FALSE(world.plan(*nav, start, goal).empty());
    ASSERT_FALSE(world.plan(*query, start, goal).empty());

    // Block the straight line; both searches must see it although they share the Grid cells.
    for (int x = 3; x <= 6; x++) {
        world.setCost(world.idx(x, 2, 1), kOccupied, {nav.get(), query.get()});
    }
    for (auto *search : {nav.get(), query.get()}) {
        const auto cells = world.plan(*search, start, goal);
        ASSERT_FALSE(cells.empty());
        EXPECT_FALSE(dsp::crossesBlocked(cells, [&](int id) { return world.blocked(id); }));
    }
}

TEST(DspGridSearch, EnclosedGoalIsReportedAsBlockedPath)
{
    World world(7, 7, 7);
    auto search = world.search();
    const int goal = world.idx(3, 3, 3);
    for (int dz = -1; dz <= 1; dz++) {
        for (int dy = -1; dy <= 1; dy++) {
            for (int dx = -1; dx <= 1; dx++) {
                if (dx || dy || dz) {
                    world.setCost(world.idx(3 + dx, 3 + dy, 3 + dz), kOccupied, {search.get()});
                }
            }
        }
    }
    // DSL still returns a path (the graph stays connected); it must cross a blocked voxel.
    const auto cells = world.plan(*search, world.idx(0, 0, 0), goal);
    ASSERT_FALSE(cells.empty());
    EXPECT_TRUE(dsp::crossesBlocked(cells, [&](int id) { return world.blocked(id); }));
}

TEST(DspVoxelUtils, PaddingOffsetsFollowVoxelSurfaceRule)
{
    // 0.5 m padding at 0.5 m voxels: exactly the 26 neighbours (3D) or 8 neighbours (2D).
    EXPECT_EQ(dsp::paddingOffsets(1.0, true).size(), 26u);
    EXPECT_EQ(dsp::paddingOffsets(1.0, false).size(), 8u);
    EXPECT_TRUE(dsp::paddingOffsets(0.0, true).empty());
}

TEST(DspVoxelUtils, NearestFreeLiftsStartOutOfGroundPadding)
{
    const dsp::VoxelLayout layout{5, 5, 5};
    // Floor at z = 0 (wall), its padding at z = 1, free above.
    const auto wall = [&](int id) { return layout.coords(id).z() == 0; };
    const auto blocked = [&](int id) { return layout.coords(id).z() <= 1; };
    const int start = layout.index(Eigen::Vector3i(2, 2, 1));
    EXPECT_EQ(dsp::nearestFree(layout, start, 2.0, blocked, wall), layout.index(Eigen::Vector3i(2, 2, 2)));
    // A free start is returned unchanged.
    const int free_start = layout.index(Eigen::Vector3i(2, 2, 3));
    EXPECT_EQ(dsp::nearestFree(layout, free_start, 2.0, blocked, wall), free_start);
    // Nothing free within the radius.
    EXPECT_EQ(dsp::nearestFree(layout, start, 0.5, blocked, wall), -1);
}

TEST(DspVoxelUtils, NearestFreeDoesNotCrossWalls)
{
    // x: 0 free | 1 wall | 2 padded (start) | 3 padded | 4 padded | 5 free
    const dsp::VoxelLayout layout{6, 1, 1};
    const auto wall = [](int id) { return id == 1; };
    const auto blocked = [](int id) { return id >= 1 && id <= 4; };
    // Voxel 0 is nearer but behind the wall; voxel 5 is reachable through padding.
    EXPECT_EQ(dsp::nearestFree(layout, 2, 3.0, blocked, wall), 5);
}

TEST(DspVoxelUtils, PathCostUsesCurrentCellCosts)
{
    const dsp::VoxelLayout layout{3, 3, 1};
    std::vector<double> cost(9, 1.0);
    cost[layout.index(Eigen::Vector3i(1, 1, 0))] = 3.0;
    const std::vector<int> cells{layout.index(Eigen::Vector3i(0, 0, 0)), layout.index(Eigen::Vector3i(1, 1, 0)),
                                 layout.index(Eigen::Vector3i(2, 1, 0))};
    // Diagonal step: (1 + (1 + 3) / 2) * sqrt(2); straight step: (1 + (3 + 1) / 2) * 1.
    EXPECT_NEAR(dsp::pathCost(layout, cells, cost), 3.0 * std::sqrt(2.0) + 3.0, 1e-12);
}

TEST(DspVoxelUtils, ComponentsSplitAtAWallAndJoinThroughAGap)
{
    // Two rooms (x < 5 and x > 5) divided by a wall at x = 5 that fills every y and z.
    World world(11, 5, 3);
    for (int z = 0; z < 3; z++) {
        for (int y = 0; y < 5; y++) {
            world.cost[world.idx(5, y, z)] = kOccupied;
        }
    }
    const auto blocked = [&world](int id) { return world.blocked(id); };
    auto labels = dsp::labelComponents(world.layout, blocked);
    EXPECT_EQ(labels[world.idx(5, 2, 1)], -1);
    EXPECT_GE(labels[world.idx(0, 0, 0)], 0);
    EXPECT_EQ(labels[world.idx(0, 0, 0)], labels[world.idx(4, 4, 2)]);
    EXPECT_NE(labels[world.idx(0, 0, 0)], labels[world.idx(10, 0, 0)]);

    // A one-voxel gap joins the rooms.
    world.cost[world.idx(5, 2, 1)] = 1.0;
    labels = dsp::labelComponents(world.layout, blocked);
    EXPECT_EQ(labels[world.idx(0, 0, 0)], labels[world.idx(10, 4, 2)]);
}

TEST(DspVoxelUtils, ComponentsUseDiagonalNeighboursLikeDsl)
{
    // Voxels that only touch at an edge are connected, as in Grid3dConnectivity.
    World world(2, 2, 1);
    world.cost[world.idx(1, 0, 0)] = kOccupied;
    world.cost[world.idx(0, 1, 0)] = kOccupied;
    const auto labels = dsp::labelComponents(world.layout, [&world](int id) { return world.blocked(id); });
    EXPECT_EQ(labels[world.idx(0, 0, 0)], labels[world.idx(1, 1, 0)]);
}

TEST(DspGridSearch, DeadlineAbortsPlanAndTheNextPlanWorks)
{
    World world(20, 20, 5);
    for (int z = 0; z < 5; z++) {
        for (int y = 0; y < 19; y++) {
            world.cost[world.idx(10, y, z)] = kOccupied;
            world.grid->cells[world.idx(10, y, z)]->cost = kOccupied;
        }
    }
    auto search = world.search();
    const int from = world.idx(0, 0, 2);
    const int to = world.idx(19, 0, 2);
    const auto center = [&world](int id) {
        return Eigen::Vector3d(world.layout.coords(id).cast<double>() + Eigen::Vector3d::Constant(0.5));
    };

    ASSERT_TRUE(search->SetStart(center(from)));
    ASSERT_TRUE(search->SetGoal(center(to)));
    search->SetDeadline(std::chrono::steady_clock::now() - std::chrono::milliseconds(1));
    dsl::GridPath<3> aborted;
    EXPECT_THROW(search->Plan(aborted), dsp::SearchBudgetExceeded);

    // The same search object plans normally afterwards and goes around the wall (gap at y = 19).
    search->SetDeadline(std::nullopt);
    const auto cells = world.plan(*search, from, to);
    ASSERT_GE(cells.size(), 2u);
    EXPECT_EQ(cells.front(), from);
    EXPECT_EQ(cells.back(), to);
    EXPECT_FALSE(dsp::crossesBlocked(cells, [&world](int id) { return world.blocked(id); }));
}
