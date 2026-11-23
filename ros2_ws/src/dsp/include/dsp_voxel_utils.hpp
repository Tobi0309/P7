#ifndef DSP_VOXEL_UTILS_HPP_
#define DSP_VOXEL_UTILS_HPP_

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <limits>
#include <unordered_set>
#include <vector>

namespace dsp
{

// Dense voxel grid layout shared by DSP's cost arrays and DSL's Grid3d:
// index = x + y * nx + z * nx * ny (the same order as dsl::Grid::Id()).
struct VoxelLayout
{
    int nx{0};
    int ny{0};
    int nz{0};

    int size() const
    {
        return nx * ny * nz;
    }

    bool inBounds(const Eigen::Vector3i &v) const
    {
        return v.x() >= 0 && v.x() < nx && v.y() >= 0 && v.y() < ny && v.z() >= 0 && v.z() < nz;
    }

    int index(const Eigen::Vector3i &v) const
    {
        return v.x() + v.y() * nx + v.z() * nx * ny;
    }

    Eigen::Vector3i coords(int idx) const
    {
        const int plane = nx * ny;
        return Eigen::Vector3i(idx % nx, (idx % plane) / nx, idx / plane);
    }
};

// Offsets (in voxels) of every voxel whose centre lies closer than `padding_voxels`
// (padding distance / resolution) to the surface of the voxel at the origin. A voxel at offset
// d has its centre at distance sqrt(sum(max(|d_i| - 0.5, 0)^2)) from that surface.
inline std::vector<Eigen::Vector3i> paddingOffsets(double padding_voxels, bool use_z)
{
    std::vector<Eigen::Vector3i> offsets;
    if (!(padding_voxels > 0.0)) {
        return offsets;
    }
    const double r2 = padding_voxels * padding_voxels - 1e-9;
    const int reach = static_cast<int>(std::ceil(padding_voxels + 0.5));
    const int reach_z = use_z ? reach : 0;
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
                    offsets.emplace_back(dx, dy, dz);
                }
            }
        }
    }
    return offsets;
}

// Nearest voxel (Euclidean, in voxels) to `start` that is not blocked, within `radius` voxels.
// The breadth-first search may pass through blocked voxels (e.g. obstacle padding) but never
// enters a wall voxel other than `start`, so it cannot jump through a wall. Returns `start` if
// it is not blocked and -1 if there is no free voxel within the radius.
template <class Blocked, class Wall>
int nearestFree(const VoxelLayout &layout, int start, double radius, Blocked blocked, Wall wall)
{
    if (start < 0 || start >= layout.size()) {
        return -1;
    }
    if (!blocked(start)) {
        return start;
    }
    const Eigen::Vector3i origin = layout.coords(start);
    const double r2 = radius * radius + 1e-9;
    std::deque<int> queue{start};
    std::unordered_set<int> seen{start};
    int best = -1;
    double best_d2 = std::numeric_limits<double>::infinity();
    while (!queue.empty()) {
        const Eigen::Vector3i current = layout.coords(queue.front());
        queue.pop_front();
        for (int dz = -1; dz <= 1; dz++) {
            for (int dy = -1; dy <= 1; dy++) {
                for (int dx = -1; dx <= 1; dx++) {
                    if (dx == 0 && dy == 0 && dz == 0) {
                        continue;
                    }
                    const Eigen::Vector3i next = current + Eigen::Vector3i(dx, dy, dz);
                    if (!layout.inBounds(next)) {
                        continue;
                    }
                    const double d2 = (next - origin).cast<double>().squaredNorm();
                    if (d2 > r2) {
                        continue;
                    }
                    const int idx = layout.index(next);
                    if (!seen.insert(idx).second || wall(idx)) {
                        continue;
                    }
                    if (!blocked(idx)) {
                        if (d2 < best_d2) {
                            best_d2 = d2;
                            best = idx;
                        }
                        continue;
                    }
                    queue.push_back(idx);
                }
            }
        }
    }
    return best;
}

// Cost of moving along consecutive voxels with DSL's GridCost::Real() formula,
// (1 + (c_a + c_b) / 2) * |a - b| with |a - b| in voxels. DSL's own path.cost is not used
// because DSL does not update an edge's stored path cost when its cost changes.
inline double pathCost(const VoxelLayout &layout, const std::vector<int> &cells,
                       const std::vector<double> &cost)
{
    double total = 0.0;
    for (std::size_t i = 1; i < cells.size(); i++) {
        const double step =
            (layout.coords(cells[i]) - layout.coords(cells[i - 1])).cast<double>().norm();
        total += (1.0 + 0.5 * (cost[cells[i - 1]] + cost[cells[i]])) * step;
    }
    return total;
}

template <class Blocked>
bool crossesBlocked(const std::vector<int> &cells, Blocked blocked)
{
    return std::any_of(cells.begin(), cells.end(), [&](int idx) { return blocked(idx); });
}

// Connected components of the voxels that are not blocked, with the same neighbourhood as DSL's
// Grid3dConnectivity (26 neighbours; 8 when nz == 1). Returns one label per voxel: -1 for a
// blocked voxel, otherwise the component number. Two voxels with the same label are joined by
// a path that never enters a blocked voxel; with different labels no such path exists, and a
// D* Lite search between them would expand every voxel it can reach before it gives up.
template <class Blocked>
std::vector<int> labelComponents(const VoxelLayout &layout, Blocked blocked)
{
    const int size = std::max(layout.size(), 0);
    const int nx = layout.nx;
    const int ny = layout.ny;
    const int nz = layout.nz;
    const int plane = nx * ny;
    std::vector<int> label(static_cast<std::size_t>(size), -1);
    std::vector<int> stack;
    int next_label = 0;
    for (int seed = 0; seed < size; seed++) {
        if (label[seed] != -1 || blocked(seed)) {
            continue;
        }
        label[seed] = next_label;
        stack.push_back(seed);
        while (!stack.empty()) {
            const int current = stack.back();
            stack.pop_back();
            const int x = current % nx;
            const int y = (current % plane) / nx;
            const int z = current / plane;
            for (int dz = -1; dz <= 1; dz++) {
                const int zz = z + dz;
                if (zz < 0 || zz >= nz) {
                    continue;
                }
                for (int dy = -1; dy <= 1; dy++) {
                    const int yy = y + dy;
                    if (yy < 0 || yy >= ny) {
                        continue;
                    }
                    for (int dx = -1; dx <= 1; dx++) {
                        const int xx = x + dx;
                        if (xx < 0 || xx >= nx) {
                            continue;
                        }
                        const int idx = xx + yy * nx + zz * plane;
                        if (label[idx] != -1 || blocked(idx)) {
                            continue;
                        }
                        label[idx] = next_label;
                        stack.push_back(idx);
                    }
                }
            }
        }
        next_label++;
    }
    return label;
}

}  // namespace dsp

#endif  // DSP_VOXEL_UTILS_HPP_
