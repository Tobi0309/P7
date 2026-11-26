// Floor height map from an OctoMap, shared by topdown_map_node and floor_height_map_node.
//
// The voxel grid is laid out exactly as DSP builds it (dsp.cpp: getMetricMin/getMetricSize,
// leaves larger than the resolution cover n^3 voxels), and the floor follows
// DSP::buildHeightMap: the lowest occupied voxel with a known-free voxel directly above,
// median-smoothed over a k x k window. Columns still without a floor take the floor of the
// nearest column that has one.

#pragma once

#include <octomap/OcTree.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace p7_floor {

enum Voxel : uint8_t { V_UNKNOWN = 0, V_FREE = 1, V_OCCUPIED = 2 };

struct VoxelGrid {
    double res{0.5}, min_x{0.0}, min_y{0.0}, min_z{0.0};
    int nx{0}, ny{0}, nz{0};
    std::vector<uint8_t> voxels;

    size_t voxel(int x, int y, int z) const { return (static_cast<size_t>(z) * ny + y) * nx + x; }
    size_t cell(int x, int y) const { return static_cast<size_t>(y) * nx + x; }
    // World z of the top surface of floor voxel index z.
    double floorTop(int z) const { return min_z + (z + 1) * res; }
};

// Returns false for an empty tree.
inline bool fromOcTree(const octomap::OcTree &tree, VoxelGrid &grid) {
    if (tree.size() == 0) {
        return false;
    }
    grid.res = tree.getResolution();
    double size_x, size_y, size_z;
    tree.getMetricSize(size_x, size_y, size_z);
    tree.getMetricMin(grid.min_x, grid.min_y, grid.min_z);
    grid.nx = static_cast<int>(std::lround(size_x / grid.res));
    grid.ny = static_cast<int>(std::lround(size_y / grid.res));
    grid.nz = static_cast<int>(std::lround(size_z / grid.res));
    if (grid.nx <= 0 || grid.ny <= 0 || grid.nz <= 0) {
        return false;
    }
    grid.voxels.assign(static_cast<size_t>(grid.nx) * grid.ny * grid.nz, V_UNKNOWN);
    for (auto it = tree.begin_leafs(), end = tree.end_leafs(); it != end; ++it) {
        const uint8_t state = tree.isNodeOccupied(*it) ? V_OCCUPIED : V_FREE;
        const double size = it.getSize();
        const int n = std::max(1, static_cast<int>(std::lround(size / grid.res)));
        const int x0 = static_cast<int>(std::lround((it.getX() - 0.5 * size - grid.min_x) / grid.res));
        const int y0 = static_cast<int>(std::lround((it.getY() - 0.5 * size - grid.min_y) / grid.res));
        const int z0 = static_cast<int>(std::lround((it.getZ() - 0.5 * size - grid.min_z) / grid.res));
        for (int k = 0; k < n; ++k) {
            for (int j = 0; j < n; ++j) {
                for (int i = 0; i < n; ++i) {
                    const int x = x0 + i, y = y0 + j, z = z0 + k;
                    if (x >= 0 && y >= 0 && z >= 0 && x < grid.nx && y < grid.ny && z < grid.nz) {
                        grid.voxels[grid.voxel(x, y, z)] = state;
                    }
                }
            }
        }
    }
    return true;
}

// DSP::buildHeightMap: lowest occupied voxel with a known-free voxel directly above (-1: none).
inline std::vector<int> rawFloor(const VoxelGrid &g) {
    std::vector<int> floor(static_cast<size_t>(g.nx) * g.ny, -1);
    for (int y = 0; y < g.ny; ++y) {
        for (int x = 0; x < g.nx; ++x) {
            for (int z = 0; z < g.nz - 1; ++z) {
                if (g.voxels[g.voxel(x, y, z)] == V_OCCUPIED && g.voxels[g.voxel(x, y, z + 1)] == V_FREE) {
                    floor[g.cell(x, y)] = z;
                    break;
                }
            }
        }
    }
    return floor;
}

// DSP::buildHeightMap smoothing: median of the known floor heights in a k x k window
// (cells within (k-1)/2 of the border keep their raw value, as in DSP). k must be odd.
inline std::vector<int> smoothFloor(const VoxelGrid &g, const std::vector<int> &raw, int kernel_size) {
    const int pad = (kernel_size - 1) / 2;
    std::vector<int> smoothed = raw;
    std::vector<int> heights;
    for (int y = pad; y < g.ny - pad; ++y) {
        for (int x = pad; x < g.nx - pad; ++x) {
            heights.clear();
            for (int dy = -pad; dy <= pad; ++dy) {
                for (int dx = -pad; dx <= pad; ++dx) {
                    const int h = raw[g.cell(x + dx, y + dy)];
                    if (h >= 0) {
                        heights.push_back(h);
                    }
                }
            }
            if (!heights.empty()) {
                const size_t mid = heights.size() / 2;
                std::nth_element(heights.begin(), heights.begin() + mid, heights.end());
                smoothed[g.cell(x, y)] = heights[mid];
            }
        }
    }
    return smoothed;
}

// Columns without a floor estimate take the floor of the nearest column that has one
// (multi-source BFS, 8-connected). The nearest floor follows slopes; a global value would not.
inline std::vector<int> fillFloor(const VoxelGrid &g, const std::vector<int> &floor) {
    std::vector<int> filled = floor;
    std::vector<size_t> queue;
    for (size_t c = 0; c < filled.size(); ++c) {
        if (filled[c] >= 0) {
            queue.push_back(c);
        }
    }
    for (size_t head = 0; head < queue.size(); ++head) {
        const int x = static_cast<int>(queue[head] % g.nx);
        const int y = static_cast<int>(queue[head] / g.nx);
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                const int nxp = x + dx, nyp = y + dy;
                if (nxp < 0 || nyp < 0 || nxp >= g.nx || nyp >= g.ny || filled[g.cell(nxp, nyp)] >= 0) {
                    continue;
                }
                filled[g.cell(nxp, nyp)] = filled[queue[head]];
                queue.push_back(g.cell(nxp, nyp));
            }
        }
    }
    return filled;
}

}  // namespace p7_floor
