"""2D (x, y) flight heatmap, drawn the same way on screen and in the PNG export.

Every trajectory sample is spread over its neighbourhood with a Gaussian (sigma in metres) on a
fine grid, so the path becomes a soft band whose intensity is the time spent (or path length
flown) per square metre nearby.
"""

import math

import numpy as np
from matplotlib import colormaps
from matplotlib.colors import ListedColormap, LogNorm, Normalize, PowerNorm
from scipy.ndimage import gaussian_filter

# Grids are kept below this many cells per axis; the resolution is coarsened if needed.
MAX_CELLS_PER_AXIS = 2000
WEIGHTS = {
    'time': ('time spent', 's/m²'),
    'path': ('path flown', 'm/m²'),
}


def _fading_colormap(name='turbo'):
    """Colormap whose low end fades to transparent, so the Gaussian tails blend into the background."""
    colours = colormaps[name](np.linspace(0.0, 1.0, 256))
    colours[:, 3] = np.clip(np.linspace(0.0, 1.0, 256) / 0.05, 0.0, 1.0)
    return ListedColormap(colours)


CMAP = _fading_colormap()


def sample_weights(trajectory, weighting):
    if weighting == 'path':
        steps = np.zeros(trajectory.shape[0])
        if trajectory.shape[0] > 1:
            steps[1:] = np.linalg.norm(np.diff(trajectory[:, :3], axis=0), axis=1)
            steps[steps > 5.0] = 0.0  # relocalisation jumps
        return steps
    return trajectory[:, 3]


def density(trajectory, resolution, sigma, weighting='time'):
    """Gaussian-smoothed density. Returns (grid[x, y] per m², x_edges, y_edges, resolution) or None."""
    if trajectory.shape[0] == 0:
        return None
    return points_density(trajectory[:, 0], trajectory[:, 1], sample_weights(trajectory, weighting),
                          resolution, sigma)


def points_density(x, y, weights, resolution, sigma):
    """Weighted points spread with a Gaussian of sigma [m] on a resolution [m] grid, per m²."""
    if len(x) == 0:
        return None
    margin = 4.0 * sigma + resolution
    x_min, x_max = x.min() - margin, x.max() + margin
    y_min, y_max = y.min() - margin, y.max() + margin
    resolution = max(resolution, (x_max - x_min) / MAX_CELLS_PER_AXIS, (y_max - y_min) / MAX_CELLS_PER_AXIS)
    x_min = math.floor(x_min / resolution) * resolution
    y_min = math.floor(y_min / resolution) * resolution
    x_edges = np.arange(x_min, x_max + resolution, resolution)
    y_edges = np.arange(y_min, y_max + resolution, resolution)
    grid, _, _ = np.histogram2d(x, y, bins=[x_edges, y_edges], weights=weights)
    if sigma > 0.0:
        grid = gaussian_filter(grid, sigma / resolution, mode='constant', truncate=4.0)
    return grid / (resolution * resolution), x_edges, y_edges, resolution


def covered_area(trajectory, cell=1.0):
    """Area of the cell x cell [m] cells the drone has flown over."""
    if trajectory.shape[0] == 0:
        return 0.0
    cells = np.unique(np.floor(trajectory[:, :2] / cell).astype(np.int64), axis=0)
    return float(len(cells)) * cell * cell


def draw(fig, ax, cax, trajectory, resolution, sigma, *, weighting='time', scale='sqrt', show_path=True,
         pose=None, viewpoints=None, title=None, frame=''):
    ax.clear()
    cax.clear()
    ax.set_facecolor('#f4f4f4')
    ax.set_xlabel(f'x [m]{" (" + frame + ")" if frame else ""}')
    ax.set_ylabel('y [m]')
    ax.set_aspect('equal', adjustable='datalim')
    ax.grid(True, color='#d0d0d0', linewidth=0.5, zorder=0)
    if title:
        ax.set_title(title, fontsize=10)

    result = density(trajectory, resolution, sigma, weighting)
    if result is None:
        ax.text(0.5, 0.5, 'No odometry recorded yet', transform=ax.transAxes, ha='center', va='center',
                color='#666666')
        cax.set_visible(False)
        return
    label, unit = WEIGHTS.get(weighting, WEIGHTS['time'])
    draw_density(fig, ax, cax, result, scale,
                 f'{label} density [{unit}] (σ {sigma:.2f} m, {result[3]:.2f} m grid)')
    draw_overlays(ax, trajectory, pose, show_path, resolution, viewpoints=viewpoints)


def draw_coverage(fig, ax, cax, coverage, resolution, sigma, *, scale='sqrt', trajectory=None, show_path=True,
                  pose=None, title=None, frame=''):
    """Camera coverage: coverage rows are (x, y, seconds in view) per cell of the coverage grid."""
    ax.clear()
    cax.clear()
    ax.set_facecolor('#f4f4f4')
    ax.set_xlabel(f'x [m]{" (" + frame + ")" if frame else ""}')
    ax.set_ylabel('y [m]')
    ax.set_aspect('equal', adjustable='datalim')
    ax.grid(True, color='#d0d0d0', linewidth=0.5, zorder=0)
    if title:
        ax.set_title(title, fontsize=10)
    if coverage is None or coverage.shape[0] == 0:
        ax.text(0.5, 0.5, 'No /topdown_map/coverage yet\n(is topdown_map_node running and the camera '
                'cloud flowing?)', transform=ax.transAxes, ha='center', va='center', color='#666666')
        cax.set_visible(False)
        return
    result = points_density(coverage[:, 0], coverage[:, 1], coverage[:, 2], resolution, sigma)
    # Per-m² density times the coverage cell area = seconds in view of a fully covered cell.
    cell = coverage_cell_size(coverage)
    grid, x_edges, y_edges, used = result
    draw_density(fig, ax, cax, (grid * cell * cell, x_edges, y_edges, used), scale,
                 f'time in camera view [s] (σ {sigma:.2f} m, {used:.2f} m grid)')
    draw_overlays(ax, trajectory, pose, show_path, resolution)


def coverage_cell_size(coverage):
    xs = np.unique(np.round(coverage[:, 0], 4))
    steps = np.diff(xs)
    steps = steps[steps > 1e-3]
    return float(steps.min()) if steps.size else 0.2


def draw_density(fig, ax, cax, result, scale, colorbar_label):
    grid, x_edges, y_edges, _ = result
    peak = float(grid.max())
    shown = grid.T
    if scale == 'log' and peak > 0.0:
        norm = LogNorm(vmin=peak * 1e-3, vmax=peak, clip=True)
        shown = np.ma.masked_less(grid.T, peak * 1e-3)
    elif scale == 'sqrt':  # lifts the path so a long hover does not wash everything else out
        norm = PowerNorm(gamma=0.5, vmin=0.0, vmax=max(peak, 1e-9))
    else:
        norm = Normalize(vmin=0.0, vmax=max(peak, 1e-9))
    image = ax.imshow(shown, origin='lower', extent=(x_edges[0], x_edges[-1], y_edges[0], y_edges[-1]),
                      cmap=CMAP, norm=norm, interpolation='bilinear', zorder=1)
    cax.set_visible(True)
    fig.colorbar(image, cax=cax, label=colorbar_label)


def draw_overlays(ax, trajectory, pose, show_path, resolution, viewpoints=None):
    if trajectory is None:
        trajectory = np.empty((0, 5))
    if show_path and trajectory.shape[0] > 1:
        ax.plot(trajectory[:, 0], trajectory[:, 1], color='black', linewidth=0.6, alpha=0.45, zorder=2,
                label='path')
    if trajectory.shape[0]:
        ax.plot(trajectory[0, 0], trajectory[0, 1], marker='o', color='#2ca02c', markersize=7, zorder=4,
                linestyle='none', markeredgecolor='white', label='start')
    if viewpoints is not None and len(viewpoints):
        ax.scatter(viewpoints[:, 0], viewpoints[:, 1], marker='x', color='#9467bd', s=30, zorder=3,
                   label='frontier viewpoints')
    draw_pose(ax, pose, max(4.0 * resolution, 1.0))
    if ax.get_legend_handles_labels()[0]:
        ax.legend(loc='upper right', fontsize=8, framealpha=0.85)


def draw_pose(ax, pose, length=1.0, colour='#e377c2'):
    if pose is None:
        return
    x, y, _, yaw = pose
    ax.annotate('', xy=(x + length * math.cos(yaw), y + length * math.sin(yaw)), xytext=(x, y),
                arrowprops=dict(arrowstyle='-|>', color=colour, lw=2), zorder=6)
    ax.plot(x, y, marker='o', color=colour, markersize=7, zorder=6, linestyle='none', markeredgecolor='white',
            label='drone')
