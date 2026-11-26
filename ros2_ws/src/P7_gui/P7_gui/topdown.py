"""Top-down explored map from P7_topdown_map (/topdown_map/image, /grid, /trees)."""

import numpy as np
from matplotlib.patches import Patch

from .heatmap import draw_pose

# Class values in /topdown_map/grid and the colours P7_topdown_map uses in /topdown_map/image.
CLASSES = [
    (-1, 'not seen by camera', (45, 45, 50)),
    (0, 'floor (shade = height)', (150, 190, 110)),
    (10, 'seen, floor not observed', (165, 175, 195)),
    (40, 'canopy (floor below not seen)', (70, 125, 75)),
    (70, 'low vegetation', (225, 185, 70)),
    (100, 'tree trunk', (140, 70, 35)),
]


def statistics(map_data):
    if map_data is None:
        return None
    classes, cell = map_data['classes'], map_data['resolution'] ** 2
    return {
        'explored': float(np.count_nonzero(classes != -1)) * cell,
        'floor': float(np.count_nonzero(classes == 0)) * cell,
        'trunk': float(np.count_nonzero(classes == 100)) * cell,
        'trees': len(map_data['trees']),
    }


def draw(ax, map_data, trajectory=None, pose=None, *, show_path=True, show_trees=True, title=None, frame=''):
    ax.clear()
    ax.set_facecolor('#2d2d32')
    ax.set_xlabel(f'x [m]{" (" + frame + ")" if frame else ""}')
    ax.set_ylabel('y [m]')
    ax.set_aspect('equal', adjustable='datalim')
    if title:
        ax.set_title(title, fontsize=10)
    if map_data is None:
        ax.text(0.5, 0.5, 'No /topdown_map/image yet\n(is topdown_map_node running?)', transform=ax.transAxes,
                ha='center', va='center', color='#dddddd')
        return
    x0, y0 = map_data['origin']
    res = map_data['resolution']
    height, width = map_data['image'].shape[:2]
    # The image has +y up (row 0 = largest y), so origin='upper' with this extent is metric.
    ax.imshow(map_data['image'], origin='upper', extent=(x0, x0 + width * res, y0, y0 + height * res),
              interpolation='nearest', zorder=1)
    ax.grid(True, color='#ffffff', linewidth=0.3, alpha=0.3, zorder=2)

    handles = [Patch(facecolor=np.array(rgb) / 255.0, edgecolor='#888888', label=label) for _, label, rgb in CLASSES]
    if show_trees and map_data['trees']:
        trees = np.array(map_data['trees'])
        ax.scatter(trees[:, 0], trees[:, 1], s=40, facecolors='none', edgecolors='#ff5a36', linewidths=1.4,
                   zorder=4, label=f'tree ({len(trees)})')
    if show_path and trajectory is not None and trajectory.shape[0] > 1:
        ax.plot(trajectory[:, 0], trajectory[:, 1], color='#4fc3f7', linewidth=1.0, alpha=0.9, zorder=3,
                label='flight path')
    draw_pose(ax, pose, max(4.0 * res, 1.0))
    extra_handles, _ = ax.get_legend_handles_labels()
    ax.legend(handles=handles + extra_handles, loc='upper right', fontsize=7, framealpha=0.85)
