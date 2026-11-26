"""Loads config/flight_height.yaml, the single source of the stack's flight heights (metres)."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory


def load_flight_height(path=None):
    """Return the validated flight-height settings plus values derived for each node."""
    if path is None:
        path = os.path.join(get_package_share_directory('P7_launch'), 'config', 'flight_height.yaml')
    with open(path) as stream:
        settings = yaml.safe_load(stream)['flight_height']

    average = float(settings['average_height'])
    minimum = float(settings['min_height'])
    maximum = float(settings['max_height'])
    resolution = float(settings['map_resolution'])
    ego_map_height = float(settings['ego_map_height'])
    if not minimum < average < maximum:
        raise ValueError(f'{path}: need min_height < average_height < max_height '
                         f'(got {minimum}, {average}, {maximum}).')
    if resolution <= 0.0:
        raise ValueError(f'{path}: map_resolution must be > 0.')
    if ego_map_height < 1.0:
        raise ValueError(f'{path}: ego_map_height must be >= 1 m (got {ego_map_height}).')

    return {
        'average_height': average,
        'min_height': minimum,
        'max_height': maximum,
        'map_resolution': resolution,
        # DSP takes its preferred height above the floor in voxels.
        'dsp_optimal_voxel_height': max(1, round(average / resolution)),
        'ego_map_size_z': ego_map_height,
    }
