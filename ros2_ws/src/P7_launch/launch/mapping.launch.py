from datetime import datetime

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    fastlio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('fast_lio'),
                'launch',
                'mapping.launch.py'
            )
        )
    )

    static_transform_world_sensor = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '3.141592653589', '0', 'world', 'camera_init']
        )
    

    octomap_node = Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            output='screen',
            parameters=[{
                'frame_id': f'camera_init',
                'resolution': 0.5,
                'base_frame_id': f'body',
                'filter_speckles': True,
                # 'filter_ground_plane': True,
                # 'ground_filter.angle': 0.3,
                # 'ground_filter.distance': 0.3,
                # 'ground_filter.plane_distance': 1.0,
                'sensor_model.max_range': 10.0,
                # 'point_cloud_max_z': 1.0,
                # 'point_cloud_min_z': -0.5,  
            }],
            remappings=[
                ('cloud_in', '/cloud_registered_body'),
                # ('/projected_map', 'map'),
                # ('/octomap_binary', 'octomap_binary'),
                # ('/octomap_full', 'octomap_full'),
                # ('/octomap_point_cloud_centers', 'octomap_point_cloud_centers'),
                # ('/occupied_cells_vis_array', 'occupied_cells_vis_array'),
                # ('/free_cells_vis_array', 'free_cells_vis_array'),
            ],

        )

    octomap_node_fov_filtered = Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            output='screen',
            parameters=[{
                'frame_id': f'camera_init',
                'resolution': 0.5,
                'base_frame_id': f'body',
                'filter_speckles': True,
                # 'filter_ground_plane': True,
                # 'ground_filter.angle': 0.3,
                # 'ground_filter.distance': 0.3,
                # 'ground_filter.plane_distance': 1.0,
                'sensor_model.max_range': 10.0,
                # 'point_cloud_max_z': 1.0,
                # 'point_cloud_min_z': -0.5,  
            }],
            remappings=[
                ('cloud_in', '/cloud_registered_body_fov_filtered'),
                ('/projected_map', '/projected_map_filtered'),
                ('/octomap_binary', '/octomap_binary_filtered'),
                ('/octomap_full', '/octomap_full_filtered'),
                ('/octomap_point_cloud_centers', '/octomap_point_cloud_centers_filtered'),
                ('/occupied_cells_vis_array', '/occupied_cells_vis_array_filtered'),
                ('/free_cells_vis_array', '/free_cells_vis_array_filtered'),
            ],
        )


    return LaunchDescription([
        fastlio,
        static_transform_world_sensor,
        octomap_node,
        octomap_node_fov_filtered,
    ])

