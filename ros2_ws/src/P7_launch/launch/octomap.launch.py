from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    octomap_server = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'frame_id': 'camera_init',
            'resolution': 0.5,
            'base_frame_id': 'body',
            'filter_speckles': True,
            'sensor_model.max_range': 10.0,
        }],
        remappings=[
            ('cloud_in', '/cloud_registered_body'),
        ],
    )

    return LaunchDescription([octomap_server])
