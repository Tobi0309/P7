import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Mission timer follows /clock (simulation) instead of wall time. '
                        'Can also be toggled in the GUI before the mission starts.',
        ),
        DeclareLaunchArgument(
            'topdown_map',
            default_value='true',
            description='Run P7_topdown_map (2D explored map from /octomap_binary_filtered) for the GUI.',
        ),
        Node(
            package='P7_gui',
            executable='p7_gui',
            name='p7_gui',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='P7_topdown_map',
            executable='topdown_map_node',
            name='topdown_map',
            output='screen',
            condition=IfCondition(LaunchConfiguration('topdown_map')),
            parameters=[
                os.path.join(get_package_share_directory('P7_topdown_map'), 'config', 'topdown_map.yaml'),
                {'use_sim_time': use_sim_time},
            ],
        ),
    ])
