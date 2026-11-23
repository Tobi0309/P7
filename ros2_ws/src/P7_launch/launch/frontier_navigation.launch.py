from pathlib import Path
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
)
from launch.logging import launch_config
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    p7_share = Path(get_package_share_directory('P7_launch'))
    custom_log_dir = str(p7_share / 'logs')
    os.makedirs(custom_log_dir, exist_ok=True)
    launch_config.log_dir = custom_log_dir

    rviz_config_argument = DeclareLaunchArgument(
        'rviz_config',
        default_value=str(p7_share / 'rviz' / 'drone.rviz'),
        description='Path to the RViz configuration file.',
    )
    use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulated time.',
    )

    with open(p7_share / 'urdf' / 'holybro_x500v2_drone.urdf') as urdf_file:
        robot_description = urdf_file.read()

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        additional_env={
            'LD_PRELOAD': '/usr/lib/x86_64-linux-gnu/liboctomap.so'
        },
        output='screen',
    )

    # The config files are keyed /** so they apply to these node names.
    use_sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time')}

    frontier_segmentation = Node(
        package='frontier_segmentation',
        executable='frontier_segmentation_node',
        name='frontier_segmentation',
        output={'both': 'log'},
        parameters=[
            str(p7_share / 'config' / 'frontier_segmentation.yaml'),
            use_sim_time,
        ],
    )

    frontier_viewpoint_optimizer = Node(
        package='frontier_viewpoint_optimizer',
        executable='frontier_viewpoint_optimizer_node',
        name='frontier_viewpoint_optimizer',
        output={'both': 'log'},
        parameters=[
            str(p7_share / 'config' / 'frontier_viewpoint_optimizer.yaml'),
            use_sim_time,
        ],
    )

    frontier_viewpoint_selection = Node(
        package='frontier_viewpoint_selection',
        executable='frontier_viewpoint_selection_node',
        name='frontier_viewpoint_selection',
        output='both',
        parameters=[
            str(p7_share / 'config' / 'frontier_viewpoint_selection.yaml'),
            use_sim_time,
        ],
    )

    return LaunchDescription(
        [
            use_sim_time_argument,
            rviz_config_argument,
            rviz,
            SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '0'),
            frontier_segmentation,
            frontier_viewpoint_optimizer,
            frontier_viewpoint_selection,
        ]
    )
