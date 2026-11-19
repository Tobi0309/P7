from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    p7_share = Path(get_package_share_directory('P7_launch'))

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

    frontier_segmentation = Node(
        package='frontier_segmentation',
        executable='frontier_segmentation_node',
        name='frontier_segmentation',
        output='log',
        parameters=[
            str(p7_share / 'config' / 'frontier_segmentation.yaml')
        ],
    )

    frontier_viewpoint_optimizer = Node(
        package='frontier_viewpoint_optimizer',
        executable='frontier_viewpoint_optimizer_node',
        name='frontier_viewpoint_optimizer',
        output='log',
        parameters=[
            str(p7_share / 'config' / 'frontier_viewpoint_optimizer.yaml')
        ],
    )

    frontier_viewpoint_selection = Node(
        package='frontier_viewpoint_selection',
        executable='frontier_viewpoint_selection_node',
        name='frontier_viewpoint_selection',
        output='both',
        parameters=[
            str(p7_share / 'config' / 'frontier_viewpoint_selection.yaml')
        ],
    )

    return LaunchDescription(
        [
            use_sim_time_argument,
            rviz_config_argument,
            rviz,
            frontier_segmentation,
            frontier_viewpoint_optimizer,
            frontier_viewpoint_selection,
        ]
    )
