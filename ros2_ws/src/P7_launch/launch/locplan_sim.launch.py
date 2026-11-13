from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_share = Path(get_package_share_directory('P7_launch'))

    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=str(package_share / 'rviz' / 'fastlio.rviz'),
        description='Path to the RViz configuration file',
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('P7_launch'),
                'launch',
                'rviz.launch.py',
            )
        ),
        launch_arguments={
            'rviz_config': LaunchConfiguration('rviz_config'),
            'use_sim_time': 'true',
        }.items(),
    )

    fastlio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('fast_lio'),
                'launch',
                'mapping.launch.py'
            )
        ),
        launch_arguments={
            'rviz': 'false',
            'use_sim_time': 'true',
        }.items()
    )

    imu_propagation = Node(
        package='fast_lio',
        executable='imu_propagation',
        name='imu_propagation',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    odometry_transformer = Node(
        package='fast_lio',
        executable='odometry_transformer',
        name='odometry_transformer',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    goal_converter = Node(
        package='fast_lio',
        executable='goal_converter',
        name='goal_converter',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        declare_rviz_config,
        rviz,
        fastlio,
        imu_propagation,
        odometry_transformer,
        goal_converter,
    ])
