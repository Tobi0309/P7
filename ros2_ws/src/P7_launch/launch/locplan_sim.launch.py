from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
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
            'use_sim_time': 'false',
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
            'use_sim_time': 'false',
        }.items()
    )

    return LaunchDescription([
        declare_rviz_config,
        rviz,
        fastlio,
    ])
