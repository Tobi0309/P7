from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_share = Path(get_package_share_directory('P7_launch'))
    default_bag_path = package_share.parents[3] / 'rosbags' / 'rosbag_1'

    bag_path = LaunchConfiguration('bag')
    declare_bag_path = DeclareLaunchArgument(
        'bag',
        default_value=str(default_bag_path),
        description='Path to the ROS 2 bag to play',
    )
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

    rosbag_player = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag_path, '--clock'],
        output='screen',
    )
    delayed_rosbag_player = TimerAction(
        period=1.0,
        actions=[rosbag_player],
    )

    return LaunchDescription([
        declare_bag_path,
        declare_rviz_config,
        rviz,
        fastlio,
        delayed_rosbag_player,
    ])
