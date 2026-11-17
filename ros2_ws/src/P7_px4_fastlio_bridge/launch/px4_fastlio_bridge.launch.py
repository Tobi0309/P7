from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_cfg = Path(get_package_share_directory('P7_px4_fastlio_bridge')) / 'config' / 'bridge.yaml'

    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(default_cfg)),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='P7_px4_fastlio_bridge',
            executable='px4_odometry_bridge',
            name='px4_odometry_bridge',
            output='screen',
            parameters=[
                LaunchConfiguration('config'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
        ),
    ])
