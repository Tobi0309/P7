import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess,IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    p7_launch_share = get_package_share_directory('P7_launch')
    default_bag = os.path.normpath(
        os.path.join(
            p7_launch_share, '..', '..', '..', '..', 'rosbag_19700101_001429',
        )
    )

    bag = DeclareLaunchArgument(
        'bag',
        default_value=default_bag,
        description=(
            'Rosbag directory to play. Use '
            'rosbag_19700101_001429 or rosbag_19700101_002438, '
            'or provide another rosbag path.'
        ),
    )

    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(p7_launch_share, 'launch', 'mapping.launch.py')
        ),
        launch_arguments={'publish_robot_description': 'true'}.items(),
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(p7_launch_share, 'launch', 'rviz.launch.py')
        ),
        launch_arguments={
            'publish_robot_description': 'false',
            'rviz_config': 'drone.rviz',
        }.items(),
    )

    frontier_segmentation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('frontier_segmentation'),
                'launch',
                'frontier_segmentation.launch.py',
            )
        )
    )

    rosbag = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag')],
        output='screen',
    )

    return LaunchDescription([
        bag,
        mapping,
        rviz,
        frontier_segmentation,
        TimerAction(period=2.0, actions=[rosbag]),
    ])
