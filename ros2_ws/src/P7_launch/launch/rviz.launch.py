import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    pkg_path = get_package_share_directory(
        "P7_launch"
    )

    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('P7_launch'),
                'launch',
                'robot_description.launch.py'
            )
        )
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            os.path.join(pkg_path, "rviz", "drone.rviz"),
        ],
        output="screen"
    )

    return LaunchDescription([
        robot_description,
        rviz
    ])