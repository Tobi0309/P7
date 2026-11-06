import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    publish_robot_description = DeclareLaunchArgument(
        'publish_robot_description',
        default_value='true',
        description='Launch the robot description and static transforms.',
    )


    pkg_path = get_package_share_directory(
        "P7_launch"
    )

    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('P7_launch'),
                'launch',
                'robot_description.launch.py'
            ),
        ),
        condition=IfCondition(
            LaunchConfiguration('publish_robot_description')
        ),
    )

    rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value='drone.rviz',
        description='RViz configuration file in the P7_launch rviz directory.',
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            # '--fullscreen',
            "-d",
            PathJoinSubstitution([
                pkg_path,
                "rviz",
                LaunchConfiguration('rviz_config'),
            ]),
        ],
        additional_env={
        'LD_PRELOAD': '/usr/lib/x86_64-linux-gnu/liboctomap.so'},
        output="screen"
    )

    return LaunchDescription([
        publish_robot_description,
        rviz_config,
        robot_description,
        rviz
    ])