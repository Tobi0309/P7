import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
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
    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_rviz_config = DeclareLaunchArgument(
        "rviz_config",
        default_value=os.path.join(pkg_path, "rviz", "fastlio.rviz"),
        description="Path to the RViz configuration file",
    )
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulated time",
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
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        additional_env={
            "LD_PRELOAD": "/usr/lib/x86_64-linux-gnu/liboctomap.so"
        },
        output="screen",
    )

    return LaunchDescription([
        publish_robot_description,
        rviz_config,
        declare_rviz_config,
        declare_use_sim_time,
        robot_description,
        rviz,
    ])
