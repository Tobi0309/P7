import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def include(package, launch_file, **kwargs):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package), 'launch', launch_file)
        ),
        **kwargs,
    )


def generate_launch_description():
    launch_rviz = LaunchConfiguration('launch_rviz')

    declare_launch_rviz = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Launch RViz (P7_launch rviz.launch.py)',
    )

    mapping = include('P7_launch', 'mapping.launch.py')
    ego_planner = include('ego_planner', 'advanced_param_live.launch.py')
    dsp = include('dsp', 'dsp_launch.py')
    rviz = include('P7_launch', 'rviz.launch.py', condition=IfCondition(launch_rviz))

    drone_velocity_mpc = Node(
        package='P7_control_io_handler',
        executable='drone_velocity_mpc',
        output='screen',
    )

    return LaunchDescription([
        declare_launch_rviz,
        mapping,
        ego_planner,
        dsp,
        drone_velocity_mpc,
        rviz,
    ])
