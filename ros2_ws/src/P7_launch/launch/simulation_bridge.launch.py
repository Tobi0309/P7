from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    joy = Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
        )

    controller_mapper = Node(
            package='P7_sim_bridge',
            executable='joy_to_drone_velocities',
            name='joy_to_drone_velocities',
            output='screen',
        )

    lidar_mapper = Node(
            package='P7_sim_bridge',
            executable='isaac_lidar_to_livox_lidar',
            name='isaac_lidar_to_livox_lidar',
            output='screen',
        )
    
    return LaunchDescription([
        joy,
        controller_mapper,
        lidar_mapper,
    ])