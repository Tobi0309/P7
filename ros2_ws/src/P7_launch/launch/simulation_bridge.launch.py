from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter

from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Every node below (also in included launch files) follows /clock when
    # use_sim_time is true, otherwise wall time.
    use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use /clock (simulation or rosbag --clock) instead of wall time.',
    )
    set_use_sim_time = SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time'))

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
    
    color_compressor = Node(
        package='image_transport',
        executable='republish',
        name='color_compressor',
        namespace='UAV/rgbd_camera/color',
        output='screen',
        parameters=[{
            'in_transport': 'raw',
            'out_transport': 'compressed',
        }],
        remappings=[
            ('in', '/UAV/rgbd_camera/color/image_raw'),
            ('out/compressed', '/UAV/rgbd_camera/color/image_raw/compressed'),
        ],
    )



    return LaunchDescription([
        use_sim_time_argument,
        set_use_sim_time,
        #joy,
        #controller_mapper,
        color_compressor,
        lidar_mapper,
    ])