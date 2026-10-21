from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    launch_dir = os.path.dirname(os.path.realpath(__file__))

    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'realsense.launch.py')
        )
    )

    # realsense_launch = os.path.join(
    #     get_package_share_directory('realsense2_camera'),
    #     'launch',
    #     'rs_launch.py'
    # )

    livox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                get_package_share_directory('livox_ros_driver2'),
                '/launch_ROS2/msg_MID360s_launch.py'
            ]
        )
    )

    tf_static = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'tf_static.launch.py')
        )
    )

    # -------------------------------------------------------------
    # Color: raw -> compressed/JPEG
    #
    # Input:
    #   /UAV/rgbd_camera/color/image_raw
    #
    # Output:
    #   /UAV/rgbd_camera/color/image_raw/compressed
    # -------------------------------------------------------------

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
            ('in', 'image_raw'),
            ('out', 'image_raw'),
        ],
    )

    # -------------------------------------------------------------
    # Aligned depth: raw -> compressedDepth
    #
    # Input:
    #   /UAV/rgbd_camera/aligned_depth_to_color/image_raw
    #
    # Output:
    #   /UAV/rgbd_camera/aligned_depth_to_color/image_raw/compressedDepth
    # -------------------------------------------------------------

    depth_compressor = Node(
        package='image_transport',
        executable='republish',
        name='depth_compressor',
        namespace='UAV/rgbd_camera/aligned_depth_to_color',
        output='screen',

        parameters=[{
            'in_transport': 'raw',
            'out_transport': 'compressedDepth',
        }],

        remappings=[
            ('in', 'image_raw'),
            ('out', 'image_raw'),
        ],
    )

    return LaunchDescription([
        realsense,
        color_compressor,
        depth_compressor,
        livox,
        tf_static,
    ])