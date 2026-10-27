from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    realsense_launch = os.path.join(
        get_package_share_directory('realsense2_camera'),
        'launch',
        'rs_launch.py'
    )

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch),
        launch_arguments={

            # Camera identification
            'camera_namespace': 'UAV',
            'camera_name': 'rgbd_camera',

            # RGB
            'enable_color': 'true',
            'rgb_camera.color_profile': '1280x720x30',
            'rgb_camera.color_format': 'RGB8',
            'rgb_camera.enable_auto_exposure': 'true',

            # Depth
            'enable_depth': 'true',
            'depth_module.depth_profile': '1280x720x30',
            'depth_module.depth_format': 'Z16',

            # Infrared stereo cameras
            'enable_infra': 'false',
            'enable_infra1': 'false',
            'enable_infra2': 'false',
            # 'depth_module.infra_profile': '1280x720x30',
            # 'depth_module.infra1_format': 'Y8',
            # 'depth_module.infra2_format': 'Y8',

            # IMU
            'enable_gyro': 'false',
            'enable_accel': 'false',

            # RGB-D synchronization
            'enable_sync': 'true',
            'align_depth.enable': 'true',

            # Processing
            'decimation_filter.enable': 'false',
            'spatial_filter.enable': 'false',
            'temporal_filter.enable': 'false',
            'disparity_filter.enable': 'false',
            'hole_filling_filter.enable': 'false',

            # Point cloud
            'pointcloud.enable': 'false',

            # Colorizer
            'colorizer.enable': 'false',

            # TF
            'publish_tf': 'true',
            # 'tf_publish_rate': '0.0',

            # QoS
            # 'color_qos': 'SENSOR_DATA',
            # 'depth_qos': 'SENSOR_DATA',
            # 'infra1_qos': 'SENSOR_DATA',
            # 'infra2_qos': 'SENSOR_DATA',
        }.items(),
    )

    return LaunchDescription([
        camera
    ])