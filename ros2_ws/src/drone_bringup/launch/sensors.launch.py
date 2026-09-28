from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os


def generate_launch_description():

    launch_dir = os.path.dirname(os.path.realpath(__file__))

    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'realsense.launch.py')
        )
    )

    livox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                get_package_share_directory('livox_ros_driver2'),
                '/launch_ROS2/msg_MID360_launch.py'
            ]
        )
    )

    # IMU = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(launch_dir, 'imu.launch.py')
    #     )
    # )

    return LaunchDescription([
        realsense,
        livox,
        # IMU,
    ])