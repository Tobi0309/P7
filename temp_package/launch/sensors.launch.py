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

    # lidar = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(launch_dir, 'lidar.launch.py')
    #     )
    # )

    return LaunchDescription([
        realsense,
        # lidar,
    ])