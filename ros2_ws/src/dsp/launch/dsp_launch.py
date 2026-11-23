import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('dsp'), 'config', 'dsp.yaml')

    params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='DSP parameter file (e.g. P7_launch/config/dsp.yaml).',
    )

    dsp = Node(
        package='dsp',
        executable='dsp',
        name='dsp',
        parameters=[LaunchConfiguration('params_file')],
    )

    return LaunchDescription([params_file, dsp])
