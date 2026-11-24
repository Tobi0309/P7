from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    # Every node below (also in included launch files) follows /clock when
    # use_sim_time is true, otherwise wall time.
    use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use /clock (simulation or rosbag --clock) instead of wall time.',
    )
    set_use_sim_time = SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time'))

    octomap_server = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[{
            'frame_id': 'camera_init',
            'resolution': 0.5,
            'base_frame_id': 'body',
            'filter_speckles': True,
            'sensor_model.max_range': 10.0,
        }],
        remappings=[
            ('cloud_in', '/cloud_registered_body'),
        ],
    )

    return LaunchDescription([use_sim_time_argument, set_use_sim_time, octomap_server])
