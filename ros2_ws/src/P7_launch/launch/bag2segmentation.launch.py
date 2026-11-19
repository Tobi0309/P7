from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    p7_share = Path(get_package_share_directory('P7_launch'))
    default_bag = p7_share.parents[3] / 'rosbag_19700101_001429'

    bag_argument = DeclareLaunchArgument(
        'bag',
        default_value=str(default_bag),
        description=(
            'Rosbag directory to play. Use '
            'rosbag_19700101_001429 or rosbag_19700101_002438, '
            'or provide another rosbag path.'
        ),
    )
    rviz_config_argument = DeclareLaunchArgument(
        'rviz_config',
        default_value=str(p7_share / 'rviz' / 'drone.rviz'),
        description='Path to the RViz configuration file.',
    )
    use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulated time.',
    )
    use_dsp_argument = DeclareLaunchArgument(
        'use_dsp',
        default_value='true',
        description='Launch the DSP 3D planner using the published OctoMap.',
    )

    fastlio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory('fast_lio')) / 'launch' / 'mapping.launch.py')
        ),
        launch_arguments={
            'rviz': 'false',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    with open(p7_share / 'urdf' / 'holybro_x500v2_drone.urdf') as urdf_file:
        robot_description = urdf_file.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )

    world_to_camera_init = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '2.74889357', 'world', 'camera_init'],
    )
    camera_init_to_world_old = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '3.1415', 'camera_init', 'world_old'],
    )
    camera_init_to_body = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'camera_init', 'body'],
    )

    octomap_parameters = {
        'frame_id': 'camera_init',
        'resolution': 0.5,
        'base_frame_id': 'body',
        'filter_speckles': True,
        'sensor_model.max_range': 10.0,
    }
    octomap = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='log',
        parameters=[octomap_parameters],
        remappings=[('cloud_in', '/cloud_registered_body')],
    )

    dsp = Node(
        package='dsp',
        executable='dsp',
        name='dsp',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_dsp')),
        parameters=[{
            'base_link_frame_id': 'body',
            'odom_frame_id': 'world',
            'use_3d': True,
            'map_topic': '/octomap_binary',
        }],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        additional_env={
            'LD_PRELOAD': '/usr/lib/x86_64-linux-gnu/liboctomap.so'
        },
        output='screen',
    )

    frontier_segmentation = Node(
        package='frontier_segmentation',
        executable='frontier_segmentation_node',
        name='frontier_segmentation',
        output='screen',
        parameters=[
            str(Path(get_package_share_directory('frontier_segmentation')) / 'config' / 'frontier_segmentation.yaml')
        ],
    )

    rosbag = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag')],
        output='screen',
    )

    return LaunchDescription(
        [
            bag_argument,
            rviz_config_argument,
            use_sim_time_argument,
            fastlio,
            world_to_camera_init,
            camera_init_to_world_old,
            camera_init_to_body,
            robot_state_publisher,
            octomap,
            # use_dsp_argument,
            # dsp,
            rviz,
            frontier_segmentation,
            TimerAction(period=2.0, actions=[rosbag]),
        ]
    )
