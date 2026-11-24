from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter

from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    publish_robot_description = DeclareLaunchArgument(
        'publish_robot_description',
        default_value='true',
        description='Launch the robot description and static transforms.',
    )

    # Every node below (also in included launch files) follows /clock when
    # use_sim_time is true, otherwise wall time.
    use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use /clock (simulation or rosbag --clock) instead of wall time.',
    )
    set_use_sim_time = SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time'))

    robot_description = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('P7_launch'),
                    'launch',
                    'robot_description.launch.py'
                ),
            ),
            condition=IfCondition(
                LaunchConfiguration('publish_robot_description')
            ),
        )

    fastlio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('fast_lio'),
                'launch',
                'mapping.launch.py'
            )
        ),
        launch_arguments={
            'rviz': 'false',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            # The simulated drone drops and bounces at start-up.
            'wait_for_still': LaunchConfiguration('use_sim_time'),
        }.items()
    )

    octomap_node = Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            output='log', #'screen',
            parameters=[{
                'frame_id': f'camera_init',
                'resolution': 0.5,
                'base_frame_id': f'body',
                'filter_speckles': True,
                # 'filter_ground_plane': True,
                # 'ground_filter.angle': 0.3,
                # 'ground_filter.distance': 0.3,
                # 'ground_filter.plane_distance': 1.0,
                'sensor_model.max_range': 10.0,
                # 'point_cloud_max_z': 1.0,
                # 'point_cloud_min_z': -0.5,  
            }],
            remappings=[
                ('cloud_in', '/cloud_registered_body'),
                # ('/projected_map', 'map'),
                # ('/octomap_binary', 'octomap_binary'),
                # ('/octomap_full', 'octomap_full'),
                # ('/octomap_point_cloud_centers', 'octomap_point_cloud_centers'),
                # ('/occupied_cells_vis_array', 'occupied_cells_vis_array'),
                # ('/free_cells_vis_array', 'free_cells_vis_array'),
            ],

        )

    # Keeps only LiDAR points inside the camera image; feeds octomap_node_fov_filtered.
    pointcloud_fov_filter = Node(
        package='P7_mapping',
        executable='pointcloud_fov_filter',
        name='pointcloud_fov_filter',
        output='screen',
    )

    octomap_node_fov_filtered = Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            output='log', #'screen',
            parameters=[{
                'frame_id': f'camera_init',
                'resolution': 0.5,
                'base_frame_id': f'body',
                'filter_speckles': True,
                # 'filter_ground_plane': True,
                # 'ground_filter.angle': 0.3,
                # 'ground_filter.distance': 0.3,
                # 'ground_filter.plane_distance': 1.0,
                'sensor_model.max_range': 10.0,
                # 'point_cloud_max_z': 1.0,
                # 'point_cloud_min_z': -0.5,  
            }],
            remappings=[
                ('cloud_in', '/cloud_registered_body_fov_filtered'),
                ('/projected_map', '/projected_map_filtered'),
                ('/octomap_binary', '/octomap_binary_filtered'),
                ('/octomap_full', '/octomap_full_filtered'),
                ('/octomap_point_cloud_centers', '/octomap_point_cloud_centers_filtered'),
                ('/occupied_cells_vis_array', '/occupied_cells_vis_array_filtered'),
                ('/free_cells_vis_array', '/free_cells_vis_array_filtered'),
            ],
        )

    imu_propagation = Node(
        package='fast_lio',
        executable='imu_propagation',
        name='imu_propagation',
        output='screen',
    )

    odometry_transformer = Node(
    	package='fast_lio',
    	executable='odometry_transformer',
    	name='odometry_transformer',
    	output='screen',
    )

    return LaunchDescription([
        publish_robot_description,
        use_sim_time_argument,
        set_use_sim_time,
        fastlio,
        robot_description,
        octomap_node,
        pointcloud_fov_filter,
        octomap_node_fov_filtered,
        imu_propagation,
        odometry_transformer,
    ])
