from pathlib import Path
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_share = Path(get_package_share_directory('P7_launch'))
    default_bag_path = package_share.parents[3] / 'rosbags' / 'rosbag_1'

    bag_path = LaunchConfiguration('bag')
    declare_bag_path = DeclareLaunchArgument(
        'bag',
        default_value=str(default_bag_path),
        description='Path to the ROS 2 bag to play',
    )
    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=str(package_share / 'rviz' / 'fastlio.rviz'),
        description='Path to the RViz configuration file',
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('P7_launch'),
                'launch',
                'rviz.launch.py',
            )
        ),
        launch_arguments={
            'rviz_config': LaunchConfiguration('rviz_config'),
            'use_sim_time': 'true',
        }.items(),
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
            'use_sim_time': 'true',
        }.items()
    )

    imu_propagation = Node(
        package='fast_lio',
        executable='imu_propagation',
        name='imu_propagation',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    odometry_transformer = Node(
    	package='fast_lio',
    	executable='odometry_transformer',
    	name='odometry_transformer',
    	output='screen',
    	parameters=[{'use_sim_time': True}],
    )

    # Goal on /goal (e.g. RViz) -> ego-planner (/move_base_simple/goal) at the current altitude
    goal_converter_ego = Node(
        package='P7_helper',
        executable='goal_converter_ego',
        name='goal_converter_ego',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    pos_cmd_to_px4 = Node(
        package='P7_control_io_handler',
        executable='pos_cmd_to_px4',
        name='pos_cmd_to_px4',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'position_command_topic': '/drone_0_planning/pos_cmd',
            'goal_topic': '/move_base_simple/goal',
            'px4_odometry_topic': '/fmu/out/vehicle_odometry',
        }],
    )

    rosbag_player = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag_path, '--clock'],
        output='screen',
    )
    delayed_rosbag_player = TimerAction(
        period=1.0,
        actions=[rosbag_player],
    )

    return LaunchDescription([
        declare_bag_path,
        declare_rviz_config,
        rviz,
        fastlio,
        imu_propagation,
	    odometry_transformer,
	    goal_converter_ego,
	    pos_cmd_to_px4,
        delayed_rosbag_player,
    ])

