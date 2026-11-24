from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_share = Path(get_package_share_directory('P7_launch'))

    # Every node below (also in included launch files) follows /clock when
    # use_sim_time is true, otherwise wall time.
    use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use /clock (simulation or rosbag --clock) instead of wall time.',
    )
    set_use_sim_time = SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time'))

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
            'use_sim_time': LaunchConfiguration('use_sim_time'),
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
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            # The simulated drone drops and bounces at start-up.
            'wait_for_still': LaunchConfiguration('use_sim_time'),
        }.items()
    )

    octomap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('P7_launch'),
                'launch',
                'octomap.launch.py',
            )
        ),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items(),
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

    goal_converter_dsp = Node(
        package='P7_helper',
        executable='goal_converter_dsp',
        name='goal_converter_dsp',
        output='screen',
    )

    pos_cmd_to_px4 = Node(
        package='P7_control_io_handler',
        executable='pos_cmd_to_px4',
        name='pos_cmd_to_px4',
        output='screen',
        parameters=[{
            'position_command_topic': '/drone_0_planning/pos_cmd',
            'goal_topic': '/move_base_simple/goal',
            'px4_odometry_topic': '/fmu/out/vehicle_odometry',
        }],
    )

    return LaunchDescription([
        use_sim_time_argument,
        set_use_sim_time,
        declare_rviz_config,
        rviz,
        fastlio,
        octomap,
        imu_propagation,
        odometry_transformer,
        goal_converter_dsp,
        pos_cmd_to_px4,
    ])
