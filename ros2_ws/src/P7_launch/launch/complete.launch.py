import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def include(package, launch_file, **kwargs):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package), 'launch', launch_file)
        ),
        **kwargs,
    )


def generate_launch_description():
    launch_rviz = LaunchConfiguration('launch_rviz')

    declare_launch_rviz = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Launch RViz (P7_launch rviz.launch.py)',
    )

    p7_config = os.path.join(get_package_share_directory('P7_launch'), 'config')

    mapping = include('P7_launch', 'mapping.launch.py')
    ego_planner = include('ego_planner', 'advanced_param_live.launch.py')
    dsp = include(
        'dsp', 'dsp_launch.py',
        launch_arguments={'params_file': os.path.join(p7_config, 'dsp.yaml')}.items(),
    )
    # mapping.launch.py already starts robot_state_publisher and the static TFs.
    rviz = include(
        'P7_launch', 'rviz.launch.py',
        condition=IfCondition(launch_rviz),
        launch_arguments={'publish_robot_description': 'false'}.items(),
    )
    px4_controller = include('P7_px4_fastlio_bridge', 'px4_fastlio_bridge.launch.py', launch_arguments={'use_sim_time': 'true'}.items())



    pos_cmd_to_px4 = Node(
        package='P7_control_io_handler',
        executable='pos_cmd_to_px4',
        name='pos_cmd_to_px4',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'position_command_topic': '/drone_0_planning/pos_cmd',
            'px4_odometry_topic': '/fmu/out/vehicle_odometry',
        }],
    )

    # Goal on /goal -> DSP (dsp/set_goal service) at a fixed height
    goal_converter_dsp = Node(
        package='P7_helper',
        executable='goal_converter_dsp',
        name='goal_converter_dsp',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'goal_height': 2.0,
        }],
    )
    
    return LaunchDescription([
        # declare_launch_rviz,
        # rviz,
        mapping,
        ego_planner,
        dsp,
        goal_converter_dsp,
        pos_cmd_to_px4,
        px4_controller,
    ])
