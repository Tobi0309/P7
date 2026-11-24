import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


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

    # Every node below (also in included launch files) follows /clock when
    # use_sim_time is true, otherwise wall time.
    use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use /clock (simulation or rosbag --clock) instead of wall time.',
    )
    set_use_sim_time = SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time'))

    sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
    mapping = include('P7_launch', 'mapping.launch.py', launch_arguments=sim_time)
    ego_planner = include('ego_planner', 'advanced_param_live.launch.py', launch_arguments=sim_time)
    dsp = include('dsp', 'dsp_launch.py')  # no use_sim_time argument; covered by SetParameter
    rviz = include('P7_launch', 'rviz.launch.py', condition=IfCondition(launch_rviz), launch_arguments=sim_time)
    px4_controller = include('P7_px4_fastlio_bridge', 'px4_fastlio_bridge.launch.py', launch_arguments=sim_time)



    pos_cmd_to_px4 = Node(
        package='P7_control_io_handler',
        executable='pos_cmd_to_px4',
        name='pos_cmd_to_px4',
        output='screen',
        parameters=[{
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
            'goal_height': 2.0,
        }],
    )
    
    return LaunchDescription([
        declare_launch_rviz,
        use_sim_time_argument,
        set_use_sim_time,
        mapping,
        ego_planner,
        dsp,
        goal_converter_dsp,
        pos_cmd_to_px4,
        rviz,
        px4_controller,
    ])
