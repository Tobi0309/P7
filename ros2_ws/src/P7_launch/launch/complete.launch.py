import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter

from P7_launch.flight_height import load_flight_height


def include(package, launch_file, **kwargs):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package), 'launch', launch_file)
        ),
        **kwargs,
    )


def generate_launch_description():
    # Every node below (also in included launch files) follows /clock when
    # use_sim_time is true, otherwise wall time.
    use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use /clock (simulation or rosbag --clock) instead of wall time.',
    )
    set_use_sim_time = SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time'))

    # All flight heights [m] come from config/flight_height.yaml.
    height = load_flight_height()

    sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
    mapping = include('P7_launch', 'mapping.launch.py', launch_arguments=sim_time)
    # ego-planner's local map rolls in x, y and z with the drone (no height limits of its own).
    ego_planner = include('ego_planner', 'advanced_param_live.launch.py', launch_arguments={
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'map_size_z_': str(height['ego_map_size_z']),
    }.items())
    # Scoped SetParameter: only reaches the DSP node (its launch file does not set these).
    # DSP keeps its path inside [floor + min_height, floor + max_height] (floor height map).
    dsp = GroupAction([  # no use_sim_time argument; covered by SetParameter
        SetParameter(name='optimal_voxel_height', value=height['dsp_optimal_voxel_height']),
        SetParameter(name='floor_height_topic', value='/floor_height_map'),
        SetParameter(name='min_height_above_ground', value=height['min_height']),
        SetParameter(name='max_height_above_ground', value=height['max_height']),
        include('dsp', 'dsp_launch.py'),
    ])
    px4_controller = include('P7_px4_fastlio_bridge', 'px4_fastlio_bridge.launch.py', launch_arguments=sim_time)
    # Frontier exploration: segmentation, viewpoint optimizer, viewpoint selection (and its RViz).
    # Selection idles until: ros2 service call /frontier_viewpoint_selection/start std_srvs/srv/Trigger
    frontier_navigation = include('P7_launch', 'frontier_navigation.launch.py', launch_arguments=sim_time)



    # 2D floor height map (/floor_height_map) from /octomap_full, DSP's floor method.
    floor_height_map = Node(
        package='P7_topdown_map',
        executable='floor_height_map_node',
        name='floor_height_map',
        output='screen',
        parameters=[os.path.join(get_package_share_directory('P7_topdown_map'), 'config',
                                 'floor_height_map.yaml')],
    )

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

    # Goal on /goal -> DSP (dsp/set_goal service) at the average flight height
    goal_converter_dsp = Node(
        package='P7_helper',
        executable='goal_converter_dsp',
        name='goal_converter_dsp',
        output='screen',
        parameters=[{
            'goal_height': height['average_height'],
        }],
    )
    
    return LaunchDescription([
        use_sim_time_argument,
        set_use_sim_time,
        mapping,
        floor_height_map,
        ego_planner,
        dsp,
        goal_converter_dsp,
        pos_cmd_to_px4,
        px4_controller,
        frontier_navigation,
    ])
