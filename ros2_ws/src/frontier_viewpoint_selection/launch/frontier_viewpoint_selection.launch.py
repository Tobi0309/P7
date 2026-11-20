from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    default_config = str(Path(get_package_share_directory("frontier_viewpoint_selection")) / "config" / "frontier_viewpoint_selection.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_config),
        Node(package="frontier_viewpoint_selection", executable="frontier_viewpoint_selection_node",
             name="frontier_viewpoint_selection_node", output="screen",
             parameters=[LaunchConfiguration("params_file")]),
    ])
