from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory("frontier_viewpoint_optimizer")) / "config" / "frontier_viewpoint_optimizer.yaml"
    return LaunchDescription([
        Node(
            package="frontier_viewpoint_optimizer",
            executable="frontier_viewpoint_optimizer_node",
            name="frontier_viewpoint_optimizer_node",
            output="screen",
            parameters=[str(config)],
        )
    ])
