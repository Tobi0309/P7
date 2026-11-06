from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    share = get_package_share_directory('frontier_viewpoint_optimizer')
    return LaunchDescription([Node(
        package='frontier_viewpoint_optimizer',
        executable='frontier_viewpoint_optimizer_node',
        name='frontier_viewpoint_optimizer_node',
        output='screen',
        parameters=[os.path.join(share, 'config', 'viewpoint_optimizer.yaml')])])
