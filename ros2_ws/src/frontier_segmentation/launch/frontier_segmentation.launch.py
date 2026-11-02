from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    params = os.path.join(get_package_share_directory('frontier_segmentation'), 'config', 'frontier_segmentation.yaml')
    return LaunchDescription([Node(package='frontier_segmentation', executable='frontier_segmentation_node', name='frontier_segmentation', output='screen', parameters=[params])])
