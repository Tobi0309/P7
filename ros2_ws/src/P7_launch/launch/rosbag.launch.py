from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    output_dir = 'rosbags/start'

    return LaunchDescription([
        Node(
            package='rosbag2_transport',
            executable='recorder',
            name='rosbag2_recorder',
            output='screen',
            arguments=['--start-paused', 
                       '-a'],
 
            parameters=[{
                'storage.uri': output_dir,
                'storage.storage_id': 'mcap'  # Standard i Jazzy
            }]
        ),
        Node(
            package='P7_control_io_handler',
            executable='rosbag_recorder',
            name='control_io_rosbag_recorder',
            output='screen',
        ),
    ])

