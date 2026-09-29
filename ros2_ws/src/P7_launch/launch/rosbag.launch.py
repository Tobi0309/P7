from datetime import datetime

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    name = datetime.now().strftime('start_dummy_bag_%Y%m%d_%H%M%S')
    
    output_dir = f'rosbags/{name}'

    return LaunchDescription([
        Node(
            package='rosbag2_transport',
            executable='recorder',
            name='rosbag2_recorder',
            output='screen',
            arguments=['--start-paused', 
                       '/livox/imu',
                       '/livox/lidar',
                       '/UAV/rgbd_camera/color/camera_info',
                       '/UAV/rgbd_camera/color/image_raw',
                       '/UAV/rgbd_camera/color/metadata',
                       '/UAV/rgbd_camera/depth/camera_info',
                       '/UAV/rgbd_camera/depth/image_rect_raw',
                       '/UAV/rgbd_camera/depth/metadata',
                       '/UAV/rgbd_camera/extrinsics/depth_to_color',
                       '/fmu/out/vehicle_attitude',
                       '/fmu/out/vehicle_gps_position',
                       '/fmu/out/vehicle_global_position',
                       '/fmu/out/vehicle_local_position',
                       '/fmu/out/rc_channels',
                        ],
 
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

