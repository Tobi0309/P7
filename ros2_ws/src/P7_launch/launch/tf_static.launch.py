from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_static_transform',
        arguments=[
            '--x', '0.133667',
            '--y', '0.000242',
            '--z', '-0.025482',
            '--qx', '-0.5',
            '--qy', '0.5',
            '--qz', '-0.5',
            '--qw', '0.5',
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_link',
        ],
        output='screen',
    )

    lidar_receiver_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_receiver_static_transform',
        arguments=[
            '--x', '0.0708171',
            '--y', '0.0002420',
            '--z', '-0.0988043',
            '--qx', '0.270598',
            '--qy', '0.653281',
            '--qz', '0.270598',
            '--qw', '0.653281',
            '--frame-id', 'base_link',
            '--child-frame-id', 'lidar_link',
        ],
        output='screen',
    )

    lidar_receiver_horizontal_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_receiver_horizontal_static_transform',
        arguments=[
            '--x', '0.052831',
            '--y', '0.0002420',
            '--z', '-0.102382',
            '--qx', '0.707106',
            '--qy', '0.707106',
            '--qz', '0.0',
            '--qw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'lidar_link',
        ],
        output='screen',
    )

    lidar_imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_imu_static_transform',
        arguments=[
            '--x', '0.0754503',
            '--y', '0.0112420',
            '--z', '-0.0491301',
            '--qx', '0.270598',
            '--qy', '0.653281',
            '--qz', '0.270598',
            '--qw', '0.653281',
            '--frame-id', 'base_link',
            '--child-frame-id', 'lidar_imu_link',
        ],
        output='screen',
    )

    return LaunchDescription([
        camera_tf,
        # lidar_receiver_tf,
        lidar_receiver_horizontal_tf,
        # lidar_imu_tf,
    ])