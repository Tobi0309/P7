from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessStart

import os
import rclpy
from std_msgs.msg import String

def generate_launch_description():

    realsense_launch = os.path.join(
        get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py'
    )

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch),
        launch_arguments={

            # Camera identification
            'camera_namespace': 'UAV',
            'camera_name': 'rgbd_camera',

            # RGB
            'enable_color': 'true',
            'rgb_camera.color_profile': '1280x720x30',
            'rgb_camera.color_format': 'RGB8',
            'rgb_camera.enable_auto_exposure': 'true',

            # Depth
            'enable_depth': 'true',
            'depth_module.depth_profile': '1280x720x30',
            'depth_module.depth_format': 'Z16',

            # Infrared stereo cameras
            'enable_infra': 'false',
            'enable_infra1': 'false',
            'enable_infra2': 'false',
            # 'depth_module.infra_profile': '1280x720x30',
            # 'depth_module.infra1_format': 'Y8',
            # 'depth_module.infra2_format': 'Y8',

            # IMU
            'enable_gyro': 'false',
            'enable_accel': 'false',

            # RGB-D synchronization
            'enable_sync': 'true',
            'align_depth.enable': 'true',

            # Processing
            'decimation_filter.enable': 'false',
            'spatial_filter.enable': 'false',
            'temporal_filter.enable': 'false',
            'disparity_filter.enable': 'false',
            'hole_filling_filter.enable': 'false',

            # Point cloud
            'pointcloud.enable': 'false',

            # Colorizer
            'colorizer.enable': 'false',

            # TF
            'publish_tf': 'true',
            # 'tf_publish_rate': '0.0',

            # QoS
            # 'color_qos': 'SENSOR_DATA',
            # 'depth_qos': 'SENSOR_DATA',
            # 'infra1_qos': 'SENSOR_DATA',
            # 'infra2_qos': 'SENSOR_DATA',
        }.items(),
    )

    # 2. Extract the specific camera node from the included launch actions
    # This searches the RealSense launch file for the actual executable action
    camera_node_action = None
    for action in realsense_launch.launch_description_source.get_launch_description().entities:
        if hasattr(action, 'executable') and action.executable == 'realsense2_camera_node':
            camera_node_action = action
            break

    # If the default structure is deeply nested, fallback to targeting it by its process name
    # directly using an event handler that registers it once it boots up.
    def on_camera_start(event, context):
        # Capture the actual Process ID assigned by the OS
        pid = event.pid
        node_name = "/camera/camera" # Default full path for rs_launch.py
        
        # Open a transient local Python node just to shout out the registration payload
        rclpy.init()
        temp_node = rclpy.create_node('realsense_top_registrar')
        
        # Match the transient local QoS that ros2top requires
        qos = rclpy.QoSProfile(
            depth=10, 
            durability=rclpy.DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        pub = temp_node.create_publisher(String, '/_ros2top/register', qos_profile=qos)
        
        # Build payload mapping the node name to its actual OS PID
        payload = json.dumps({"node": node_name, "pid": pid})
        msg = String(data=payload)
        
        # Broadcast registration
        pub.publish(msg)
        
        # Clean up the registrar node safely
        temp_node.destroy_node()
        rclpy.shutdown()
        print(f"[ros2top Integration] Registered {node_name} with PID {pid} over Zenoh!")

    # 3. Bind the event handler to fire the moment the camera process spins up
    register_event = RegisterEventHandler(
        OnProcessStart(
            target_action=camera_node_action,
            on_start=on_camera_start
        )
    )

    return LaunchDescription([
        camera,
        register_event
    ])