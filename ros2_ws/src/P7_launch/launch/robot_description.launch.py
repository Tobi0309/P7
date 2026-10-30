import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_path = get_package_share_directory(
        "P7_launch"
    )

    urdf_path = os.path.join(
        pkg_path,
        "urdf",
        "holybro_x500v2_drone.urdf"
    )

    with open(urdf_path, "r") as f:
        robot_description = f.read()

    world_to_camera_init = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0", "0", "0",
            "0", "0", "2.74889357",
            "world",
            "camera_init"
        ]
    )

    camera_init_to_body = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0", "0", "0",
            "0", "0", "0",
            "camera_init",
            "body"
        ]
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description}
        ]
    )

    return LaunchDescription([
        world_to_camera_init,
        camera_init_to_body,
        robot_state_publisher
    ])