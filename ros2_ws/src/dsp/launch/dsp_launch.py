from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import Parameter
import os
import getpass


def generate_launch_description():
    name_space = getpass.getuser()
    return LaunchDescription([
        Node(
            package='dsp',
            # namespace=name_space,
            executable='dsp',
            # name=name_space + '_dsp',
            name='dsp',
            parameters=[
            #   Parameter("base_link_frame_id", name_space + "/body"),
                Parameter("base_link_frame_id", "body"),
                Parameter("odom_frame_id", "camera_init"),
                Parameter("use_3d", True),
            #   Parameter("map_topic", name_space + "/octomap_full"),
                Parameter("map_topic", "/octomap_full"),
                # Voxels closer than this [m] to an obstacle surface are treated as occupied.
                Parameter("obstacle_padding", 0.5),
                # Replanning rate [Hz] and rate [Hz] of the look-ahead goal sent to ego-planner.
                Parameter("plan_rate", 0.2),
                Parameter("goal_publish_rate", 1.0),
                Parameter("goal_voxels_ahead", 12),
            ]
        )
    ])
