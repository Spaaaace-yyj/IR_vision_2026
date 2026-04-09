#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params_file = os.path.join(
        get_package_share_directory("camera_mount_calibration"),
        "config",
        "camera_mount_calibration_params.yaml",
    )

    params_file = LaunchConfiguration("params_file")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params_file,
            description="Path to the ROS2 parameters file for camera_mount_calibration",
        ),
        Node(
            package="camera_mount_calibration",
            executable="camera_mount_calibration_node",
            name="camera_mount_calibration_node",
            output="screen",
            parameters=[params_file],
        ),
    ])
