#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params_file = os.path.join(
        get_package_share_directory("car_control"),
        "config",
        "car_control_params.yaml",
    )

    params_file = LaunchConfiguration("params_file")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params_file,
            description="Path to the ROS2 parameters file for car_control",
        ),
        Node(
            package="car_control",
            executable="car_control",
            name="car_control_node",
            output="screen",
            parameters=[params_file],
        ),
    ])
