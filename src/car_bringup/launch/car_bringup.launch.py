#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    car_control_launch = os.path.join(
        get_package_share_directory("car_control"),
        "launch",
        "car_control_launch.py",
    )

    car_control_default_params = os.path.join(
        get_package_share_directory("car_control"),
        "config",
        "car_control_params.yaml",
    )

    rplidar_launch = os.path.join(
        get_package_share_directory("rplidar_ros"),
        "launch",
        "rplidar_a3_launch.py",
    )

    # ===== Launch 参数 =====
    car_control_params_file = LaunchConfiguration("car_control_params_file")

    lidar_channel_type = LaunchConfiguration("lidar_channel_type")
    lidar_serial_port = LaunchConfiguration("lidar_serial_port")
    lidar_serial_baudrate = LaunchConfiguration("lidar_serial_baudrate")
    lidar_frame_id = LaunchConfiguration("lidar_frame_id")
    lidar_inverted = LaunchConfiguration("lidar_inverted")
    lidar_angle_compensate = LaunchConfiguration("lidar_angle_compensate")
    lidar_scan_mode = LaunchConfiguration("lidar_scan_mode")

    enable_camera = LaunchConfiguration("enable_camera")

    # ===== MindVision 相机 launch =====
    mv_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("mindvision_camera"),
                "launch",
                "mv_launch.py",
            ])
        ),
        condition=IfCondition(enable_camera),
    )

    return LaunchDescription([
        # ===== 参数声明 =====
        DeclareLaunchArgument(
            "car_control_params_file",
            default_value=car_control_default_params,
            description="Path to the ROS 2 parameters file for car_control",
        ),

        DeclareLaunchArgument(
            "lidar_channel_type",
            default_value="serial",
            description="RPLIDAR channel type",
        ),
        DeclareLaunchArgument(
            "lidar_serial_port",
            default_value="/dev/ttyUSB0",
            description="RPLIDAR serial device path",
        ),
        DeclareLaunchArgument(
            "lidar_serial_baudrate",
            default_value="256000",
            description="RPLIDAR serial baudrate",
        ),
        DeclareLaunchArgument(
            "lidar_frame_id",
            default_value="laser",
            description="RPLIDAR frame id",
        ),
        DeclareLaunchArgument(
            "lidar_inverted",
            default_value="false",
            description="Whether to invert RPLIDAR scan data",
        ),
        DeclareLaunchArgument(
            "lidar_angle_compensate",
            default_value="true",
            description="Whether to enable RPLIDAR angle compensation",
        ),
        DeclareLaunchArgument(
            "lidar_scan_mode",
            default_value="Sensitivity",
            description="RPLIDAR scan mode",
        ),

        DeclareLaunchArgument(
            "enable_camera",
            default_value="true",
            description="Whether to start the MindVision camera",
        ),

        # ===== 启动 car_control =====
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(car_control_launch),
            launch_arguments={
                "params_file": car_control_params_file,
            }.items(),
        ),

        # ===== 启动雷达 =====
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rplidar_launch),
            launch_arguments={
                "channel_type": lidar_channel_type,
                "serial_port": lidar_serial_port,
                "serial_baudrate": lidar_serial_baudrate,
                "frame_id": lidar_frame_id,
                "inverted": lidar_inverted,
                "angle_compensate": lidar_angle_compensate,
                "scan_mode": lidar_scan_mode,
            }.items(),
        ),

        # ===== 启动 MindVision 相机 =====
        mv_camera_launch,
    ])