from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='imu_odometry',
                executable='imu_odometry_node',
                name='imu_odometry',
                output='screen',
                parameters=[
                    {
                        'imu_topic': '/imu/data_raw',
                        'odom_topic': '/imu/odometry',
                        'odom_frame': 'odom',
                        'base_frame': 'base_footprint',
                        'publish_tf': True,
                        'use_imu_orientation': True,
                        'stationary_accel_threshold': 0.35,
                        'stationary_gyro_threshold': 0.12,
                        'stationary_time_threshold': 0.25,
                        'stationary_velocity_threshold': 0.08,
                        'stationary_jerk_threshold': 1.2,
                        'stationary_filter_alpha': 0.2,
                        'stationary_velocity_decay': 12.0,
                        'bias_correction_time_constant': 8.0,
                        'accel_low_pass_alpha': 0.35,
                        'velocity_damping': 0.0,
                    }
                ],
            )
        ]
    )
