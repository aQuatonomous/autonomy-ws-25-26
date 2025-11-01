from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package='pointcloud_filters',
            executable='lidar_range_filter',
            name='lidar_range_filter',
            output='screen',
            parameters=[
                {
                    # Vendor defaults:
                    #   cloud topic: /unilidar/cloud  (frame: unilidar_lidar)
                    #   IMU topic:   /unilidar/imu    (frame: unilidar_imu)
                    # You can override these as needed.
                    'input_topic': '/unilidar/cloud',
                    'output_topic': '/points_filtered',
                    'base_frame': 'base_link',
                    'use_tf_transform': True,
                    'z_min': 0.0,
                    'z_max': 10.0,
                    'range_max': 30.0,
                }
            ],
        ),
    ])
