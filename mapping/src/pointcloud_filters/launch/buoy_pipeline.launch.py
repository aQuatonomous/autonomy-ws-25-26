"""
All-in-one LiDAR buoy pipeline: Unitree driver → range filter → detector → tracker → visualizer.
Optionally launches RViz. LiDAR is always /dev/ttyUSB0.
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    lidar_port = '/dev/ttyUSB0'
    if not os.path.exists(lidar_port):
        raise RuntimeError(
            f"LiDAR port {lidar_port!r} not found. Is the LiDAR plugged in? Check: ls /dev/ttyUSB*"
        )
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='false',
        description='If true, launch RViz with buoy_pipeline config. Default false (no GUI).',
    )
    # Detector tuning: e.g. ros2 launch ... buoy_detector_log_level:=debug
    buoy_detector_log_level_arg = DeclareLaunchArgument(
        'buoy_detector_log_level', default_value='info', description='buoy_detector log level (e.g. debug)',
    )

    unitree_node = Node(
        package='unitree_lidar_ros2',
        executable='unitree_lidar_ros2_node',
        name='unitree_lidar_ros2_node',
        output='screen',
        parameters=[
            {'port': '/dev/ttyUSB0'},
            {'rotate_yaw_bias': 0.0},
            {'range_scale': 0.001},
            {'range_bias': 0.0},
            {'range_max': 50.0},
            {'range_min': 0.0},
            {'cloud_frame': 'unilidar_lidar'},
            {'cloud_topic': 'unilidar/cloud'},
            {'cloud_scan_num': 18},
            {'imu_frame': 'unilidar_imu'},
            {'imu_topic': 'unilidar/imu'},
        ],
    )

    range_filter_node = Node(
        package='pointcloud_filters',
        executable='lidar_range_filter',
        name='lidar_range_filter',
        output='screen',
        parameters=[
            {
                'input_topic': '/unilidar/cloud',
                'output_topic': '/points_filtered',
                'z_min': -0.37,
                'z_max': 10.0,
                'range_max': 30.0,
            }
        ],
    )

    buoy_detector_node = Node(
        package='pointcloud_filters',
        executable='buoy_detector',
        name='buoy_detector',
        output='screen',
        parameters=[
            {
                'input_topic': '/points_filtered',
                'output_topic': '/buoy_detections',
                'eps': 0.8,
                'min_samples': 2,
                'min_lateral_extent': 0.01,
                'max_lateral_extent': 1.0,
                'min_points_final': 2,
                'confidence_scale': 15.0,
                'ransac_enabled': False,
                'ransac_iterations': 80,
                'ransac_distance_threshold': 0.15,
                'ransac_min_inlier_ratio': 0.3,
            }
        ],
    )

    buoy_tracker_node = Node(
        package='pointcloud_filters',
        executable='buoy_tracker',
        name='buoy_tracker',
        output='screen',
    )

    # LiDAR-only: shows raw detections as markers in RViz (topic /buoy_markers)
    buoy_visualizer_node = Node(
        package='pointcloud_filters',
        executable='buoy_visualizer',
        name='buoy_visualizer',
        output='screen',
    )
    # Fused (CV+LiDAR): shows tracked buoys with class from CV (topic /tracked_buoy_markers); only has data when fusion runs
    tracked_buoy_visualizer_node = Node(
        package='pointcloud_filters',
        executable='tracked_buoy_visualizer',
        name='tracked_buoy_visualizer',
        output='screen',
    )

    # RViz (optional)
    rviz_config_path = os.path.join(
        get_package_share_directory('pointcloud_filters'), 'rviz', 'buoy_pipeline.rviz'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='log',
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
    )

    return LaunchDescription([
        launch_rviz_arg,
        buoy_detector_log_level_arg,
        unitree_node,
        range_filter_node,
        buoy_detector_node,
        buoy_tracker_node,
        buoy_visualizer_node,
        tracked_buoy_visualizer_node,
        rviz_node,
    ])
