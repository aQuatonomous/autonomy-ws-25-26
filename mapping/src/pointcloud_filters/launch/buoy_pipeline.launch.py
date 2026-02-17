"""
All-in-one LiDAR buoy pipeline: Unitree driver → range filter → detector → tracker → visualizer.
Optionally launches RViz. Use lidar_port to pin the LiDAR to a stable by-path device.
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _assert_lidar_port_exists(context):
    """Fail at launch if lidar_port does not exist (avoids unitree node silent fail)."""
    port = LaunchConfiguration('lidar_port').perform(context)
    if not port or not os.path.exists(port):
        by_path = '/dev/serial/by-path'
        hint = f"Run: ls -l {by_path}" if os.path.isdir(by_path) else "Check /dev/ttyUSB* and set lidar_port:=/dev/ttyUSB0"
        raise RuntimeError(
            f"LiDAR port does not exist: {port!r}. {hint}"
        )
    return []


def generate_launch_description() -> LaunchDescription:
    # Default LiDAR port: by-path for stable device identity.
    # - 2.4.1.2: LiDAR on hub or as detected by list_usb_ports.sh.
    # - 2.1: LiDAR plugged directly into Jetson (no hub). Override: lidar_port:=/dev/serial/by-path/...2.1:1.0-port0
    default_lidar_port = '/dev/serial/by-path/platform-3610000.usb-usb-0:2.4.1.2:1.0-port0'

    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port',
        default_value=default_lidar_port,
        description='Serial port for Unitree LiDAR (by-path or /dev/ttyUSB0). Override if different: lidar_port:=/dev/serial/by-path/...',
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
            {'port': LaunchConfiguration('lidar_port')},
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

    visualizer_node = Node(
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
        lidar_port_arg,
        launch_rviz_arg,
        buoy_detector_log_level_arg,
        OpaqueFunction(function=_assert_lidar_port_exists),
        unitree_node,
        range_filter_node,
        buoy_detector_node,
        buoy_tracker_node,
        visualizer_node,
        rviz_node,
    ])
