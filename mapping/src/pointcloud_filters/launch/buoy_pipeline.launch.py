"""
All-in-one launch: LiDAR driver, range filter, buoy detector, tracker, visualizer, and RViz.

Usage:
  # Full pipeline with RViz (default; requires unitree_lidar_ros2 and rviz2)
  ros2 launch pointcloud_filters buoy_pipeline.launch.py

  # Without launching the LiDAR driver (e.g. driver already running elsewhere)
  ros2 launch pointcloud_filters buoy_pipeline.launch.py launch_lidar_driver:=false

  # Without RViz
  ros2 launch pointcloud_filters buoy_pipeline.launch.py launch_rviz:=false

  # Inside-bay / short range (0.5 m)
  ros2 launch pointcloud_filters buoy_pipeline.launch.py range_max:=0.5 z_max:=0.5

  # Inside-bay with detector tuning
  ros2 launch pointcloud_filters buoy_pipeline.launch.py range_max:=0.5 z_max:=0.5 eps:=0.15 min_samples:=15

  # Stricter tracking (fewer false tracks): require more observations before creating a track
  ros2 launch pointcloud_filters buoy_pipeline.launch.py min_observations_to_add:=8 candidate_max_consecutive_misses:=3
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_pointcloud_filters = get_package_share_directory('pointcloud_filters')

    # ----- Launch arguments -----
    launch_lidar_driver_arg = DeclareLaunchArgument(
        'launch_lidar_driver',
        default_value='true',
        description='If true, start the Unitree LiDAR driver (unitree_lidar_ros2). Set false if driver runs elsewhere.',
    )
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='If true, start RViz2 with buoy pipeline config.',
    )
    # Range filter (defaults: normal range ~30 m)
    range_max_arg = DeclareLaunchArgument('range_max', default_value='30.0', description='Max horizontal range (m). Use 0.5 for inside-bay.')
    z_min_arg = DeclareLaunchArgument('z_min', default_value='-0.37', description='Min height z (m).')
    z_max_arg = DeclareLaunchArgument('z_max', default_value='10.0', description='Max height z (m). Use 0.5 for inside-bay.')
    use_tf_arg = DeclareLaunchArgument('use_tf_transform', default_value='false', description='Transform cloud to base_frame via TF.')
    # Buoy detector (optional overrides for inside-bay)
    eps_arg = DeclareLaunchArgument('eps', default_value='0.8', description='DBSCAN eps (m). Max distance between points in same cluster; larger = looser clusters.')
    min_samples_arg = DeclareLaunchArgument('min_samples', default_value='2', description='DBSCAN min_samples. Min points to form a cluster; use 2–3 for small buoys.')
    max_lateral_extent_arg = DeclareLaunchArgument('max_lateral_extent', default_value='1.0', description='Max cluster width (m); reject walls/large surfaces. Buoys ~0.3-0.8 m.')
    min_points_final_arg = DeclareLaunchArgument('min_points_final', default_value='2', description='Min points per cluster after validation; use 2–3 so small buoys pass.')
    min_isolation_margin_arg = DeclareLaunchArgument('min_isolation_margin', default_value='0.0', description='Min distance (m) from cluster to nearest other point; 0 = off. Non-zero can reject buoys near water/ground.')
    max_aspect_ratio_arg = DeclareLaunchArgument('max_aspect_ratio', default_value='10.0', description='Max bbox aspect ratio; small clusters (2–3 pts) are often elongated along beam.')
    max_external_points_nearby_arg = DeclareLaunchArgument('max_external_points_nearby', default_value='-1', description='Max other points within external_density_radius of centroid; -1 = off. Non-zero can reject buoys near water.')
    external_density_radius_arg = DeclareLaunchArgument('external_density_radius', default_value='0.8', description='Radius (m) for counting nearby external points.')
    # Tracker (stricter = fewer false tracks)
    min_observations_arg = DeclareLaunchArgument(
        'min_observations_for_publish',
        default_value='3',
        description='Frames a tracked buoy must be seen before publishing to output.',
    )
    min_observations_to_add_arg = DeclareLaunchArgument(
        'min_observations_to_add',
        default_value='5',
        description='Frames a new object must be seen in the same area before becoming a track (higher = stricter, fewer false buoys).',
    )
    candidate_max_misses_arg = DeclareLaunchArgument(
        'candidate_max_consecutive_misses',
        default_value='4',
        description='Frames a candidate can go unmatched before it is dropped (must re-see to become a track).',
    )
    association_threshold_arg = DeclareLaunchArgument(
        'association_threshold',
        default_value='0.5',
        description='Max distance (m) to match a detection to a track or candidate (smaller = stricter).',
    )
    max_consecutive_misses_arg = DeclareLaunchArgument(
        'max_consecutive_misses',
        default_value='10',
        description='Frames a tracked buoy can be missed before it is removed.',
    )
    buoy_detector_log_level_arg = DeclareLaunchArgument(
        'buoy_detector_log_level',
        default_value='info',
        description='Log level for buoy_detector (e.g. debug to see why clusters are rejected).',
    )

    # ----- Unitree LiDAR driver (optional) -----
    lidar_driver_node = Node(
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
        condition=IfCondition(LaunchConfiguration('launch_lidar_driver')),
    )

    # ----- Range filter -----
    range_filter_node = Node(
        package='pointcloud_filters',
        executable='lidar_range_filter',
        name='lidar_range_filter',
        output='screen',
        parameters=[
            {'input_topic': '/unilidar/cloud'},
            {'output_topic': '/points_filtered'},
            {'base_frame': 'base_link'},
            {'use_tf_transform': LaunchConfiguration('use_tf_transform')},
            {'z_min': LaunchConfiguration('z_min')},
            {'z_max': LaunchConfiguration('z_max')},
            {'range_max': LaunchConfiguration('range_max')},
            {'max_buffer_duration_sec': 5.0},
            {'keep_publisher': True},
            {'enable_accumulation': False},
            {'accumulation_window': 0.2},
        ],
    )

    # ----- Buoy detector -----
    buoy_detector_node = Node(
        package='pointcloud_filters',
        executable='buoy_detector',
        name='buoy_detector',
        output='screen',
        arguments=[['--ros-args', '--log-level', LaunchConfiguration('buoy_detector_log_level')]],
        parameters=[
            {'input_topic': '/points_filtered'},
            {'output_topic': '/buoy_detections'},
            {'eps': LaunchConfiguration('eps')},
            {'min_samples': LaunchConfiguration('min_samples')},
            {'min_lateral_extent': 0.01},
            {'max_lateral_extent': LaunchConfiguration('max_lateral_extent')},
            {'min_points_final': LaunchConfiguration('min_points_final')},
            {'min_isolation_margin': LaunchConfiguration('min_isolation_margin')},
            {'max_aspect_ratio': LaunchConfiguration('max_aspect_ratio')},
            {'max_external_points_nearby': LaunchConfiguration('max_external_points_nearby')},
            {'external_density_radius': LaunchConfiguration('external_density_radius')},
            {'confidence_scale': 15.0},
            {'ransac_enabled': False},
            {'ransac_iterations': 80},
            {'ransac_distance_threshold': 0.15},
            {'ransac_min_inlier_ratio': 0.3},
        ],
    )

    # ----- Buoy tracker -----
    tracker_node = Node(
        package='pointcloud_filters',
        executable='buoy_tracker',
        name='buoy_tracker',
        output='screen',
        parameters=[
            {'association_threshold': LaunchConfiguration('association_threshold')},
            {'max_consecutive_misses': LaunchConfiguration('max_consecutive_misses')},
            {'min_observations_for_publish': LaunchConfiguration('min_observations_for_publish')},
            {'min_observations_to_add': LaunchConfiguration('min_observations_to_add')},
            {'candidate_max_consecutive_misses': LaunchConfiguration('candidate_max_consecutive_misses')},
            {'position_alpha': 0.7},
        ],
    )

    # ----- Tracked buoy visualizer -----
    tracked_visualizer_node = Node(
        package='pointcloud_filters',
        executable='tracked_buoy_visualizer',
        name='tracked_buoy_visualizer',
        output='screen',
        parameters=[
            {'marker_height': 0.3},
            {'marker_radius': 0.1},
            {'marker_lifetime': 2.0},
            {'show_text_labels': True},
        ],
    )

    # ----- RViz (optional) -----
    rviz_config_path = os.path.join(pkg_pointcloud_filters, 'rviz', 'buoy_pipeline.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
    )

    return LaunchDescription([
        launch_lidar_driver_arg,
        launch_rviz_arg,
        range_max_arg,
        z_min_arg,
        z_max_arg,
        use_tf_arg,
        eps_arg,
        min_samples_arg,
        max_lateral_extent_arg,
        min_points_final_arg,
        min_isolation_margin_arg,
        max_aspect_ratio_arg,
        max_external_points_nearby_arg,
        external_density_radius_arg,
        min_observations_arg,
        min_observations_to_add_arg,
        candidate_max_misses_arg,
        association_threshold_arg,
        max_consecutive_misses_arg,
        buoy_detector_log_level_arg,
        lidar_driver_node,
        range_filter_node,
        buoy_detector_node,
        tracker_node,
        tracked_visualizer_node,
        rviz_node,
    ])
