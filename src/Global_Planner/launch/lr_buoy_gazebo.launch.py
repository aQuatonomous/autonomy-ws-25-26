"""
Launch LR buoy nav for Gazebo testing.

Requires: Gazebo sim + MAVROS already running (e.g. aquatonomous_simulation_full.bash).

Launches:
  - global_frame (boat_state_node) for /boat_pose from MAVROS GPS+heading
  - CV sim pipeline (launch_cv_sim) for /combined/detection_info_with_distance
  - lr_buoy_node: publishes to /mavros/setpoint_position/local (ENU meters)

Usage (after colcon build, with sim+MAVROS running):
  ros2 launch global_planner lr_buoy_gazebo.launch.py

  # Single camera (recommended on Jetson to avoid GPU OOM):
  ros2 launch global_planner lr_buoy_gazebo.launch.py single_camera:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Resolve planning path for lr_buoy_node (same as global_planner.launch.py)
    try:
        share = get_package_share_directory("global_planner")
        for rel in [
            os.path.join(share, "..", "..", "..", "src", "planning"),
            os.path.join(share, "..", "..", "..", "..", "planning"),
        ]:
            p = os.path.abspath(rel)
            if os.path.isdir(p):
                planning_path = p
                break
        else:
            planning_path = os.path.abspath(os.path.join(share, "..", "..", "..", ".."))
    except Exception:
        planning_path = os.path.abspath(
            os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", "planning")
        )

    cv_share = get_package_share_directory("cv_ros_nodes")
    global_frame_share = get_package_share_directory("global_frame")

    return LaunchDescription([
        DeclareLaunchArgument(
            "single_camera",
            default_value="false",
            description="If true, run only camera1 (use on Jetson to avoid GPU OOM).",
        ),
        SetEnvironmentVariable(name="PLANNING_PATH", value=planning_path),

        # Boat pose from MAVROS (GPS + compass -> east, north, heading)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(global_frame_share, "launch", "global_frame.launch.py")
            ]),
        ),

        # CV pipeline for sim (no v4l2; /cameraN/image_raw from Gazebo bridges)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(cv_share, "launch", "launch_cv_sim.py")
            ]),
            launch_arguments={
                "single_camera": LaunchConfiguration("single_camera"),
            }.items(),
        ),

        # LR buoy nav node
        Node(
            package="global_planner",
            executable="lr_buoy_node",
            name="lr_buoy_node",
            output="screen",
        ),
    ])
