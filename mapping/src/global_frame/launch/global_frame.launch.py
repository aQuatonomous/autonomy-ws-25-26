#!/usr/bin/env python3
"""Launch boat_state_node and detection_to_global_node."""

import launch
import launch_ros.actions


def generate_launch_description():
    boat_state = launch_ros.actions.Node(
        package="global_frame",
        executable="boat_state_node",
        name="boat_state_node",
        output="screen",
        parameters=[
            {"global_position_topic": "mavros/global_position/global"},
            {"heading_topic": "mavros/global_position/compass_hdg"},
            {"heading_topic_type": "compass_hdg"},
            {"use_first_fix_as_origin": True},
            {"publish_pose_stamped": True},
            {"publish_tf": True},
        ],
    )
    detection_to_global = launch_ros.actions.Node(
        package="global_frame",
        executable="detection_to_global_node",
        name="detection_to_global_node",
        output="screen",
    )
    return launch.LaunchDescription([boat_state, detection_to_global])
