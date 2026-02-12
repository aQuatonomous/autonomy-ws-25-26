from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os


def _create_camera_nodes(context):
    res = LaunchConfiguration('resolution').perform(context)
    devs = LaunchConfiguration('camera_devices').perform(context)
    w, h = map(int, res.split(','))
    dev_list = [d.strip() for d in devs.split(',')]
    if len(dev_list) != 3:
        raise ValueError(
            "camera_devices must be 3 comma-separated paths, e.g. /dev/v4l/by-path/platform-3610000.usb-usb-0:1.2.2:1.0-video-index0,/dev/v4l/by-path/platform-3610000.usb-usb-0:1.2.3:1.0-video-index0,/dev/v4l/by-path/platform-3610000.usb-usb-0:1.2.4:1.0-video-index0"
        )
    return [
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='camera0_node',
            namespace='camera0',
            parameters=[{'video_device': dev_list[0], 'image_size': [w, h]}]
        ),
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='camera1_node',
            namespace='camera1',
            parameters=[{'video_device': dev_list[1], 'image_size': [w, h]}]
        ),
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='camera2_node',
            namespace='camera2',
            parameters=[{'video_device': dev_list[2], 'image_size': [w, h]}]
        ),
    ]


def generate_launch_description():
    default_engine_path = os.path.join(
        os.path.expanduser('~'),
        'autonomy-ws-25-26',
        'computer_vision',
        'cv_scripts',
        'model.engine'
    )

    resolution_arg = DeclareLaunchArgument(
        'resolution',
        default_value='1920,1200',
        description='Camera resolution as W,H (e.g. 1920,1200)'
    )

    camera_devices_arg = DeclareLaunchArgument(
        'camera_devices',
        default_value='/dev/v4l/by-path/platform-3610000.usb-usb-0:1.2.2:1.0-video-index0,/dev/v4l/by-path/platform-3610000.usb-usb-0:1.2.3:1.0-video-index0,/dev/v4l/by-path/platform-3610000.usb-usb-0:1.2.4:1.0-video-index0',
        description='Comma-separated paths to 3 camera devices (using persistent USB port paths)'
    )

    engine_path_arg = DeclareLaunchArgument(
        'engine_path',
        default_value=default_engine_path,
        description='Path to TensorRT engine file'
    )

    conf_threshold_arg = DeclareLaunchArgument(
        'conf_threshold',
        default_value='0.25',
        description='Confidence threshold for detections'
    )

    staleness_threshold_arg = DeclareLaunchArgument(
        'staleness_threshold',
        default_value='1.0',
        description='Staleness threshold for combiner (seconds)'
    )

    enable_task4_arg = DeclareLaunchArgument(
        'enable_task4',
        default_value='false',
        description='Enable Task 4 supply drop processor'
    )

    enable_indicator_buoy_arg = DeclareLaunchArgument(
        'enable_indicator_buoy',
        default_value='false',
        description='Enable indicator buoy processor (red/green colour indicator buoy)'
    )

    default_number_engine = os.path.join(
        os.path.expanduser('~'),
        'autonomy-ws-25-26',
        'computer_vision',
        'cv_scripts',
        'number_detection.engine'
    )
    enable_number_detection_arg = DeclareLaunchArgument(
        'enable_number_detection',
        default_value='false',
        description='Enable docking number detection (digits 1, 2, 3)'
    )
    number_detection_engine_arg = DeclareLaunchArgument(
        'number_detection_engine',
        default_value=default_number_engine,
        description='Path to number detection TensorRT engine'
    )
    number_conf_threshold_arg = DeclareLaunchArgument(
        'number_conf_threshold',
        default_value='0.25',
        description='Confidence threshold for number detection'
    )

    # Preprocessing nodes
    preprocess0 = Node(
        package='cv_ros_nodes',
        executable='vision_preprocessing',
        name='preprocess_camera0',
        arguments=['--camera_id', '0']
    )
    preprocess1 = Node(
        package='cv_ros_nodes',
        executable='vision_preprocessing',
        name='preprocess_camera1',
        arguments=['--camera_id', '1']
    )
    preprocess2 = Node(
        package='cv_ros_nodes',
        executable='vision_preprocessing',
        name='preprocess_camera2',
        arguments=['--camera_id', '2']
    )

    # Inference nodes
    inference0 = Node(
        package='cv_ros_nodes',
        executable='vision_inference',
        name='inference_camera0',
        arguments=[
            '--camera_id', '0',
            '--engine_path', LaunchConfiguration('engine_path'),
            '--conf_threshold', LaunchConfiguration('conf_threshold'),
            '--enable_number_detection', LaunchConfiguration('enable_number_detection'),
            '--number_detection_engine', LaunchConfiguration('number_detection_engine'),
            '--number_conf_threshold', LaunchConfiguration('number_conf_threshold'),
        ]
    )
    inference1 = Node(
        package='cv_ros_nodes',
        executable='vision_inference',
        name='inference_camera1',
        arguments=[
            '--camera_id', '1',
            '--engine_path', LaunchConfiguration('engine_path'),
            '--conf_threshold', LaunchConfiguration('conf_threshold'),
            '--enable_number_detection', LaunchConfiguration('enable_number_detection'),
            '--number_detection_engine', LaunchConfiguration('number_detection_engine'),
            '--number_conf_threshold', LaunchConfiguration('number_conf_threshold'),
        ]
    )
    inference2 = Node(
        package='cv_ros_nodes',
        executable='vision_inference',
        name='inference_camera2',
        arguments=[
            '--camera_id', '2',
            '--engine_path', LaunchConfiguration('engine_path'),
            '--conf_threshold', LaunchConfiguration('conf_threshold'),
            '--enable_number_detection', LaunchConfiguration('enable_number_detection'),
            '--number_detection_engine', LaunchConfiguration('number_detection_engine'),
            '--number_conf_threshold', LaunchConfiguration('number_conf_threshold'),
        ]
    )

    combiner = Node(
        package='cv_ros_nodes',
        executable='vision_combiner',
        name='detection_combiner',
        arguments=['--staleness_threshold', LaunchConfiguration('staleness_threshold')]
    )

    task4_supply_processor = Node(
        package='cv_ros_nodes',
        executable='task4_supply_processor',
        name='task4_supply_processor',
        condition=IfCondition(LaunchConfiguration('enable_task4'))
    )

    indicator_buoy_processor = Node(
        package='cv_ros_nodes',
        executable='indicator_buoy_processor',
        name='indicator_buoy_processor',
        condition=IfCondition(LaunchConfiguration('enable_indicator_buoy'))
    )

    return LaunchDescription([
        resolution_arg,
        camera_devices_arg,
        engine_path_arg,
        conf_threshold_arg,
        staleness_threshold_arg,
        enable_task4_arg,
        enable_indicator_buoy_arg,
        enable_number_detection_arg,
        number_detection_engine_arg,
        number_conf_threshold_arg,
        LogInfo(msg='Starting computer vision pipeline...'),
        OpaqueFunction(function=_create_camera_nodes),
        preprocess0,
        preprocess1,
        preprocess2,
        inference0,
        inference1,
        inference2,
        combiner,
        task4_supply_processor,
        indicator_buoy_processor,
        LogInfo(msg='All computer vision nodes started!')
    ])
