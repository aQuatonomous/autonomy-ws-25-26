from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    # Default engine path - can be in package or workspace
    # User can override with launch argument
    default_engine_path = os.path.join(
        os.path.expanduser('~'), 
        'autonomy-ws-25-26', 
        'cv_ros_nodes', 
        'model.engine'
    )

    # Launch arguments
    engine_path_arg = DeclareLaunchArgument(
        'engine_path',
        default_value=default_engine_path,
        description='Path to TensorRT engine file'
    )
    
    conf_threshold_arg = DeclareLaunchArgument(
        'conf_threshold',
        default_value = '0.25',
        description = 'Confidence threshold for detections'
    )
    
    staleness_threshold_arg = DeclareLaunchArgument(
        'staleness_threshold', 
        default_value='1.0',
        description= 'Staleness threshold for combiner (seconds)'
    )
    
    # Camera Nodes (v4l2_camera)
    camera0_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera0_node',
        namespace='camera0',
        parameters=[{
            'video_device': '/dev/video0',
            'image_size': [1920,1200],
            'framerate': 30.0
        }]
    )
    
    camera1_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera1_node',
        namespace='camera1',
        parameters=[{
            'video_device': '/dev/video2',
            'image_size': [1920,1200],
            'framerate': 30.0
        }]
    )
    
    camera2_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera2_node',
        namespace='camera2',
        parameters=[{
            'video_device': '/dev/video4',
            'image_size': [1920,1200],
            'framerate': 30.0
        }]
    )
    
    # Preprocessing nodes (installed executables)
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
    # Inference nodes (installed executables)
    inference0 = Node(
        package='cv_ros_nodes',
        executable='vision_inference',
        name='inference_camera0',
        arguments=[
            '--camera_id', '0',
            '--engine_path', LaunchConfiguration('engine_path'),
            '--conf_threshold', LaunchConfiguration('conf_threshold')
        ]
    )
    
    inference1 = Node(
        package='cv_ros_nodes',
        executable='vision_inference',
        name='inference_camera1',
        arguments=[
            '--camera_id', '1',
            '--engine_path', LaunchConfiguration('engine_path'),
            '--conf_threshold', LaunchConfiguration('conf_threshold')
        ]
    )
    
    inference2 = Node(
        package='cv_ros_nodes',
        executable='vision_inference',
        name='inference_camera2',
        arguments=[
            '--camera_id', '2',
            '--engine_path', LaunchConfiguration('engine_path'),
            '--conf_threshold', LaunchConfiguration('conf_threshold')
        ]
    )
    
    # Combiner node (installed executable)
    combiner = Node(
        package='cv_ros_nodes',
        executable='vision_combiner',
        name='detection_combiner',
        arguments=[
            '--staleness_threshold', LaunchConfiguration('staleness_threshold')
        ]
    )
    


    return LaunchDescription([
        engine_path_arg,
        conf_threshold_arg,
        staleness_threshold_arg,
        LogInfo(msg='Starting computer vision pipeline...'),
        camera0_node,
        camera1_node,
        camera2_node,
        preprocess0,
        preprocess1,
        preprocess2,
        inference0,
        inference1,
        inference2,
        combiner,
        LogInfo(msg='All computer vision nodes started!')
    ])