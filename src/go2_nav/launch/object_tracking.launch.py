import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Launch Configurations & Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    target_class = LaunchConfiguration('target_class', default='person')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    declare_target_class = DeclareLaunchArgument(
        'target_class',
        default_value='person',
        description='Target class name to track (e.g., person, sports_ball, stairs)'
    )

    # 1. Static TF: base_link -> camera_link (Camera mounting calibration)
    camera_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf_broadcaster',
        arguments=[
            '--x', '0.28',
            '--y', '0.0',
            '--z', '0.05',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_link'
        ],
        output='screen'
    )

    # 2. Static TF: base_link -> utlidar_lidar (Lidar mounting calibration)
    lidar_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_tf_broadcaster',
        arguments=[
            '--x', '0.28945',
            '--y', '0.0',
            '--z', '-0.046825',
            '--roll', '0.0',
            '--pitch', '2.8782',
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'utlidar_lidar'
        ],
        output='screen'
    )

    # 3. Camera Info Publisher (Publishes standard calibrated intrinsic matrix)
    camera_info_node = Node(
        package='go2_camera_info',
        executable='camera_info_publisher',
        name='camera_info_publisher',
        output='screen',
        parameters=[{
            'frame_id': 'camera_link',
            'publish_rate_hz': 30.0,
            'use_sim_time': use_sim_time
        }]
    )

    # 4. Object Tracking Fusion Node
    # Scheduled to start 5 seconds after static transforms and camera info publisher boot
    object_tracker_node = Node(
        package='go2_nav',
        executable='object_tracker',
        name='object_tracking_fusion_node',
        output='screen',
        parameters=[{
            'target_class': target_class,
            'use_sim_time': use_sim_time,
            'detection_topic': '/yolo/detections',
            'cloud_topic': '/utlidar/cloud_deskewed_restamped',
            'camera_info_topic': '/front_camera/camera_info',
            'goal_pose_topic': '/goal_pose',
            'min_points_in_box': 5
        }]
    )

    timed_object_tracker = TimerAction(
        period=5.0,
        actions=[object_tracker_node]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_target_class,
        camera_tf_node,
        lidar_tf_node,
        camera_info_node,
        timed_object_tracker
    ])
