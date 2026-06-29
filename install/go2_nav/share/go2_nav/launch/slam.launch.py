# Copyright (c) 2026 Safal
# Licensed under the MIT License

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():

    this_package = FindPackageShare('go2_nav')

    # RTAB-Map launch path
    rtabmap_launch_path = PathJoinSubstitution(
        [FindPackageShare('rtabmap_launch'), 'launch', 'rtabmap.launch.py']
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            name='rviz',
            default_value='false',
            description='Launch RViz'
        ),

        DeclareLaunchArgument(
            name='rtabmap_viz',
            default_value='false',
            description='Launch RTAB-Map visualizer'
        ),

        # Odom TF broadcaster — odom → base_link
        Node(
            package='go2_nav',
            executable='odom_broadcast',
            name='odom_tf_broadcaster',
            output='screen'
        ),

        # RTAB-Map SLAM
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rtabmap_launch_path),
            launch_arguments={
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'map_frame_id': 'map',
                'subscribe_scan_cloud': 'true',
                'scan_cloud_topic': '/utlidar/cloud_deskewed',
                'subscribe_scan': 'false',
                'visual_odometry': 'false',
                'icp_odometry': 'false',
                'odom_topic': '/utlidar/robot_odom',
                'approx_sync': 'true',
                'approx_sync_max_interval': '0.5',
                'publish_tf_map': 'true',
                'use_sim_time': 'false',
                'rtabmap_viz': LaunchConfiguration('rtabmap_viz'),
                'rviz': LaunchConfiguration('rviz'),
                'localization': 'false',
                'args': '--delete_db_on_start',  # fresh map every run
                'qos': '1',
                'wait_for_transform': '0.5',
            }.items()
        ),
    ])