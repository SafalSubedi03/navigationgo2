# Copyright (c) 2026 Safal
# Licensed under the MIT License

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetRemap


def generate_launch_description():

    this_package = FindPackageShare('go2_nav')
    go2_nav_dir = get_package_share_directory('go2_nav')

    # Paths
    slam_nav2_params_path = PathJoinSubstitution(
        [this_package, 'config', 'slam_nav2_params.yaml']
    )

    rtabmap_launch_path = PathJoinSubstitution(
        [FindPackageShare('rtabmap_launch'), 'launch', 'rtabmap.launch.py']
    )

    nav2_launch_path = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py']
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

        # PointCloud to LaserScan
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            remappings=[
                ('cloud_in', '/utlidar/cloud_deskewed'),
                ('scan', '/scan')
            ],
            parameters=[{
                'target_frame': 'odom',
                'transform_tolerance': 0.5,
                'min_height': 0.20,
                'max_height': 1.5,
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'angle_increment': 0.0043,
                'scan_time': 0.1,
                'range_min': 0.2,
                'range_max': 30.0,
                'use_inf': True,
                'use_sim_time': False,
            }],
            output='screen'
        ),

        # RTAB-Map SLAM — builds map in real time
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
                'args': '--delete_db_on_start',
                'qos': '1',
                'wait_for_transform': '0.5',
                # Connect RTAB-Map frontier goals directly to Nav2
                'use_action_for_goal': 'true',
                'output_goal_topic': '/goal_pose',
            }.items()
        ),

        # Nav2 navigation — no map server needed, RTAB-Map provides the map
        GroupAction([
            SetRemap(src='/cmd_vel', dst='/cmd_vel_manual'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch_path),
                launch_arguments={
                    'use_sim_time': 'false',
                    'params_file': slam_nav2_params_path,
                }.items()
            )
        ]),
    ])