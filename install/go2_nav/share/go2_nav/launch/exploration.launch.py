# Copyright (c) 2026 Safal
# Licensed under the MIT License

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetRemap


def generate_launch_description():

    this_package = FindPackageShare('go2_nav')

    slam_nav2_params_path = PathJoinSubstitution(
        [this_package, 'config', 'slam_nav2_params.yaml']
    )
    nav2_launch_path = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py']
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            name='rtabmap_viz',
            default_value='false',
            description='Launch RTAB-Map visualizer'
        ),

        # Odom TF broadcaster — odom → base_link (uses current time)
        Node(
            package='go2_nav',
            executable='odom_broadcast',
            name='odom_tf_broadcaster',
            output='screen'
        ),

        # Cloud relay — fixes frame_id (odom→base_link) and timestamp (104s delay)
        Node(
            package='go2_nav',
            executable='cloud_relay',
            name='cloud_relay',
            output='screen'
        ),

        # RTAB-Map SLAM — uses restamped cloud in base_link frame
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'map_frame_id': 'map',
                # Subscriptions — lidar only, no camera
                'subscribe_depth': False,
                'subscribe_rgb': False,
                'subscribe_rgbd': False,
                'subscribe_stereo': False,
                'subscribe_scan': False,
                'subscribe_scan_cloud': True,
                # Disable visual loop closure — no camera
                'RGBD/LoopClosureEnabled': 'false',
                'Kp/MaxFeatures': '-1',
                'Vis/FeatureType': '0',
                # Odometry
                'visual_odometry': False,
                # Synchronization
                'approx_sync': True,
                'approx_sync_max_interval': 2.0,
                'wait_for_transform': 0.5,
                # Map publishing
                'publish_tf_map': True,
                'use_sim_time': False,
                # Frontier exploration
                'use_action_for_goal': True,
                'output_goal_topic': '/goal_pose',
                # Grid map for Nav2
                'Grid/Sensor': '1',
                'Grid/RangeMin': '0.5',
                'Grid/RangeMax': '20.0',
                'Grid/CellSize': '0.05',
                'Grid/MaxObstacleHeight': '1.5',
                'Grid/MinGroundHeight': '-0.1',
                'Grid/3D': 'false',
                # ICP scan matching
                'Icp/VoxelSize': '0.1',
                'Icp/Iterations': '30',
                'Icp/RangeMin': '0.5',
                'Icp/RangeMax': '30.0',
                'Icp/PointToPlane': 'true',
                # QoS
                'qos_scan': 1,
                'qos_odom': 1,
                'Rtabmap/DetectionRate': '1.0',
                'database_path': '~/.ros/rtabmap_exploration.db',
            }],
            remappings=[
                ('scan_cloud', '/cloud_restamped'),  # use relay output
                ('odom', '/utlidar/robot_odom'),
            ],
            arguments=['-d']  # delete db on start
        ),

        # RTAB-Map visualizer
        Node(
            condition=IfCondition(LaunchConfiguration('rtabmap_viz')),
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            output='screen',
            parameters=[{
                'subscribe_depth': False,
                'subscribe_rgb': False,
                'subscribe_rgbd': False,
                'subscribe_scan': False,
                'subscribe_scan_cloud': True,
                'approx_sync': True,
                'approx_sync_max_interval': 2.0,
            }],
            remappings=[
                ('scan_cloud', '/cloud_restamped'),
                ('odom', '/utlidar/robot_odom'),
            ]
        ),

        # Nav2 — RTAB-Map provides map and localization
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