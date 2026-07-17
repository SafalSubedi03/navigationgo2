# Follow-object navigation stack (no autonomous exploration)
# follow_object_server publishes /goal_pose (map frame) whenever it locks
# onto a tracked object; Nav2's bt_navigator consumes it directly via its
# built-in goal_pose topic subscription and drives the robot there.
#
# Run with:
#   ros2 launch go2_config goal_pose_nav.launch.py

import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetRemap
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, GroupAction


def generate_launch_description():

    this_package = FindPackageShare('go2_nav')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    slam_launch_path = PathJoinSubstitution(
        [this_package, 'launch', 'slam_explore.launch.py']
    )
    nav2_params_path = PathJoinSubstitution(
        [this_package, 'config', 'nav2_slam_params.yaml']
    )

    # -------------------------------------------------------------------
    # 1. RTAB-Map SLAM
    # -------------------------------------------------------------------
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_path),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # -------------------------------------------------------------------
    # 2. Nav2 -- controller, planner, behaviors, bt_navigator ONLY.
    #    bt_navigator's built-in 'goal_pose' topic subscription is what
    #    picks up follow_object_server's published goals -- no explore_lite,
    #    no relay node needed.
    # -------------------------------------------------------------------
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    navigation_launch = GroupAction([
        SetRemap(src='/cmd_vel', dst='/cmd_vel_manual'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file':  nav2_params_path,
            }.items(),
        ),
    ])

    restamp_node = Node(
        package='go2_nav',
        executable='restamp_node',
        name='restamp_node',
        output='screen',
    )

    # -------------------------------------------------------------------
    # 3. follow_object_server -- publishes /goal_pose when a tracked
    #    object is detected and localized via camera+lidar fusion.
    # -------------------------------------------------------------------
    # follow_object_node = Node(
    #     package='go2_nav',           # <-- set to your actual package name
    #     executable='follow_object_server',
    #     name='follow_object_server',
    #     output='screen',
    #     parameters=[{'use_sim_time': use_sim_time}],
    # )

    # -------------------------------------------------------------------
    # 4. Static TFs -- base_link -> camera_link, base_link -> lidar frame.
    #    Quick-fix placeholder values; replace once extrinsic calibration
    #    is done (see calibration guide).
    # -------------------------------------------------------------------
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf_broadcaster',
        arguments=['--x', '0.28', '--y', '0', '--z', '0.05',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'base_link',
                   '--child-frame-id', 'camera_link'],
    )
    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_tf_broadcaster',
        arguments=['--x', '0.28945', '--y', '0', '--z', '-0.046825',
                   '--roll', '0', '--pitch', '2.8782', '--yaw', '0',
                   '--frame-id', 'base_link',
                   '--child-frame-id', 'utlidar_lidar'],  # <-- set to your real lidar frame_id
    )

    return LaunchDescription([
        declare_use_sim_time,
        restamp_node,
        camera_tf,
        lidar_tf,
        TimerAction(
            period=3.0,
            actions=[slam_launch],
        ),
        navigation_launch,
        # TimerAction(
        #     period=20.0,
        #     actions=[follow_object_node],
        # ),
    ])