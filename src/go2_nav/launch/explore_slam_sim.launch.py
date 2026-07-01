# explore_slam_sim.launch.py
#
# Full autonomous frontier-exploration stack, simulation only.
# Robot builds the map live with RTAB-Map while explore_lite drives it
# toward unexplored frontiers via Nav2.
#
# Pipeline:
#   slam_explore_sim.launch.py (included) -> RTAB-Map produces /map (live)
#   /scan                                  -> already published by your sim setup
#   nav2_bringup/navigation_launch.py      -> controller/planner/bt_navigator
#                                              (NO AMCL, NO map_server)
#   explore_lite                           -> reads costmap, sends NavigateToPose goals
#
# Run with:
#   ros2 launch go2_config explore_slam_sim.launch.py

import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, GroupAction

def generate_launch_description():

    this_package = FindPackageShare('go2_config')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    slam_launch_path = PathJoinSubstitution(
        [this_package, 'launch', 'slam_explore_sim.launch.py']
    )
    nav2_params_path = PathJoinSubstitution(
        [this_package, 'config/autonomy', 'nav2_slam_params.yaml']
    )
    explore_params_path = PathJoinSubstitution(
        [this_package, 'config/autonomy', 'explore_lite_params.yaml']
    )

    # -------------------------------------------------------------------
    # 1. RTAB-Map SLAM (included from slam_explore_sim.launch.py)
    # -------------------------------------------------------------------
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_path),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # -------------------------------------------------------------------
    # 2. Nav2 -- controller, planner, behaviors, bt_navigator ONLY.
    #    Uses navigation_launch.py (lifecycle subset WITHOUT AMCL/map_server),
    #    not bringup_launch.py.
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

    # -------------------------------------------------------------------
    # 3. explore_lite -- frontier selection, sends NavigateToPose goals
    # -------------------------------------------------------------------
    explore_node = Node(
        package='explore_lite',
        executable='explore',
        name='explore_node',
        output='screen',
        parameters=[explore_params_path, {'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        declare_use_sim_time,
        slam_launch,
        navigation_launch,
        TimerAction(
            period=10.0,
            actions=[explore_node],
        ),
    ])