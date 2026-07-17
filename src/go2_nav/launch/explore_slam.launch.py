
# Full autonomous frontier-exploration stack
# Robot builds the map live with RTAB-Map while explore_lite drives it
# toward unexplored frontiers via Nav2.
# Run with:
#   ros2 launch go2_config explore_slam.launch.py

import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetRemap
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, GroupAction
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
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
    # explore_params_path = os.path.join(
    #     get_package_share_directory('go2_nav'),
    #     'config',
    #     'explore_lite_params.yaml'
    # )

    declare_explore = DeclareLaunchArgument(
        'explore',
        default_value='true',
        description='Start exploration or not'
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
    restamp_node = Node(
        package='go2_nav',
        executable='restamp_node',
        name='restamp_node',
        output='screen',
    )
    # -------------------------------------------------------------------
    # 3. explore_lite -- frontier selection, sends NavigateToPose goals
    # -------------------------------------------------------------------
    explore_node = Node(
        package='explore_lite',
        executable='explore',
        condition = IfCondition(LaunchConfiguration('explore')),
        name='explore_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_base_frame': 'base_link',
            'costmap_topic': '/global_costmap/costmap',
            'costmap_updates_topic': '/global_costmap/costmap_updates',
            'visualize': True,
            'planner_frequency': 0.5,
            'progress_timeout': 60.0,
            'potential_scale': 3.0,
            'orientation_scale': 0.0,
            'gain_scale': 1.0,
            'transform_tolerance': 0.5,
            'min_frontier_size': 1.0,
            'return_to_init': False,
        }],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_explore,
        restamp_node,          
        TimerAction(
            period=3.0,        
            actions=[slam_launch],
        ),
        
        navigation_launch,

        TimerAction(
            period=20.0,
            actions=[explore_node],
        ),
    ])