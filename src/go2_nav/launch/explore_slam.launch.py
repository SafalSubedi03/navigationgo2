# Full autonomous frontier-exploration stack
# Robot builds the map live with RTAB-Map while explore_lite drives it
# toward unexplored frontiers via Nav2.
# Run with:
#   ros2 launch go2_config explore_slam.launch.py

import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetRemap
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, GroupAction
from launch.conditions import IfCondition
from nav2_common.launch import RewrittenYaml

def generate_launch_description():

    this_package = FindPackageShare('go2_nav')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    lidar_source = LaunchConfiguration('lidar_source')
    declare_lidar_source = DeclareLaunchArgument(
        'lidar_source', default_value='unitree',
        description="Which lidar to use for mapping: 'unitree' or 'livox'"
    )

    scan_cloud_topic = PythonExpression([
        "'/livox/lidar' if '", lidar_source, "' == 'livox' else '/utlidar/cloud_deskewed_restamped'"
    ])

    slam_launch_path = PathJoinSubstitution(
        [this_package, 'launch', 'slam_explore.launch.py']
    )
    nav2_params_path = PathJoinSubstitution(
        [this_package, 'config', 'nav2_slam_params.yaml']
    )

    declare_explore = DeclareLaunchArgument(
        'explore',
        default_value='true',
        description='Start exploration or not'
    )

    # -------------------------------------------------------------------
    # 1. RTAB-Map SLAM (included from slam_explore.launch.py)
    # -------------------------------------------------------------------
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_path),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'lidar_source': lidar_source,
        }.items(),
    )

    # -------------------------------------------------------------------
    # 2. Nav2 -- controller, planner, behaviors, bt_navigator ONLY.
    #    params_file is rewritten at launch time to substitute the correct
    #    lidar observation-source topic based on lidar_source.
    # -------------------------------------------------------------------
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    rewritten_nav2_params = RewrittenYaml(
        source_file=nav2_params_path,
        root_key='',
        param_rewrites={'LIDAR_OBSERVATION_TOPIC': scan_cloud_topic},
        convert_types=True,
    )

    navigation_launch = GroupAction([
        SetRemap(src='/cmd_vel', dst='/cmd_vel_manual'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file':  rewritten_nav2_params,
            }.items(),
        ),
    ])

    # restamp_node fixes Unitree's clock offset for BOTH the Unitree cloud
    # AND the robot's own odometry -- odometry restamping is needed
    # regardless of which lidar is active, so this always runs.
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
        declare_lidar_source,
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