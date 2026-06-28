import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap

def generate_launch_description():
    go2_nav_dir = get_package_share_directory('go2_nav')

    # 2. Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    params_file = LaunchConfiguration('params_file', default=os.path.join(go2_nav_dir, 'config', 'nav2_params.yaml'))
    map_yaml_file = LaunchConfiguration('map', default=os.path.join(go2_nav_dir, 'maps', 'rtabmap.yaml'))

    # 3. PointCloud to LaserScan Node
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/utlidar/cloud_deskewed'),
            ('scan', '/scan')
        ],
        parameters=[{
            'target_frame': 'odom',
            'transform_tolerance': 0.01,
            'min_height': 0.20,
            'max_height': 1.5,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0043,
            'scan_time': 0.1,
            'range_min': 0.2,
            'range_max': 30.0,
            'use_inf': True,
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    # 4. Include Nav2 Bringup Launch wrapped in GroupAction for cmd_vel remapping
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch = GroupAction([
        SetRemap(src='/cmd_vel', dst='/cmd_vel_manual'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'map': map_yaml_file,
                'use_sim_time': use_sim_time,
                'params_file': params_file
            }.items()
        )
    ])

    odomtfBroadcaster = Node(
        package='go2_nav',
        executable='odom_broadcast',
        name='odom_tf_broadcaster',
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation clock if true'),
        DeclareLaunchArgument('params_file', default_value=params_file, description='Full path to the ROS2 parameters file to use'),
        DeclareLaunchArgument('map', default_value=map_yaml_file, description='Full path to map yaml file to load'),
        DeclareLaunchArgument(
            name='rviz',
            default_value='false',
            description='Run rviz'
        ),
        # Nodes to run
        pointcloud_to_laserscan_node,
        nav2_launch,
        odomtfBroadcaster
    ])