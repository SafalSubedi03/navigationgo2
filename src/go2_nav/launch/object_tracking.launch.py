from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    lidar_source = LaunchConfiguration('lidar_source')

    declare_lidar_source = DeclareLaunchArgument(
        'lidar_source',
        default_value='internal',
        description="'internal' (Unitree onboard LiDAR) or 'external' (Livox Mid-360)"
    )

    cloud_topic = PythonExpression([
        "'/utlidar/cloud_deskewed_restamped' if '", lidar_source, "' == 'internal' else '/livox/lidar'"
    ])

    return LaunchDescription([
        declare_lidar_source,

        Node(
            package='go2_nav',
            executable='camera_info_publisher',
            name='camera_info_publisher',
            output='screen',
            parameters=[{'frame_id': 'camera_link', 'publish_rate_hz': 30.0}]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_tf_broadcaster',
            arguments=['--x', '0.28', '--y', '0', '--z', '0.05',
                       '--roll', '0', '--pitch', '0', '--yaw', '0',
                       '--frame-id', 'base_link',
                       '--child-frame-id', 'camera_link'],
        ),



        Node(
            package='go2_nav',
            executable='object_pursuit_node',
            name='object_pursuit_node',
            output='screen',
            parameters=[{
                'target_class': 'person',
                'cloud_topic': cloud_topic,
            }]
        ),
    ])