from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='go2_camera_info',
            executable='camera_info_publisher',
            name='camera_info_publisher',
            output='screen',
            parameters=[{
                'frame_id': 'camera_link',
                'publish_rate_hz': 30.0,
                # 'calib_path': '/custom/path/if/you/want/to/override.json',
            }]
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
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_tf_broadcaster',
            arguments=['--x', '0.28945', '--y', '0', '--z', '-0.046825',
                       '--roll', '0', '--pitch', '2.8782', '--yaw', '0',
                       '--frame-id', 'base_link',
                       '--child-frame-id', '<REAL_LIDAR_FRAME>'],  # fill this in
        )
    ])