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
        )
    ])