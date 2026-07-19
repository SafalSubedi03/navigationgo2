from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # --- Static camera intrinsics ---
        Node(
            package='go2_nav',
            executable='camera_info_publisher',
            name='camera_info_publisher',
            output='screen',
            parameters=[{
                'frame_id': 'camera_link',
                'publish_rate_hz': 30.0,
            }]
        ),

        # --- Static TF: base_link -> camera_link ---
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_tf_broadcaster',
            arguments=['--x', '0.28', '--y', '0', '--z', '0.05',
                       '--roll', '0', '--pitch', '0', '--yaw', '0',
                       '--frame-id', 'base_link',
                       '--child-frame-id', 'camera_link'],
        ),

        # --- Static TF: base_link -> LiDAR ---
        # IMPORTANT: <REAL_LIDAR_FRAME> must exactly match the frame_id
        # actually present in the restamped point cloud's header. Find it with:
        #   ros2 topic echo /utlidar/cloud_deskewed_restamped --field header.frame_id --once
        # then replace the placeholder below before launching.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_tf_broadcaster',
            arguments=['--x', '0.28945', '--y', '0', '--z', '-0.046825',
                       '--roll', '0', '--pitch', '2.8782', '--yaw', '0',
                       '--frame-id', 'base_link',
                       '--child-frame-id', 'odom'],
        ),

        # --- Merged sensor-fusion + pursuit node ---
        Node(
            package='go2_nav',
            executable='object_pursuit_node',
            name='object_pursuit_node',
            output='screen',
            parameters=[{
                'target_class': 'person',
            }]
        ),
    ])