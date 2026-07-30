from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():

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

    rtabmap_parameters = {
        'use_sim_time': use_sim_time,
        'frame_id':      'base_link',
        'odom_frame_id': 'odom',

        'subscribe_depth':  False,
        'subscribe_rgb':    False,
        'subscribe_stereo': False,
        'subscribe_odom':       False,
        'subscribe_scan':       False,
        'subscribe_scan_cloud': True,
        'wait_for_transform': 0.2,
        'approx_sync':          True,
        'sync_queue_size':      5,
        'topic_queue_size':     5,
        'qos_image': 2,
        'qos_scan_cloud': 2,
        'Reg/Strategy':   '1',
        'Grid/FromDepth': 'false',
        'Grid/RangeMax':  '20.0',

        'Icp/PM':                       'false',
        'Icp/VoxelSize':                '0.1',
        'Icp/MaxCorrespondenceDistance':'0.2',
        'Icp/PointToPlaneRadius':       '1.0',
        'Icp/PointToPlaneK':            '0',

        'RGBD/ProximityBySpace':          'true',
        'RGBD/ProximityPathMaxNeighbors': '10',

        'Reg/Force3DoF': 'false',
        'Grid/NormalsSegmentation': 'true',
        'Grid/MaxGroundHeight': '0.2',
        'Grid/MaxObstacleHeight': '1.5',
        'Grid/RangeMax':  '20.0',
    }

    remappings = [
        ('scan_cloud', scan_cloud_topic),
    ]

    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        output='screen',
        parameters=[rtabmap_parameters],
        remappings=remappings,
        arguments=['-d'],
    )

    rtabmap_viz_node = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        output='screen',
        parameters=[rtabmap_parameters],
        remappings=remappings,
    )

    odomtfBroadcaster = Node(
        package='go2_nav',
        executable='odom_broadcast',
        name='odom_tf_broadcaster',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    livox_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_livox_tf',
        output='screen',
        arguments=[
            '--x', '0.05', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0.17453', '--yaw', '0',
            '--frame-id', 'base_link', '--child-frame-id', 'livox_frame'
        ],
        condition=IfCondition(PythonExpression(["'", lidar_source, "' == 'livox'"])),
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_lidar_source,
        odomtfBroadcaster,
        livox_static_tf,
        # icp_odometry_node,
        rtabmap_node,
        rtabmap_viz_node,
    ])