# slam_explore_sim.launch.py
#
# Live SLAM mapping with RTAB-Map for simulation.
# No pre-saved map -- RTAB-Map builds /map from scratch as the robot moves.
#

# Run standalone to test SLAM only:
#   ros2 launch go2_config slam_explore_sim.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    rtabmap_parameters = {
        'use_sim_time': use_sim_time,
        'frame_id':      'base_link',
        'odom_frame_id': 'odom',      # Keeps reading TF from 'odom' -> 'base_link'

        # Camera fully disabled -- pure 3D LiDAR pipeline
        'subscribe_depth':  False,
        'subscribe_rgb':    False,
        'subscribe_stereo': False,
        
        # --- FIX 1: Turn off topic subscription, use TF instead ---
        'subscribe_odom':       False, 
        
        'subscribe_scan':       False,
        'subscribe_scan_cloud': True,
        'wait_for_transform': 0.2,
        # --- FIX 2: Since there's only 1 sensor topic now, approx_sync is for TF synchronization ---
        'approx_sync':          True,
        'sync_queue_size':      5,    # Give it room to match frames
        'topic_queue_size':     5,
        'qos_image': 2,          # BEST_EFFORT -- skip old buffered messages
        'qos_scan_cloud': 2,     # BEST_EFFORT for point cloud
        'Reg/Strategy':   '1',     # ICP-based registration
        'Grid/FromDepth': 'false', # 2D occupancy grid built from LiDAR, not depth
        'Grid/RangeMax':  '20.0',

        'Icp/PM':                       'false',
        'Icp/VoxelSize':                '0.1',
        'Icp/MaxCorrespondenceDistance':'0.2',
        'Icp/PointToPlaneRadius':       '1.0',
        'Icp/PointToPlaneK':            '0',  

        'RGBD/ProximityBySpace':          'true',
        'RGBD/ProximityPathMaxNeighbors': '10',

        'Reg/Force3DoF': 'false',
    }

    # --- FIX 4: Clean up remappings (Odom is handled via TF now) ---
    remappings = [
        ('scan_cloud', '/utlidar/cloud_deskewed_restamped'),
    ]

    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        output='screen',
        parameters=[rtabmap_parameters],
        remappings=remappings,
        arguments=['-d'],   # wipe rtabmap.db on every boot
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
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time,
        odomtfBroadcaster,
        # icp_odometry_node,
        rtabmap_node,
        rtabmap_viz_node,
        
    ])