# slam_explore_sim.launch.py
#
# Live SLAM mapping with RTAB-Map for simulation.
# No pre-saved map -- RTAB-Map builds /map from scratch as the robot moves.
#
# Topics:
#   IN:  /velodyne_points (sensor_msgs/PointCloud2) -- Gazebo LiDAR plugin
#   OUT: /rtabmap/odom    (nav_msgs/Odometry)        -- ICP-computed odometry
#   OUT: /map             (nav_msgs/OccupancyGrid)   -- live growing map
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
        'odom_frame_id': 'odom',

        # Camera fully disabled -- pure 3D LiDAR pipeline
        'subscribe_depth':  False,
        'subscribe_rgb':    False,
        'subscribe_stereo': False,

        'subscribe_scan':       False,
        'subscribe_scan_cloud': True,

        'approx_sync': True,

        'Reg/Strategy':   '1',     # ICP-based registration
        'Grid/FromDepth': 'false', # 2D occupancy grid built from LiDAR, not depth
        'Grid/RangeMax':  '20.0',

        'Icp/PM':                       'false',
        'Icp/VoxelSize':                '0.1',
        'Icp/MaxCorrespondenceDistance':'0.2',
        'Icp/PointToPlaneRadius':       '1.0',
        'Icp/PointToPlaneK':            '0',  

        'guess_frame_id':        'odom',
        'guess_min_translation': '0.0',

        'RGBD/ProximityBySpace':          'true',
        'RGBD/ProximityPathMaxNeighbors': '10',

        'Reg/Force3DoF': 'false',
    }

    remappings = [
        ('scan_cloud', '/utlidar/cloud_deskewed'),
        ('odom',       '/utlidar/robot_odom'),
        # ('guess',      '/odom'), 
    ]

    icp_odometry_node = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        output='screen',
        parameters=[rtabmap_parameters],
        remappings=remappings,
    )

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

    return LaunchDescription([
        declare_use_sim_time,
        icp_odometry_node,
        rtabmap_node,
        rtabmap_viz_node,
    ])