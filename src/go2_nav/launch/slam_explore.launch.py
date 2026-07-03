# Run standalone to test SLAM only:
#   ros2 launch go2_config slam_explore.launch.py

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
        ('scan_cloud', '/utlidar/cloud_deskewed_restamped'),
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
        parameters=[{'use_sim_time':use_sim_time}]
    )

    return LaunchDescription([
        declare_use_sim_time,
        odomtfBroadcaster,
        # icp_odometry_node,
        rtabmap_node,
        rtabmap_viz_node,
        
    ])