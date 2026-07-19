# Unitree Go2 Vision & Navigation Source Code Map

This directory (`/workspace/src/`) contains the source code packages deployed on the **Unitree Go2 Jetson platform**. Below is a summary of the source directory structure, package responsibilities, topic connections, and potential configuration issues.

---

## 1. Directory Tree & Package Roles

The source space is mapped as follows:

```text
/workspace/src/
├── follow_object_client.py   # Top-level action client test script (duplicating go2_nav/follow_object_client.py)
├── follow_object_server.py   # Top-level action server test script (duplicating go2_nav/follow_object_server.py)
│
├── go2_vision/               # GPU-accelerated YOLO Node Package (runs in go2vision container)
│   ├── go2_vision/
│   │   ├── __init__.py
│   │   └── yolo_node.py      # ROS 2 node running TensorRT YOLOv11 tracker
│   ├── package.xml
│   └── setup.py
│
├── go2_nav/                  # Control, Bridges, and Sensor Fusion Package (runs in go2nav container)
│   ├── go2_nav/
│   │   ├── __init__.py
│   │   ├── cameraInfoPublisher.py # Publishes CameraInfo parameters on /front_camera/camera_info (Integrated)
│   │   ├── cameraaccess.py   # Grabs video samples from Unitree SDK, compresses to JPEG, and publishes
│   │   ├── moveapicall.py    # Low-level movement calls using Unitree python bindings
│   │   ├── moveapinode.py    # Go2SportapiBridge node subscribing to /cmd_vel_manual
│   │   ├── odomBroadcast.py  # OdomTFBroadcaster publishing odom -> base_link
│   │   ├── restamp_node.py   # RestampNode correcting the 126s clock offset on lidar/odom topics
│   │   ├── follow_object_client.py  # Action client for FollowObject action (Mission Supervisor)
│   │   ├── follow_object_server.py  # Action server implementing depth projection & YOLO bounding box mapping
│   │   └── object_tracking_fusion.py # Unified tracking & depth-lifting sensor fusion node (Integrated)
│   ├── config/
│   │   ├── explore_lite_params.yaml # Frontier exploration parameters
│   │   ├── nav2_params.yaml         # Standalone navigation parameters
│   │   ├── nav2_slam_params.yaml    # Navigation with SLAM parameters
│   │   └── go2_front_calib.json     # Calibrated camera intrinsic parameters (Integrated)
│   ├── launch/
│   │   ├── explore_slam.launch.py   # Full SLAM + Nav2 + frontier exploration launch file
│   │   ├── slam_explore.launch.py   # RTAB-Map SLAM standalone launcher
│   │   ├── navigatio.launch.py      # Nav2 bringup launcher
│   │   ├── detectionnavigate.launch.py # Target follow-object navigation launch file
│   │   └── object_tracking.launch.py # Integrated launch file for transforms, camera info & tracker (Integrated)
│   ├── maps/                 # Map storage directory
│   ├── package.xml
│   └── setup.py
│
├── go2_vision_msgs/          # Custom Interfaces Package (Action and Message Definitions)
│   ├── action/
│   │   └── FollowObject.action      # ROS 2 action definition for object tracking missions
│   ├── CMakeLists.txt
│   └── package.xml
│
└── m-explore-ros2/           # Frontier-based Exploration Package (explore_lite)
    ├── explore/              # explore_lite package source directory
    ├── explore_lite_msgs/    # Messages for explore_lite
    └── map_merge/            # Multi-robot map merging utility
```

---

## 2. Topic and Action Data Flow (Linking)

The packages link together through topics, actions, and the TF frame tree:

```mermaid
flowchart TD
    %% Define Containers & Nodes
    subgraph HW ["Unitree Go2 Hardware Services"]
        CamSample["Camera Feed (video RPC)"]
        LiDARRaw["Lidar /utlidar/cloud_deskewed"]
        OdomRaw["Odom /utlidar/robot_odom"]
        MotionSDK["Unitree Sport SDK API"]
    end

    subgraph go2vision_container ["go2vision Container"]
        YoloNode["yolo_camera_node<br><i>go2_vision</i>"]
    end

    subgraph go2nav_container ["go2nav Container"]
        CamAccess["cameraimg<br><i>go2_nav</i>"]
        Restamp["restamp_node<br><i>go2_nav</i>"]
        OdomTF["odom_tf_broadcaster<br><i>go2_nav</i>"]
        CamInfoPub["camera_info_publisher<br><i>go2_nav</i>"]
        
        FObjectServer["follow_object_server<br><i>go2_nav</i>"]
        FObjectClient["mission_supervisor_node<br><i>go2_nav</i>"]
        
        SLAM["rtabmap<br><i>rtabmap_slam</i>"]
        Laserscan["pointcloud_to_laserscan<br><i>pointcloud_to_laserscan</i>"]
        ExploreLite["explore_node<br><i>explore_lite</i>"]
        Nav2["Nav2 Stack<br><i>nav2_bringup</i>"]
        SportBridge["go2_Sportapi_bridge<br><i>go2_nav</i>"]
    end

    %% Connections
    CamSample -->|video RPC| CamAccess
    CamAccess -->|/go2/camera/compressed| YoloNode
    
    YoloNode -->|/yolo/detections<br>[Detection2DArray]| FObjectServer
    YoloNode -.->|/yolo/annotated_image/compressed| RViz(["RViz Visualizer"])
    
    CamInfoPub -->|/front_camera/camera_info| FObjectServer
    
    LiDARRaw --> Restamp
    OdomRaw --> Restamp
    
    Restamp -->|/utlidar/robot_odom_restamped<br>[Odometry]| OdomTF
    Restamp -->|/utlidar/cloud_deskewed_restamped<br>[PointCloud2]| Laserscan
    Restamp -->|/utlidar/cloud_deskewed_restamped<br>[PointCloud2]| SLAM
    Restamp -->|/utlidar/cloud_deskewed_restamped<br>[PointCloud2]| FObjectServer
    
    OdomTF -->|TF: odom -> base_link| SLAM
    OdomTF -->|TF: odom -> base_link| FObjectServer
    
    SLAM -->|TF: map -> odom| Nav2
    SLAM -->|/map<br>[OccupancyGrid]| ExploreLite
    
    Laserscan -->|/scan| Nav2
    
    ExploreLite -->|NavigateToPose Action Goal| Nav2
    
    FObjectClient -->|FollowObject Action Goal| FObjectServer
    FObjectServer -->|/goal_pose<br>[PoseStamped]| Nav2
    FObjectServer -->|Action Feedback| FObjectClient
    
    Nav2 -->|/cmd_vel_manual| SportBridge
    SportBridge -->|Move commands| MotionSDK
```

---

## 3. Configuration & Compile Safeguards

1. **Compilation Paths**: Compile each package independently targeting its specific build directory to avoid polluting python/library prefix environments:
   * **`go2vision` Container**: Build using `--build-base /vision_build/build --install-base /vision_build/install`.
   * **`go2nav` Container**: Build using `--build-base /nav_build/build --install-base /nav_build/install`.
2. **Clock Offset Alignments**: Sensors `/utlidar/robot_odom` and `/utlidar/cloud_deskewed` are ~126s out of phase with system ROS time. The `restamp_node` must run to publish their `_restamped` equivalents.
3. **RMW Implementations**: SDK bridges (`cameraimg` and `go2_sport_bridge`) must run with `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` environment variable overlay since they directly talk to the hardware's DDS interface. Other ROS 2 nodes use the standard native `rmw_cyclonedds_cpp` setup.

---

## 4. Key Workspace Discrepancies (For Debugging)

> [!WARNING]
> **Synchronized Callback Topic Type Mismatch in `follow_object_server.py`**
> * **Code**: `follow_object_server.py` subscribes to `/utlidar/robot_odom_restamped` expecting a message of type `sensor_msgs/msg/PointCloud2`.
> * **Reality**: `restamp_node.py` publishes `/utlidar/robot_odom_restamped` as `nav_msgs/msg/Odometry`.
> * **Fix**: Change `follow_object_server.py` to subscribe to `/utlidar/cloud_deskewed_restamped` instead.

> [!IMPORTANT]
> **Wrong Package Namespaces in `go2_nav/setup.py` Entry Points**
> * **Code**: 
>   * `cameraimg = go2_vision.cameraaccess:main`
>   * `object_client = follow_object_client:main`
>   * `object_server= follow_object_server:main`
> * **Reality**: `cameraaccess.py`, `follow_object_client.py` and `follow_object_server.py` are all located inside the `go2_nav` python module directory.
> * **Fix**: The entry points should map to:
>   * `'cameraimg = go2_nav.cameraaccess:main'`
>   * `'object_client = go2_nav.follow_object_client:main'`
>   * `'object_server = go2_nav.follow_object_server:main'`
