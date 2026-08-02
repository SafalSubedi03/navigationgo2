# Unitree Go2 Autonomous Vision & Navigation Stack

An end-to-end, GPU-accelerated autonomous vision-guided navigation and mapping system deployed on the **Unitree Go2 quadruped platform** powered by an onboard **NVIDIA Jetson Orin NX**.

The system combines real-time deep-learning object detection and tracking (**YOLOv11** via **TensorRT**), 2D-to-3D point cloud coordinate lifting, **RTAB-Map SLAM**, **Nav2** navigation, and **`explore_lite`** autonomous frontier exploration into a dual-container architecture.

---

## 1. System Overview & Core Capabilities

The stack allows the Unitree Go2 quadruped to map unknown environments, autonomously explore unmapped frontiers, and detect, track, and navigate to target objects of interest (e.g., `person`, `ball`, custom classes) without interrupting SLAM mapping state or navigation stability.

```mermaid
flowchart TD
    subgraph Hardware ["Unitree Go2 Jetson Platform & Sensors"]
        CamRPC["Unitree Video RPC"]
        UnitreeLiDAR["Unitree LiDAR (/utlidar/cloud_deskewed)"]
        LivoxLiDAR["Livox Mid-360 LiDAR (/livox/lidar)"]
        RobotSDK["Unitree Sport SDK API"]
    end

    subgraph Container_Vision ["go2vision Container (Ubuntu 20.04 | CUDA & TensorRT | Python 3.8)"]
        YoloDetector["yolo_detector<br>(yolo_node.py)"]
    end

    subgraph Container_Nav ["go2nav Container (Ubuntu 22.04 | ROS 2 Humble | Python 3.10)"]
        CamBridge["cameraimg<br>(cameraaccess.py)"]
        CamInfo["camera_info_publisher<br>(cameraInfoPublisher.py)"]
        Restamp["restamp_node<br>(restamp_node.py)"]
        OdomTF["odom_tf_broadcaster<br>(odomBroadcast.py)"]
        
        SLAM["rtabmap_slam<br>(RTAB-Map)"]
        Nav2Stack["Nav2 Navigation Stack"]
        ExploreLite["explore_node<br>(m-explore-ros2)"]
        
        FusionNode["object_pursuit_node<br>(object_tracking_fusion.py)"]
        SportBridge["go2_sport_bridge<br>(moveapinode.py)"]
    end

    %% Data Connections
    CamRPC -->|Video Samples| CamBridge
    CamBridge -->|/go2/camera/compressed| YoloDetector
    
    YoloDetector -->|/yolo/detections<br>[Detection2DArray]| FusionNode
    CamInfo -->|/front_camera/camera_info| FusionNode
    
    UnitreeLiDAR -->|Clock Skewed Cloud| Restamp
    Restamp -->|/utlidar/cloud_deskewed_restamped| SLAM
    Restamp -->|/utlidar/cloud_deskewed_restamped| FusionNode
    LivoxLiDAR -.->|Optional External Cloud| SLAM
    LivoxLiDAR -.->|Optional External Cloud| FusionNode
    
    Restamp -->|/utlidar/robot_odom_restamped| OdomTF
    OdomTF -->|TF: odom -> base_link| SLAM
    OdomTF -->|TF: odom -> base_link| FusionNode
    
    SLAM -->|TF: map -> odom| Nav2Stack
    SLAM -->|/map OccupancyGrid| ExploreLite
    
    ExploreLite -->|Frontier Goals| Nav2Stack
    FusionNode -->|/goal_pose Target Centroid| Nav2Stack
    
    Nav2Stack -->|/cmd_vel_manual| SportBridge
    SportBridge -->|Low-level Motion Calls| RobotSDK
```

### Key Technical Features
- **Dual-Container Isolation**: Keeps high-throughput CUDA/TensorRT deep-learning runtimes isolated from ROS 2 Humble navigation libraries, preventing Python and package dependency corruption.
- **Real-Time Deep-Learning Tracking**: TensorRT INT8-optimized YOLOv11 detector paired with a BoT-SORT multi-object tracker processing camera feeds in real-time.
- **2D-to-3D Target Fusion**: Projects 3D LiDAR point clouds into 2D bounding boxes using calibrated pinhole intrinsics ($f_x, f_y, c_x, c_y$) and publishes target 3D centroids directly to `/goal_pose` in the `map` coordinate frame.
- **Multi-LiDAR Hardware Support**: Native support for both internal Unitree onboard LiDAR (`/utlidar/cloud_deskewed_restamped`) and external **Livox Mid-360** LiDAR (`/livox/lidar` via `livox_ros_driver2`).
- **Autonomous Mapping & Frontier Exploration**: Integrated RTAB-Map 3D/2D SLAM paired with `explore_lite` for automatic environment coverage.
- **Locomotion Control Bridge**: Direct integration with the Unitree Go2 SDK `SportClient` API, featuring safety-clamped velocity constraints.

---

## 2. Repository Directory Structure

All active ROS 2 source packages and configuration scripts reside in the `/workspace/src` directory:

```text
unitreego2nav/
├── setup_nav.sh                # Automated setup script for go2nav container
├── setup_vision.sh             # Automated setup script for go2vision container
├── start_go2.sh                # TMUX master orchestrator launcher script
├── ARCHITECTURE.md             # Detailed system architecture document
├── README.md                   # System user guide & documentation
├── objective.md                # System design & vision specification reference
├── sdk/                        # Unitree SDK Python bindings repository
│   └── unitree_sdk2_python/
├── src/                        # Main ROS 2 Workspace Source
│   ├── go2_nav/                # Navigation, Control, Sensor-Fusion & Bridges
│   │   ├── config/
│   │   │   ├── explore_lite_params.yaml  # Frontier exploration configuration
│   │   │   ├── nav2_params.yaml          # Standalone Nav2 parameters
│   │   │   ├── nav2_slam_params.yaml     # Nav2 + SLAM integration parameters
│   │   │   └── go2_front_calib.json      # Pinhole camera intrinsic parameters
│   │   ├── go2_nav/
│   │   │   ├── cameraaccess.py           # Camera bridge (RPC capture & JPEG compression)
│   │   │   ├── cameraInfoPublisher.py    # Static CameraInfo publisher
│   │   │   ├── moveapicall.py            # Low-level Unitree Python SDK wrappers
│   │   │   ├── moveapinode.py            # Locomotion bridge (subscribes to /cmd_vel_manual)
│   │   │   ├── odomBroadcast.py          # TF broadcaster (odom -> base_link)
│   │   │   ├── restamp_node.py           # Hardware clock offset correction node
│   │   │   ├── follow_object_client.py   # Action client for FollowObject mission control
│   │   │   ├── follow_object_server.py   # Action server implementing 3D target projection
│   │   │   └── object_tracking_fusion.py # Core integrated 2D-to-3D sensor fusion node
│   │   ├── launch/
│   │   │   ├── explore_slam.launch.py    # Full SLAM + Nav2 + frontier exploration launch
│   │   │   ├── slam_explore.launch.py    # RTAB-Map SLAM & odometry launcher
│   │   │   ├── navigatio.launch.py       # Nav2 stack bringup launcher
│   │   │   ├── detectionnavigate.launch.py # Action-based pursuit launcher
│   │   │   └── object_tracking.launch.py # Integrated pursuit & TF launcher
│   │   ├── package.xml
│   │   └── setup.py
│   ├── go2_vision/             # GPU Deep Learning Detection Package
│   │   ├── config/
│   │   │   └── custombotsort.yaml        # BoT-SORT multi-object tracker settings
│   │   ├── go2_vision/
│   │   │   └── yolo_node.py              # TensorRT YOLOv11 detection & tracking node
│   │   ├── package.xml
│   │   └── setup.py
│   ├── go2_vision_msgs/        # Custom ROS 2 Interface Definitions
│   │   ├── action/
│   │   │   └── FollowObject.action       # Mission control action interface
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   └── m-explore-ros2/         # Frontier Exploration Package (explore_lite)
│       ├── explore/            # Autonomous frontier finder
│       └── map_merge/          # Multi-robot map merging utilities
```

---

## 3. Container Topology & Build Instructions

To isolate GPU libraries (CUDA, TensorRT, PyTorch) from ROS 2 Humble control packages, runtime execution is divided into two separate Docker containers mounting the shared workspace directory `/workspace/src`.

| Container | Operating System & Python | Hardware Runtime | Primary Purpose | Build Output Path |
|---|---|---|---|---|
| **`go2vision`** | Ubuntu 20.04 (Focal)<br>Python 3.8 | NVIDIA GPU (`--runtime nvidia`) | YOLOv11 TensorRT inference & BoT-SORT tracking | `/vision_build/build`<br>`/vision_build/install` |
| **`go2nav`** | Ubuntu 22.04 (Jammy)<br>Python 3.10 | Host Network (`eth0`) | SLAM, Nav2, sensor-fusion projection, locomotion | `/nav_build/build`<br>`/nav_build/install` |

> [!IMPORTANT]
> **Build Target Isolation**: Each container MUST use its dedicated build directory (`/vision_build` for `go2vision` and `/nav_build` for `go2nav`). Do not build into `/workspace/install`, as conflicting Python versions will corrupt shared dependencies.

### Initial Setup & Compilation

#### 1. Setup Vision Container (`go2vision`)
Inside the `go2vision` container terminal:
```bash
cd /workspace
bash setup_vision.sh
```
This script validates GPU access, installs PyTorch (NVIDIA Jetson wheel `torch-2.0.0+nv23.05`), compiles `torchvision` (cached in `/workspace/wheels`), installs Ultralytics, and builds the workspace into `/vision_build`.

#### 2. Setup Navigation Container (`go2nav`)
Inside the `go2nav` container terminal:
```bash
cd /workspace
bash setup_nav.sh
```
This script installs Nav2, RTAB-Map, builds CycloneDDS 0.10.2 C library, configures Python bindings, clones the Unitree SDK, and builds the workspace into `/nav_build`.

---

## 4. Quick Start: Deploying the System

System deployment is automated using [start_go2.sh](file:///home/safal/Desktop/unitreego2nav/start_go2.sh), which launches a background **TMUX** session named `go2_stack` with staggered execution delays.

### Execute Master Launcher
On the host Jetson platform:
```bash
bash start_go2.sh
```

To re-attach to the running TMUX workspace at any time:
```bash
tmux attach -t go2_stack
```

### TMUX Window Layout Allocation

The launcher orchestrates system startup across 6 TMUX windows:

```text
go2_stack (TMUX Session)
├── Window 0 [nav]      : Launches explore_slam.launch.py (RTAB-Map SLAM + Nav2)
├── Window 1 [camera]   : Runs cameraimg bridge (Unitree RPC capture & JPEG compression)
├── Window 2 [yolo]     : Runs yolo_detector in go2vision container (TensorRT tracking)
├── Window 3 [pursuit]  : Launches object_tracking.launch.py (camera intrinsics + fusion)
├── Window 4 [scratch]  : Interactive bash prompt inside go2nav container for debugging
└── Window 5 [livox]    : Launches Livox Mid-360 driver (livox_ros_driver2)
```

To navigate between TMUX windows, use `Ctrl+b` followed by the window index number (0-5).

---

## 5. System Data Flow & Topic Architecture

### ROS 2 Communication Matrix

| Topic Name | Message Type | Publisher Node | Subscriber Node | Function / Description |
|---|---|---|---|---|
| `/go2/camera/compressed` | `sensor_msgs/msg/CompressedImage` | `cameraimg` | `yolo_detector` | JPEG compressed camera frames (12.5 Hz, Quality=80) |
| `/front_camera/camera_info` | `sensor_msgs/msg/CameraInfo` | `camera_info_publisher` | `object_pursuit_node` | Calibrated pinhole camera intrinsic parameters |
| `/yolo/detections` | `vision_msgs/msg/Detection2DArray` | `yolo_detector` | `object_pursuit_node` | 2D bounding boxes, class labels, and confidence scores |
| `/yolo/annotated_image/compressed` | `sensor_msgs/msg/CompressedImage` | `yolo_detector` | RViz / User | Real-time tracking feed with drawn bounding boxes |
| `/utlidar/cloud_deskewed` | `sensor_msgs/msg/PointCloud2` | Unitree LiDAR Hardware | `restamp_node` | Raw 3D point cloud from onboard Unitree LiDAR |
| `/utlidar/cloud_deskewed_restamped` | `sensor_msgs/msg/PointCloud2` | `restamp_node` | SLAM, Fusion | Point cloud with corrected Jetson ROS system timestamp |
| `/livox/lidar` | `sensor_msgs/msg/PointCloud2` | `livox_ros_driver2` | SLAM, Fusion | Point cloud feed from external Livox Mid-360 LiDAR |
| `/utlidar/robot_odom_restamped` | `nav_msgs/msg/Odometry` | `restamp_node` | `odom_tf_broadcaster` | Restamped robot odometry for TF tree generation |
| `/map` | `nav_msgs/msg/OccupancyGrid` | `rtabmap_slam` | `explore_node`, Nav2 | 2D grid map built live during SLAM |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | `object_pursuit_node` | Nav2 (`bt_navigator`) | Projected 3D target centroid in map frame |
| `/cmd_vel_manual` | `geometry_msgs/msg/Twist` | Nav2 / Pursuit Node | `go2_sport_bridge` | Locomotion commands passed to Unitree SDK |
| `/go2/select_target_class` | `std_msgs/msg/String` | User / Script | `yolo_detector`, Fusion | Dynamic target class switching (e.g. `person`) |

---

## 6. Sensor Configurations: Internal vs. External LiDAR

The system provides flexible launch parameters to switch between internal Unitree LiDAR and external Livox Mid-360 LiDAR setups.

### 1. Internal Unitree LiDAR Setup
Default mode. Uses `/utlidar/cloud_deskewed_restamped` provided by [restamp_node.py](file:///home/safal/Desktop/unitreego2nav/src/go2_nav/go2_nav/restamp_node.py).
```bash
ros2 launch go2_nav explore_slam.launch.py lidar_source:=unitree
```

### 2. External Livox Mid-360 LiDAR Setup
Configured via `lidar_source:=livox`. Automatically activates static TF broadcast `base_link` $\rightarrow$ `livox_frame` and maps point clouds from `/livox/lidar`.
```bash
ros2 launch go2_nav explore_slam.launch.py lidar_source:=livox
```
*Note*: Requires `livox_ros_driver2` running in `livox_ws` (handled automatically by Window 5 in `start_go2.sh`).

---

## 7. Critical Architectural Workarounds & Safeguards

### 1. Hardware Clock Offset Realignment (`restamp_node`)
> [!WARNING]
> The onboard Unitree hardware processor uses an un-synchronized internal clock with a **~126-second offset** relative to the Jetson Orin system clock.
> 
> Without intervention, ROS 2 TF buffer lookups and message synchronizers immediately fail with time out errors. [restamp_node.py](file:///home/safal/Desktop/unitreego2nav/src/go2_nav/go2_nav/restamp_node.py) intercepts `/utlidar/cloud_deskewed` and `/utlidar/robot_odom`, replaces their headers with `node.get_clock().now()`, and publishes clean `_restamped` topics.

### 2. Network Bandwidth Compression (`cameraimg`)
Uncompressed 1080p frames at 12.5 Hz require upwards of **77 MB/s** network bandwidth. [cameraaccess.py](file:///home/safal/Desktop/unitreego2nav/src/go2_nav/go2_nav/cameraaccess.py) compresses video frames to JPEG format (`JPEG_QUALITY = 80`), reducing network bandwidth consumption to **100–300 KB/s** while maintaining sub-second latency.

### 3. DDS Middleware Coexistence (`FastRTPS` vs `CycloneDDS`)
The Unitree Python SDK bindings require `FastRTPS` (`rmw_fastrtps_cpp`), whereas native ROS 2 Humble navigation packages rely on `CycloneDDS` (`rmw_cyclonedds_cpp`). Hardware bridge nodes (`cameraimg` and `go2_sport_bridge`) explicitly set environment overrides:
```bash
RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 run go2_nav go2_sport_bridge
```

---

## 8. Visualization in RViz

To visualize real-time mapping, point clouds, detection bounding boxes, and goal poses, launch RViz 2 on a remote workstation connected to the Unitree Go2 network:

```bash
ros2 run rviz2 rviz2
```

### Display Panel Configuration
Add the following displays to your RViz setup:
- **Map**: Topic `/map` (`nav_msgs/msg/OccupancyGrid`)
- **LiDAR Point Cloud**: Topic `/utlidar/cloud_deskewed_restamped` or `/livox/lidar` (`sensor_msgs/msg/PointCloud2`)
- **YOLO Annotated Camera Stream**: Topic `/yolo/annotated_image/compressed` (`sensor_msgs/msg/CompressedImage`)
- **Target Tracking Goal**: Topic `/goal_pose` (`geometry_msgs/msg/PoseStamped`)
- **TF Tree**: Enable frames `map`, `odom`, `base_link`, `camera_link`, `livox_frame`.

---

## 9. License & Maintenance

- **Maintainer**: Safal Subedi (080bei034.safal@pcampus.edu.np)
- **Repository**: Unitree Go2 Autonomous Vision & Navigation Stack
