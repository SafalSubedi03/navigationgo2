# Unitree Go2 Vision & Navigation Stack

This repository contains the complete autonomous vision-guided navigation and mapping stack deployed on the **Unitree Go2 Jetson platform**. By splitting workloads across dual Docker containers, the system isolates high-throughput, GPU-accelerated AI object tracking from low-latency ROS 2 navigation, SLAM, and low-level SDK robot locomotion.

---

## 1. System Overview

The stack integrates real-time object detection (YOLOv11 via TensorRT), LiDAR sensor fusion, 3D pinhole coordinate projection, frontier-based autonomous exploration, and RTAB-Map SLAM to enable the Unitree Go2 quadruped to explore environments autonomously, map them, and dynamically locate and navigate to objects of interest.

![System Architecture](images/system_architecture.png)
*Figure 1: High-level dual-container system architecture mapping vision and control workloads.*

### Core Capabilities:
- **Autonomous Mapping & SLAM**: Uses RTAB-Map SLAM and `explore_lite` for autonomous frontier exploration.
- **Deep-Learning Vision**: GPU-accelerated YOLOv11 tracker running in INT8 precision via TensorRT on the Jetson Orin NX.
- **Real-time 2D-to-3D Target Fusion**: Projects 3D LiDAR point clouds onto 2D camera images using pinhole calibration parameters, isolating points inside the YOLO bounding box to calculate the target's 3D centroid in the `map` coordinate frame.
- **Locomotion Control Bridge**: Direct bridge nodes communicating with the Unitree Go2 SDK `SportClient` API.

---

## 2. Directory and Source Code Structure

All active ROS 2 packages and configurations are located in the `/workspace/src` directory:

```text
/workspace/src/
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
│   │   ├── cameraInfoPublisher.py # Publishes CameraInfo parameters on /front_camera/camera_info
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
│   │   └── go2_front_calib.json     # Calibrated camera intrinsic parameters
│   ├── launch/
│   │   ├── explore_slam.launch.py   # Full SLAM + Nav2 + frontier exploration launch file
│   │   ├── slam_explore.launch.py   # RTAB-Map SLAM standalone launcher
│   │   ├── navigatio.launch.py      # Nav2 bringup launcher
│   │   ├── detectionnavigate.launch.py # Target follow-object navigation launch file
│   │   └── object_tracking.launch.py # Integrated launch file for transforms, camera info & tracker
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

## 3. Container & Build Isolation

Due to library and platform dependencies, runtime execution is split across two Docker containers running on the onboard Jetson Orin NX. Both containers mount the same source folder (`/workspace/src`), but build artifacts are completely isolated to prevent Python version and package prefix corruption.

| Container | Base OS & Python | HW Acceleration | Primary Role | Target Build Paths |
|---|---|---|---|---|
| **`go2vision`** | Ubuntu 20.04 <br> Python 3.8 | NVIDIA GPU (CUDA & TensorRT) | YOLOv11 deep-learning inference & target tracking | Build: `/vision_build/build` <br> Install: `/vision_build/install` |
| **`go2nav`** | Ubuntu 22.04 <br> Python 3.10 | Host Network (`eth0`) | SLAM, Nav2, sensor-fusion projection, robot locomotion | Build: `/nav_build/build` <br> Install: `/nav_build/install` |

### Build Instructions

1. **Inside `go2vision` Container**:
   ```bash
   cd /workspace
   colcon build --build-base /vision_build/build --install-base /vision_build/install
   ```

2. **Inside `go2nav` Container**:
   ```bash
   cd /workspace
   colcon build --build-base /nav_build/build --install-base /nav_build/install
   ```

---

## 4. Quick Start: Deploying the System

We use a tmux-based launch script (`start_go2.sh`) to automate and orchestrate the staggered startup of the dual-container stack.

```bash
# Execute the full stack launcher
bash start_go2.sh
```

### Tmux Workspace Layout:
The launcher creates a tmux session named `go2_stack` with the following window allocations:
* **`nav` (Window 0)**: Launches the core mapping/exploration stack: Nav2, RTAB-Map SLAM, and frontier exploration (`explore_lite`).
* **`camera` (Window 1)**: Captures raw images from the Unitree video RPC server, compresses them to JPEG, and publishes them.
* **`yolo` (Window 2)**: Runs the GPU-accelerated YOLO detector/tracker inside the `go2vision` container.
* **`pursuit` (Window 3)**: Publishes camera intrinsics, static TFs, and the sensor-fusion `object_pursuit_node`.
* **`scratch` (Window 4)**: Drops into an interactive bash shell in the `go2nav` container for quick topic echo or TF visualization checks.

To attach to the running session at any time:
```bash
tmux attach -t go2_stack
```

![Tmux Workspace Layout](images/tmux_workspace_layout.png)
*Figure 2: Tmux session workspace layout monitoring the running navigation and vision nodes.*

---

## 5. System Data Flow & Topic Architecture

Nodes communicate over the host-network interface. A diagram of the linked topics and services:

```mermaid
flowchart TD
    %% HW Layer
    subgraph HW ["Unitree Go2 Hardware Services"]
        CamSample["Camera Feed (video RPC)"]
        LiDARRaw["Lidar /utlidar/cloud_deskewed"]
        OdomRaw["Odom /utlidar/robot_odom"]
        MotionSDK["Unitree Sport SDK API"]
    end

    %% Container 1
    subgraph go2vision_container ["go2vision Container"]
        YoloNode["yolo_camera_node<br><i>go2_vision</i>"]
    end

    %% Container 2
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

    %% Node linkages
    CamSample -->|video RPC| CamAccess
    CamAccess -->|/go2/camera/compressed| YoloNode
    
    YoloNode -->|/yolo/detections<br>[Detection2DArray]| FObjectServer
    
    CamInfoPub -->|/front_camera/camera_info| FObjectServer
    
    LiDARRaw --> Restamp
    OdomRaw --> Restamp
    
    Restamp -->|/utlidar/robot_odom_restamped| OdomTF
    Restamp -->|/utlidar/cloud_deskewed_restamped| Laserscan
    Restamp -->|/utlidar/cloud_deskewed_restamped| SLAM
    Restamp -->|/utlidar/cloud_deskewed_restamped| FObjectServer
    
    OdomTF -->|TF: odom -> base_link| SLAM
    OdomTF -->|TF: odom -> base_link| FObjectServer
    
    SLAM -->|TF: map -> odom| Nav2
    SLAM -->|/map| ExploreLite
    
    Laserscan -->|/scan| Nav2
    
    ExploreLite -->|NavigateToPose Action Goal| Nav2
    
    FObjectClient -->|FollowObject Action Goal| FObjectServer
    FObjectServer -->|/goal_pose<br>[PoseStamped]| Nav2
    FObjectServer -->|Action Feedback| FObjectClient
    
    Nav2 -->|/cmd_vel_manual| SportBridge
    SportBridge -->|Move commands| MotionSDK
```

---

## 6. Documented "New Changes" & Optimizations

This version contains major consolidations and safety enhancements to the repository pipeline:

1. **Package Consolidation**:
   - The standalone `go2camerainfo` package was deprecated and removed. All functions and launch parameters were unified under `go2_nav` inside [cameraInfoPublisher.py](file:///home/safal/Desktop/unitreego2nav/src/go2_nav/go2_nav/cameraInfoPublisher.py) for structural simplicity and reduced package dependencies.
2. **Unified Pursuit Logic**:
   - Integrated the sensor-fusion math and mission state-control loops directly into [object_tracking_fusion.py](file:///home/safal/Desktop/unitreego2nav/src/go2_nav/go2_nav/object_tracking_fusion.py) as `object_pursuit_node` (executable: `object_pursuit_node`). This allows testing sensor-fusion projection limits directly without initiating an Action Server/Client sequence.
3. **Safety Control Constraints**:
   - Tightened velocity limits inside [moveapinode.py](file:///home/safal/Desktop/unitreego2nav/src/go2_nav/go2_nav/moveapinode.py) to prevent fast, erratic movements on hardware. Limits are constrained as follows:
     - `vxlimit` (Forward/Backward velocity): **0.25 m/s** (formerly 0.4 m/s)
     - `vylimit` (Lateral velocity): **0.25 m/s** (formerly 0.4 m/s)
     - `vyawlimit` (Rotation rate): **0.4 rad/s** (formerly 0.55 rad/s)
4. **Staggered TMUX Session Launcher**:
   - Refactored `start_go2.sh` to enforce workspace-directory mounts (`-w /workspace`), spin up containers if stopped (`docker start`), and apply generous execution delay offsets to ensure RTAB-Map SLAM and TF frames are fully initialized before sensor-fusion node starts tracking.
5. **Restamped Topic Alignment**:
   - Aligned point cloud callback subscriptions. In the sensor-fusion nodes, callbacks now bind cleanly to `/utlidar/cloud_deskewed_restamped` (type `sensor_msgs/msg/PointCloud2`) instead of `/utlidar/robot_odom_restamped` (which actually publishes `nav_msgs/msg/Odometry`), eliminating a topic type mismatch bug.

---

## 7. Critical Architectural Workarounds

To guarantee real-time performance and avoid hardware compatibility conflicts, the stack implements three architectural workarounds:

### 1. Clock Offset Realignment (`restamp_node`)
The onboard Unitree hardware publishes LiDAR and Odometry topics using its own internal hardware clock, which has a persistent **~126-second clock offset** relative to the Jetson Orin's system time. Because ROS 2 TF buffers and time synchronizers reject messages with large time gaps, `restamp_node` intercepts these topics and overwrites headers with the current ROS time stamp.

### 2. JPEG Bandwidth Compression
Raw camera frames (1920x1080) at 12.5 Hz require upwards of **77 MB/s** of sustained network bandwidth, causing massive network lag. The `cameraimg` node uses OpenCV compression (`JPEG_QUALITY = 80`) to drop packet payloads down to **100–300 KB** per frame, preserving real-time transmission speeds.

### 3. DDS Middleware Separation
The Unitree Python SDK relies on custom **FastRTPS** bindings. However, native ROS 2 humble packages default to **CycloneDDS**. Running both on a single node causes middleware translation conflicts.
- **Fix**: The custom hardware bridge nodes (`cameraimg` and `go2_sport_bridge`) are initialized with `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` environment variable overrides to talk to the physical robot SDK, while all other navigation nodes run on standard `rmw_cyclonedds_cpp`.

---

## 8. Visualization in RViz

To visualize mapping, localization, point clouds, and tracking in real-time, configure your host computer to connect to the robot's network and launch RViz 2.

```bash
# Run on your local PC
ros2 run rviz2 rviz2
```
Configure your display panels to subscribe to:
- `/map` (OccupancyGrid)
- `/utlidar/cloud_deskewed_restamped` (PointCloud2)
- `/yolo/annotated_image/compressed` (CompressedImage)
- `/goal_pose` (PoseStamped target tracker centroid)

![RViz Visualizer](images/rviz_visualization.png)
*Figure 3: RViz dashboard illustrating SLAM occupancy mapping, real-time lidar scanning, and projected goal targets.*
