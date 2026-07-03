# Go2 Nav2 — Autonomous Navigation for Unitree Go2

A ROS 2 Nav2 navigation stack implementation for the **Unitree Go2** quadruped robot, running on real hardware. This package provides a complete autonomous navigation pipeline including localization, costmap configuration, frontier exploration, and velocity bridging to the Go2's high-level sport client API.

> ⚠️ **Work in Progress** — Tested on real hardware. Active development ongoing.

---

## Overview

This package enables fully autonomous point-to-point navigation on the Unitree Go2 using a pre-built map, as well as **autonomous frontier exploration** for building maps without a human driving the robot. All computation currently runs on an external laptop connected to the robot over Ethernet. The pipeline uses AMCL (or RTAB-Map SLAM, for exploration) for localization, Nav2 for path planning and obstacle avoidance, `explore_lite` for frontier detection, and a custom node to bridge Nav2 velocity commands to the Go2's sport client API.

### Architecture — Navigation (pre-built map)

```
/utlidar/cloud_deskewed  →  pointcloud_to_laserscan  →  /scan  →  AMCL
/utlidar/robot_odom      →  odom_tf_broadcaster      →  TF (odom → base_link)
Nav2 (AMCL + Costmaps + Planner + Controller)  →  /cmd_vel  →  go2_sport_bridge  →  Go2 Sport Client
```

### Architecture — Frontier Exploration (SLAM, no pre-built map)

```
/utlidar/robot_odom      →  odom_tf_broadcaster  →  TF (odom → base_link)
/utlidar/robot_odom, /utlidar/cloud_deskewed  →  restamp_node  →  restamped odom + lidar topics
restamped topics  →  RTAB-Map (SLAM)  →  map + global costmap
global costmap  →  explore_lite  →  frontier goal pose
Nav2 (planner)  →  /cmd_vel_manual  →  go2_sport_bridge  →  Go2 Sport Client
```

RTAB-Map builds the map and provides odometry/loop-closure entirely from LiDAR — **no camera is used for loop detection**. `explore_lite` detects frontier nodes from the occupancy grid and sends them to Nav2 as goals; Nav2 plans the path and publishes the resulting Twist to `/cmd_vel_manual`.

---

## Features

- Full Nav2 bringup with AMCL localization on real Go2 hardware
- **Frontier exploration** — tested and verified working on real hardware, using `explore_lite` + RTAB-Map SLAM
- Custom `odom_tf_broadcaster` node — converts `/utlidar/robot_odom` odometry topic to a TF broadcast (`odom → base_link`)
- Custom `restamp_node` — corrects a ~126s clock offset between the robot's internal clock and the laptop by restamping odometry and LiDAR data before they're used in the exploration pipeline
- Custom `go2_sport_bridge` node — redirects Nav2 velocity output to the Go2 high-level sport client move API
- 3D voxel layer costmaps using the Go2's native UtiLidar pointcloud
- Separate, tuned configuration files for navigation, SLAM, and `explore_lite`

---

## Requirements

- ROS 2 Humble
- Unitree Go2 with CycloneDDS configured
- External laptop connected to Go2 over Ethernet
- The following ROS 2 packages:
  - `nav2_bringup`
  - `pointcloud_to_laserscan`
  - `tf2_ros`
  - `rtabmap_ros`
  - `explore_lite`

---

## Branches

- **`navigation`** — the original, initial navigation pipeline (AMCL + pre-built map). This branch is frozen and won't receive further changes.
- **`frontier_exploration`** — active development branch containing the SLAM + frontier exploration pipeline described below.

---

## Installation

```bash
# Clone the repository
git clone <your-repo-url> ~/unitreego2nav
cd ~/unitreego2nav

# Build the package
colcon build --packages-select go2_nav
source install/setup.bash
```

---

## Mapping (Navigation branch only)

If you're using the `navigation` branch with a pre-built map, you'll need a map of your environment first. This package uses RTAB-Map for mapping. Place your map files in the `maps/` directory:

```
go2_nav/maps/
├── rtabmap.yaml   # Nav2-compatible map metadata
└── rtabmap.pgm    # Occupancy grid image
```

The `rtabmap.yaml` must follow Nav2 map server format:

```yaml
image: rtabmap.pgm
resolution: 0.05
origin: [x, y, 0.0]
negate: 0
occupied_thresh: 0.5
free_thresh: 0.196
```

Refer to the [RTAB-Map ROS 2 documentation](http://wiki.ros.org/rtabmap_ros) for mapping instructions.

> On the `frontier_exploration` branch, no pre-built map is needed — SLAM and mapping happen live during exploration.

---

## Usage

### 1. Connect to the Robot

Connect your laptop to the Go2 over Ethernet and verify the robot's ROS 2 topics are visible:

```bash
ros2 topic list
```

Verify the key sensor topics are publishing:

```bash
ros2 topic hz /utlidar/robot_odom
ros2 topic hz /utlidar/cloud_deskewed
```

You should see valid frequency output for both. If not, refer to the [official Unitree Go2 ROS 2 setup guide](https://github.com/unitreerobotics) to establish the connection.

### 2a. Launch Navigation (pre-built map)

In terminal 1, source the workspace and launch the navigation stack:

```bash
source install/setup.bash
ros2 launch go2_nav navigatio.launch.py
```

### 2b. Launch Frontier Exploration (SLAM, no pre-built map)

In terminal 1, source the workspace and launch the exploration stack, which brings up RTAB-Map SLAM together with `explore_lite`:

```bash
source install/setup.bash
ros2 launch go2_nav explore_slam.launch.py
```

> Note: the launch/config files are mid-rename — the `_sim` suffix is being dropped from all launch and config filenames, so you may still see `explore_slam_sim.launch.py` / `slam_explore_sim.launch.py` in the repo until that cleanup is finished.

### 3. Start the Sport Client Bridge

In terminal 2, run the velocity bridge node:

```bash
ros2 run go2_nav go2_sport_bridge
```

### 4. Visualize in RViz

In terminal 3, open RViz:

```bash
rviz2
```

Configure RViz:
- Set **Fixed Frame** to `map`
- Add `/map` topic (Global Costmap → Static Layer)
- Add `/global_costmap/costmap` for obstacle visualization
- Add `/plan` or `global_received_path` for path visualization
- For exploration, also add the frontier markers topic to view detected frontier nodes

### 5. Navigate

**Pre-built map:**
1. Use the **2D Pose Estimate** tool in RViz to set the robot's initial position on the map — place it at the robot's actual physical starting location
2. Use the **2D Nav Goal** tool to set a goal position
3. The robot will plan a path and begin moving autonomously

> **Important:** The robot's initial pose must match the robot's actual position in the mapped environment. The map origin is defined by where you started mapping.

**Frontier exploration:**
1. Launch `explore_slam.launch.py` and the `go2_sport_bridge` as above
2. The robot builds the map live via RTAB-Map while `explore_lite` continuously detects unexplored frontiers and sends them to Nav2 as goals
3. No manual goal-setting is required — the robot explores autonomously

---

## Package Structure

```
go2_nav/
├── config/
│   ├── nav2_params.yaml        # Nav2 stack configuration (navigation)
│   ├── slam_params.yaml        # RTAB-Map SLAM configuration (exploration)
│   └── explore_params.yaml     # explore_lite configuration
├── launch/
│   ├── navigatio.launch.py     # Main navigation launch file (pre-built map)
│   └── explore_slam.launch.py  # SLAM + frontier exploration launch file
├── maps/
│   ├── rtabmap.yaml            # Map metadata (navigation branch)
│   └── rtabmap.pgm             # Occupancy grid (navigation branch)
├── go2_nav/
│   ├── odomBroadcast.py        # odom → base_link TF broadcaster
│   ├── restamp_node.py         # corrects robot/laptop clock offset on odom + lidar data
│   └── moveapinode.py          # Nav2 → Go2 sport client bridge
├── package.xml
└── setup.py
```

---

## Roadmap

- [x] Navigation pipeline tested on real Go2 hardware
- [x] Custom odom TF broadcaster
- [x] Nav2 velocity to sport client bridge
- [x] Frontier exploration — tested and verified on real hardware
- [x] Timestamp offset correction between robot and laptop clocks (`restamp_node`)
- [ ] Full indoor navigation testing
- [ ] Onboard computation — migrate from external laptop to Go2's internal computer
- [ ] Dynamic obstacle avoidance improvements
- [ ] Finish renaming launch/config files to drop the `_sim` suffix

---

## Known Issues & Notes

- All computation currently runs on an external laptop. Onboard deployment is planned for a future release.
- The robot's initial pose in RViz must closely match the actual physical starting position for AMCL to localize correctly (navigation branch).
- Ensure CycloneDDS is configured correctly on both the laptop and the robot before launching.
- The robot's internal clock and the laptop clock have been observed to drift ~126s out of phase; `restamp_node` corrects this for the exploration pipeline before odometry/LiDAR data is consumed downstream.
- Launch and config filenames containing `_sim` are being renamed — expect some inconsistency until that cleanup lands.

---

## Contributing

Pull requests and issues are welcome. This is an ongoing project and contributions are appreciated.

---

## License

MIT License
Copyright (c) 2026 Safal
