# Go2 Nav2 — Autonomous Navigation for Unitree Go2

A ROS 2 Nav2 navigation stack implementation for the **Unitree Go2** quadruped robot, running on real hardware. This package provides a complete autonomous navigation pipeline including localization, costmap configuration, and velocity bridging to the Go2's high-level sport client API.

> ⚠️ **Work in Progress** — Tested on real hardware. Active development ongoing.

---

## Overview

This package enables fully autonomous point-to-point navigation on the Unitree Go2 using a pre-built map. All computation currently runs on an external laptop connected to the robot over Ethernet. The pipeline uses AMCL for localization, Nav2 for path planning and obstacle avoidance, and a custom node to bridge Nav2 velocity commands to the Go2's sport client API.

### Architecture

```
/utlidar/cloud_deskewed  →  pointcloud_to_laserscan  →  /scan  →  AMCL
/utlidar/robot_odom      →  odom_tf_broadcaster      →  TF (odom → base_link)
Nav2 (AMCL + Costmaps + Planner + Controller)  →  /cmd_vel  →  go2_sport_bridge  →  Go2 Sport Client
```

---

## Features

- Full Nav2 bringup with AMCL localization on real Go2 hardware
- Custom `odom_tf_broadcaster` node — converts `/utlidar/robot_odom` odometry topic to a TF broadcast (`odom → base_link`)
- Custom `go2_sport_bridge` node — redirects Nav2 velocity output to the Go2 high-level sport client move API
- 3D voxel layer costmaps using the Go2's native UtiLidar pointcloud
- Pre-configured `nav2_params.yaml` tuned for Go2's sensor topics and frame IDs

---

## Requirements

- ROS 2 Humble
- Unitree Go2 with CycloneDDS configured
- External laptop connected to Go2 over Ethernet
- The following ROS 2 packages:
  - `nav2_bringup`
  - `pointcloud_to_laserscan`
  - `tf2_ros`

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

## Mapping

Before running navigation you must have a map of your environment. This package uses RTAB-Map for mapping. Place your map files in the `maps/` directory:

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

### 2. Launch Navigation

In terminal 1, source the workspace and launch the navigation stack:

```bash
source install/setup.bash
ros2 launch go2_nav navigatio.launch.py
```

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

### 5. Navigate

1. Use the **2D Pose Estimate** tool in RViz to set the robot's initial position on the map — place it at the robot's actual physical starting location
2. Use the **2D Nav Goal** tool to set a goal position
3. The robot will plan a path and begin moving autonomously

> **Important:** The robot's initial pose must match the robot's actual position in the mapped environment. The map origin is defined by where you started mapping.

---

## Package Structure

```
go2_nav/
├── config/
│   └── nav2_params.yaml        # Nav2 stack configuration
├── launch/
│   └── navigatio.launch.py     # Main navigation launch file
├── maps/
│   ├── rtabmap.yaml            # Map metadata
│   └── rtabmap.pgm             # Occupancy grid
├── go2_nav/
│   ├── odomBroadcast.py        # odom → base_link TF broadcaster
│   └── moveapinode.py          # Nav2 → Go2 sport client bridge
├── package.xml
└── setup.py
```

---

## Roadmap

- [x] Navigation pipeline tested on real Go2 hardware
- [x] Custom odom TF broadcaster
- [x] Nav2 velocity to sport client bridge
- [ ] Frontier exploration for autonomous mapping
- [ ] Full indoor navigation testing
- [ ] Onboard computation — migrate from external laptop to Go2's internal computer
- [ ] Dynamic obstacle avoidance improvements

---

## Known Issues & Notes

- All computation currently runs on an external laptop. Onboard deployment is planned for a future release.
- The robot's initial pose in RViz must closely match the actual physical starting position for AMCL to localize correctly.
- Ensure CycloneDDS is configured correctly on both the laptop and the robot before launching.

---

## Contributing

Pull requests and issues are welcome. This is an ongoing project and contributions are appreciated.

---

## License

MIT License
Copyright (c) 2026 Safal