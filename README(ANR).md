#  Autonomous Navigation Robot — ROS 2

[![ROS 2 Humble](https://img.shields.io/badge/ROS_2-Humble-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.10+-3776AB?logo=python&logoColor=white)](https://www.python.org/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Classic_11-FF6600?logo=gazebo)](https://gazebosim.org/)
[![Nav2](https://img.shields.io/badge/Nav2-Humble-brightgreen)](https://nav2.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Build](https://img.shields.io/badge/build-passing-brightgreen)](.github/workflows/)

A complete, production-ready **autonomous self-driving robot** built on ROS 2 Humble. The robot maps an unknown environment using 2D LiDAR SLAM, localises itself on the saved map, and navigates autonomously to user-defined goals — all while a real-time safety layer prevents collisions with unexpected obstacles.

> **One command to run it all:**
> ```bash
> ros2 launch autonomous_nav_bot bringup.launch.py map:=/path/to/map.yaml
> ```

---

## Table of Contents

- [Features](#-features)
- [System Architecture](#-system-architecture)
- [Robot Specifications](#-robot-specifications)
- [Folder Structure](#-folder-structure)
- [Prerequisites](#-prerequisites)
- [Installation](#-installation)
- [Quick Start](#-quick-start)
- [Workflows](#-workflows)
  - [1 — Visualise the Robot Model](#workflow-1--visualise-the-robot-model)
  - [2 — Build a Map (SLAM)](#workflow-2--build-a-map-slam)
  - [3 — Autonomous Navigation](#workflow-3--autonomous-navigation)
  - [4 — Waypoint Following](#workflow-4--waypoint-following)
- [Configuration Guide](#-configuration-guide)
- [Topic Reference](#-topic-reference)
- [Troubleshooting](#-troubleshooting)
- [Contributing](#-contributing)
- [License](#-license)

---

##  Features

### Core Capabilities
- **Full 360° 2D LiDAR SLAM** — builds occupancy grid maps using SLAM Toolbox with loop closure correction
- **AMCL Localisation** — particle-filter localisation on saved maps, corrected by LiDAR scan matching
- **EKF Sensor Fusion** — fuses wheel odometry + IMU via Extended Kalman Filter for accurate odometry
- **Autonomous Navigation** — Nav2 stack with A* global planning and DWB local trajectory following
- **Waypoint Following** — ordered multi-goal navigation from topic or YAML file
- **Gazebo Simulation** — full physics simulation with realistic sensor noise models

### Sensor Processing Pipeline
- **LiDAR Processor** — circular moving-average smoothing, NaN/Inf replacement, range clamping
- **IMU Processor** — spike rejection, per-axis moving-average filtering
- **Camera Processor** — frame validation, QoS bridging (Gazebo BEST\_EFFORT → RELIABLE)

### Safety Architecture
- **3-Zone Emergency Stop** — DANGER (<20 cm, full stop) / CAUTION (<40 cm, 40% speed) / CLEAR
- **Dead-Man Switch** — automatic zero velocity if no command received for 500 ms
- **Hard Velocity Clamping** — independent of Nav2 config, enforced at the final output gate

### Developer Experience
- **Confidence Monitor** — AMCL quality score (0–1) with escalating warnings when localisation fails
- **Map Saver** — save finished maps via topic or service with timestamped filenames
- **5-Phase Bringup** — staggered startup ensures every component is ready before the next starts

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         SENSOR LAYER  (Step 2)                              │
│                                                                             │
│  Gazebo Plugins         ROS 2 Processors         Clean Topics               │
│  ─────────────         ────────────────         ────────────                │
│  LiDAR Ray Sensor  ──► lidar_processor    ──►  /scan/filtered               │
│  IMU Plugin        ──► imu_processor      ──►  /imu/filtered                │
│  Camera Plugin     ──► camera_processor   ──►  /camera/image_processed      │
└───────────────────────────────────┬─────────────────────────────────────────┘
                                    │
┌───────────────────────────────────┴─────────────────────────────────────────┐
│                      LOCALISATION LAYER  (Steps 3 & 4)                      │
│                                                                             │
│  /scan/filtered  ──► [slam_toolbox]   ──► /map  (SLAM mode)                 │
│                                                                             │
│  /map + /scan    ──► [AMCL]           ──► TF: map → odom                   │
│  /odom           ──► [EKF]            ──► /odometry/filtered                │
│  /imu/filtered   ──►    │                 TF: odom → base_footprint         │
│                         └─► [localization_monitor] ──► /localization/confidence │
└───────────────────────────────────┬─────────────────────────────────────────┘
                                    │
┌───────────────────────────────────┴─────────────────────────────────────────┐
│                       NAVIGATION LAYER  (Step 5)                            │
│                                                                             │
│  /goal_pose      ──► [goal_navigator]        ──► NavigateToPose action      │
│  /waypoints      ──► [waypoint_navigator]    ──► FollowWaypoints action     │
│                                                                             │
│  [bt_navigator]  orchestrates:                                              │
│    [planner_server]    A* global path ──────────────► /plan                 │
│    [controller_server] DWB local ctrl ──────────────► /cmd_vel_nav          │
│    [behavior_server]   spin/backup/wait (recovery)                          │
│  [velocity_smoother]   /cmd_vel_nav ──────────────► /cmd_vel_raw            │
└───────────────────────────────────┬─────────────────────────────────────────┘
                                    │
┌───────────────────────────────────┴─────────────────────────────────────────┐
│                       CONTROL LAYER  (Step 6)                               │
│                                                                             │
│  /scan/filtered  ──► [emergency_stop]  ──► /emergency_stop (Bool)           │
│                                             /control/zone  (String)         │
│                                                                             │
│  /cmd_vel_raw                                                               │
│  /emergency_stop ──► [velocity_controller] ──► /cmd_vel ──► robot wheels   │
│  /control/zone                                                              │
│                                                                             │
│  Safety gates (in order): dead-man switch → e-stop → hard clamp            │
└─────────────────────────────────────────────────────────────────────────────┘

TF Tree:
  map → odom → base_footprint → base_link → left_wheel_link
                                           → right_wheel_link
                                           → lidar_link
                                           → camera_link → camera_optical_link
                                           → imu_link
```

---

##  Robot Specifications

| Property | Value |
|---|---|
| Drive type | Differential drive |
| Chassis | 0.40 m × 0.30 m × 0.10 m |
| Drive wheels | 2 × radius 0.05 m, separation 0.34 m |
| Caster wheels | 2 × radius 0.025 m (front & rear) |
| Total mass | ~6.1 kg |
| Max linear speed | 0.26 m/s |
| Max angular speed | 1.82 rad/s |
| **LiDAR** | RPLIDAR A1 equivalent — 360°, 0.12–3.5 m, 10 Hz |
| **Camera** | RGB 640×480, 80° FOV, 30 fps |
| **IMU** | 6-DOF, 100 Hz, gyroscope + accelerometer |

---

##  Folder Structure

```
autonomous_nav_bot/
│
├── urdf/                          # Robot description (XACRO → URDF)
│   ├── robot.urdf.xacro           # Main file: chassis, wheels, sensor mounts
│   ├── materials.xacro            # RViz2 colour definitions
│   └── gazebo.xacro               # Gazebo plugins: diff-drive, LiDAR, camera, IMU
│
├── config/                        # All tunable parameters
│   ├── robot_description.yaml     # Physical dimensions reference
│   ├── sensors.yaml               # LiDAR / IMU / camera processor params
│   ├── slam_params.yaml           # SLAM Toolbox: 47 tuned parameters
│   ├── ekf_params.yaml            # EKF: 15×15 covariance matrices, sensor config
│   ├── localization_params.yaml   # AMCL particle filter + map_server
│   ├── nav2_params.yaml           # Full Nav2 stack: 9 nodes configured
│   └── control_params.yaml        # Safety zones, dead-man timeout, speed limits
│
├── src/
│   ├── sensor_integration/
│   │   ├── lidar_processor.py     # Circular smoothing, NaN/Inf filter, /scan/filtered
│   │   ├── imu_processor.py       # Spike rejection, moving average, /imu/filtered
│   │   └── camera_processor.py    # Frame validation, QoS bridge, /camera/image_processed
│   ├── mapping/
│   │   └── map_saver.py           # Saves .pgm + .yaml via topic or service
│   ├── localization/
│   │   └── localization_monitor.py # AMCL confidence [0–1], spread + covariance score
│   ├── navigation/
│   │   ├── goal_navigator.py      # /goal_pose → NavigateToPose action client
│   │   └── waypoint_follower.py   # /waypoints or YAML file → FollowWaypoints action
│   └── control/
│       ├── emergency_stop.py      # 3-zone LiDAR proximity monitor
│       └── velocity_controller.py # Dead-man + e-stop + clamp → /cmd_vel
│
├── launch/
│   ├── bringup.launch.py          # ★ MAIN: 5-phase full-system launcher
│   ├── mapping.launch.py          # SLAM mapping session (Gazebo + SLAM + RViz2)
│   ├── localization.launch.py     # Localisation only (AMCL + EKF + RViz2)
│   ├── navigation.launch.py       # Nav2 stack only
│   ├── sensors.launch.py          # Sensor processing nodes only
│   └── display.launch.py          # Robot model viewer (RViz2 + joint sliders)
│
├── rviz/
│   ├── navigation.rviz            # Costmaps + global path + Nav Goal tool
│   ├── mapping.rviz               # Top-down map growth + LiDAR overlay
│   ├── localization.rviz          # Particle cloud + covariance ellipse
│   └── robot_display.rviz         # Robot model + TF frames
│
├── maps/                          # Saved maps from SLAM (.pgm + .yaml)
├── tests/                         # Unit and integration tests
├── .github/workflows/             # CI/CD pipelines
├── package.xml                    # ROS 2 package manifest + all dependencies
└── CMakeLists.txt                 # Build configuration (ament_cmake)
```

---

##  Prerequisites

### System Requirements
- **OS**: Ubuntu 22.04 LTS
- **ROS 2**: Humble Hawksbill — [installation guide](https://docs.ros.org/en/humble/Installation.html)
- **Gazebo**: Classic 11 (ships with `ros-humble-gazebo-ros-pkgs`)
- **Python**: 3.10+

### ROS 2 Package Dependencies

All dependencies are declared in `package.xml` and can be installed with one `rosdep` command (see [Installation](#-installation)):

| Package | Used for |
|---|---|
| `slam_toolbox` | SLAM mapping (Step 3) |
| `nav2_bringup`, `nav2_msgs` | Autonomous navigation (Step 5) |
| `robot_localization` | EKF sensor fusion (Step 4) |
| `nav2_amcl` | Particle filter localisation (Step 4) |
| `nav2_map_server` | Map serving + map saving |
| `gazebo_ros`, `gazebo_plugins` | Physics simulation |
| `robot_state_publisher`, `xacro` | URDF/TF broadcasting |
| `rviz2` | 3D visualisation |
| `python3-numpy` | LiDAR and IMU signal processing |

---

##  Installation

### 1. Create a ROS 2 workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 2. Clone the repository

```bash
git clone https://github.com/<your-username>/autonomous_nav_bot.git
cd ~/ros2_ws
```

### 3. Install ROS 2 package dependencies

```bash
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

> **Note:** `rosdep` reads `package.xml` and installs everything automatically. If any package is not found via rosdep, install it manually:
>
> ```bash
> sudo apt install -y \
>   ros-humble-slam-toolbox \
>   ros-humble-nav2-bringup \
>   ros-humble-robot-localization \
>   ros-humble-nav2-amcl \
>   ros-humble-nav2-map-server \
>   ros-humble-gazebo-ros-pkgs \
>   ros-humble-joint-state-publisher-gui \
>   python3-numpy
> ```

### 4. Install teleop keyboard (for driving during mapping)

```bash
sudo apt install -y ros-humble-teleop-twist-keyboard
```

### 5. Build the package

```bash
cd ~/ros2_ws
colcon build --packages-select autonomous_nav_bot --symlink-install
```

> **Tip:** `--symlink-install` means you can edit Python scripts and YAML files without rebuilding.

### 6. Source the workspace

```bash
source ~/ros2_ws/install/setup.bash
```

> Add this to `~/.bashrc` so it runs on every terminal:
> ```bash
> echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
> ```

---

##  Quick Start

If you already have a saved map and want to navigate immediately:

```bash
ros2 launch autonomous_nav_bot bringup.launch.py \
  map:=~/ros2_ws/src/autonomous_nav_bot/maps/my_map.yaml
```

Then in RViz2:
1. Click **"2D Pose Estimate"** → click the robot's starting location on the map
2. Watch the green particle cloud converge
3. Click **"2D Nav Goal"** → click + drag to a destination

---

## Workflows

### Workflow 1 — Visualise the Robot Model

Verify the URDF is correct before running simulations:

```bash
ros2 launch autonomous_nav_bot display.launch.py
```

**What you'll see:**
- RViz2 opens with the full 3D robot model
- A slider window lets you rotate the wheels manually
- TF axes show every coordinate frame on the robot

---

### Workflow 2 — Build a Map (SLAM)

Run this once to map your environment. The saved map is used for all future navigation sessions.

#### Step 1 — Launch the mapping session

```bash
ros2 launch autonomous_nav_bot mapping.launch.py
```

Gazebo opens with the robot in an empty world. To use a custom world:

```bash
ros2 launch autonomous_nav_bot mapping.launch.py \
  world:=/path/to/your/world.sdf
```

#### Step 2 — Drive the robot

In a **new terminal**, start the teleop keyboard:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/cmd_vel
```

| Key | Action |
|---|---|
| `i` | Forward |
| `,` | Backward |
| `j` / `l` | Rotate left / right |
| `k` | Full stop |
| `u` / `o` | Diagonal forward-left / forward-right |

Drive the robot around all areas you want to include in the map. Watch the occupancy grid grow in RViz2. Grey cells = unexplored, white = free, black = walls.

#### Step 3 — Save the map

When the map looks complete:

```bash
# Save with a specific name
ros2 topic pub /save_map std_msgs/msg/String \
  "{data: 'office_floor_1'}" --once

# Or use auto-timestamp naming via service
ros2 service call /save_map_service std_srvs/srv/Trigger
```

Map files appear in the package `maps/` directory:
```
maps/
├── office_floor_1.pgm    # greyscale image (black=wall, white=free)
└── office_floor_1.yaml   # metadata (resolution, origin, thresholds)
```

---

### Workflow 3 — Autonomous Navigation

Requires a saved map from Workflow 2.

#### Launch the full system

```bash
ros2 launch autonomous_nav_bot bringup.launch.py \
  map:=$(ros2 pkg prefix autonomous_nav_bot)/share/autonomous_nav_bot/maps/office_floor_1.yaml
```

The 5-phase startup takes ~20 seconds. Watch the terminal for phase messages.

#### Set the initial pose

AMCL starts with particles spread across the entire map. Help it localise quickly:

1. In RViz2, click the **"2D Pose Estimate"** arrow button (top toolbar)
2. Click on the map where the robot physically is
3. Drag to set the robot's initial heading
4. Watch the particle cloud snap to that location and converge

Alternatively, from the command line:

```bash
ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  '{ header: {frame_id: "map"},
     pose: { pose: {
       position: {x: 1.0, y: 2.0, z: 0.0},
       orientation: {w: 1.0}
     }}}' --once
```

#### Send a navigation goal

**RViz2** (easiest):
1. Click the **"2D Nav Goal"** button (top toolbar)
2. Click on the map → drag to set the target heading
3. Watch the robot plan a path and drive to the goal

**Command line:**

```bash
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped \
  '{ header: {frame_id: "map"},
     pose: {
       position: {x: 3.0, y: 1.5, z: 0.0},
       orientation: {w: 1.0}
     }}' --once
```

**Cancel active navigation:**

```bash
ros2 topic pub /cancel_navigation std_msgs/msg/Bool '{data: true}' --once
```

#### Monitor navigation status

```bash
# Goal progress (distance remaining, recovery count)
ros2 topic echo /navigation/status

# AMCL localisation confidence (0.0 = lost, 1.0 = certain)
ros2 topic echo /localization/confidence

# Safety zone (CLEAR / CAUTION / DANGER)
ros2 topic echo /control/zone
```

---

### Workflow 4 — Waypoint Following

Navigate to a sequence of locations in order.

#### Via topic (runtime-defined waypoints)

```bash
ros2 topic pub /waypoints geometry_msgs/msg/PoseArray \
  '{ header: {frame_id: "map"},
     poses: [
       {position: {x: 1.0, y: 0.5, z: 0.0}, orientation: {w: 1.0}},
       {position: {x: 3.5, y: 2.0, z: 0.0}, orientation: {w: 0.707, z: 0.707}},
       {position: {x: 0.5, y: 3.5, z: 0.0}, orientation: {w: 0.0,  z: 1.0}},
       {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}
     ]}' --once
```

#### Via YAML file (repeatable routes)

Create a waypoints file, e.g. `config/patrol_route.yaml`:

```yaml
frame_id: map
waypoints:
  - {x: 1.0, y: 0.5, yaw: 0.0}
  - {x: 3.5, y: 2.0, yaw: 1.57}
  - {x: 0.5, y: 3.5, yaw: 3.14}
  - {x: 0.0, y: 0.0, yaw: 0.0}
```

```bash
ros2 run autonomous_nav_bot waypoint_follower.py \
  --ros-args -p waypoints_file:=/path/to/patrol_route.yaml
```

**Cancel waypoint route:**

```bash
ros2 topic pub /cancel_waypoints std_msgs/msg/Bool '{data: true}' --once
```

---

##  Configuration Guide

All parameters live in `config/`. Change them without rebuilding (thanks to `--symlink-install`).

| File | Configures | Key parameters to tune |
|---|---|---|
| `sensors.yaml` | Sensor processing | `range_min/max`, `smoothing_window`, `filter_window` |
| `slam_params.yaml` | SLAM mapping quality | `resolution`, `minimum_travel_distance`, `loop_match_minimum_response_fine` |
| `ekf_params.yaml` | Sensor fusion | `odom0_config`, `imu0_config`, covariance matrices |
| `localization_params.yaml` | AMCL particle filter | `min/max_particles`, `alpha1–4`, `laser_likelihood_max_dist` |
| `nav2_params.yaml` | Navigation behaviour | `max_vel_x`, `inflation_radius`, `xy_goal_tolerance`, DWB critic weights |
| `control_params.yaml` | Motor safety layer | `stop_distance`, `slow_distance`, `cmd_timeout_s` |

### Common tuning scenarios

**Robot drives too close to walls**
```yaml
# config/nav2_params.yaml → global_costmap and local_costmap
inflation_radius: 0.45   # increase from 0.35
```

**Navigation is too slow**
```yaml
# config/nav2_params.yaml → controller_server → FollowPath
max_vel_x: 0.26          # already at physical limit
sim_time: 1.2            # reduce from 1.7 for more responsive turning
```

**SLAM map is blurry**
```yaml
# config/slam_params.yaml
resolution: 0.03          # finer grid (default 0.05)
minimum_travel_distance: 0.3   # add nodes more frequently
```

**AMCL loses localisation frequently**
```yaml
# config/localization_params.yaml
max_particles: 5000       # more hypotheses (default 2000)
alpha1: 0.3               # increase motion model noise
```

**Emergency stop fires too eagerly**
```yaml
# config/control_params.yaml → emergency_stop
stop_distance: 0.15       # reduce from 0.20
min_danger_rays: 5        # require more rays to trigger
```

---

## 📡 Topic Reference

### Sensor topics

| Topic | Type | Published by | Consumed by |
|---|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | Gazebo | `lidar_processor` |
| `/scan/filtered` | `sensor_msgs/LaserScan` | `lidar_processor` | SLAM, AMCL, costmaps, `emergency_stop` |
| `/imu` | `sensor_msgs/Imu` | Gazebo | `imu_processor` |
| `/imu/filtered` | `sensor_msgs/Imu` | `imu_processor` | EKF |
| `/camera/image_raw` | `sensor_msgs/Image` | Gazebo | `camera_processor` |
| `/camera/image_processed` | `sensor_msgs/Image` | `camera_processor` | Vision pipeline |
| `/odom` | `nav_msgs/Odometry` | Gazebo diff-drive | EKF |
| `/odometry/filtered` | `nav_msgs/Odometry` | EKF | Nav2 |

### Navigation topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/goal_pose` | `geometry_msgs/PoseStamped` | → robot | Send a single goal |
| `/waypoints` | `geometry_msgs/PoseArray` | → robot | Send a waypoint route |
| `/cancel_navigation` | `std_msgs/Bool` | → robot | Cancel active goal |
| `/cancel_waypoints` | `std_msgs/Bool` | → robot | Cancel waypoint route |
| `/map` | `nav_msgs/OccupancyGrid` | ← robot | Loaded/built map |
| `/plan` | `nav_msgs/Path` | ← robot | Global planned path |
| `/save_map` | `std_msgs/String` | → robot | Trigger map save (SLAM mode) |

### Status and diagnostics

| Topic | Type | Description |
|---|---|---|
| `/navigation/status` | `std_msgs/String` | Goal navigator state + distance |
| `/navigation/active` | `std_msgs/Bool` | True while navigating |
| `/waypoint/status` | `std_msgs/String` | Waypoint follower progress |
| `/localization/confidence` | `std_msgs/Float32` | AMCL quality: 0.0–1.0 |
| `/localization/status` | `std_msgs/String` | Particle spread + covariance |
| `/emergency_stop` | `std_msgs/Bool` | True = full stop active |
| `/control/zone` | `std_msgs/String` | `CLEAR` / `CAUTION` / `DANGER` |
| `/control/status` | `std_msgs/String` | Velocity controller diagnostics |
| `/scan/stats` | `std_msgs/String` | LiDAR filter counters |
| `/imu/stats` | `std_msgs/String` | IMU spike rejection counters |

---

## 🔍 Troubleshooting

### Robot doesn't appear in RViz2

```bash
# Check if robot_state_publisher is broadcasting TF
ros2 topic echo /robot_description --no-arr | head -5

# Verify TF tree is complete
ros2 run tf2_tools view_frames
```

The generated `frames.pdf` should show:
`map → odom → base_footprint → base_link → [sensors]`

---

### AMCL particles don't converge

1. **Check the map is loaded:** `ros2 topic echo /map --no-arr`
2. **Check scan is arriving:** `ros2 topic hz /scan/filtered` (should be ~10 Hz)
3. **Set a pose hint:** use "2D Pose Estimate" in RViz2
4. **Check localisation confidence:** `ros2 topic echo /localization/confidence`

If confidence stays below 0.3 after 30 seconds, increase `max_particles` in `localization_params.yaml`.

---

### Nav2 goal is rejected or fails immediately

```bash
# Check Nav2 lifecycle state
ros2 lifecycle get /bt_navigator
# Should print: Active

# Check costmap is receiving scan
ros2 topic hz /global_costmap/costmap  # should be ~1 Hz

# Check TF map→odom exists
ros2 run tf2_ros tf2_echo map odom
```

If lifecycle is not `Active`, check that `lifecycle_manager_navigation` started without errors.

---

### Robot stops unexpectedly (emergency stop)

```bash
# Check which zone triggered it
ros2 topic echo /control/zone
ros2 topic echo /control/e_stop_status

# View minimum LiDAR distance in real time
ros2 topic echo /scan/filtered | grep range_min
```

If it fires spuriously in open space, increase `min_danger_rays` or `stop_distance` in `control_params.yaml`.

---

### "Waiting for transform" warnings in terminal

These usually mean a node started before the TF publishers it depends on. The bringup launch has built-in delays to prevent this, but if you're running individual launch files:

```bash
# Always start localisation before navigation
ros2 launch autonomous_nav_bot localization.launch.py map:=...
# Wait for "lifecycle_manager: Managed nodes are active" before running:
ros2 launch autonomous_nav_bot navigation.launch.py
```

---

### Gazebo physics behave oddly (robot spins or flies)

The spawned robot starts with all joints unlocked. If it behaves erratically:
1. Restart Gazebo — `Ctrl+C` then relaunch
2. Reduce `max_wheel_torque` in `urdf/gazebo.xacro` (currently 10 N·m)
3. Check that `wheel_separation` in the diff-drive plugin matches the URDF (both should be 0.34 m)

---

## 🛠 Development Notes

### Running the sensor unit tests

The processing logic for each sensor node is pure Python and doesn't require ROS 2:

```bash
cd ~/ros2_ws/src/autonomous_nav_bot

# LiDAR filter: NaN replacement, circular smoothing, range clamping
python3 -c "
import numpy as np
from src.sensor_integration.lidar_processor import LidarProcessor
# ... (see tests/ directory)
"
```

### Validating all config files

```bash
# XML validity (URDF/XACRO)
for f in urdf/*.xacro; do xmllint --noout \$f && echo OK: \$f; done

# YAML validity
for f in config/*.yaml; do python3 -c \"import yaml; yaml.safe_load(open('\$f'))\"; echo OK: \$f; done

# XACRO expansion
xacro urdf/robot.urdf.xacro > /tmp/expanded.urdf && echo "XACRO OK"
```

### Adding a new sensor

1. Add the URDF link + joint in `urdf/robot.urdf.xacro`
2. Add the Gazebo plugin in `urdf/gazebo.xacro`
3. Create `src/sensor_integration/<sensor>_processor.py` (follow the existing pattern)
4. Register it in `CMakeLists.txt` under `install(PROGRAMS ...)`
5. Add parameters to `config/sensors.yaml`
6. Add the node to `launch/sensors.launch.py` and `launch/bringup.launch.py`

---

##  Contributing

Contributions are welcome. Please follow these steps:

1. Fork the repository
2. Create a feature branch: `git checkout -b feature/my-new-sensor`
3. Make your changes with clear, well-commented code
4. Run validation: `colcon test --packages-select autonomous_nav_bot`
5. Submit a pull request with a description of what changed and why

### Code style
- Python: PEP 8, type hints encouraged, docstrings for all public methods
- YAML: comments explaining *why* each parameter is set to its value
- Launch files: a startup message for each phase/node group

---

##  License

This project is licensed under the **MIT License** — see the [LICENSE](LICENSE) file for details.

---

##  References

- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Nav2 Documentation](https://nav2.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [robot_localization](https://docs.ros.org/en/humble/p/robot_localization/)
- [Gazebo Classic](https://classic.gazebosim.org/tutorials)
- [URDF / XACRO Tutorial](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html)

---

<div align="center">
  Built with ROS 2 Humble · Gazebo · Nav2 · SLAM Toolbox
</div>
