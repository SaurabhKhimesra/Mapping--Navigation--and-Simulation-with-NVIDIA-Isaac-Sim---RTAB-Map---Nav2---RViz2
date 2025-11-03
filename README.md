# Mapping, Navigation, and Simulation with NVIDIA Isaac Sim + RTAB-Map + Nav2 + RViz2

End-to-end demo of autonomous navigation using **NVIDIA Isaac Sim** for simulation, **RTAB-Map** for SLAM, **Navigation2 (Nav2)** for planning & control, and **RViz2** for visualization.

[🎥 **Demo Video**](https://youtu.be/MkWIAPUYG_Y)  
[![IMAGE](images/image2.png)](https://youtu.be/NRXYgfIl9uc)

---

## Table of Contents
- [Overview](#overview)
- [Key Components](#key-components)
- [Architecture (High Level)](#architecture-high-level)
- [Tested Environment](#tested-environment)
- [Prerequisites](#prerequisites)
- [Install & Build](#install--build)
- [Quick Start](#quick-start)
- [Run Components Individually](#run-components-individually)
- [Topic & Frame Conventions](#topic--frame-conventions)
- [Project Layout](#project-layout)
- [Configuration Snippets](#configuration-snippets)
- [Troubleshooting](#troubleshooting)
- [Resources](#resources)
- [Contributing](#contributing)

---

## Overview
This package wires up a simulated mobile robot in **NVIDIA Isaac Sim** with **RTAB-Map** SLAM and **Navigation2 (Nav2)**, while **RViz2** provides live visualization. It’s meant for rapid experimentation with mapping, loop closures, localization, and autonomous navigation behaviors in a realistic physics-based virtual world.

---

## Key Components

1. **Isaac Sim**
   - Physically based robotics simulation with ROS 2 bridges for sensors, TF, and control.
   - Great for testing perception and navigation safely before real deployment.

2. **RTAB-Map**
   - Real-time appearance-based SLAM (2D/3D) with RGB-D, LiDAR, or stereo inputs.
   - Supports loop-closure detection, graph optimization, and memory management for large maps.

3. **Navigation2 (Nav2)**
   - Full navigation stack for path planning, obstacle avoidance, and recovery behaviors.
   - Works with SLAM (online) or AMCL (map-based) localization; integrates with global/local costmaps.

4. **RViz2**
   - Visualizes robot state, sensor data, TF tree, maps, and navigation goals.
   - Essential for debugging and verifying SLAM/localization/planning in real time.

---

## Architecture (High Level)

```
Isaac Sim (USD world, sensors) ──ROS 2 Bridge──>  /scan, /camera, /tf, /clock
           │
           └──────────────>  /cmd_vel  <──── Nav2 (planner, controller, costmaps)
                                   │
                               RTAB-Map (SLAM)  ──>  /map, /odom (or /amcl_pose)
                                   │
                                   └────> RViz2 (visualization, goal setting)
```

---

## Tested Environment
- **OS:** Ubuntu 22.04 LTS  
- **ROS 2:** Humble Hawksbill  
- **Isaac Sim:** 2023.x / 2024.x (ROS 2 bridge enabled)

> Other versions may work with minor adjustments (topic names, QoS, or bridges).

---

## Prerequisites

- **Isaac Sim:** Install and enable the ROS 2 bridge  
  Docs: https://docs.omniverse.nvidia.com/isaacsim/latest/

- **ROS 2 Humble:** https://docs.ros.org/en/humble/Installation.html

- **RTAB-Map (ROS 2):**
  ```bash
  sudo apt update
  sudo apt install ros-humble-rtabmap-ros
  ```

- **Navigation2 + Bringup:**
  ```bash
  sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
  ```

- **Teleop (optional):**
  ```bash
  sudo apt install ros-humble-teleop-twist-keyboard
  ```

---

## Install & Build

Clone this repository into your ROS 2 workspace and build with `colcon`:

```bash
# Create workspace if needed
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone this repo (replace with your fork if applicable)
git clone <THIS_REPO_URL> isaac-sim-mobile-robot-rtab-map

# Build
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Quick Start

1. **Launch Isaac Sim scene**
   - Open the provided USD world (recommended):
     ```
     isaac-sim/ros2-turtlebot.usd
     ```
   - Or run the helper script (example):
     ```bash
     ISAACSIM_PYTHON script/run-sim.py
     ```
     > Depending on your Isaac Sim install, you may prefer:
     > ```
     > ./python.sh script/run-sim.py
     > ```
     > Ensure sensors and TF publishers are enabled in the stage.

2. **Start RViz2 (preconfigured)**
   ```bash
   ros2 launch isaac-sim-mobile-robot-rtab-map rviz.launch.py
   ```

3. **Start RTAB-Map (scan/RGB-D)**
   ```bash
   ros2 launch isaac-sim-mobile-robot-rtab-map rtab-map-scan.launch.py
   ```

4. **Bring up Navigation2**
   ```bash
   ros2 launch isaac-sim-mobile-robot-rtab-map navigation2.launch.py
   ```

- In **RViz2**, set a **2D Goal Pose** once you see `/map`, TF, and costmaps populating.

---

## Run Components Individually

1. **Teleop (keyboard)**
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```

2. **RTAB-Map demo (alt)**
   ```bash
   ros2 launch rtabmap_demos turtlebot3_scan.launch.py
   ```
   > Use only if you want the stock demo; prefer this repo’s launch for Isaac-specific topics.

3. **Navigation2 (generic bringup)**
   ```bash
   ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true
   ```

4. **RViz2 (Nav2 default view)**
   ```bash
   ros2 launch nav2_bringup rviz_launch.py
   ```

---

## Topic & Frame Conventions

> Exact topics/frames depend on your USD + bridge config; these are the usual suspects.

- **Frames:** `map` → `odom` → `base_link` → `base_scan` / `camera_link`
- **SLAM / Map:** `/map`, `/map_metadata`, `/rtabmap/odom`, `/tf`, `/tf_static`
- **Sensors:** `/scan` (LiDAR) **or** `/camera/*/image_raw`, `/camera/*/camera_info`, `/points`
- **Control:** `/cmd_vel`
- **Nav2:** `/amcl_pose` (if using AMCL), `/goal_pose`, `/global_costmap/*`, `/local_costmap/*`
- **Clock:** `/clock` (ensure `use_sim_time:=true` everywhere in sim)

---

## Project Layout

```
isaac-sim-mobile-robot-rtab-map/
├─ launch/
│  ├─ rviz.launch.py                  # RViz2 with pre-set displays (map, TF, costmaps, etc.)
│  ├─ rtab-map-scan.launch.py         # RTAB-Map for 2D scan-based SLAM (example)
│  └─ navigation2.launch.py           # Nav2 bringup wired to Isaac Sim topics
├─ config/
│  ├─ rtabmap.yaml                    # RTAB-Map parameters (sensor topics, mem/loop-closure)
│  ├─ nav2_params.yaml                # Planner/controller/costmap params
│  └─ rviz.rviz                       # RViz2 saved configuration
├─ isaac-sim/
│  └─ ros2-turtlebot.usd              # Example USD world
├─ script/
│  └─ run-sim.py                      # Helper to open/load the stage with ROS 2 bridge
├─ src/                               # (Optional) packages/nodes if you extend this project
└─ README.md
```

> File names may differ slightly in your repo; keep topics/frames consistent across configs.

---

## Configuration Snippets

Minimal **RTAB-Map** (scan-based) example (`config/rtabmap.yaml`):
```yaml
rtabmap:
  ros__parameters:
    use_sim_time: true
    frame_id: "base_link"
    odom_frame_id: "odom"
    subscribe_scan: true
    subscribe_rgbd: false
    map_update_interval: 1.0

    # Loop closure / memory (tune as needed)
    RGBD/LoopClosureReextractFeatures: "true"
    RGBD/OptimizeFromGraphEnd: "true"
    Rtabmap/DetectionRate: "5.0"
    Rtabmap/MemoryThr: "0.1"

    # Topics
    scan_topic: "/scan"
```

Minimal **Nav2** params (`config/nav2_params.yaml`):
```yaml
amcl:
  ros__parameters:
    use_sim_time: true

controller_server:
  ros__parameters:
    use_sim_time: true
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "nav2_controller::FollowPath"

planner_server:
  ros__parameters:
    use_sim_time: true
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"

bt_navigator:
  ros__parameters:
    use_sim_time: true
    default_bt_xml_filename: "navigate_w_replanning_and_recovery.xml"

global_costmap:
  ros__parameters:
    use_sim_time: true
    global_frame: "map"
    robot_base_frame: "base_link"
    plugin_names: ["static_layer", "obstacle_layer", "inflation_layer"]
    plugin_types:
      - "nav2_costmap_2d::StaticLayer"
      - "nav2_costmap_2d::ObstacleLayer"
      - "nav2_costmap_2d::InflationLayer"

local_costmap:
  ros__parameters:
    use_sim_time: true
    global_frame: "odom"
    robot_base_frame: "base_link"
    plugin_names: ["obstacle_layer", "inflation_layer"]
    plugin_types:
      - "nav2_costmap_2d::ObstacleLayer"
      - "nav2_costmap_2d::InflationLayer"

behavior_server:
  ros__parameters:
    use_sim_time: true
```

---

## Troubleshooting

- **No movement or delayed time:** Confirm simulation time is used everywhere.
  ```bash
  ros2 param get /<node> use_sim_time
  ```
  Launch with `use_sim_time:=true` and verify `/clock` is published by Isaac Sim.

- **Empty map or SLAM not updating:** Check sensor topics (`/scan` or RGB-D topics) and QoS. Isaac Sim often needs matching QoS profiles (reliable/best-effort).

- **TF errors (no transform from map→base_link):** Ensure TF tree is continuous (`map → odom → base_link`). Use `rtabmap` (or `amcl`) to publish `map→odom`, your robot publishes `odom→base_link`.

- **Nav2 won’t plan / costmaps empty:** Verify global/local costmap frames, footprint/radius, and obstacle sources. Check `global_frame`, `robot_base_frame`, and `observation_sources` in params.

- **Topic mismatches:** Confirm the ROS 2 bridge in Isaac Sim maps the USD sensors to the exact topics expected by RTAB-Map/Nav2. Adjust either the bridge or your configs.

- **ROS Domain:** Ensure `ROS_DOMAIN_ID` matches across terminals:
  ```bash
  export ROS_DOMAIN_ID=0
  ```

---

## Resources
- **NVIDIA Isaac Sim:** https://developer.nvidia.com/isaac/sim  
- **ROS 2 (Humble):** https://docs.ros.org/en/humble/index.html  
- **tf2 Intro:** https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Introduction-To-Tf2.html  
- **Navigation2:** https://github.com/ros-navigation/navigation2  
- **RTAB-Map:** https://introlab.github.io/rtabmap/  
- **RTAB-Map ROS 2:** https://github.com/introlab/rtabmap_ros/tree/ros2#rtabmap_ros

---

## Contributing
Contributions are welcome. If you find a bug or have a feature request, please open an issue or submit a pull request.
