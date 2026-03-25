# Dynamic Obstacle Detector — Voxel Subtraction Node

## Overview

A ROS2 Humble C++ node that detects dynamic obstacles by subtracting a static voxelized map from live LiDAR scans. This follows the voxel subtraction approach:

```
dynamic_obstacles = live_scan_voxels - inflated_static_voxels
```

If a voxel appears in the live scan but NOT in the inflated static map, it is a dynamic obstacle.

## How It Works (Step by Step)

1. **Load static map** — Reads `bitlab.pcd` (268K points, XYZI format) from disk. Done once at startup.
2. **Voxelize static map** — Converts each 3D point to a voxel grid index using `floor(point / voxel_size)`. Voxel size = 0.2m. Done once.
3. **Inflate static voxels (3x3x3)** — For every occupied voxel, also mark all 26 neighbors as occupied. This creates tolerance for small pose errors and noise. Done once.
4. **Per live LiDAR scan:**
   - Receive point cloud from `/cloud` topic (already in `map` frame from localization)
   - Receive robot pose from `/amcl_pose` topic
   - For each point: compute voxel key, check if it exists in inflated static set
   - If NOT in static set AND within activation zone (distance + FOV) → dynamic obstacle
5. **Activation zone** — Only voxels within `activation_distance` (default 1m) AND within `fov_angle_deg` (default 120°) frontal FOV of the robot are considered. This avoids unnecessary processing when the path is clear.

## Key Design Decisions

- **No external voxel library** (no octomap). Voxelization is just integer division: `voxel_index = floor(point / 0.2)`. Uses `unordered_set` with spatial hashing for O(1) lookup.
- **C++ for performance** — hash set operations on every point at 10Hz LiDAR rate.
- **frame_id aware** — If `/cloud` is already in `map` frame (as in our bag file), no extra transform is applied. If in sensor frame, the robot pose is used to transform.
- **3x3x3 inflation** — Adds ±1 voxel tolerance in all directions (~0.2m buffer for pose error).

## Workspace & Files

```
/home/iko/voxel_subtraction_ws/
└── src/dynamic_obstacle_detector/
    ├── src/voxel_subtraction_node.cpp   # Main node (all logic here)
    ├── config/params.yaml               # Parameters (topics, voxel size, thresholds)
    ├── launch/voxel_subtraction.launch.py
    ├── CMakeLists.txt
    └── package.xml
```

## Static Map PCD File

Located at: `/home/iko/localization_ws/src/ndt_omp_ros2/data/bitlab.pcd`
- 268,250 points, XYZI format, binary
- Same file used by the NDT localization node

## Topics

### Subscribed
| Topic | Type | Description |
|-------|------|-------------|
| `/cloud` | sensor_msgs/PointCloud2 | Live LiDAR scan (frame_id: map) |
| `/amcl_pose` | geometry_msgs/PoseWithCovarianceStamped | Robot pose from localization |

### Published
| Topic | Type | Description |
|-------|------|-------------|
| `/dynamic_obstacles_markers` | visualization_msgs/MarkerArray | Red cubes for dynamic obstacles (RViz) |
| `/dynamic_obstacles_cloud` | sensor_msgs/PointCloud2 | Dynamic obstacle voxel centers |
| `/static_map_voxelized` | sensor_msgs/PointCloud2 | Full voxelized static map (republished every 2s) |

## Parameters (config/params.yaml)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `static_map_path` | (required) | Path to static PCD map file |
| `voxel_size` | 0.2 | Voxel cube size in meters |
| `cloud_topic` | `/cloud` | Live LiDAR topic |
| `pose_topic` | `/amcl_pose` | Robot pose topic |
| `activation_distance` | 1.0 | Max distance (m) for obstacle detection |
| `fov_angle_deg` | 120.0 | Frontal field of view in degrees |
| `min_z` | -0.3 | Min Z height filter (ignore ground) |
| `max_z` | 2.5 | Max Z height filter (ignore ceiling) |

## Build

```bash
# IMPORTANT: conda interferes with ROS2 build. Use system python:
source /opt/ros/humble/setup.bash
cd /home/iko/voxel_subtraction_ws
colcon build --packages-select dynamic_obstacle_detector \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

## Run

```bash
# Terminal 1: Play bag file (for testing)
source /opt/ros/humble/setup.bash
ros2 bag play /home/iko/localization_ws/bitlab_ros2 --loop

# Terminal 2: Run the node
source /opt/ros/humble/setup.bash
source ~/voxel_subtraction_ws/install/setup.bash
ros2 launch dynamic_obstacle_detector voxel_subtraction.launch.py

# Terminal 3: RViz
rviz2
# Fixed Frame: map
# Add: PointCloud2 -> /cloud (live scan, white)
# Add: PointCloud2 -> /static_map_voxelized (static map voxels)
# Add: MarkerArray -> /dynamic_obstacles_markers (red cubes = dynamic obstacles)
```

## Bag File

Location: `/home/iko/localization_ws/bitlab_ros2/`
- Duration: ~92 seconds, 923 LiDAR scans (~10Hz)
- Key topics: `/cloud` (PointCloud2, map frame), `/amcl_pose` (200 msgs, ~2Hz)
- Converted from ROS1 to ROS2 Humble

## Related Workspaces

- `/home/iko/localization_ws/` — NDT localization (lidar_localization_ros2, ndt_omp_ros2)
- `/home/iko/ros2_ws/` — Navigation, path planning, vehicle description

## Concept (from PhD advisor)

The PhD student's key advice:
- Voxel size 0.2m is the default
- 3x3x3 inflation handles pose error tolerance
- No need for external libraries like octomap — simple hash-based voxelization is sufficient
- `dynamic = live_voxels - inflated_static_voxels`
- C/C++ for performance (Python arrays too slow for real-time)
