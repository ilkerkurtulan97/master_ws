# navmesh_planner

ROS2 C++ path planner for an Ackermann-type rover using Recast Navigation's Detour library. Loads a pre-baked navigation mesh (.bin file) and performs A* pathfinding to produce waypoint paths.

Part of a master's thesis project on dynamic obstacle detection and avoidance for autonomous rovers, targeting multi-floor environments (parking garages with spiral ramps).

---

## System Architecture

```
                         /pcl_pose (NDT localization)
         ┌─────────────────────┼───────────────────────┐
         v                     v                        v
┌──────────────────┐  ┌─────────────────┐  ┌──────────────────┐
│ dynamic_obstacle │  │  navmesh_planner │  │ drive_controller │
│    _detector     │  │                  │  │                  │
│                  │  │ Loads .bin       │  │ Pure pursuit     │
│ Voxel subtraction│  │ findPath()       │  │ Ackermann steer  │
│ Static vs dynamic│  │ dtTileCache      │  │ Obstacle stop    │
└──────┬───────────┘  └───────┬──┬───────┘  └──┬──────┬───────┘
       │                  ^   │  ^              ^      │
       │/dynamic_obstacles│   │  │/replan_      │      │
       │_cloud            │   │  │request       │      │
       │                  │   │  └──────────────┘      │
       ├──────────────────┘   │ /planned_path          │
       │  (TSET: obstacle     └────────────────────────┘
       │   injection)                                  │
       └───────────────────────────────────────────────┘
                                                   /cmd_vel → Rover
```

### Topic Wiring

| Topic | Publisher | Subscriber | Type |
|-------|----------|------------|------|
| `/pcl_pose` | NDT localization | All 3 nodes | PoseStamped / PoseWithCovarianceStamped |
| `/goal_pose` | RViz2 "2D Goal Pose" | navmesh_planner | PoseStamped |
| `/planned_path` | navmesh_planner | drive_controller | nav_msgs/Path |
| `/planned_path_markers` | navmesh_planner | RViz2 | visualization_msgs/Marker |
| `/replan_request` | drive_controller | navmesh_planner | std_msgs/Bool |
| `/dynamic_obstacles_cloud` | dynamic_obstacle_detector | drive_controller, navmesh_planner (TSET only) | PointCloud2 |
| `/cmd_vel` | drive_controller | Rover motor driver | geometry_msgs/Twist |

All topics use `map` frame ID.

---

## How the NavMesh is Created

The navmesh is NOT created at runtime. It's a pre-baked binary file produced offline:

```
bitlab_big.pcd          Raw 3D point cloud of the environment
      |
      |  export_obj.py       Voxelize + mesh (Nafis's voxelize_navmesh tool)
      v
bitlab.obj              Triangle mesh (Y-UP for Recast)
      |
      |  RecastDemo GUI      Configure agent params, build navmesh, save
      v
bitlab_big.bin          Binary navmesh (MSET format, ~44KB)
      |
      |  navmesh_planner     Loaded at startup, used for all path queries
      v
  findPath() → /planned_path
```

### File Purposes

| File | Size | Used at runtime? | By whom? |
|------|------|-----------------|----------|
| `bitlab_big.pcd` | ~7 GB | YES | dynamic_obstacle_detector (voxel subtraction reference) |
| `bitlab.obj` | ~varies | NO | Only used offline in RecastDemo |
| `bitlab_big.bin` | ~44 KB | YES | navmesh_planner (path planning) |

The .pcd and .bin represent the same environment but serve different purposes:
- **PCD**: "What does the world look like?" (for obstacle detection)
- **BIN**: "Where can the rover drive?" (walkable surfaces only)

They must be spatially aligned (same coordinate origin and scale) so that obstacles detected in PCD-world coordinates map correctly onto the navmesh.

---

## Coordinate System

Detour uses **Y-UP**. ROS2 uses **Z-UP**. Every position must be converted.

```
ROS2 (Z-UP):        Detour (Y-UP):
  Z (up)               Y (up)
  |                     |
  |__ Y (left)          |__ Z
 /                     /
X (forward)           X (forward)
```

Conversion (see `include/navmesh_planner/coordinate_utils.hpp`):
```cpp
// ROS → Detour: X stays, Z→Y(up), Y→Z
void rosToDetour(double rx, double ry, double rz, float* d) {
    d[0] = (float)rx;   // X unchanged
    d[1] = (float)rz;   // ROS Z (up) → Detour Y (up)
    d[2] = (float)ry;   // ROS Y → Detour Z
}

// Detour → ROS: X stays, Y(up)→Z, Z→Y
void detourToRos(const float* d, double& rx, double& ry, double& rz) {
    rx = d[0];   // X unchanged
    ry = d[2];   // Detour Z → ROS Y
    rz = d[1];   // Detour Y (up) → ROS Z (up)
}
```

---

## Parameters

```yaml
navmesh_planner_node:
  ros__parameters:
    navmesh_path: "/path/to/bitlab_big.bin"   # Path to pre-baked navmesh (MSET or TSET)
    pose_topic: "/pcl_pose"                    # Localization pose topic
    poly_search_extent_x: 2.0                  # findNearestPoly search box X (meters)
    poly_search_extent_y: 4.0                  # findNearestPoly search box Y-UP height
    poly_search_extent_z: 2.0                  # findNearestPoly search box Z (meters)
    max_path_polys: 2048                       # Max polygons in path corridor
    max_straight_path: 256                     # Max waypoints after string-pulling
    waypoint_spacing: 0.5                      # Interpolation spacing (0 = disabled)

    # dtTileCache clustering (only active with TSET navmesh)
    cluster_tolerance: 0.5                     # DBSCAN distance threshold (meters)
    min_cluster_size: 5                        # Minimum points per cluster
    max_cluster_size: 5000                     # Maximum points per cluster
    obstacle_radius_default: 0.3               # Minimum cylinder radius (meters)
    obstacle_height: 2.0                       # Cylinder height in navmesh (meters)
    obstacle_replan_interval: 0.5              # Throttle between replans (seconds)
```

---

## Runtime Behavior

1. **Startup**: Load `.bin` (auto-detect MSET/TSET) → init `dtNavMesh` + `dtNavMeshQuery` (+ `dtTileCache` for TSET) → subscribe to topics
2. **Wait** for pose (`/pcl_pose`) and goal (`/goal_pose` from RViz2)
3. **On goal received**:
   - Convert start (current pose) and goal to Detour Y-UP coordinates
   - `findNearestPoly()` for both positions
   - `findPath()` → polygon corridor (A* on navmesh)
   - `findStraightPath()` → waypoints (string-pulling)
   - Convert waypoints back to ROS Z-UP
   - Optionally interpolate for smoother driving
   - Publish as `nav_msgs/Path` on `/planned_path`
4. **On `/replan_request`**: Re-run planning from current pose to same goal
5. **On `/dynamic_obstacles_cloud`** (TSET only):
   - Throttle check (skip if too soon since last update)
   - Remove all previous obstacle cylinders from tile cache
   - DBSCAN cluster the incoming point cloud
   - For each cluster: compute centroid + radius, add as cylinder obstacle
   - Update tile cache (rebuild affected tiles)
   - Replan path around injected obstacles

---

## Getting a Better NavMesh

If the navmesh has artifacts (walkable patches on ceiling, floating in mid-air), the issue is the input OBJ quality, not Recast.

### Step 1: Filter the PCD before meshing

Remove ceiling and noise points:
```python
import open3d as o3d
import numpy as np

pcd = o3d.io.read_point_cloud("bitlab_big.pcd")
points = np.asarray(pcd.points)

# Keep only floor-level points (adjust Z range for your environment)
mask = (points[:, 2] > -0.5) & (points[:, 2] < 0.3)
pcd_filtered = pcd.select_by_index(np.where(mask)[0])

# Remove outliers
pcd_clean, _ = pcd_filtered.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
o3d.io.write_point_cloud("bitlab_floor_only.pcd", pcd_clean)
```

### Step 2: Use finer voxel size when exporting OBJ

```bash
python export_obj.py bitlab_floor_only.pcd bitlab_floor.obj --voxel-size 0.1
```

### Step 3: Tune RecastDemo parameters

| Parameter | Recommended | Why |
|-----------|-------------|-----|
| Cell Size | 0.05-0.1m | Finer detail |
| Cell Height | 0.05-0.1m | Better ramp/slope resolution |
| Agent Height | 0.5m | Rover height, excludes low-clearance areas |
| Agent Radius | 0.3m | Rover half-width |
| Agent Max Climb | 0.1m | Small step tolerance |
| Agent Max Slope | 15-20 deg | Allows ramps, rejects walls |
| Region Min Size | 50-100 | Removes small floating patches (ceiling artifacts) |

### Step 4: Verify visually in RecastDemo before saving

Rotate around the navmesh. Blue polygons should only appear on floors and ramps, not ceilings.

### Alternative meshing: Poisson reconstruction

For smoother meshes (especially for the parking garage), use Open3D Poisson reconstruction instead of the voxel box approach:
```python
pcd.estimate_normals()
mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=9)
```

---

## Dynamic Obstacle Avoidance (dtTileCache)

The planner supports dynamic obstacle avoidance via Detour's `dtTileCache`. When a TSET-format navmesh is loaded, the node subscribes to `/dynamic_obstacles_cloud`, clusters the points using DBSCAN, injects them as cylinder obstacles into the navmesh, and replans around them automatically.

### How It Works

```
/dynamic_obstacles_cloud (PointCloud2)
        |
        v
   DBSCAN clustering (pcl::EuclideanClusterExtraction)
        |
        v
   For each cluster: compute centroid + enclosing radius
        |
        v
   tile_cache_->addObstacle(pos, radius, height)
        |
        v
   tile_cache_->update() — rebuilds affected tiles (~1ms each)
        |
        v
   findPath() now routes around obstacle areas
```

Each callback cycle removes all previous obstacles and re-adds from the current cloud (clear-all + re-add pattern). A time-based throttle (default 0.5s) prevents flooding from high-frequency obstacle updates.

### MSET vs TSET

The loader auto-detects the format by reading the first 4 magic bytes:

| | MSET (static) | TSET (dynamic) |
|---|---|---|
| Format magic | `'MSET'` | `'TSET'` |
| Structure | Single monolithic mesh | Grid of compressed tiles |
| Dynamic obstacles | Not supported | Cylinders via dtTileCache |
| Created by | RecastDemo "Solo Mesh" | RecastDemo "Temp Obstacles" |
| Loading | `dtNavMesh` only | `dtTileCache` + `dtNavMesh` |

Both formats are supported. MSET files load and plan paths normally but log a warning that dynamic obstacles are not available.

### Building a TSET NavMesh (One-Time Setup)

1. Run RecastDemo:
   ```bash
   /home/iko/Desktop/master_thesis/recastnavigation/build/RecastDemo/RecastDemo
   ```
2. Load your environment's `.obj` file (e.g., `bitlab.obj`)
3. Select the **"Temp Obstacles"** sample (not "Solo Mesh")
4. Configure agent parameters to match the rover:
   - Agent Radius: ~0.3m (rover half-width)
   - Agent Height: ~0.5m (rover height)
   - Max Climb: ~0.1m (step tolerance)
   - Max Slope: 15-20 deg (allows ramps)
5. Click **Build**, then **Save**
6. The output is a TSET-format `.bin` file
7. Update `navmesh_path` in `config/params.yaml` to point to this file

### Clustering Parameters

```yaml
# dtTileCache: DBSCAN clustering parameters
cluster_tolerance: 0.5       # Euclidean distance threshold (meters)
min_cluster_size: 5          # Minimum points per cluster
max_cluster_size: 5000       # Maximum points per cluster
obstacle_radius_default: 0.3 # Minimum cylinder radius (meters)
obstacle_height: 2.0         # Cylinder height in navmesh (meters)
obstacle_replan_interval: 0.5 # Throttle between obstacle-triggered replans (seconds)
```

### End-to-End Test

1. Build and launch all three nodes:
   ```bash
   ros2 launch navmesh_planner navmesh_planner.launch.py
   ros2 launch dynamic_obstacle_detector voxel_subtraction.launch.py
   ros2 launch drive_controller drive_controller.launch.py
   ```
2. Set a goal in RViz2 using "2D Goal Pose"
3. Walk in front of the rover:
   - Red cubes appear on the dynamic obstacle (from detector)
   - Green path replans around the obstacle
   - When obstacle moves away, path returns to original route

### Reference Code

- `Sample_TempObstacles.cpp`: RecastDemo reference for TSET save/load and obstacle API
- `recast_cli.cpp`: MSET loading reference
- `recast_ros/`: ROS1 wrapper with dtTileCache services

---

## Why NavMesh Over 2D Grid

The final acceptance test will be on a **multi-floor parking garage with spiral ramps**. A 2D occupancy grid cannot handle:

- Spiral ramps where the rover is at different Z heights on the same XY position
- Two floors stacked vertically (same XY, different Z) — a 2D grid collapses them
- Walkable surface reasoning (slopes, steps, clearance)

NavMesh handles all of these naturally as connected polygon surfaces at varying heights. dtTileCache adds dynamic obstacle support on top.

---

## Research Paper Insights

These papers from `/home/iko/Desktop/master_thesis/articles/` informed the design:

| Paper | Key Insight for Our System |
|-------|---------------------------|
| **Asvadi et al. 2016** (jrobot2016) | Voxel-based static/moving detection using LLR — validates our voxel subtraction approach |
| **Dong et al.** (Tmech) | Models obstacles as circles/lines → maps directly to dtTileCache cylindrical obstacles |
| **Xu et al. 2025** (LV-DOT, CMU) | DBSCAN clustering on LiDAR → bounding boxes — exactly what we need for clustering dynamic voxels |
| **Lin et al. 2022** (sensors) | Bimodal EKF pedestrian prediction — future work for predictive avoidance |

The pipeline combines Asvadi's detection concept + LV-DOT's clustering + Dong's geometric primitive model + dtTileCache.

---

## Known Issues

1. **drive_controller replan logic**: Line 190 has inverted condition (`elapsed < 0.5` should reference `obstacle_stop_timeout_`). Doesn't block stop-and-wait behavior but should be fixed.
2. **Absolute file paths**: `navmesh_path` in params.yaml uses absolute path. Must be updated when deploying to different machines (e.g., Jetson Nano).

---

## Future Work

1. **Kalman filter tracking** — Track obstacle movement over time for smoother obstacle lifecycle
2. **Predictive avoidance** — Use trajectory prediction (Lin et al.) to anticipate where obstacles will be
3. **Parking garage deployment** — Test on multi-floor environment with spiral ramps
4. **Better OBJ generation** — Poisson reconstruction or normal-based filtering for cleaner navmesh input

---

## Build and Run

### Build
```bash
cd /home/iko/master_thesis_ws
colcon build --packages-select navmesh_planner --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

Note: The `--cmake-args` flag is needed if conda is active (interferes with Python discovery).

### Run
```bash
# With launch file
ros2 launch navmesh_planner navmesh_planner.launch.py

# Or directly with params
ros2 run navmesh_planner navmesh_planner_node --ros-args \
  --params-file src/navmesh_planner/config/params.yaml
```

### Select a goal
In RViz2, click the **"2D Goal Pose"** button and click on the map. The planner will compute a path and publish it on `/planned_path`.

---

## Package Structure

```
navmesh_planner/
  CMakeLists.txt
  package.xml
  README.md
  config/
    params.yaml
  data/
    bitlab_big.bin              Pre-baked navmesh (MSET or TSET format)
  launch/
    navmesh_planner.launch.py
  include/navmesh_planner/
    coordinate_utils.hpp        ROS Z-UP <-> Detour Y-UP conversion
    navmesh_loader.hpp          MSET + TSET format structs, loadNavMesh(), loadNavMeshAuto()
    tilecache_helpers.hpp       LinearAllocator, FastLZCompressor, MeshProcess for dtTileCache
  src/
    navmesh_loader.cpp          MSET/TSET .bin file loader with auto-detection
    navmesh_planner_node.cpp    Main ROS2 node (pathfinding + dynamic obstacle injection)
  third_party/
    Detour/                     Detour library source (from recastnavigation)
      Include/                  DetourNavMesh.h, DetourNavMeshQuery.h, etc.
      Source/                   DetourNavMesh.cpp, DetourNavMeshQuery.cpp, etc.
    DetourTileCache/            DetourTileCache library (dynamic obstacle support)
      Include/                  DetourTileCache.h, DetourTileCacheBuilder.h
      Source/                   DetourTileCache.cpp, DetourTileCacheBuilder.cpp
    fastlz/                     FastLZ compression (required for TSET tile decompression)
      fastlz.h, fastlz.c
```
