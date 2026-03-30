# Voxel Subtraction - Isaac Sim Alignment Debug Log

## Problem
Live lidar from Isaac Sim (`/front_3d_lidar/lidar_points`) was not aligning with the static PCD map in RViz. The voxel subtraction was producing incorrect results because the lidar-to-map transformation was wrong.

## Root Cause Analysis

### Issue 1: Wrong transform (robot base vs lidar pose)
The original code used TF2 to look up `map -> front_3d_lidar` via Isaac Sim's TF tree. This composed `map -> base_link -> front_3d_lidar`, but `/robotPose` (Odometry) already gives the **lidar's ground-truth pose** in the map frame directly - not the robot base pose. So the extra `base_link -> front_3d_lidar` offset was wrong.

Previously the node worked with `/pcl_pose` (PoseWithCovarianceStamped) which was the lidar's own pose from PCL-based localization. Switching to `/robotPose` (Odometry from Isaac Sim) required treating it the same way: as the lidar pose, applied directly.

### Issue 2: TF tree conflict with Isaac Sim
When we broadcast `map -> front_3d_lidar` TF from robotPose, Isaac Sim was also publishing its own TF involving `front_3d_lidar` (e.g. `base_link -> front_3d_lidar`). This caused two parents for the `front_3d_lidar` frame - TF2 doesn't allow this and the frame "fights" between two positions in RViz.

### Issue 3: RViz can't display raw lidar without TF
The raw `/front_3d_lidar/lidar_points` topic has `frame_id: front_3d_lidar`. RViz needs a TF from that frame to the fixed frame (`map`) to display it. Since we can't broadcast our own TF without conflicting with Isaac Sim's tree, the raw topic can't be reliably shown.

## Solution
Removed all TF dependency. The node now:

1. **Subscribes to `/robotPose`** (Odometry) - stores as `robot_pose_` (lidar's ground-truth pose in map frame)
2. **Subscribes to `/front_3d_lidar/lidar_points`** - raw lidar points in sensor-local frame
3. **Transforms directly**: `p_map = robot_pose_ * p_lidar` for every point - no TF lookup, no frame_id checks
4. **Publishes `/lidar_map_frame`** - all lidar points already transformed to `map` frame as a new PointCloud2 with `frame_id: "map"`. This is the key debug/visualization topic.
5. **Publishes `/static_map_voxelized`** - the PCD static map, also in `map` frame

In RViz, both `/lidar_map_frame` (green) and `/static_map_voxelized` (gray) are in the `map` frame. If alignment is correct, green lidar traces (walls, furniture) visually overlap the gray PCD geometry.

## What was removed
- All TF2 includes (`tf2_ros`, `tf2_eigen`, `tf2_sensor_msgs`)
- TF buffer, listener, broadcaster
- `lidar_frame` parameter
- TF broadcasting from `odom_callback`
- Frame_id checks in `cloud_callback` (no more `cloud_in_map` logic)

## Topics for RViz verification
| Topic | Frame | Color | Purpose |
|-------|-------|-------|---------|
| `/static_map_voxelized` | map | Gray | Static PCD map (ground truth geometry) |
| `/lidar_map_frame` | map | Green | Live lidar transformed to map frame |
| `/dynamic_obstacles_cloud` | map | Red | Detected dynamic obstacle points |
| `/dynamic_obstacles_markers` | map | Red cubes | Detected dynamic obstacle markers |

## How to verify alignment
Launch: `ros2 launch dynamic_obstacle_detector isaac_sim.launch.py`

RViz opens with the config at `rviz/isaac_sim.rviz`. Check:
- **Aligned**: green lidar walls overlap gray PCD walls = transform is correct
- **Offset/shifted**: constant translation error between PCD origin and Isaac Sim world origin
- **Rotated**: orientation mismatch between coordinate systems
- **Scaled**: units mismatch (meters vs something else)

## Files changed
- `src/voxel_subtraction_node.cpp` - simplified transform, added `/lidar_map_frame` publisher
- `config/isaac_sim_params.yaml` - removed `lidar_frame` param
- `launch/isaac_sim.launch.py` - added RViz2 node with `use_sim_time: true`
- `rviz/isaac_sim.rviz` - new RViz config showing all relevant topics
- `CMakeLists.txt` - removed TF2 deps, added `rviz/` install dir
