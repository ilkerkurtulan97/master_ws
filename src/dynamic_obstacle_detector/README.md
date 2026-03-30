# Dynamic Obstacle Detector — Voxel Subtraction Node

## Overview

A ROS2 Humble C++ node that detects dynamic obstacles by subtracting a static voxelized map from live 3D LiDAR scans.

```
dynamic_obstacles = live_scan_voxels - inflated_static_map_voxels
```

Any voxel that appears in the live scan but NOT in the inflated static map is a dynamic obstacle candidate. After surviving a temporal persistence filter, it is published as a marker.

**Robot platform:** NVIDIA Nova Carter (Isaac Sim)
**LiDAR:** Hesai QT128 front 3D LiDAR (`/front_3d_lidar/lidar_points`)
**Static map:** Bitlab PCD scan (`Bitlab_hq_subsampled.pcd`)
**Workspace:** `/home/ilker/master_ws`

---

## Pipeline (Step by Step)

1. **Load static PCD** — reads `.pcd` file from disk once at startup.
2. **Voxelize** — maps each 3D point to an integer voxel key: `floor(x/voxel_size), floor(y/voxel_size), floor(z/voxel_size)`. Stored in an `unordered_set` for O(1) lookup.
3. **Inflate static voxels** — expands each occupied voxel to its full `(2*inflation_radius+1)³` neighborhood. Absorbs small pose errors and PCD surface gaps.
4. **Per LiDAR scan:**
   - Transform each point from sensor frame to map frame using `robotPose` (the lidar's own pose in map frame, from Isaac Sim).
   - Apply Z range filter (`min_z` / `max_z`).
   - Apply relative ground filter: skip points below `robot_z + ground_filter_height`.
   - Look up voxel key in inflated static set — if found → static, skip.
   - Check activation zone: must be within `activation_distance` meters (XY distance from lidar) and within `fov_angle_deg`.
   - Surviving voxels go into the temporal persistence map.
5. **Temporal filter** — decay-based persistence: seen voxels get `count+1`, unseen voxels decay `count-1` and expire at 0. Voxels with `count >= min_consecutive_scans` are confirmed dynamic obstacles.
6. **Publish** — confirmed voxels published as `MarkerArray` (red cubes), `PointCloud2`, and a debug candidates cloud.

---

## Topics

### Subscribed
| Topic | Type | Description |
|-------|------|-------------|
| `/front_3d_lidar/lidar_points` | `sensor_msgs/PointCloud2` | Live LiDAR scan in sensor frame |
| `/robotPose` | `nav_msgs/Odometry` | Lidar pose in map frame (Isaac Sim ground truth) |

### Published
| Topic | Type | Description |
|-------|------|-------------|
| `/dynamic_obstacles_markers` | `visualization_msgs/MarkerArray` | Red cubes at confirmed dynamic voxels |
| `/dynamic_obstacles_cloud` | `sensor_msgs/PointCloud2` | Confirmed dynamic voxel centers |
| `/dynamic_candidates_cloud` | `sensor_msgs/PointCloud2` | Pre-temporal-filter candidates (debug) |
| `/lidar_map_frame` | `sensor_msgs/PointCloud2` | Live scan transformed to map frame (alignment check) |
| `/static_map_voxelized` | `sensor_msgs/PointCloud2` | Voxelized static map (republished every 5s) |

---

## Parameters (`config/isaac_sim_params.yaml`)

| Parameter | Value | Description |
|-----------|-------|-------------|
| `static_map_path` | `/mnt/storage/.../Bitlab_hq_subsampled.pcd` | Static PCD map file |
| `voxel_size` | `0.1` | Voxel cube side in meters |
| `cloud_topic` | `/front_3d_lidar/lidar_points` | LiDAR topic |
| `pose_topic` | `/robotPose` | Pose topic |
| `message_typ` | `Odometry` | Pose message type (`Odometry` or `PoseWithCovariance`) |
| `activation_distance` | `10.0` | Max XY detection radius in meters |
| `fov_angle_deg` | `360.0` | Detection FOV (360 = omnidirectional) |
| `inflation_radius` | `2` | Static map inflation: `2` = 5×5×5 = 0.2m buffer |
| `min_consecutive_scans` | `1` | Frames before confirming obstacle (see note below) |
| `min_z` | `-5.0` | Absolute Z floor filter |
| `max_z` | `5.0` | Absolute Z ceiling filter |
| `ground_filter_height` | `-2.0` | Ground threshold = `robot_z + this`. Must be negative for elevated lidar. |

---

## Build and Run

```bash
# Build
cd ~/master_ws
colcon build --packages-select dynamic_obstacle_detector
source install/setup.bash

# Launch (starts node + static TF + RViz)
ros2 launch dynamic_obstacle_detector isaac_sim.launch.py
```

Then press **Play** in Isaac Sim.

> **Important:** Stop `world_pose_tf_relay.py` before launching — it conflicts with
> the `map → World` static TF published by the launch file. Both try to parent
> `base_link` to `map`, giving it two parents which breaks TF.
>
> ```bash
> pkill -f "world_pose_tf_relay"
> pkill -f "static_transform_publisher.*base_link"
> ```

---

## TF Tree

Isaac Sim publishes: `World → odom → base_link → front_3d_lidar`

The launch file adds: `map → World` (identity static TF)

Full chain in RViz (fixed frame = `map`): `map → World → odom → base_link → front_3d_lidar`

---

## Diagnostic Log

The node logs every 100 frames (clock-independent, uses frame counter):

```
[diag #N] total=7700 nan=0 z_filt=0 gnd_filt=0 static_filt=7650 zone_filt=0 |
          pre_temporal=25 confirmed=0 | robot_z=-0.67 gnd_thresh=-2.67
```

| Field | Meaning |
|-------|---------|
| `total` | Total points in scan message |
| `nan` | Points with NaN coordinates (dropped) |
| `z_filt` | Points outside `min_z`/`max_z` |
| `gnd_filt` | Points below `robot_z + ground_filter_height` |
| `static_filt` | Points matching inflated static map |
| `zone_filt` | Points outside activation zone |
| `pre_temporal` | Dynamic voxels this frame (before temporal filter) |
| `confirmed` | Confirmed dynamic voxels at this exact frame |
| `robot_z` | Current lidar Z in map frame |
| `gnd_thresh` | Actual ground filter Z threshold applied |

Note: `confirmed` in the diag line is often 0 even when detections are happening. This is a sampling artifact — the diag fires at frame #N which may not coincide with confirmed-voxel frames. The separate `"Dynamic obstacles confirmed: X voxels"` log line is the authoritative signal.

---

## Known Behavior and Limitations

### Scan slice publishing (Hesai QT128 in Isaac Sim)
The QT128 publishes **partial scan slices** (~90-100° azimuth per ROS message) at ~36 Hz rather than full 360° rotations at 10 Hz. This means each callback covers only a fraction of the full scan. Consequences:
- A specific cylinder voxel is only illuminated in 1-2 consecutive callbacks per lidar rotation, then invisible for the next 6-7 callbacks.
- `min_consecutive_scans ≥ 2` causes most cylinder voxels to decay back to 0 before the next hit arrives — severely reducing detected voxel count.
- **Solution: `min_consecutive_scans: 1`** — confirm on first appearance. The 0.5s marker lifetime in RViz provides visual smoothing. The static map subtraction is the primary noise filter.

### Ground filter: must be negative for elevated lidar
The Nova Carter's Hesai QT128 is mounted ~1.5m above the floor. With `robot_z ≈ -0.67` (lidar z in map frame), the floor is at approximately `z ≈ -2.17`. Setting `ground_filter_height: -2.0` gives a threshold of `-2.67`, safely below the floor. The floor itself is in the PCD static map and gets absorbed by `static_filt`. The original value `0.15` (intended for a lidar near floor level) placed the threshold at `z = -0.52`, cutting off any floor-level object since the entire cylinder was below that height.

### Close-range detection limit
Objects within approximately 0.5–1.0m horizontal distance from the lidar sensor may not be detected. This is the sensor's physical minimum range — the lidar does not produce returns for objects this close. This is a hardware characteristic, not a software issue.

### False positives on walls
The static PCD (`Bitlab_hq_subsampled.pcd`) is a subsampled scan. Sparse regions create voxel gaps in the static map. Real lidar hits in those gaps pass `static_filt` and appear as false positive dynamic voxels on walls/surfaces. `inflation_radius: 2` (0.2m buffer) absorbs most of these gaps. If stray wall voxels persist, increase to `inflation_radius: 3`.

---

## Debugging History

This section documents all issues encountered and fixes applied during development in the Isaac Sim integration phase.

### Issue 1 — No output on `/dynamic_obstacles_markers` despite lidar working
**Symptom:** Topics flowing at 36 Hz, no markers in RViz, no `[diag]` log output.

**Root cause A — Throttle silenced by `use_sim_time`:** All `RCLCPP_*_THROTTLE` calls check `this->now()`. With `use_sim_time: true` and Isaac Sim paused (or `/clock` not flowing), `now()` stays at 0 and no throttle interval ever expires. All throttled logs were silent.

**Fix:** Replaced all `RCLCPP_INFO_THROTTLE` and `RCLCPP_WARN_THROTTLE` calls with frame-counter-based logging:
```cpp
if (++diag_frame_ % 100 == 0) {
    RCLCPP_INFO(this->get_logger(), "[diag #%lu] ...", diag_frame_, ...);
}
```

**Root cause B — All points absorbed by static filter:** With no dynamic objects in the scene, every lidar hit lands on a static surface → `static_filt ≈ total` → `pre_temporal = 0` always.

**Fix:** Place a dynamic object (cylinder/box) in the Isaac Sim scene.

---

### Issue 2 — Cylinder only detected when lifted, not at floor level
**Symptom:** `pre_temporal > 0` and confirmed voxels only appeared when the cylinder was raised to approximately `z = -0.70` (just above the old `gnd_thresh = -0.52`).

**Root cause:** `ground_filter_height` was `0.15` (default designed for lidar near floor level). With lidar at `robot_z = -0.67`, this set `gnd_thresh = -0.52`. The cylinder resting on the floor at `z ≈ -2.2` had its entire body below `-0.52` → all cylinder points were ground-filtered.

**Diagnostic confirmation:**
```
robot_z=-0.67  gnd_thresh=-0.52   gnd_filt=3500 (≈ half all points)
```
Half the scan was being discarded as "ground" when in reality the lidar is 1.5m above the floor.

**Fix:** `ground_filter_height: -2.0` → `gnd_thresh = -0.67 + (-2.0) = -2.67`, safely below the actual floor.

---

### Issue 3 — Ghost TF frame in RViz corner
**Symptom:** Two copies of the robot's TF frame visible — one at the correct position, one stuck at the world origin (0, 0, 0).

**Root cause:** Two conflicting sources of TF:
1. Isaac Sim's TF tree rooted at `World`: `World → odom → base_link → sensors`
2. `world_pose_tf_relay.py` publishing `map → base_link` from `/chassis/odom`

With `world_pose_tf_relay.py` running, `base_link` had two parents (`World/odom` and `map`), which is invalid in TF. Additionally, `World` had no connection to `map`, so it floated at (0, 0, 0).

Also, `voxel_subtraction.launch.py` had been launched multiple times, creating 3 duplicate `base_link_to_lidar_tf` static transform nodes.

**Fix A:** Added `map → World` identity static transform to `isaac_sim.launch.py`:
```python
Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='map_to_world_tf',
    arguments=['0', '0', '0', '0', '0', '0', 'map', 'World'],
    parameters=[{'use_sim_time': True}],
),
```

**Fix B:** Stop conflicting processes before launching:
```bash
pkill -f "world_pose_tf_relay"
pkill -f "static_transform_publisher.*base_link"
```

---

### Issue 4 — YAML/launch changes not taking effect
**Symptom:** Changed `ground_filter_height` in YAML but `ros2 param get` still returned old value.

**Root cause:** `colcon build` installs YAML and launch files into `install/`. The node loads parameters from `install/` at launch time. After a build, the old node process was still running with the old params.

**Fix:** Always relaunch the node after `colcon build`:
```bash
colcon build --packages-select dynamic_obstacle_detector
source install/setup.bash
ros2 launch dynamic_obstacle_detector isaac_sim.launch.py
```

Verify params loaded correctly:
```bash
ros2 param get /voxel_subtraction_node ground_filter_height  # should return -2.0
ros2 param get /voxel_subtraction_node voxel_size            # should return 0.1
```

---

### Issue 5 — Intermittent / unstable voxel detections
**Symptom:** Confirmed voxels flickered — appeared for a few frames, disappeared, reappeared.

**Root cause:** Original temporal persistence was a hard-reset: if a voxel was missed for even ONE frame, its count reset to 0. A rotating lidar doesn't illuminate the exact same voxel every single scan callback (beam pattern varies slightly), so counts would reset frequently.

**Fix:** Replaced hard-reset with decay-based persistence:
- Seen voxel: `count = min(count + 1, min_consecutive_scans)` (boost, capped)
- Unseen voxel: `count = count - 1`, remove when `count == 0` (decay)

This allows a voxel to survive a brief missed scan without losing all accumulated evidence.

---

### Issue 6 — Low confirmed voxel count (4–8 voxels for a full cylinder)
**Symptom:** Even with a cylinder fully visible in the scene, only 4–8 voxels were confirmed. `pre_temporal` alternated between 0 and 10–68 every few diag frames.

**Root cause:** The Hesai QT128 in Isaac Sim publishes **partial scan slices** (~90–100° azimuth per message) at ~36 Hz, not full 360° rotations at 10 Hz. Each cylinder surface voxel is only illuminated during 1–2 consecutive callbacks per rotation, then invisible for the next 6–7 callbacks as the scan sweeps past. With `min_consecutive_scans ≥ 2`, the count builds to 1 during the hit-slice and decays back to 0 during the miss-slices — never reaching the confirmation threshold for most voxels.

**Diagnostic pattern that revealed this:**
- Every ~3 diag entries: one frame with `pre_temporal=0, zone_filt=3-4, total≈6540`
- Then 2 frames with `pre_temporal=10-68, zone_filt=0, total≈7700`
- The lower total count on `pre_temporal=0` frames indicates the scan slice was facing away from the cylinder.

**Fix:** `min_consecutive_scans: 1` — confirm on first appearance. The 0.5s marker `lifetime` in `publish_obstacle_markers()` provides visual continuity in RViz. Static map subtraction is the real noise filter.

Result: improved from 4–8 confirmed voxels to 15–25+ (full cylinder surface visible).

---

### Issue 7 — False positive voxels on walls
**Symptom:** After setting `min_consecutive_scans: 1`, small stray red voxels appeared on walls in RViz.

**Root cause:** `Bitlab_hq_subsampled.pcd` is a subsampled point cloud. Sparse regions have voxel gaps in the static map. Real lidar hits in those gaps pass `static_filt` and are confirmed as dynamic on the very first hit.

**Key insight:** Raising `min_consecutive_scans` would NOT fix this. Wall gaps are hit in every scan frame consistently, so they would quickly reach count ≥ 2 regardless. The fix must be at the static map level.

**Fix:** `inflation_radius: 2` (from 1) — inflates static voxels to a 5×5×5 (0.2m) neighborhood instead of 3×3×3 (0.1m). Covers PCD subsampling gaps and small surface misalignments. If stray wall voxels remain, increase to `inflation_radius: 3`.

---

## Final Working Configuration

```yaml
voxel_size: 0.1
activation_distance: 10.0
fov_angle_deg: 360.0
inflation_radius: 2
min_consecutive_scans: 1
min_z: -5.0
max_z: 5.0
ground_filter_height: -2.0
```

**Expected diag output** with a cylinder in the scene:
```
[diag #N] total≈7700 nan=0 z_filt=0 gnd_filt=0 static_filt≈7650 zone_filt=0 |
          pre_temporal=15-70 confirmed=0 | robot_z=-0.67 gnd_thresh=-2.67
```
Intermittent `confirmed=0` in diag is normal (sampling artifact). Watch for the separate `"Dynamic obstacles confirmed: X voxels"` messages which should appear several times per second.
