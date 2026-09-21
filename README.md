<h1 align="center">master_ws — Dynamic Obstacle Avoidance for a Mobile Robot</h1>

<p align="center">
  Autonomous 3D navigation for the <b>Innok Heros</b> rover — plan a path on a baked
  <b>Recast/Detour navigation mesh</b>, detect dynamic obstacles from live LiDAR,
  re-plan around them at runtime, and track the route with a pure-pursuit controller.
</p>

<p align="center">
  <img alt="ROS 2 Humble" src="https://img.shields.io/badge/ROS_2-Humble-22314E?logo=ros&logoColor=white">
  <img alt="C++17" src="https://img.shields.io/badge/C%2B%2B-17-00599C?logo=cplusplus&logoColor=white">
  <img alt="Python 3.10" src="https://img.shields.io/badge/Python-3.10-3776AB?logo=python&logoColor=white">
  <img alt="Platform" src="https://img.shields.io/badge/edge-Jetson_Orin_NX-76B900?logo=nvidia&logoColor=white">
  <img alt="Sim" src="https://img.shields.io/badge/sim-Isaac_Sim-76B900?logo=nvidia&logoColor=white">
</p>

<p align="center">
  <img src="docs/images/rover-innok-heros.jpg" width="88%" alt="Innok Heros rover with Hesai LiDAR and ZED 2i camera">
  <br>
  <sub><b>The Innok Heros rover</b> — Hesai Pandar LiDAR (top), ZED 2i stereo camera, and the Jetson Orin NX compute box.</sub>
</p>

Developed as part of a master's thesis (*Dynamic Obstacle Avoidance for Mobile Robot*,
Ilker Kurtulan, FH Dortmund — Embedded Systems Engineering). The rover autonomously
plans a path across a pre-scanned indoor environment, detects dynamic obstacles from
live LiDAR, re-plans around them at runtime, and tracks the resulting path with a
pure-pursuit controller. The stack is validated in **NVIDIA Isaac Sim** and deployed
on an **NVIDIA Jetson Orin NX** aboard the real rover — replacing the earlier manual,
waypoint-based navigation prototype with full 3D NavMesh planning plus reactive
obstacle avoidance.

<p align="center">
  <img src="docs/images/planning-avoidance-rviz.jpg" width="46%" alt="RViz: green path bending around red obstacle cylinders">
  &nbsp;
  <img src="docs/images/replan-detour-rviz.jpg" width="46%" alt="RViz: replanned detour around a dynamic obstacle">
  <br>
  <sub><b>Live avoidance in RViz</b> — the planned path (green) bends around dynamic obstacles carved as cylinders (red) into the navigation mesh.</sub>
</p>

---

## 1. System at a glance

The system is organised as a **five-layer architecture** running on ROS 2 Humble:

| Layer | Responsibility | Where it lives |
|---|---|---|
| **Sensor** | Hesai Pandar LiDAR (3D point clouds) + ZED 2i stereo camera | Hardware + drivers |
| **Localization** | Real-time 6-DoF pose in the static map (LiDAR-based, NDT) | `lidar_localization_ros2` (external workspace) → `/pcl_pose` |
| **Perception** | Dynamic obstacle detection by static-map voxel subtraction | `dynamic_obstacle_detector` |
| **Planning** | NavMesh path planning + runtime obstacle injection & replanning | `navmesh_planner` |
| **Control** | Pure-pursuit path tracking + safety stops + human-aware speed cap | `drive_controller`, `zed_speed_governor` |


<p align="center">
  <img src="docs/images/architecture-layers.png" width="82%" alt="Five-layer architecture: Sensor, Localization, Perception, Planning, Control">
  <br>
  <sub><b>Five-layer architecture.</b> Each layer depends only on the one below it; the offline-baked navigation mesh and reference point cloud are shared inputs.</sub>
</p>

The end-to-end obstacle-avoidance flow — from a pedestrian stepping into the route to
the detour, the speed cap, and the recovery once the way is clear:

<p align="center">
  <img src="docs/images/sequence-obstacle-avoidance.png" width="90%" alt="Sequence diagram of the obstacle avoidance flow across all nodes">
</p>

---

## 2. Hardware / deployment topology

| Component | Role |
|---|---|
| **Innok Heros** rover | Mobile base. Its onboard computer runs **ROS 1** and only serves LiDAR packets. |
| **NVIDIA Jetson Orin NX** | Primary compute. Runs the full **ROS 2 Humble** stack (perception, planning, control). |
| **Hesai Pandar** LiDAR | 3D point clouds for static map construction and live obstacle detection. |
| **ZED 2i** stereo camera | Person detection for the human-aware speed governor. |
| **Simulation workstation** | Remote RTX workstation hosting Isaac Sim (accessed via VNC). Runs *only* the simulator — the navigation stack runs unchanged on the Jetson. |

The Innok base (ROS 1) and the Jetson (ROS 2) are bridged over one **direct Gigabit
Ethernet link** using two ZMQ channels:

- **LiDAR** — Innok → Jetson (`:5555`), republished on the Jetson as `/lidar/points`.
- **cmd_vel** — Jetson → Innok (`:5556`), driving the wheels.

See `cmd_vel_zmq_sender.py` (Jetson) and `cmd_vel_zmq_receiver_ros1.py` (Innok).

---

## 3. Packages

```
src/
├── dynamic_obstacle_detector/   # C++  — perception (voxel subtraction)
├── navmesh_planner/             # C++  — planning (Detour navmesh + tile cache)
├── drive_controller/            # C++  — control (pure pursuit + safety stops)
├── zed_speed_governor/          # Py   — person-aware speed cap
└── navmesh_gui/                 # Py   — operator interface (goal selection)
```

### `dynamic_obstacle_detector` (`voxel_subtraction_node`)
Subtracts a pre-baked static voxel map from each live LiDAR scan; whatever remains is
a dynamic obstacle.

```
dynamic_obstacles = live_scan_voxels − inflated_static_map_voxels
```

Key steps: load static PCD → voxelize (0.1 m) → inflate occupied voxels (XY and Z
inflation are separate) → per scan, transform to map frame using the pose, apply
Z-range + ground + self-filter, subtract the inflated static set, apply temporal
persistence, and publish confirmed obstacles.

<p align="center">
  <img src="docs/images/perception-rviz.jpg" width="82%" alt="Left: the BitLab test room. Right: RViz showing live LiDAR with detected dynamic obstacles as red voxels.">
  <br>
  <sub><b>Perception.</b> Left: the physical BitLab test space. Right: the live scan in RViz — everything matching the static map is subtracted away, leaving only the dynamic objects (red).</sub>
</p>

| Dir | Topic | Type |
|---|---|---|
| Sub | `/lidar/points` (real) or `/front_3d_lidar/lidar_points` (sim) | PointCloud2 |
| Sub | `/pcl_pose` (real) or `/chassis/odom` (sim) | PoseWithCovariance / Odometry |
| Pub | `/dynamic_obstacles_cloud` | PointCloud2 |
| Pub | `/dynamic_obstacles_markers` | MarkerArray (red cubes) |
| Pub | `/static_map_voxelized`, `/lidar_map_frame`, `/dynamic_candidates_cloud` | PointCloud2 (debug/alignment) |

### `navmesh_planner` (`navmesh_planner_node`)
Loads a pre-baked **Recast/Detour** navmesh and runs A* pathfinding. With a **tiled
(TSET)** navmesh it uses `dtTileCache` to carve dynamic obstacles as cylinders at
runtime and replan around them.

Pipeline: `findNearestPoly` (start + goal) → `findPath` (A* polygon corridor) →
`findStraightPath` (string-pull) → optional interpolation. Dynamic obstacles are
DBSCAN-clustered from `/dynamic_obstacles_cloud`, each cluster becomes a cylinder
injected via `addObstacle()` + `update()`, then the path is replanned.

| Dir | Topic | Type |
|---|---|---|
| Sub | `/pcl_pose` | PoseStamped / PoseWithCovariance |
| Sub | `/goal_pose` | PoseStamped (RViz "2D Goal Pose" or the GUI) |
| Sub | `/dynamic_obstacles_cloud` | PointCloud2 (TSET only) |
| Sub | `/replan_request` | Bool |
| Pub | `/planned_path` | nav_msgs/Path |
| Pub | `/planned_path_markers`, `/navmesh_polys`, `/navmesh_obstacles_markers` | Marker(Array) |

Recast/Detour and FastLZ are **vendored as static libraries** under
`src/navmesh_planner/third_party/` — there is no external navigation dependency.

### `drive_controller` (`drive_controller_node`)
Pure-pursuit differential-drive follower of `/planned_path`. Speed-scaled lookahead,
pose-latency compensation, an omega low-pass, an acceleration ramp, a LiDAR safety
bubble with a bubble-escape state machine, a shy-away repulsion layer, and an external
`/speed_limit` cap.

| Dir | Topic | Type |
|---|---|---|
| Sub | `/pcl_pose`, `/planned_path`, `/dynamic_obstacles_cloud`, `/speed_limit` | — |
| Pub | `/cmd_vel`, `/replan_request`, `/lookahead_marker` | Twist / Bool / Marker |

### `zed_speed_governor` (`zed_speed_governor_node`)
Opens the ZED 2i, runs person detection, and if a person is inside a **forward gate**
(distance ≤ `gate_distance`, |lateral| ≤ `corridor_half_width`) publishes a reduced
`/speed_limit`; otherwise it publishes `-1.0` ("no limit"). **Fail-open:** a dead
camera means normal driving — the LiDAR safety bubble remains the real collision guard.

<p align="center">
  <img src="docs/images/zed-governor-garage.jpg" width="70%" alt="ZED speed governor operator view: a detected person 4.78 m away, status CLEAR - full speed">
  <br>
  <sub><b>Speed governor operator view.</b> A person is detected (green box, with range) but sits outside the forward gate, so the banner reads <code>CLEAR — full speed</code>. Inside the gate it turns red and caps the speed.</sub>
</p>

### `navmesh_gui`
Operator interface for selecting and publishing a goal pose, and for monitoring the
planned path / rover pose.

---

## 4. Environment setup

### 4.1 Prerequisites

- **Ubuntu 22.04** with **ROS 2 Humble** (the Jetson runs the same distribution under
  **JetPack 6**, so no cross-compilation is needed).
- **C++17** toolchain (GCC 11), `ament_cmake`, `rclcpp`.
- **PCL**, **tf2**, **Eigen** (the remaining third-party dependencies for the C++ nodes).
- **Python 3.10** with `rclpy` (operator interface, speed governor, bridge/conversion scripts).
- **Open3D** + NumPy (offline map preparation only).
- **ZED SDK 5.0.1** + `pyzed` (only for `zed_speed_governor`; the MEDIUM/NEURAL
  TensorRT engine is optimized once on first run and needs internet for that download).

> ⚠️ **Always `conda deactivate` before any ROS 2 work.** ROS 2 Humble requires the
> system Python 3.10 (`/usr/bin/python3`). A conda `base` environment (Python 3.13)
> silently breaks `ros2 launch` and colcon's Python discovery.

### 4.2 Build

```bash
cd ~/master_ws
conda deactivate                      # use system Python 3.10
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Build a single package during development:

```bash
colcon build --packages-select navmesh_planner --symlink-install
```

> `--symlink-install` is important: **YAML and launch edits take effect on a node
> restart, no rebuild.** Only C++ source changes require `colcon build`. If conda is
> active and interferes with Python discovery, add
> `--cmake-args -DPython3_EXECUTABLE=/usr/bin/python3`.

---

## 5. The two map artifacts (offline preparation)

Every environment needs exactly **two files**, both derived from the same LiDAR scan
and in the **same coordinate frame** as the pose topic:

| File | Used by | Purpose |
|---|---|---|
| `<scene>.tcbin` | `navmesh_planner` | Tiled Detour navmesh — *where the rover can drive*. Supports runtime obstacle carving. |
| `<scene>.pcd` | `dynamic_obstacle_detector` | Static reference point cloud — subtracted from each live scan to find dynamic objects. |

### Baking pipeline

```
                pcd_to_obj.py                 bake_tcbin.py
  scene.pcd ───────────────────► scene_yup.obj ───────────────────► scene.tcbin
             (voxelize @ 0.1 m,                (Recast build:
              exposed-face mesh,                heightfield → regions →
              Z-up → Y-up swap)                 contours → polys → TSET tiles)
```

- **`scripts/pcd_to_obj.py`** (written for this work) voxelizes the point cloud and
  emits a triangle mesh of only the *exposed* voxel faces, performing the
  **Z-up → Y-up** axis swap so the output is already in Detour's convention.
- **`bake_tcbin.py`** (from the `voxelize_navmesh` toolchain) runs the Recast pipeline
  and serializes a **tiled TSET** `.tcbin`.

<p align="center">
  <img src="docs/images/navmesh-baked.jpg" width="70%" alt="The baked navigation mesh of the BitLab environment, split into walkable tiles">
  <br>
  <sub><b>The baked navigation mesh.</b> Walkable floor and ramps only, split into the fixed-size tiles that <code>dtTileCache</code> regenerates at runtime for obstacle carving.</sub>
</p>

Typical bake parameters (tune per scene / platform):

```bash
python3 bake_tcbin.py scene_yup.obj scene.tcbin \
  --cell-size 0.10 --cell-height 0.10 \
  --agent-radius 0.30 \        # rover half-width (Innok ≈ 0.35 m)
  --agent-max-climb 0.30 \     # raise for ramps/curbs; 0.15 misses ramp junctions
  --agent-max-slope 45 \
  --tile-size 32 --max-obstacles 128
```

The planner logs the tile count at startup:

```
[navmesh_planner] TSET NavMesh loaded: N tiles, dtTileCache active
```

A tile count that seems too low for the scene means **undercoverage** — usually a
simplified/holey source mesh, `--agent-max-climb` too low, or a missing/incorrect
Z-up handling. Use the full-geometry mesh, not a simplified collision hull.

### NavMesh formats

| | MSET (static) | TSET (dynamic) |
|---|---|---|
| Magic bytes | `'MSET'` | `'TSET'` |
| Dynamic obstacles | Not supported | Cylinders via `dtTileCache` |

The loader auto-detects the format. **Dynamic obstacle avoidance requires a TSET
navmesh.** MSET files plan static paths but log a warning that carving is unavailable.

---

## 6. Coordinate frames (read this before debugging)

Detour is **Y-up**; ROS 2 is **Z-up**. Every position is converted, consistently,
via `include/navmesh_planner/coordinate_utils.hpp`:

```
ROS → Detour:  d.x = ros.x   d.y = ros.z   d.z = ros.y
Detour → ROS:  ros.x = d.x   ros.y = d.z   ros.z = d.y
```

A forgotten swap is the single most common source of silent bugs (paths through
walls, wrong floor, wildly offset positions). All topics use the `map` frame.

---

## 7. Running the stack

Two parallel configuration profiles are maintained per package: **Isaac Sim** and
**real robot**. Switching profiles is a launch-file / YAML change only — no rebuild.

### 7.1 Isaac Sim (validation)

<p align="center">
  <img src="docs/images/isaac-sim.jpg" width="78%" alt="The rover in the reconstructed BitLab scene inside NVIDIA Isaac Sim">
  <br>
  <sub><b>Isaac Sim validation.</b> The reconstructed environment is imported as a visual + collision scene; OmniGraph supplies ground-truth pose on <code>/chassis/odom</code>, so planning and avoidance are tested repeatably before real-world deployment.</sub>
</p>

```bash
# Terminal 1 — obstacle detector (also brings up the map→World static TF + RViz)
ros2 launch dynamic_obstacle_detector isaac_sim.launch.py

# Terminal 2 — navmesh planner
ros2 launch navmesh_planner navmesh_planner.launch.py

# Terminal 3 — drive controller
ros2 launch drive_controller drive_controller.launch.py

# Terminal 4 — operator GUI (optional)
ros2 run navmesh_gui navmesh_gui
```

Then press **Play** in Isaac Sim and set a goal via RViz "2D Goal Pose" or the GUI.

- Isaac Sim odometry is published on **`/chassis/odom`** (the front-LiDAR world pose).
  All three sim nodes subscribe to it. **Do not use `/robotPose`** — leftover
  OmniGraph nodes leave it with conflicting publishers.
- **Do not run `lidar_pose_relay.py`** — it is superseded and adds a wrong
  `/robotPose` publisher.
- Stop any `world_pose_tf_relay.py` before launching (it double-parents `base_link`
  and breaks TF).

### 7.2 Real robot (Jetson Orin NX)

Prerequisite: bring up the **network link** and the **localizer** first.

```bash
# Terminal 0 — localization (separate workspace), publishes /pcl_pose
ros2 launch lidar_localization_ros2 lidar_localization.launch.py   # + hesai_tf

# Terminals 1–4 — navigation stack (real_robot2 profile)
ros2 launch dynamic_obstacle_detector real_robot2.launch.py
ros2 launch navmesh_planner          real_robot2.launch.py
ros2 launch drive_controller         real_robot2.launch.py
ros2 launch zed_speed_governor       real_robot2.launch.py         # person speed cap
```

The `real_robot2` profile uses `/pcl_pose` (NDT), `/lidar/points` (ZMQ bridge), and
`message_typ: PoseWithCovariance`. Ensure the cmd_vel ZMQ bridge is running with
exactly **one** receiver on the Innok and **one** sender on the Jetson.

### 7.3 Pre-drive verification checklist

- [ ] `ros2 topic hz /lidar/points` ≈ 10 Hz (real) — see network note below.
- [ ] `ros2 topic echo /pcl_pose --once` — localizer producing a pose.
- [ ] Planner log: `TSET NavMesh loaded: N tiles` with N > 0.
- [ ] RViz `/navmesh_polys` covers the actual floor you scanned.
- [ ] RViz `/static_map_voxelized` aligns with walls/floor (no vertical offset).
- [ ] Detector diag with no obstacles present: `static_filt ≈ total` (near-zero residual).
- [ ] Set a 2D Goal → `/planned_path` appears; step in front → path replans around you.

---

## 8. Things to be careful about

These are hard-won field lessons — violating them reintroduces known failures.

**Environment / build**
- `conda deactivate` before every ROS 2 command (Python 3.10 vs conda 3.13).
- `--symlink-install`: config/launch = restart only; C++ = rebuild. After a build,
  **relaunch** the node or it keeps the old params from `install/`.
- **Absolute paths in the YAML configs** (`navmesh_path`, `static_map_path`) must be
  updated for each machine — they are the first thing to break on a new deployment.

**Network (real robot)**
- The 10 Hz LiDAR depends on the **direct Gigabit cable** (a 100 Mb router drops it
  to ~4 Hz and degrades everything downstream). The **Innok side is not persistent** —
  after an Innok reboot, re-apply the network recovery and re-check
  `ros2 topic hz /lidar/points` ≈ 10 before testing.
- The cmd_vel receiver on the Innok **must** be started with
  `_connect_address:=tcp://<jetson-ip>:5556` or it silently connects to loopback.
- Exactly **one** cmd_vel receiver on the Innok and **one** sender on the Jetson.
- **Do not touch the Innok `enp3s0` interface** (Hesai LiDAR + emergency-stop path).

**Planner / safety tuning (invariants that must hold)**
- Safety-zone ordering must always be:
  `planner blind disc (obstacle_min_rover_dist 0.35) < safety bubble (safety_stop_radius 0.5) < detour clearance (cylinder_padding + bake agent radius ≈ 0.75) ≤ repulsion_radius (0.7)`.
  If the planner's blind disc grows *larger* than the safety bubble, the rover
  deadlocks nose-to-obstacle.
- `obstacle_replan_interval` (0.2 s) must stay **below** the drive controller's
  `obstacle_stop_cooldown` (1.0 s), so replan heartbeats keep the rover driving the
  avoidance route instead of stop-starting.
- `snap_vextent` must stay **≤ ~1.3**. The Bitlab navmesh has walkable polys on the
  floor *and* on ceiling beams / mezzanine; a large vertical snap window pulls the
  start/goal onto the ceiling ("path drawn on the ceiling").
- `path_block_clearance` must stay below the bake agent radius, or a freshly replanned
  path that skirts a carved hole re-triggers "blocked" every cycle.

**Perception**
- XY and Z inflation are **separate**. The floor is a static surface, so vertical
  inflation eats the bottom of every object standing on it — keep `inflation_radius_z`
  small (1) or short objects (e.g. a 45 cm box) get missed.
- Keep `activation_distance` modest (≈ 6 m on the real robot). Localization heading
  error displaces static points proportionally to range, producing phantom obstacles
  at long range.

---

## 9. Key parameters

### `navmesh_planner`
| Parameter | Typical | Notes |
|---|---|---|
| `navmesh_path` | `…/scene.tcbin` | TSET required for dynamic obstacles. |
| `pose_topic` / `message_typ` | `/pcl_pose` / `PoseWithCovariance` | Sim: `/chassis/odom` / `Odometry`. |
| `snap_vextent` | 1.0 | Vertical snap half-extent; keep ≤ 1.3. |
| `cluster_tolerance` | 0.25 | DBSCAN distance for the dynamic cloud. |
| `obstacle_radius_default` | 0.55 | Minimum carved cylinder radius. |
| `cylinder_padding` | 0.35 | Detour standoff vs. corridor width trade-off. |
| `obstacle_max_radius` | 1.1 | Hard cap on a carved radius (mis-merge guard). |
| `obstacle_decay_s` | 0.6 | Evict a tracked obstacle this long after last seen. |
| `obstacle_replan_interval` | 0.2 | Replan/heartbeat throttle. |

### `drive_controller`
| Parameter | Typical | Notes |
|---|---|---|
| `lookahead_distance` / `lookahead_time` | 0.6–1.2 | Speed-scaled lookahead. |
| `max_linear_speed` | 0.4–0.5 | Reduce for first real-world tests. |
| `safety_stop_radius` | 0.5 | Hard LiDAR safety bubble. |
| `repulsion_radius` / `repulsion_gain` | 0.70 / 2.0 | Shy-away layer. |
| `pose_latency` / `omega_smoothing_tau` | 0.20 / 0.15 | Latency comp. + jitter filter. |
| `speed_limit_timeout` | 1.0 | Fresh, non-negative `/speed_limit` caps speed. |

### `dynamic_obstacle_detector`
| Parameter | Sim | Real | Notes |
|---|---|---|---|
| `voxel_size` | 0.1 | 0.1 | Voxel resolution. |
| `activation_distance` | 10.0 | 6.0 | Long range → phantoms on the real robot. |
| `inflation_radius` / `inflation_radius_z` | 2 | 3 / 1 | XY vs Z absorb are independent. |
| `min_consecutive_scans` | 1 | 2 | Sim QT128 publishes partial slices → 1. |
| `ground_filter_height` | −2.0 | −2.5 | Relative to robot Z; must clear the floor. |
| `self_filter_*` | — | on | Sensor-frame chassis box (real robot only). |

### `zed_speed_governor`
| Parameter | Typical | Notes |
|---|---|---|
| `gate_distance` | 2.0 | Forward trigger distance for a person. |
| `corridor_half_width` | 1.0 | Someone to the side does not trigger. |
| `human_slow_speed` | 0.2 | Speed cap while triggered (detour-crawl). |
| `hold_time` | 1.2 | Hold after last detection; keep ≥ 1.0 s. |

---

## 10. Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `findNearestPoly failed for start/goal` | Pose off the navmesh, or snap window too small | Increase `snap_vextent` (≤ 1.3) or re-localize |
| Path drawn on the ceiling | `snap_vextent` too large | Lower to ~1.0 |
| Path planned but rover doesn't move | `message_typ` mismatch across configs | Align `message_typ` in all nodes |
| Dozens of phantom obstacles | Long `activation_distance` + localization drift | Lower `activation_distance`, raise `inflation_radius` |
| Short object (≈45 cm) missed | Vertical inflation eating its base | Lower `inflation_radius_z` to 1 |
| Rover stops right in front of a box | Safety-zone ordering violated | Keep `obstacle_min_rover_dist` < `safety_stop_radius` |
| Left/right path flapping / "wiggle" | Replan feedback loop / pose latency | Epoch re-arm + speed-scaled lookahead (already in) |
| Navmesh has floor but no ramp | `--agent-max-climb` too low at bake time | Re-bake with `--agent-max-climb 0.30`+ |
| LiDAR at ~4 Hz | Traffic going through the router | Restore the direct Gigabit link |
| No detector output, `static_filt ≈ total` | No dynamic object present (correct) | Place/observe a real dynamic object |

---

## 11. Repository layout

```
master_ws/
├── src/
│   ├── dynamic_obstacle_detector/   # voxel_subtraction_node (C++)
│   ├── navmesh_planner/             # navmesh_planner_node (C++) + third_party/ (Detour, DetourTileCache, fastlz)
│   ├── drive_controller/            # drive_controller_node (C++)
│   ├── zed_speed_governor/          # governor_node (Python)
│   └── navmesh_gui/                 # operator interface (Python)
├── scripts/
│   ├── pcd_to_obj.py                # PCD → Y-up OBJ (navmesh baking, step 1)
│   └── lidar_pose_relay.py          # (legacy; do not run in the current architecture)
├── cmd_vel_zmq_sender.py            # Jetson → Innok cmd_vel bridge
├── cmd_vel_zmq_receiver_ros1.py     # Innok-side ROS 1 cmd_vel receiver
└── data/, src/navmesh_planner/data/ # baked navmeshes (.tcbin) + reference clouds
```

Each package keeps its scene/platform parameters in `config/*.yaml` and a matching
`launch/*.launch.py`. Two profiles are maintained: Isaac Sim and real robot
(`real_robot2_*`). Large runtime artifacts (`build/`, `install/`, `log/`, some map
files) are git-ignored.

---

*Claude Opus 4.8 and 5.0 is used to develop and integrate parts of the code.*
