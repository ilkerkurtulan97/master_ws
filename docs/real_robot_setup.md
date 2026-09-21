# mesh_navigation on the REAL robot — from-scratch guide (Innok Heros + Jetson Orin)

How to take a raw environment scan and end up driving the real rover with
mesh_navigation, dynamic-obstacle avoidance included. Written for the Bitlab test, but
the pipeline is general.

> **Sibling doc:** `mesh_nav_isaac_sim_guide.md` is the *simulation* guide (Isaac Sim).
> This file is the *real-robot* guide. Where they differ, this file wins for hardware.

---

## 0. Hardware / process topology (who runs what)

```
INNOK HEROS (ROS1 onboard computer)              JETSON ORIN NX (ROS2 Humble — all nav runs here)
┌──────────────────────────────┐                 ┌───────────────────────────────────────────────┐
│ Hesai Pandar lidar            │                 │ lidar_zmq receiver  ──► /hesai/pandar (PointCloud2)
│ lidar_zmq_sender.py (PUB 5555)│ ══ZMQ 5555══►   │ ros2_lidar_localization (NDT) ──► TF map→base_link
│                               │                 │     + pose topic (e.g. /ndt_pose)              │
│ cmd_vel_zmq_receiver.py (SUB) │ ◄══ZMQ 5556══   │ mesh_nav (MBF + Dijkstra + MeshController)     │
│   └─► /cmd_vel ─► wheels      │                 │ dynamic_obstacle_detector ──► /dynamic_obstacles_cloud
└──────────────────────────────┘                 │ replan_trigger.py                             │
                                                  │ cmd_vel_zmq_sender.py (PUB 5556)              │
                                                  └───────────────────────────────────────────────┘
```

- **Lidar in:** robot → Jetson over ZMQ (your existing `lidar_zmq_bridge`, port 5555).
- **cmd_vel out:** Jetson → robot over ZMQ (NEW: `cmd_vel_zmq_sender.py` /
  `cmd_vel_zmq_receiver_ros1.py`, port 5556).
- The Jetson has **no `/clock`, no odom, no TF from the robot** — only lidar points.
  Localization (NDT) is what manufactures the pose/TF on the Jetson.

---

## 1. THE GOLDEN RULE — one common frame

Everything must live in **one map frame**. Three artifacts must share the *same origin*:

1. **NDT localization map** (the PCD `ros2_lidar_localization` matches the live lidar to).
2. **Planning mesh** (`blender_welded_mesh.h5`) that mesh_nav plans on.
3. **Detector static PCD** (`dynamic_obstacle_detector` subtracts this).

If these don't share an origin, the rover localizes in one place, plans on a mesh that's
shifted/rotated, and the detector flags the whole world as "dynamic." **Derive all three
from the SAME Bitlab point cloud.** Verify with the RViz overlay test in §6 BEFORE driving.

---

## 2. Do you need a fresh point cloud? (short answer: reuse, but re-verify alignment)

You're testing in **Bitlab**, which is already scanned — so you do **not** need a new
*scan*. What you need is for the three artifacts above to agree on a frame:

| Artifact | Reuse candidate | Action |
|---|---|---|
| NDT map PCD | `Desktop/real_life_maps/Bitlab_hq_subsampled.pcd` (69 MB) | Use this as your NDT map. It defines the `map` origin. |
| Planning mesh | `blender_welded_mesh.h5` | Reuse **only if** it shares the PCD's origin. If the §6 overlay is offset → rebuild it from the same Bitlab OBJ/PCD (§3). |
| Detector static PCD | the NDT map PCD, OR a fresh capture (§4) | Simplest: point the detector at the **same** Bitlab PCD as NDT. A fresh real-lidar capture (§4) better matches real sensor noise. |

So: **no new scan**, but confirm the welded mesh is in the Bitlab-PCD frame, and feed the
detector the same Bitlab map. If the mesh is offset (it came through the Isaac/Blender
path, which may have a different origin), rebuild it via §3 from `bitlab_yup.obj` /
`Bitlab_hq.obj`.

---

## 3. Pipeline: point cloud / OBJ → planning mesh (`.h5`)

`lvr2` reconstruct tools are **not built** in this workspace, so the PCD→surface step is
done externally (handheld-scanner software or Blender → an OBJ). You already have OBJs:
`Desktop/real_life_maps/bitlab_yup.obj`, `Desktop/Bitlab_hq.obj`. Starting from an OBJ:

### 3a. OBJ → exploded lvr2 HDF5
`mesh_map` converts OBJ→HDF5 on first load (via Assimp) **when `mesh_file != working_file`**.
Temporarily point a launch/config at the OBJ with a *different* working file:
```yaml
mesh_map:
  mesh_file:         "/home/ilker/.../bitlab.obj"      # input OBJ
  mesh_working_file: "/home/ilker/.../bitlab_clean.h5" # different path -> triggers conversion
  mesh_part: "Low_Bitlab"
  mesh_working_part: "Low_Bitlab"
```
Launch once; let it write `bitlab_clean.h5`, then Ctrl-C. This h5 is an **exploded
triangle soup** (Assimp's JoinIdenticalVertices is broken here) — not usable directly.

### 3b. Weld + remesh → planning mesh  (`weld_mesh.py`)
```bash
cd /home/ilker/meshnavigation_ws
conda deactivate                              # ROS/Open3D need system python3
python3 weld_mesh.py bitlab_clean.h5 bitlab_welded.h5 Low_Bitlab
```
This (1) merges coincident vertices → shared edges (so Dijkstra's wavefront can propagate),
(2) vertex-cluster remeshes @0.20 m to uniform resolution (so MeshController's
`searchContainingFace` works), removes non-manifold edges, keeps the largest component, and
recomputes normals. Output is the runtime mesh.

### 3c. Verify the mesh  (`check_mesh.py`)
```bash
python3 check_mesh.py    # edit the path inside if needed; expect "V/F < 1 -> GOOD"
```
`V/F < 1` (≈0.5) and ONE connected component = good. `V/F ≈ 3` = still exploded (weld failed).

### 3d. Point mesh_nav at it
In `mesh_nav_params.yaml` set BOTH to the welded file (equal ⇒ mesh_map loads it directly,
skipping the broken Assimp path):
```yaml
mesh_file:         "/home/ilker/.../bitlab_welded.h5"
mesh_working_file: "/home/ilker/.../bitlab_welded.h5"
```
> First clean launch, mesh_map writes the computed layers back INTO this h5. Don't Ctrl-C in
> the first ~10 s (corrupts it → `-6` crash). After one clean run, `chmod 444` it to be safe.

---

## 4. Pipeline: static reference PCD for the detector (optional but recommended)

The detector subtracts a static map from the live lidar; whatever remains is "dynamic."
Two options:
- **Reuse the NDT map PCD** (simplest; guaranteed same frame). Point
  `real_robot_params.yaml: static_map_path` at it.
- **Capture a fresh one from the real lidar** (best subtraction quality). With NDT running
  and the lab empty of moving things, drive/stand and record:
  ```bash
  python3 src/dynamic_obstacle_detector/scripts/capture_static_pcd.py \
      --topic /hesai/pandar --duration 30 --out .../bitlab_static.pcd
  ```
  This must be in the **map frame** — make sure NDT/TF is publishing while you capture, or
  the points won't align with the mesh. Then set `static_map_path` to it.

---

## 5. Pipeline: localization + the ZMQ bridges

### 5a. NDT localization (this is the real-robot pose source — replaces Isaac odom)
Run your `ros2_lidar_localization` (NDT) on the Jetson with:
- the Bitlab **map PCD** (§2),
- the live lidar `/hesai/pandar`,
- an **initial pose** (place the rover at a known start, or set it in RViz),
- output: it must publish the **`map → base_link` TF** (and a pose topic for the detector).

mesh_nav, the detector, and `replan_trigger` all rely on this TF. No NDT ⇒ nothing works.

### 5b. Lidar IN (you already have this)
Robot: `lidar_zmq_sender.py` (PUB `tcp://0.0.0.0:5555`).
Jetson: your lidar receiver → republishes `/hesai/pandar`.

### 5c. cmd_vel OUT (NEW — built this session)
- **Jetson (ROS2):** `cmd_vel_zmq_sender.py` — subscribes MBF's `/mbf_mesh_nav/cmd_vel`
  (TwistStamped), PUBs over ZMQ on `tcp://0.0.0.0:5556`. Launched by `mesh_nav_real.launch.py`.
  (It replaces `cmd_vel_relay.py`; don't run that on the robot.)
- **Robot (ROS1):** copy `cmd_vel_zmq_receiver_ros1.py` into your catkin
  `lidar_zmq_bridge/scripts/` (rename to `cmd_vel_zmq_receiver.py`, `chmod +x`,
  add to `CMakeLists` install or `rosrun` directly):
  ```bash
  rosrun lidar_zmq_bridge cmd_vel_zmq_receiver.py \
      _connect_address:=tcp://<JETSON_IP>:5556 _output_topic:=/cmd_vel _timeout:=0.5
  ```
  It deserializes and publishes `geometry_msgs/Twist`, and **safety-stops** (zero Twist)
  if no command arrives within `timeout` (Jetson died / network dropped). Keep that on.

Wire format is a fixed 60-byte little-endian struct (`<4s 2I 6d`: magic `CMDV`, stamp,
linear xyz, angular xyz) — identical on both ends. Pattern mirrors your lidar bridge
(PUB/SUB + `CONFLATE=1`). Default ports: lidar 5555, cmd_vel 5556.

---

## 6. Bring-up order & verification (do these IN ORDER)

```text
ON THE ROBOT (ROS1):
  1. roscore + drivers up; Hesai publishing.
  2. rosrun lidar_zmq_bridge lidar_zmq_sender.py
  3. rosrun lidar_zmq_bridge cmd_vel_zmq_receiver.py _connect_address:=tcp://<JETSON_IP>:5556
     (rover should sit still — no commands yet — and safety-stop is the default state)

ON THE JETSON (ROS2):
  4. lidar receiver up:    ros2 topic hz /hesai/pandar          # lidar arriving
  5. NDT localization up + initial pose set:
       ros2 run tf2_ros tf2_echo map base_link                 # TF exists & is sane
  6. ros2 launch mesh_nav_isaac_sim mesh_nav_real.launch.py
```

**Verification checklist (RViz, Fixed Frame = `map`):**
- [ ] **Overlay test (THE alignment check):** add a PointCloud2 on `/hesai/pandar` and a
      MeshDisplay on `/mbf_mesh_nav/mesh`. The live lidar must overlay the mesh walls. If it's
      shifted/rotated → the mesh is in the wrong frame → rebuild it (§3) from the NDT-map PCD,
      or fix your NDT map origin. **Do not drive until this overlays.**
- [ ] `ros2 topic hz /dynamic_obstacles_cloud` ~empty when static; lights up on the cube.
- [ ] Send a 2D Nav Goal (tool topic `/goal_pose`) → a path appears, rover drives.
- [ ] Drag an object into the path → `replan_trigger` logs "re-issued goal", path bends.
- [ ] Kill the Jetson launch mid-drive → rover STOPS within `timeout` (watchdog works).

---

## 7. What changed vs the sim launch (quick diff)

| Sim (`mesh_nav_isaac_sim.launch.py`) | Real (`mesh_nav_real.launch.py`) |
|---|---|
| `use_sim_time: True` (needs `/clock`) | `use_sim_time: False` |
| `map_world_anchor.py` (cancels Isaac spawn offset) | **removed** — TF comes from NDT |
| `cmd_vel_relay.py` (Twist→/cmd_vel local) | **`cmd_vel_zmq_sender.py`** (→ ZMQ → robot) |
| lidar `/front_3d_lidar/lidar_points` | `/hesai/pandar` (detector `real_robot_params.yaml`) |
| pose `/chassis/odom` (Isaac) | NDT pose/TF |
| detector `isaac_sim_params.yaml` | `real_robot_params.yaml` |
| planner Dijkstra (NOT CVP) | same — Dijkstra (CVP crashes this mesh) |

Files added this session: `cmd_vel_zmq_sender.py`, `cmd_vel_zmq_receiver_ros1.py`,
`src/dynamic_obstacle_detector/config/real_robot_params.yaml`,
`src/mesh_nav_isaac_sim/launch/mesh_nav_real.launch.py`.

---

## 8. Things to CONFIRM before tomorrow (grep `CONFIRM` in the new files)

1. **NDT output:** exact pose topic + type (`/ndt_pose` PoseWithCovariance? `/odom`
   Odometry?) → set in `real_robot_params.yaml` (`pose_topic`, `message_typ`).
2. **NDT publishes `map → base_link` TF** (not just a pose). mesh_nav needs the TF.
3. **base frame name** on the Innok (`base_link`?) → matches `robot_frame` in the launch.
4. **lidar topic name** your Jetson receiver publishes (assumed `/hesai/pandar`).
5. **Jetson IP** in the robot-side `cmd_vel_zmq_receiver` `_connect_address`.
6. **Mesh alignment** (the §6 overlay) — the single most likely thing to bite.
7. **Lidar mount height** → detector `ground_filter_height` so the real floor is filtered.

---

## 9. Troubleshooting

- **Every node hangs at startup** → `use_sim_time` still true somewhere / no `/clock`. The
  real launch sets it false; make sure you didn't source a stray param.
- **No path / rover won't move** → no `map→base_link` TF (`tf2_echo map base_link`). Fix NDT.
- **Lidar doesn't overlay the mesh** → frame mismatch (§1/§6). Rebuild mesh from the NDT-map PCD.
- **Whole floor flagged dynamic** → detector `ground_filter_height`/`min_z` wrong for the real
  lidar height, or static PCD in the wrong frame.
- **Rover runs away after a glitch** → the ROS1 receiver watchdog isn't running / `timeout` too
  high. It must publish zero Twist on staleness.
- **`-6 corrupted` on launch** → killed mbf mid-write to the h5; restore a `.bak` or rebuild (§3).
- **`HalfEdgeMesh panicked! / Path 0m` spam** → CVP planner selected. Keep Dijkstra
  (`dijkstra_mesh_planner/DijkstraMeshPlanner`).
