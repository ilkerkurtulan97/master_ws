# mesh_navigation × Isaac Sim — Operating Guide & Session Notes

**Scope:** running `naturerobots/mesh_navigation` (Move Base Flex + mesh_map +
dijkstra_mesh_planner + mesh_controller) inside NVIDIA Isaac Sim on the Bitlab
parking-garage scene (`blender_with_rover.usd`, Nova Carter rover). This is the
second navigation contender vs. the existing recast/Detour stack in `master_ws`.

Last updated: 2026-06-09. Read the **Quick Start** and **Golden Rules** first; the
rest is reference + the history of bugs so they don't get re-introduced.

**Status:** static navigation **working**; **dynamic obstacle avoidance working**
(drag a physical object in front of the rover → it reroutes around it). Planner =
**DijkstraMeshPlanner** (CVP was tried and rejected — see *Why Dijkstra, not CVP*).

---

## Quick Start (the ONE-command workflow)

1. **Open Isaac Sim** with `/home/ilker/Downloads/blender_with_rover.usd`.
2. **Press PLAY in Isaac first.** This starts `/clock`, `/chassis/odom`, the lidar,
   and the TF tree. Launching before Play makes mbf (which uses `use_sim_time`)
   **block forever** = "doesn't start properly." Optionally confirm:
   `ros2 topic hz /clock` (should be ~18 Hz).
3. **One launch starts the whole stack:**
   ```bash
   source /opt/ros/humble/setup.bash
   source /home/ilker/meshnavigation_ws/install/setup.bash
   ros2 launch mesh_nav_isaac_sim mesh_nav_isaac_sim.launch.py
   ```
4. In **RViz**: Fixed Frame = `map`; set the **"2D Nav Goal" tool topic to
   `/goal_pose`**; send a goal → rover plans + drives.

The launch now starts **everything** (no separate terminals needed):
`map_world_anchor.py` · `mbf_mesh_nav` · `cmd_vel_relay.py` · `goal_bridge.py` ·
`voxel_subtraction_node` (obstacle perception) · `replan_trigger.py` (reroute trigger).

The rover does **not** need to be at spawn.

**Dynamic obstacle test:** send a goal so the rover drives, then drop/drag a physical
object (e.g. an Isaac *Create → Mesh → Cube*) into its path → the path reroutes around
it. With nothing in the way the rover drives the straight/optimal route.

---

## Golden Rules (learned the hard way)

- **Isaac must be Playing before you launch.** No `/clock` → mbf hangs.
- **Exactly ONE `map_world_anchor` may run.** The launch starts one. Never run it
  by hand too — duplicates publish conflicting `map→World` → the lidar flickers/tilts.
  Check: `pgrep -af "map_world_anchor|goal_bridge|mbf_mesh_nav"` → one of each.
- **Do NOT edit `map_world_anchor.py`.** The original works (continuous full-rotation
  recompute). Known-good copy: `/home/ilker/Desktop/backup_mesh/meshnavigation_ws/`.
  If it ever breaks, restore that file + the launch from there.
- **Don't Ctrl-C `mbf_mesh_nav` during the first ~10 s of a launch.** It writes
  computed layers back into `blender_welded_mesh.h5`; a kill mid-write corrupts the
  file → `-6 corrupted size`/`double-linked list` crash on the next launch.
- **Never Stop/Restart Isaac while the stack runs** (clock reset → `TF_OLD_DATA`
  flood / "jump back in time"). Stop the ROS stack first.
- **`master_ws/` is read-only.** Never edit code in `master_ws/`. To reuse a package
  (we did this for `dynamic_obstacle_detector`), **copy the source** into
  `meshnavigation_ws/src/` and build/modify the COPY. The original stays pristine.
- **Keep the planner = `dijkstra_mesh_planner/DijkstraMeshPlanner`.** CVP hard-crashes
  on this mesh (see *Why Dijkstra, not CVP*). Do not swap it without repairing the mesh.

---

## Architecture

```
Isaac Sim (blender_with_rover.usd, Nova Carter)   [PLAY first]
  pub: /clock, /chassis/odom, /front_3d_lidar/lidar_points
  TF:  World → odom → base_link → front_3d_lidar   (SPAWN-anchored, not stage origin)
  sub: /cmd_vel

ros2 launch mesh_nav_isaac_sim mesh_nav_isaac_sim.launch.py starts:
  ├─ map_world_anchor.py   /chassis/odom + TF → publishes map→World (full rotation)
  ├─ mbf_mesh_nav          mesh_map (blender_welded_mesh.h5) + Dijkstra + MeshController
  │                        pub: /mbf_mesh_nav/mesh, /mbf_mesh_nav/path, /mbf_mesh_nav/cmd_vel
  ├─ cmd_vel_relay.py      /mbf_mesh_nav/cmd_vel (TwistStamped) → /cmd_vel (Twist)
  ├─ goal_bridge.py        /goal_pose (RViz/navmesh_gui) → /mbf_mesh_nav/move_base ACTION
  │                        (snaps goal z to robot floor height)
  ├─ voxel_subtraction_node   (vendored dynamic_obstacle_detector) PERCEPTION:
  │                        /front_3d_lidar/lidar_points + /chassis/odom − static blender.pcd
  │                        → /dynamic_obstacles_cloud  (sparse dynamic-only points, map frame)
  └─ replan_trigger.py     REROUTE TRIGGER: if an obstacle in /dynamic_obstacles_cloud sits on
                           the current path AHEAD of the rover, re-publishes the goal → fresh plan

Dynamic obstacle data flow (avoidance):
  /dynamic_obstacles_cloud → mesh_map ObstacleLayer (marks hit faces lethal)
    → MaxCombinationLayer (OR with steepness) → InflationLayer (re-inflate)
    → MeshMap::layerChanged updates planner vertex_costs/edge weights
    → (replan_trigger re-issues goal) → Dijkstra plans AROUND the inflated obstacle

Goal:  RViz "2D Nav Goal" (topic = /goal_pose)  OR  navmesh_gui  → goal_bridge → MBF
```

---

## Key files

| File | Role |
|---|---|
| `src/mesh_nav_isaac_sim/launch/mesh_nav_isaac_sim.launch.py` | starts the whole stack (symlink-installed) |
| `src/mesh_nav_isaac_sim/config/mesh_nav_params.yaml` | all mbf/mesh_map/layer/controller params (symlink-installed) |
| `map_world_anchor.py` | publishes `map→World` (DO NOT EDIT; restore from backup if broken) |
| `goal_bridge.py` | `/goal_pose` → MBF `move_base` action; snaps goal z to floor |
| `cmd_vel_relay.py` | TwistStamped → Twist for Isaac |
| `replan_trigger.py` | re-issues the goal when an obstacle blocks the path ahead → reroute |
| `src/dynamic_obstacle_detector/` | **vendored** from `master_ws` (copy, not edited in place); voxel/PCD subtraction → `/dynamic_obstacles_cloud` |
| `src/dynamic_obstacle_detector/data/blender.pcd` | static reference map for subtraction (self-contained copy) |
| `blender_welded_mesh.h5` | the runtime nav mesh (geometry-only after a clean rewrite) |
| `blender_welded_mesh.h5.bak`, `.bak2` | backups of the h5 |
| `/home/ilker/Desktop/backup_mesh/meshnavigation_ws/` | **known-good full backup** |

`--symlink-install` is used: edits to YAML / launch / the standalone `*.py` scripts
go live **without** a rebuild. Only C++ changes need `colcon build`.

---

## Bugs found & fixed THIS session

### 1. Lidar cloud tilted / didn't overlay the mesh (MAIN issue)
- **Symptom:** path/planning fine, but the 3D lidar appeared rotated/tilted vs the mesh;
  sometimes the rover drove to the wrong place.
- **Real cause:** `map→World` was wrong. Contributors, all eventually traced to the anchor:
  (a) **duplicate anchor processes** (debugging leftovers) publishing conflicting
  latched `map→World` → flicker; (b) edits that **froze** the anchor or **hardcoded it
  with zeroed roll/pitch** — zeroed roll/pitch *is* the tilt.
- **Fix:** restored the **original** `map_world_anchor.py` + launch from
  `Desktop/backup_mesh`. The original continuously recomputes `map→World` with the
  **full quaternion** and self-corrects every frame. Ensure only one runs.
- **Lesson:** the original anchor works — don't "improve" it. `map→World` is the offset
  between the mesh/map frame and Isaac's **spawn-anchored** TF "World" (NOT the USD
  stage origin, even though the `Low_Bitlab` prim is at identity in the stage).

### 2. `mbf_mesh_nav` crash at startup: `-6 corrupted size` / `double-linked list`
- **Cause:** mbf writes computed layers back into `blender_welded_mesh.h5`
  (`mesh_file == mesh_working_file`). A run killed **mid-write** leaves corrupt
  `vertex_/edge_/face_attributes` groups; lvr2 then corrupts the heap reading them.
- **Fix:** rewrite the h5 **fresh, geometry-only** (drop the `*_attributes` groups;
  full rewrite, not just an h5py delete). Next launch recomputes them cleanly.
- **Prevent:** don't Ctrl-C during the first ~10 s. Durable option: after one clean
  launch, `chmod 444 blender_welded_mesh.h5` (mbf then only *reads* the cached layers).

### 3. Planned path not visible in RViz (but rover drives)
- **Cause:** `/mbf_mesh_nav/path` is published **TRANSIENT_LOCAL (latched)** once
  (plan-once, `planner_frequency: 0.0`). A RViz Path display with **Volatile** QoS that
  connects after the publish never receives it.
- **Fix (RViz only):** Path display on `/mbf_mesh_nav/path` → set **Durability =
  Transient Local**, **Reliability = Reliable**.

### 4. Dynamic obstacle avoidance — **SOLVED** (full write-up below)

---

## Findings / gotchas reference

- **`map→World` is between two fixed frames → a constant**, but the anchor *computes*
  it from `odom ∘ inv(World→lidar TF)`. That product was observed to be **pose-dependent**
  in a corrupted state (yaw -0.67° at spawn vs ~-72° off-spawn). In healthy operation the
  continuous-recompute anchor stays correct; the pose-dependence we measured coincided with
  a messed-up odom/TF state during debugging. If alignment ever drifts as the rover moves,
  suspect odom/TF inconsistency, not the anchor code.
- **ICP of one lidar scan to the dense USD building cloud is unreliable** (many local
  minima; gave fitness 1.0 one run, 0% the next). Don't use it to derive `map→World`.
- **`mesh_map` loads `.h5` directly (bypasses broken Assimp) only when
  `mesh_file == mesh_working_file`.** Assimp's JoinIdenticalVertices is broken in this
  build (explodes OBJ into triangle soup → Dijkstra can't leave the goal triangle).
- **The ramp** in Isaac (separate `Cube/Cube_001/Cube_002` USD prims) is **not** in the
  nav mesh (mesh = `Low_Bitlab` only). That's why the ramp doesn't appear in the mesh map.
  Real ramp slope ~20°. Deferred; `blender_ramp_mesh.h5` was built but not wired in.
- **SteepnessLayer** threshold is radians (angle from vertical); lethal if steepness >
  threshold. Currently 0.4 rad (~23°).
- **Isaac `/chassis/odom` twist is 0** (only pose populated) → mbf's "ignoring velocity
  commands" check is disabled in the YAML (`cmd_vel_ignored_tolerance: 0.0`).
- **Goal z:** RViz 2D Nav Goal publishes z=0 but the floor is ~ -1.2 m; `goal_bridge`
  snaps goal z to the robot's current floor height (same-floor goals only).

---

## Dynamic obstacle avoidance — how it works (SOLVED)

Goal (matching the recast/`master_ws` behavior): drag a physical object in front of the
rover → the global path reroutes around it; remove it → the path straightens.

### The architecture we ended up with
We use **mesh_navigation's own avoidance core** (ObstacleLayer → combination → inflation →
planner), exactly like the official tutorial. The tutorial's Gazebo demo gets its two
surrounding pieces from elsewhere; in Isaac we supply equivalents:

| Piece | Tutorial (Gazebo) | Ours (Isaac) |
|---|---|---|
| **Perception** (segment the obstacle) | RMCL mesh-localization `outlier_scan` → `obstacle_points` | **vendored `dynamic_obstacle_detector`** (voxel/PCD subtraction) → `/dynamic_obstacles_cloud` |
| **Avoidance core** | ObstacleLayer→combination→inflation→planner | **same** (unchanged) |
| **Replan trigger** | relies on MBF | **`replan_trigger.py`** (MBF's replanning is broken here — see below) |

Why a *segmented* cloud and not raw lidar: ObstacleLayer projects every point straight
down onto the mesh and marks the hit face lethal. Raw lidar includes floor returns →
the whole floor goes lethal, **and** raw dense lidar deterministically crashed
`processPointCloud` (heap corruption) in earlier attempts. The detector outputs a sparse,
**dynamic-only, map-frame** cloud (static building subtracted out), which is the correct,
tutorial-aligned input and is what made ObstacleLayer stable.

### The two real problems we solved

**Problem A — MBF's continuous replanning is broken in this build.** Setting
`planner_frequency > 0` (the "normal" way to replan) fails two ways:
1. It **never arms** `MoveBaseAction::replanning_period_` at startup (it's only set inside
   the param-callback, so it stays 0 → off by default), and
2. when forced on at runtime it replans **UNTHROTTLED (~1600 Hz)** because
   `AbstractPlannerExecution`'s run-loop reads `frequency_` but never uses it to rate-limit
   → it thrashes mbf into an unresponsive "Node not found" state.

   So we keep `planner_frequency: 0.0` (**plan-once**, which also keeps driving smooth —
   every replan calls `MeshController::setPlan` which resets `current_face_` and risks a
   re-bootstrap stall). With plan-once, the obstacle layer keeps updating costs live, but
   **nothing re-invokes the planner** — the path is computed once and never reflects a
   newly-appeared obstacle. That was the *entire* reason "it doesn't reroute."

**Problem B — provide the missing replan trigger.** `replan_trigger.py` mirrors how
`master_ws/navmesh_planner` does it (`is_path_blocked() → plan_and_publish()`): it watches
`/dynamic_obstacles_cloud`, `/mbf_mesh_nav/path` (TRANSIENT_LOCAL latched), and the rover
TF; when an obstacle point lands within `block_dist` (0.6 m) of the current path **ahead**
of the rover, it re-publishes the active goal on `/goal_pose` → `goal_bridge` → a fresh
plan-once. Because the obstacle layer has already pushed the obstacle into the planner's
edge weights, that fresh plan routes **around** it. Throttled (`min_replan_period` 1.5 s)
so it can't thrash; self-limiting (once the new path clears the obstacle, nothing blocks).

### Why the cost actually reaches the planner (verified in source)
`ObstacleLayer::processPointCloud` marks lethal vertices → `notifyChange` →
`LayerManager` propagates down the DAG via `onInputChanged`: `MaxCombinationLayer` ORs the
obstacle+steepness lethals → `InflationLayer` re-reads `input->lethals()` and fully
re-inflates → only the **default layer (inflation)** triggers `MeshMap::layerChanged`,
which updates `vertex_costs` + edge weights the planner reads. `cost_limit: 0.99` (below
the inflation `inscribed_value: 1.0`) makes Dijkstra avoid the whole inflated ring, not
just the lethal core. **The avoidance math was never the problem — the missing replan
trigger was.**

### Key params that make it work (`mesh_nav_params.yaml`)
- `obstacle.topic: /dynamic_obstacles_cloud`, `qos: Reliable` (match the detector),
  `robot_height: 2.5` (project person/cube-height points down to the floor face),
  `max_obstacle_dist: 1000000.0` (cloud is map-frame, so the layer's `p.norm()` filter
  measures from the **map origin**, not a sensor — keep huge or valid points get dropped).
- `inflation`: `inflation_radius: 1.0`, `inscribed_radius: 0.4`, `inscribed_value: 1.0`,
  `lethal_value: .inf`; `dijkstra.cost_limit: 0.99`.
- `planner_frequency: 0.0` (plan-once; replanning done by `replan_trigger.py`, NOT MBF).

---

## Why Dijkstra, not CVP (valid reasons we reverted)

mesh_navigation ships two global planners; the official tutorial demo uses **CVP**
(`cvp_mesh_planner/CVPMeshPlanner`), which produces a smooth vector-field path instead of
Dijkstra's vertex-to-vertex polyline. We tried swapping to CVP to give mesh_nav its best
planner for the recast comparison. **It does not work on this mesh** — these are the
concrete reasons we switched back to `dijkstra_mesh_planner/DijkstraMeshPlanner`:

1. **CVP hard-crashes the navigation server.** With CVP, every `GetPath` returned
   `Path length: 0m` accompanied by
   `ERROR: Could not find a valid path, while back-tracking from the goal: HalfEdgeMesh
   panicked!`, spamming ~400×/second, and **`mbf_mesh_nav` then aborted with SIGABRT
   (exit code -6)**. The rover never moved on a 2D Nav Goal. This is a fatal crash, not a
   tuning issue — no parameter fixes it.
2. **Root cause is a mesh-topology mismatch, not a config error.** CVP propagates a
   *continuous* vector field outward from the goal and then **back-tracks** from the start
   by walking the lvr2 half-edge topology. That walk hit a fatal assert (`HalfEdgeMesh
   panicked!`) on this mesh's defects (boundary edges / thin / near-non-manifold faces).
   Dijkstra just hops vertex-to-vertex along edges and **tolerates the same mesh fine**.
3. **The mesh was built for Dijkstra, not CVP.** `blender_welded_mesh.h5` was welded +
   vertex-clustered + non-manifold-stripped specifically to satisfy Dijkstra +
   MeshController. CVP needs a cleaner mesh than that; fixing it would mean a non-trivial
   mesh-repair pass with an uncertain payoff, and CVP could remain fragile afterward.
4. **It doesn't block the actual goal (stack comparison).** DijkstraMeshPlanner is a
   first-class, native mesh_navigation planner — not a workaround. It gives a **stable,
   fully-working** mesh_nav stack (static + dynamic avoidance) to benchmark against the
   recast/Detour stack. CVP would derail the comparison for no guaranteed benefit.

**Decision:** keep Dijkstra. The CVP attempt is preserved in git as the diff against the
baseline commit, and a safety commit (`e5af518`, "Working dynamic obstacle avoidance …
Dijkstra baseline") was made **before** the CVP swap so it could be reverted instantly —
which is exactly what we did (`git checkout HEAD -- …/mesh_nav_params.yaml`).
**If you ever revisit CVP, repair/clean the mesh first; expect it may still panic.**

---

## Paused / pending work
- Ramp not in nav mesh (see findings). `blender_ramp_mesh.h5` built, not wired in.
- (Optional) `replan_trigger.py` re-issues the goal as a fresh `move_base` action, which
  resets the MeshController (`current_face_`) — there can be a brief hitch at the moment of
  reroute. Acceptable today; a cleaner path would be patching MBF to rate-limit its own
  replanner (C++ rebuild) instead of the external trigger.

---

## Recovery cheatsheet

```bash
# Clean slate (kill leftovers that cause duplicate-anchor tilt):
pkill -9 -f "ros2 launch mesh_nav_isaac_sim"
pkill -9 -f "map_world_anchor|goal_bridge|cmd_vel_relay|mbf_mesh_nav|voxel_subtraction|replan_trigger"

# Anchor broken / lidar tilted → restore the known-good files:
cp /home/ilker/Desktop/backup_mesh/meshnavigation_ws/map_world_anchor.py \
   /home/ilker/meshnavigation_ws/map_world_anchor.py
cp /home/ilker/Desktop/backup_mesh/meshnavigation_ws/src/mesh_nav_isaac_sim/launch/mesh_nav_isaac_sim.launch.py \
   /home/ilker/meshnavigation_ws/src/mesh_nav_isaac_sim/launch/mesh_nav_isaac_sim.launch.py

# mbf "-6 corrupted" at startup → rewrite the h5 geometry-only (drop *_attributes groups),
# backups are blender_welded_mesh.h5.bak / .bak2

# mbf "-6" + "HalfEdgeMesh panicked! / Path length: 0m" spam → CVP planner is selected.
# Restore Dijkstra (the working baseline):
git -C /home/ilker/meshnavigation_ws checkout e5af518 -- \
   src/mesh_nav_isaac_sim/config/mesh_nav_params.yaml

# Verify exactly one of each is running:
pgrep -af "map_world_anchor|goal_bridge|mbf_mesh_nav|voxel_subtraction|replan_trigger"

# Dynamic obstacle avoidance not rerouting? Check, in order:
ros2 topic hz /dynamic_obstacles_cloud     # perception alive (lights up when obstacle present)
ros2 topic echo /mbf_mesh_nav/path --once  # planner produced a path (TRANSIENT_LOCAL)
# replan_trigger logs "Obstacle blocking path ahead ... re-issued goal" when it fires
```
