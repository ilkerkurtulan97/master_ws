# RUNBOOK — mesh_navigation real-robot test (Bitlab, tomorrow)

Step-by-step run-day guide for driving the **Innok Heros** with mesh_navigation on the
**Jetson Orin NX**. Do the steps **in order**. Background/theory lives in
`real_robot_setup.md`; this file is the operational checklist for the test itself.

> **One-line summary:** lidar comes from the robot over ZMQ → NDT makes the pose →
> mesh_nav plans on `bitlab_real_mesh.h5` → velocity goes back to the robot over ZMQ.
> The robot computer only drives; **all navigation runs on the Jetson.**

---

## 0. Topology (who runs what)

```
INNOK HEROS (ROS1)                          JETSON ORIN NX (ROS2 Humble — all nav)
┌───────────────────────────┐               ┌──────────────────────────────────────────┐
│ Hesai Pandar lidar         │               │ lidar_zmq receiver  ─► /hesai/pandar       │
│ lidar_zmq_sender.py PUB 5555│ ═ZMQ 5555═►   │ ros2_lidar_localization (NDT)              │
│                            │               │     ─► TF map→base_link + /ndt_pose         │
│ cmd_vel_zmq_receiver.py SUB │ ◄═ZMQ 5556═   │ mesh_nav (MBF+Dijkstra+MeshController)     │
│   └► /cmd_vel ─► wheels     │               │ dynamic_obstacle_detector                  │
└───────────────────────────┘               │ replan_trigger.py · cmd_vel_zmq_sender 5556 │
                                             └──────────────────────────────────────────┘
```

---

## 1. What is already prepared (no action needed)

| Artifact | Path | State |
|---|---|---|
| Planning mesh | `bitlab_real_mesh.h5` (+ `.bak`) | Built from `Bitlab_nav.pcd`; 68.4k v / 130k f, one component, lvr2-clean. |
| NDT / detector map | `~/Desktop/real_life_maps/Bitlab_nav.pcd` | 1.11M pts, lowercase `intensity`. Feeds NDT **and** the detector. |
| Real launch | `src/mesh_nav_isaac_sim/launch/mesh_nav_real.launch.py` | Loads the real mesh, `use_sim_time=False`, ZMQ cmd_vel. Built into `install/`. |
| Detector config | `src/dynamic_obstacle_detector/config/real_robot_params.yaml` | `static_map_path → Bitlab_nav.pcd`. |
| cmd_vel bridges | `cmd_vel_zmq_sender.py` (Jetson), `cmd_vel_zmq_receiver_ros1.py` (Innok) | 60-byte `CMDV` struct, watchdog safety-stop. |

**The mesh, NDT map, and detector map all share one origin** (all derived from
`Bitlab_nav.pcd`) → the golden-rule alignment is satisfied by construction. You still
verify it with the overlay test in §5 before driving.

---

## 2. CONFIRM these before you start (the only things I couldn't know)

Edit and re-check these — they are real-stack facts. Grep `CONFIRM` in the files.

1. **NDT pose topic + type** → `real_robot_params.yaml`: `pose_topic` / `message_typ`
   (e.g. `/ndt_pose` + `PoseWithCovariance`, or `/odom` + `Odometry`).
2. **NDT publishes the `map → base_link` TF** (not just a pose topic). mesh_nav needs the TF.
3. **Innok base frame name** = `base_link`? → `robot_frame` in `mesh_nav_real.launch.py`.
4. **Lidar topic** your Jetson receiver publishes = `/hesai/pandar`? → `cloud_topic`.
5. **Jetson IP** → used by the Innok-side `cmd_vel_zmq_receiver` `_connect_address`.
6. **Lidar mount height** → detector `ground_filter_height` (so the real floor is filtered,
   low obstacles kept).

If any YAML/launch value changes, no rebuild is needed (symlink-install) **except** if you
add a brand-new file — then `colcon build --packages-select <pkg> --symlink-install`.

---

## 3. Bring-up — ON THE ROBOT (ROS1, Innok)

```bash
# 3.1  roscore + drivers; confirm the Hesai is publishing.
rostopic hz /<your_hesai_topic>

# 3.2  ship lidar to the Jetson
rosrun lidar_zmq_bridge lidar_zmq_sender.py          # PUB tcp://0.0.0.0:5555

# 3.3  cmd_vel return path + SAFETY watchdog (rover stays still — no commands yet)
rosrun lidar_zmq_bridge cmd_vel_zmq_receiver.py \
    _connect_address:=tcp://<JETSON_IP>:5556 \
    _output_topic:=/cmd_vel _timeout:=0.5
```
> If `cmd_vel_zmq_receiver.py` isn't in the catkin pkg yet: copy
> `cmd_vel_zmq_receiver_ros1.py` into `lidar_zmq_bridge/scripts/`, `chmod +x`, rebuild.
> The watchdog publishes a **zero Twist** if no command arrives within `timeout` — keep it on.

---

## 4. Bring-up — ON THE JETSON (ROS2 Humble)

Open a terminal per step; source ROS + the workspace in each:
```bash
source /opt/ros/humble/setup.bash
source /home/ilker/meshnavigation_ws/install/setup.bash
```

```bash
# 4.1  lidar receiver up -> /hesai/pandar arriving
ros2 topic hz /hesai/pandar

# 4.2  NDT localization up, with: the Bitlab_nav.pcd map, /hesai/pandar, and an INITIAL POSE
#      (place the rover at a known start, or set it in RViz). It must output map→base_link TF.
ros2 run tf2_ros tf2_echo map base_link              # must print a sane, stable transform

# 4.3  launch mesh navigation
ros2 launch mesh_nav_isaac_sim mesh_nav_real.launch.py
```

`4.3` starts: `mbf_mesh_nav` · `goal_bridge` · `voxel_subtraction_node` (detector) ·
`replan_trigger` · `cmd_vel_zmq_sender`.

> **First launch writes the computed layers back INTO `bitlab_real_mesh.h5`.** Do **not**
> Ctrl-C in the first ~10 s (corrupts it → `-6` crash). After one clean launch:
> `chmod 444 bitlab_real_mesh.h5`. If you ever see `-6`: `cp bitlab_real_mesh.h5.bak
> bitlab_real_mesh.h5` and relaunch. **Never run two mesh_nav launches at once.**

---

## 5. Verify BEFORE driving (RViz, Fixed Frame = `map`)

Do these in order. **Do not send a goal until the overlay passes.**

- [ ] **THE alignment check (most important):** add a **PointCloud2** on `/hesai/pandar`
      and a **MeshDisplay** on `/mbf_mesh_nav/mesh`. Live lidar must **overlay the mesh
      walls**. Shifted/rotated → NDT pose is wrong or map origin mismatched → fix NDT
      (re-set initial pose); do **not** drive.
- [ ] **Mesh visible & colored** (`/mbf_mesh_nav/mesh`): the floor is drivable (low cost),
      walls/steep faces are lethal. If the whole floor is lethal → steepness too aggressive.
- [ ] **Detector quiet when static:** `ros2 topic hz /dynamic_obstacles_cloud` ≈ empty
      with nothing moving. If the floor lights up → raise `ground_filter_height` / fix `min_z`.
- [ ] **TF sane:** `ros2 run tf2_ros tf2_echo map base_link` updates as the rover is nudged.

---

## 6. Drive it

1. In RViz pick **2D Nav Goal** (tool topic must be `/goal_pose`) and click a reachable spot
   on the floor.
2. A path appears on the mesh; the rover should start moving. Watch:
   ```bash
   ros2 topic echo /mbf_mesh_nav/cmd_vel --once   # MBF is emitting velocity
   ros2 topic hz /cmd_vel                          # (on the robot) commands arriving
   ```
3. **Dynamic-obstacle reroute:** walk an object into the planned path. `/dynamic_obstacles_cloud`
   lights up on it → `replan_trigger` logs a re-issued goal → the path bends around it.
   Remove the object → the path straightens on the next replan.

---

## 7. Safety

- **Watchdog:** kill the Jetson launch mid-drive → the rover must **STOP within `timeout`
  (0.5 s)**. This is the ROS1 receiver publishing zero Twist on staleness. Test it once,
  gently, at low speed early on.
- **Speed caps** live in `mesh_nav_params.yaml`: `mesh_controller.max_lin_velocity: 0.4`,
  `max_ang_velocity: 1.0`. Lower them for the first runs if you want a slower rover.
- **Manual override / E-stop:** keep the Innok's manual stop within reach; it is independent
  of this stack.

---

## 8. Shutdown (clean, avoids mesh corruption)

1. Stop sending goals; let the rover halt.
2. On the Jetson: Ctrl-C the `mesh_nav_real.launch.py` terminal (safe after the first run).
3. On the robot: Ctrl-C the receiver (rover safety-stops), then the lidar sender.

---

## 9. Troubleshooting (fast table)

| Symptom | Likely cause → fix |
|---|---|
| Every node hangs at startup | `use_sim_time` true / no `/clock`. Real launch sets it false — don't source a stray param. |
| No path / rover won't move | No `map→base_link` TF. `tf2_echo map base_link`; fix NDT / set initial pose. |
| Lidar doesn't overlay mesh | Frame mismatch — NDT pose wrong or wrong map. Re-set initial pose; confirm NDT uses `Bitlab_nav.pcd`. |
| Whole floor flagged dynamic | `ground_filter_height` / `min_z` wrong for the real lidar height; or static PCD wrong frame. |
| Whole floor is lethal on mesh | `steepness.threshold` (0.4) too low for a noisy floor — raise slightly. |
| Rover runs away after a glitch | ROS1 receiver watchdog not running / `timeout` too high. It must zero-Twist on staleness. |
| `-6 corrupted` on launch | Killed mbf mid-write. `cp bitlab_real_mesh.h5.bak bitlab_real_mesh.h5`; relaunch; don't Ctrl-C early. |
| `HalfEdgeMesh panicked / Path 0m` | CVP planner selected. Keep Dijkstra (`dijkstra_mesh_planner/DijkstraMeshPlanner`). |
| Rover reaches goal then spins | Heading tolerance — already loosened (`angle_tolerance: 3.15`). If it recurs, confirm the param loaded. |
| cmd_vel arrives but no motion | Robot-side `/cmd_vel` topic name / drive mux. Confirm the receiver `_output_topic` matches the robot. |

---

## 10. One-screen quick start (once §2 confirmed)

```text
ROBOT (ROS1):                              JETSON (ROS2):
 lidar driver up                            ros2 topic hz /hesai/pandar      # lidar in
 rosrun lidar_zmq_bridge lidar_zmq_sender.py
 rosrun lidar_zmq_bridge cmd_vel_zmq_receiver.py \
   _connect_address:=tcp://<JETSON_IP>:5556  NDT up + initial pose set
                                            ros2 run tf2_ros tf2_echo map base_link
                                            ros2 launch mesh_nav_isaac_sim mesh_nav_real.launch.py
                                            RViz: overlay test → 2D Nav Goal
```
