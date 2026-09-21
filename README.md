# Mesh Navigation — Innok Heros

Mesh-based navigation with dynamic obstacle avoidance for the Innok Heros rover
(ROS 2 Humble), built on
[naturerobots/mesh_navigation](https://github.com/naturerobots/mesh_navigation).
Comparison stack to the Recast/Detour planner on `main`.

- **Planner:** `DijkstraMeshPlanner` · **Controller:** `MeshController`
- **Obstacles:** `voxel_subtraction_node` → `ObstacleLayer`; `replan_trigger.py`
  requests a new plan when the path ahead is blocked
- **Runs** in Isaac Sim and on the real robot (Jetson Orin NX; `cmd_vel` to the
  Innok over ZeroMQ)

## Setup

Requires ROS 2 Humble and `python3-vcstool`. Run `conda deactivate` first.
Clone into `~/meshnavigation_ws` — the launch files use this path.

```bash
git clone -b mesh_navigation git@github.com:ilkerkurtulan97/master_ws.git ~/meshnavigation_ws
cd ~/meshnavigation_ws
vcs import src < dependencies.repos
git -C src/mesh_navigation apply ../../patches/mesh_map-fixes.patch
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install && source install/setup.bash
```

**The patch is required.** It fixes an lvr2 heap overflow and a `/clock` deadlock
in `mesh_map` that otherwise crash or hang `mbf_mesh_nav`.

## Run

**Isaac Sim** — press Play first, then send goals with RViz *2D Nav Goal* on
`/goal_pose`:

```bash
ros2 launch mesh_nav_isaac_sim mesh_nav_isaac_sim.launch.py
```

**Real robot** — check the `CONFIRM` values in `mesh_nav_real.launch.py` and
`real_robot_params.yaml` first.

```bash
# Jetson
ros2 launch mesh_nav_isaac_sim mesh_nav_real.launch.py
# Innok (ROS 1)
rosrun lidar_zmq_bridge cmd_vel_zmq_receiver.py \
    _connect_address:=tcp://<JETSON_IP>:5556 _output_topic:=/cmd_vel _timeout:=0.5
```

## Building a mesh

```bash
python3 make_nav_pcd.py  --src <scan.pcd> --out nav.pcd --voxel 0.05
python3 make_nav_mesh.py --pcd nav.pcd --out mesh.h5 --template bitlab_real_mesh.h5 --part Low_Bitlab
```

The localization map, planning mesh and detector cloud must all come from the
same point cloud.

Detailed notes: [`docs/`](docs/).
