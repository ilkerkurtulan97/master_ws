"""
mesh_navigation Isaac Sim launch — blender parking garage scene.

Pre-conditions (same as existing navmesh_planner stack):
  1. Isaac Sim open with blender_with_rover.usd  (/home/ilker/Downloads/)
  2. fix_odom_definitive.py run  →  /chassis/odom z ≈ −0.664
  3. Press Play in Isaac Sim

Usage:
  source /opt/ros/humble/setup.bash
  source /home/ilker/meshnavigation_ws/install/setup.bash
  ros2 launch mesh_nav_isaac_sim mesh_nav_isaac_sim.launch.py

First-run note: mesh_map converts blender.obj (133 MB) to HDF5 on startup.
This takes 1–3 minutes.  Subsequent launches load from blender_mesh.h5 (~10 s).

Goal setting (two options):
  Option A — RViz "2D Nav Goal" tool   →  publishes /move_base_simple/goal directly
  Option B — navmesh_gui Drive button  →  /goal_pose relay bridges to MBF
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('mesh_nav_isaac_sim')
    config  = os.path.join(pkg_dir, 'config', 'mesh_nav_params.yaml')
    # Vendored dynamic_obstacle_detector (copied from master_ws into this workspace).
    detector_config = os.path.join(
        get_package_share_directory('dynamic_obstacle_detector'),
        'config', 'isaac_sim_params.yaml')

    return LaunchDescription([

        # ── TF: anchor Isaac's spawn-relative tree to absolute coordinates ───
        # fix_odom_definitive.py (already baked into the saved USD) makes the
        # /chassis/odom TOPIC report the lidar's ABSOLUTE Bitlab world pose, but
        # Isaac's TF tree (World→odom→base_link) is still SPAWN-RELATIVE and
        # spawn-YAWED. A static map→World offset can't fix the yaw, so the mesh
        # ends up rotated/shifted vs the lidar (ramp looks displaced, lidar
        # doesn't overlay). This node publishes map→World live, computed as
        # (map_T_lidar from /chassis/odom) ∘ inv(World_T_lidar from TF), which
        # cancels the spawn translation AND rotation exactly. Result: the whole
        # chain map→World→odom→base_link→sensors is absolute and overlays the mesh.
        ExecuteProcess(
            cmd=['python3', '/home/ilker/meshnavigation_ws/map_world_anchor.py',
                 '--ros-args', '-p', 'use_sim_time:=true'],
            output='screen',
        ),

        # ── Move Base Flex mesh navigation server ────────────────────────────
        # Loads mesh_map (Low_Bitlab.obj → HDF5), computes steepness/inflation/
        # obstacle layers, runs DijkstraMeshPlanner + MeshController.
        # Exposes:
        #   /mbf_mesh_nav/get_path   (action)
        #   /mbf_mesh_nav/exe_path   (action)
        #   /move_base              (action — MBF backward-compat)
        #   /move_base_simple/goal  (PoseStamped topic — simplest interface)
        #   /cmd_vel                (Twist → Isaac Sim Nova Carter)
        Node(
            package='mbf_mesh_nav',
            executable='mbf_mesh_nav',
            name='mbf_mesh_nav',
            output='screen',
            parameters=[config, {
                'use_sim_time': True,
                'mesh_map.global_frame': 'map',
                'global_frame': 'map',
                'robot_frame': 'base_link',
            }],
        ),

        # ── cmd_vel type bridge: MBF → Isaac Sim ─────────────────────────────
        # MBF publishes geometry_msgs/TwistStamped on /mbf_mesh_nav/cmd_vel, but
        # Isaac's Nova Carter (like master_ws) subscribes to geometry_msgs/Twist
        # on /cmd_vel. Mismatched types never connect -> rover ignores commands
        # -> MBF aborts. This relay strips the header and republishes Twist.
        ExecuteProcess(
            cmd=['python3', '/home/ilker/meshnavigation_ws/cmd_vel_relay.py',
                 '--ros-args', '-p', 'use_sim_time:=true'],
            output='screen',
        ),

        # ── Goal bridge: /goal_pose → MBF move_base action ───────────────────
        # MBF is ACTION-ONLY (no /move_base_simple/goal subscriber). RViz "2D Nav
        # Goal" and navmesh_gui publish a PoseStamped on /goal_pose; this node turns
        # it into a MoveBase action goal on /mbf_mesh_nav/move_base (plan+drive+
        # replan) and snaps the goal z to the robot's floor height. Previously run by
        # hand in a separate terminal; folded in here so one launch starts everything.
        # NOTE: set the RViz "2D Nav Goal" tool's topic to /goal_pose.
        ExecuteProcess(
            cmd=['python3', '/home/ilker/meshnavigation_ws/goal_bridge.py',
                 '--ros-args', '-p', 'use_sim_time:=true'],
            output='screen',
        ),

        # ── Dynamic obstacle perception: vendored dynamic_obstacle_detector ───
        # Subtracts the static Bitlab map (blender.pcd) from the live lidar using the
        # rover pose from /chassis/odom, publishing ONLY dynamic points on
        # /dynamic_obstacles_cloud (map frame). That cloud feeds the mesh ObstacleLayer
        # (see mesh_nav_params.yaml obstacle.topic). Copied from master_ws and built in
        # THIS workspace (master_ws is left untouched). We launch only the node, NOT the
        # detector's own isaac_sim.launch.py (which would publish a conflicting identity
        # map->World fighting map_world_anchor).
        Node(
            package='dynamic_obstacle_detector',
            executable='voxel_subtraction_node',
            name='voxel_subtraction_node',
            output='screen',
            parameters=[detector_config, {'use_sim_time': True}],
        ),

        # ── Replan trigger: reroute around dynamic obstacles ─────────────────
        # mesh_nav's ObstacleLayer DOES inject obstacle cost into the planner's
        # edge weights live (obstacle->combination->inflation->MeshMap::layerChanged),
        # but with planner_frequency=0.0 (plan-once, required for smooth driving) and
        # MBF's broken continuous replanner, GetPath is only ever called ONCE — at the
        # initial goal — so the path never reflects a newly-appeared obstacle. This node
        # is the missing trigger (mirrors master_ws navmesh_planner's
        # is_path_blocked()->plan_and_publish()): when a /dynamic_obstacles_cloud point
        # lands within block_dist of the CURRENT path AHEAD of the rover, it re-publishes
        # the active goal on /goal_pose -> goal_bridge -> a fresh plan-once that, reading
        # the now-updated obstacle costs, routes AROUND it. Throttled so it can't thrash;
        # self-limiting once the path clears the obstacle.
        ExecuteProcess(
            cmd=['python3', '/home/ilker/meshnavigation_ws/replan_trigger.py',
                 '--ros-args', '-p', 'use_sim_time:=true'],
            output='screen',
        ),
    ])
