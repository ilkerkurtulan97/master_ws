"""
mesh_navigation REAL-ROBOT launch — Innok Heros + Hesai Pandar + Jetson Orin NX.

This is the real-world counterpart of mesh_nav_isaac_sim.launch.py. Key differences
from the sim launch (see README_real_robot_setup.md for the full story):

  * use_sim_time = FALSE everywhere. There is no /clock on the real robot; leaving it
    true makes every node block forever.
  * NO map_world_anchor.py. That node cancels Isaac's spawn offset using /chassis/odom.
    On the real robot the map->base_link TF comes from your NDT localization instead.
  * NO cmd_vel_relay.py. It is replaced by cmd_vel_zmq_sender.py, which ships the
    velocity command to the Innok (ROS1) over ZeroMQ.
  * Detector uses real_robot_params.yaml (real lidar topic /hesai/pandar, NDT pose,
    real Bitlab static PCD).

PRE-CONDITIONS (must already be running BEFORE this launch):
  1. lidar_zmq receiver on the Jetson -> a ROS2 PointCloud2 (e.g. /hesai/pandar).
  2. ros2_lidar_localization (NDT) -> publishes the map->base_link TF (and a pose
     topic for the detector). Give it the initial pose + the Bitlab map PCD.
  3. cmd_vel_zmq_receiver.py running on the Innok (ROS1) -> /cmd_vel to the wheels.

Then on the Jetson:
  source /opt/ros/humble/setup.bash
  source /home/ilker/meshnavigation_ws/install/setup.bash
  ros2 launch mesh_nav_isaac_sim mesh_nav_real.launch.py

Goal: RViz "2D Nav Goal" (tool topic = /goal_pose). Fixed Frame = map.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

# ── Things to CONFIRM against your real stack ────────────────────────────────
ODOM_TOPIC   = "/odom"                  # CONFIRM: odom source for MBF velocity (check is disabled, so non-critical)
ZMQ_BIND     = "tcp://0.0.0.0:5556"     # Jetson cmd_vel PUB endpoint (receiver connects here)
WS = "/home/ilker/meshnavigation_ws"
# Planning mesh built DIRECTLY from the NDT map cloud Bitlab_nav.pcd by make_nav_mesh.py
# (same origin as NDT localization => the GOLDEN RULE alignment is guaranteed, unlike the
# Blender/OBJ-derived sim mesh). Overridden here so the shared mesh_nav_params.yaml (which
# the sim launch uses) is left untouched. First clean launch writes layers back into this
# file; don't Ctrl-C in the first ~10 s, and `chmod 444` it afterwards (restore .bak if -6).
REAL_MESH = f"{WS}/bitlab_real_mesh.h5"


def generate_launch_description():
    pkg_dir = get_package_share_directory('mesh_nav_isaac_sim')
    # Reuse the SAME tuned planner/controller/layer config as sim; override only the
    # real-robot bits below. (mesh_file etc. stay the same — assumes the Bitlab welded
    # mesh is in your NDT map frame; verify the lidar overlays the mesh in RViz.)
    config = os.path.join(pkg_dir, 'config', 'mesh_nav_params.yaml')
    detector_config = os.path.join(
        get_package_share_directory('dynamic_obstacle_detector'),
        'config', 'real_robot_params.yaml')

    use_sim_time = False

    return LaunchDescription([

        # ── Move Base Flex mesh navigation server ────────────────────────────
        Node(
            package='mbf_mesh_nav',
            executable='mbf_mesh_nav',
            name='mbf_mesh_nav',
            output='screen',
            parameters=[config, {
                'use_sim_time': use_sim_time,
                'mesh_map.global_frame': 'map',
                'global_frame': 'map',
                'robot_frame': 'base_link',     # CONFIRM the Innok base frame name
                'odom_topic': ODOM_TOPIC,
                # Use the PCD-derived real mesh (NOT the sim mesh in mesh_nav_params.yaml).
                # mesh_file == mesh_working_file => mesh_map loads the .h5 directly.
                'mesh_map.mesh_file': REAL_MESH,
                'mesh_map.mesh_working_file': REAL_MESH,
                'mesh_map.mesh_part': 'Low_Bitlab',
                'mesh_map.mesh_working_part': 'Low_Bitlab',
            }],
        ),

        # ── Goal bridge: /goal_pose -> MBF move_base action ──────────────────
        ExecuteProcess(
            cmd=['python3', f'{WS}/goal_bridge.py',
                 '--ros-args', '-p', f'use_sim_time:={str(use_sim_time).lower()}'],
            output='screen',
        ),

        # ── Dynamic obstacle perception (vendored detector, real config) ─────
        Node(
            package='dynamic_obstacle_detector',
            executable='voxel_subtraction_node',
            name='voxel_subtraction_node',
            output='screen',
            parameters=[detector_config, {'use_sim_time': use_sim_time}],
        ),

        # ── Replan trigger: reroute around dynamic obstacles ─────────────────
        ExecuteProcess(
            cmd=['python3', f'{WS}/replan_trigger.py',
                 '--ros-args', '-p', f'use_sim_time:={str(use_sim_time).lower()}'],
            output='screen',
        ),

        # ── cmd_vel OUT: ROS2 -> ZeroMQ -> Innok (ROS1) ──────────────────────
        # Replaces cmd_vel_relay.py. Subscribes MBF's TwistStamped and ships it to the
        # robot. The Innok must run cmd_vel_zmq_receiver.py connecting to ZMQ_BIND.
        ExecuteProcess(
            cmd=['python3', f'{WS}/cmd_vel_zmq_sender.py', '--ros-args',
                 '-p', f'use_sim_time:={str(use_sim_time).lower()}',
                 '-p', f'bind_address:={ZMQ_BIND}',
                 '-p', 'input_topic:=/mbf_mesh_nav/cmd_vel',
                 '-p', 'input_stamped:=true'],
            output='screen',
        ),
    ])
