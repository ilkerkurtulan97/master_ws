"""Full navigation stack against NVIDIA Isaac Sim.

Starts the simulation profile of every package, all on simulated time:
  map->World static TF, voxel_subtraction_node, RViz   dynamic_obstacle_detector
  navmesh_planner_node                                 navmesh_planner
  drive_controller_node                                drive_controller
  navmesh_gui (optional)                               navmesh_gui

Launch, then press Play in Isaac Sim and set a goal in RViz or the GUI.

    ros2 launch rover_bringup isaac_sim.launch.py
    ros2 launch rover_bringup isaac_sim.launch.py gui:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def include(package, launch_file, condition=None, **args):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package), 'launch', launch_file])),
        launch_arguments=args.items(),
        condition=condition)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'gui', default_value='false',
            description='Start the navmesh_gui operator interface'),

        include('dynamic_obstacle_detector', 'isaac_sim.launch.py'),
        include('navmesh_planner', 'navmesh_planner.launch.py', use_sim_time='true'),
        include('drive_controller', 'drive_controller.launch.py', use_sim_time='true'),
        include('navmesh_gui', 'navmesh_gui.launch.py',
                IfCondition(LaunchConfiguration('gui'))),
    ])
