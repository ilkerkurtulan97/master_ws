"""Full navigation stack on the real Innok Heros (Jetson Orin NX).

Starts the real_robot2 profile of every package:
  lidar_localization_ros2   NDT localization -> /pcl_pose, hesai_tf  (external workspace)
  voxel_subtraction_node    dynamic_obstacle_detector
  navmesh_planner_node      navmesh_planner
  drive_controller_node     drive_controller
  zed_speed_governor_node   zed_speed_governor

    ros2 launch rover_bringup real_robot.launch.py
    ros2 launch rover_bringup real_robot.launch.py localization:=false   # localizer already running
    ros2 launch rover_bringup real_robot.launch.py zed:=false            # no camera attached

Not started here: the LiDAR and cmd_vel ZMQ bridges (see README).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def include(package, launch_file, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package), 'launch', launch_file])),
        condition=condition)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'localization', default_value='true',
            description='Start lidar_localization_ros2 (its workspace must be sourced)'),
        DeclareLaunchArgument(
            'zed', default_value='true',
            description='Start the ZED person-aware speed governor'),

        include('lidar_localization_ros2', 'lidar_localization.launch.py',
                IfCondition(LaunchConfiguration('localization'))),
        include('dynamic_obstacle_detector', 'real_robot2.launch.py'),
        include('navmesh_planner', 'real_robot2.launch.py'),
        include('drive_controller', 'real_robot2.launch.py'),
        include('zed_speed_governor', 'real_robot2.launch.py',
                IfCondition(LaunchConfiguration('zed'))),
    ])
