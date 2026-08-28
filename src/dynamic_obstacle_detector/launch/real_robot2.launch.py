import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('dynamic_obstacle_detector'),
        'config', 'real_robot2_params.yaml'
    )

    return LaunchDescription([
        # base_link -> lidar TF is published by lidar_localization.launch.py (hesai_tf node).
        # Do not duplicate it here.
        Node(
            package='dynamic_obstacle_detector',
            executable='voxel_subtraction_node',
            name='voxel_subtraction_node',
            output='screen',
            parameters=[config],
        ),
    ])
