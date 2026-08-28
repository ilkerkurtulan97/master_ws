import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('drive_controller'),
        'config', 'real_robot_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='drive_controller',
            executable='drive_controller_node',
            name='drive_controller_node',
            output='screen',
            parameters=[config],
        ),
    ])
