import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('zed_speed_governor'),
        'config', 'real_robot2_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='zed_speed_governor',
            executable='zed_speed_governor_node',
            name='zed_speed_governor_node',
            output='screen',
            parameters=[config],
        ),
    ])
