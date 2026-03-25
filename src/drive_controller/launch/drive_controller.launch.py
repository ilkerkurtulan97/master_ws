import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('drive_controller'),
        'config',
        'params.yaml'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock'
        ),
        Node(
            package='drive_controller',
            executable='drive_controller_node',
            name='drive_controller_node',
            output='screen',
            parameters=[config, {'use_sim_time': use_sim_time}],
        ),
    ])
