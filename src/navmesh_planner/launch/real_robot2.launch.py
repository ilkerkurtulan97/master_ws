import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('navmesh_planner'),
        'config', 'real_robot2_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='navmesh_planner',
            executable='navmesh_planner_node',
            name='navmesh_planner_node',
            output='screen',
            parameters=[config],
        ),
    ])
