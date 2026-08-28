from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='navmesh_gui',
            executable='navmesh_gui',
            name='navmesh_gui',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
    ])
