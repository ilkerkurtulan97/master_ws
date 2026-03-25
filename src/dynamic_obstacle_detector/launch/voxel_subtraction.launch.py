import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('dynamic_obstacle_detector'),
        'config',
        'params.yaml'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (bag) clock'
        ),

        # Static TF: base_link -> lidar (x=0.4m forward, z=0.85m up)
        # Needed because bag's /tf_static has incompatible QoS
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_lidar_tf',
            arguments=['0.4', '0', '1.0', '0', '0', '0', 'base_link', 'lidar'],
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        Node(
            package='dynamic_obstacle_detector',
            executable='voxel_subtraction_node',
            name='voxel_subtraction_node',
            output='screen',
            parameters=[config, {'use_sim_time': use_sim_time}],
        ),
    ])
