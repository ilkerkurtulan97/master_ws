import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('dynamic_obstacle_detector')

    config = os.path.join(pkg_dir, 'config', 'isaac_sim_params.yaml')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'isaac_sim.rviz')

    return LaunchDescription([
        # Connect Isaac Sim's world frame to the map frame (identity — confirmed aligned).
        # Isaac Sim publishes: World -> odom -> base_link -> sensors
        # This gives RViz the full chain: map -> World -> odom -> base_link -> front_3d_lidar
        # NOTE: stop world_pose_tf_relay.py before launching — it conflicts by also
        # parenting base_link to map, giving base_link two parents.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_world_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'World'],
            parameters=[{'use_sim_time': True}],
        ),

        # Voxel subtraction node
        Node(
            package='dynamic_obstacle_detector',
            executable='voxel_subtraction_node',
            name='voxel_subtraction_node',
            output='screen',
            parameters=[config, {'use_sim_time': True}],
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
        ),
    ])
