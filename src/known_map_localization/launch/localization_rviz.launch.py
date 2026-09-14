"""Known-map NDT localization with RViz initial-pose selection.

The sensor pipeline and base_link -> lidar_link extrinsic TF must already
be running. map -> base_link is published dynamically by ndt_localizer.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory('known_map_localization')
    map_path = LaunchConfiguration('map_path')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_path',
            default_value='/home/yez/lio_sam_maps/floor1/GlobalMap.pcd',
            description='PCD map loaded by both the map publisher and NDT',
        ),
        Node(
            package='known_map_localization',
            executable='known_map_publisher',
            name='known_map_publisher',
            parameters=[{'map_path': map_path}],
            output='screen',
        ),
        Node(
            package='known_map_localization',
            executable='ndt_localizer',
            name='ndt_localizer',
            parameters=[{
                'map_path': map_path,
                'wait_for_initialpose': True,
                'map_voxel_size': 0.30,
                'scan_voxel_size': 0.50,
                'ndt_resolution': 1.0,
                'ndt_max_iterations': 15,
                'base_to_lidar_x': 0.0,
                'base_to_lidar_y': 0.0,
                'base_to_lidar_z': 0.14,
                'use_odom_prior': True,
                'odom_topic': '/odom',
                'max_odom_time_difference': 0.10,
                'fitness_accept_threshold': 0.017,
                'max_position_jump': 0.15,
                'max_yaw_jump': 0.20,
                'max_prediction_position_error': 0.15,
                'max_prediction_yaw_error': 0.15,
            }],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='localization_rviz',
            arguments=['-d', os.path.join(share_dir, 'config', 'localization.rviz')],
            output='screen',
        ),
    ])
