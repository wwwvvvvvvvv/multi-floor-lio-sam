import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
    ExecuteProcess,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    sim_share = get_package_share_directory('multi_floor_sim')
    nav2_share = get_package_share_directory('nav2_bringup')

    sim_launch = os.path.join(
        sim_share,
        'launch',
        'floor1_system.launch.py'
    )

    nav2_launch = os.path.join(
        nav2_share,
        'launch',
        'navigation_launch.py'
    )

    nav2_params = (
        '/home/yez/multi_floor_ws/src/'
        'multi_floor_sim/config/nav2_params.yaml'
    )

    map_yaml = '/home/yez/nav_maps/floor1/map.yaml'
    pcd_map = '/home/yez/lio_sam_maps/floor1/GlobalMap.pcd'

    # ============================================================
    # 1. Gazebo + sensor bridge + basic simulation
    # ============================================================
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch)
    )

    # ============================================================
    # 2. Gazebo simulation clock -> ROS2 /clock
    # ============================================================
    clock_bridge = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='clock_bridge',
                arguments=[
                    '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'
                ],
                output='screen',
            )
        ]
    )

    # ============================================================
    # 3. odom -> base_link
    # ============================================================
    odom_tf = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='known_map_localization',
                executable='odom_tf_broadcaster',
                name='odom_tf_broadcaster',
                parameters=[
                    {'use_sim_time': True}
                ],
                output='screen',
            )
        ]
    )

    # ============================================================
    # 4. 3D PointCloud -> 2D LaserScan
    # ============================================================
    laser_scan = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='pointcloud_to_laserscan',
                executable='pointcloud_to_laserscan_node',
                name='pointcloud_to_laserscan',
                remappings=[
                    ('cloud_in', '/lio_sam/points'),
                    ('scan', '/scan'),
                ],
                parameters=[{
                    'use_sim_time': True,
                    'target_frame': 'lidar_link',

                    'min_height': -0.05,
                    'max_height': 0.30,

                    'angle_min': -3.14159,
                    'angle_max': 3.14159,
                    'angle_increment': 0.0087,

                    'range_min': 0.35,
                    'range_max': 20.0,

                    'scan_time': 0.1,
                    'use_inf': True,
                }],
                output='screen',
            )
        ]
    )

    # ============================================================
    # 5. NDT known-map localization
    #
    # NDT:
    #   low-rate global correction
    #
    # TF:
    #   map -> odom refreshed at high rate by /odom callback
    # ============================================================
    ndt_localizer = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='known_map_localization',
                executable='ndt_localizer',
                name='ndt_localizer',
                parameters=[{
                    'use_sim_time': True,

                    'map_path': pcd_map,

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
            )
        ]
    )

    # ============================================================
    # 6. 2D navigation map
    # ============================================================
    # Start map_server first.
    map_server = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                parameters=[{
                    'use_sim_time': True,
                    'yaml_filename': map_yaml,
                }],
                output='screen',
            )
        ]
    )

    # Start its lifecycle manager slightly later so that map_server
    # is already registered before configure/activate is requested.
    map_lifecycle_manager = TimerAction(
        period=5.5,
        actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_map_server',
                parameters=[{
                    'use_sim_time': True,
                    'autostart': True,
                    'node_names': ['map_server'],
                }],
                output='screen',
            )
        ]
    )

    # ============================================================
    # 7. Floor1 simulation initial pose
    #
    # Current Gazebo world always spawns robot at:
    # x=0, y=0, yaw=0
    #
    # If spawn pose changes later, change/remove this block.
    # ============================================================
    initial_pose = TimerAction(
        period=6.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2',
                    'topic',
                    'pub',
                    '--once',
                    '/initialpose',
                    'geometry_msgs/msg/PoseWithCovarianceStamped',
                    (
                        "{header: {frame_id: 'map'}, "
                        "pose: {pose: {"
                        "position: {x: 0.0, y: 0.0, z: 0.0}, "
                        "orientation: "
                        "{x: 0.0, y: 0.0, z: 0.0, w: 1.0}"
                        "}}}"
                    ),
                ],
                output='screen',
            )
        ]
    )

    # ============================================================
    # 8. Nav2
    #
    # IMPORTANT:
    # navigation_launch.py does NOT launch AMCL.
    # Our NDT publishes map -> odom.
    # ============================================================
    navigation = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch),
                launch_arguments={
                    'use_sim_time': 'true',
                    'autostart': 'true',
                    'params_file': nav2_params,
                }.items(),
            )
        ]
    )

    # ============================================================
    # 9. Nav2 RViz
    # ============================================================
    rviz = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='nav2_rviz',
                arguments=[
                    '-d',
                    os.path.join(
                        nav2_share,
                        'rviz',
                        'nav2_default_view.rviz'
                    )
                ],
                parameters=[
                    {'use_sim_time': True}
                ],
                output='screen',
            )
        ]
    )

    return LaunchDescription([
        simulation,
        clock_bridge,
        odom_tf,
        laser_scan,
        ndt_localizer,
        map_server,
        map_lifecycle_manager,
        initial_pose,
        navigation,
        rviz,
    ])
