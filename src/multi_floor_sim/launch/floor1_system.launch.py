from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

import os


def launch_gazebo(context, world):
    """Start Gazebo with the selected rendering backend."""
    render_mode = LaunchConfiguration('render_mode').perform(context).lower()
    render_environment = {
        # Fortress uses Qt 5.  Its native Wayland 3D view crashes on this
        # hybrid-GPU system, so keep it on the stable XWayland GLX path.
        'QT_QPA_PLATFORM': 'xcb',
        'QT_XCB_GL_INTEGRATION': 'xcb_glx',
        'QSG_RENDER_LOOP': 'basic',
    }

    if render_mode == 'nvidia':
        render_environment.update({
            '__NV_PRIME_RENDER_OFFLOAD': '1',
            '__GLX_VENDOR_LIBRARY_NAME': 'nvidia',
            '__VK_LAYER_NV_optimus': 'NVIDIA_only',
            '__EGL_VENDOR_LIBRARY_FILENAMES':
                '/usr/share/glvnd/egl_vendor.d/10_nvidia.json',
            '__GL_SYNC_TO_VBLANK': '1',
        })
    elif render_mode == 'software':
        render_environment.update({
            'LIBGL_ALWAYS_SOFTWARE': '1',
        })
    elif render_mode == 'auto':
        pass
    else:
        raise RuntimeError(
            "render_mode must be one of: 'nvidia', 'auto', or 'software'"
        )

    return [
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-r', world],
            additional_env=render_environment,
            output='screen',
        )
    ]


def generate_launch_description():
    pkg_multi_floor = get_package_share_directory(
        'multi_floor_sim'
    )

    world = os.path.join(
        pkg_multi_floor,
        'worlds',
        'floor1.sdf'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'render_mode',
            default_value='nvidia',
            description='Gazebo renderer: nvidia, auto, or software',
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[
                '--x', '0',
                '--y', '0',
                '--z', '0.14',
                '--roll', '0',
                '--pitch', '0',
                '--yaw', '0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'lidar_link'
            ],
            output='screen'
        ),

        # =========================
        # Gazebo
        # =========================
        OpaqueFunction(
            function=launch_gazebo,
            args=[world],
        ),

        # =========================
        # ROS-Gazebo bridge
        # =========================

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/imu/data@sensor_msgs/msg/Imu[ignition.msgs.IMU',
                '/lidar/points@sensor_msgs/msg/PointCloud2'
                '[ignition.msgs.PointCloudPacked',
                '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            ],
            output='screen'
        ),

        # =========================
        # TF
        # =========================
        # =========================
        # LIO-SAM point adapter
        # =========================

        Node(
            package='lio_sam_sim_adapter',
            executable='lidar_time_converter',
            output='screen'
        )

    ])
