#!/usr/bin/env python3
"""Launch outdoor Gazebo world + stereo UGV + IMU + depth bridge."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('ugv_sim')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world = os.path.join(pkg, 'worlds', 'outdoor_path.sdf')
    urdf = os.path.join(pkg, 'urdf', 'ugv_bot.urdf')
    sdf = os.path.join(pkg, 'models', 'ugv_bot', 'model.sdf')
    bridge = os.path.join(pkg, 'config', 'bridge.yaml')

    with open(urdf, 'r', encoding='utf-8') as f:
        robot_desc = f.read()

    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    yaw = LaunchConfiguration('yaw')

    set_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(pkg, 'models'),
    )

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r -v2 {world}',
            'on_exit_shutdown': 'true',
        }.items(),
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_desc,
        }],
        output='screen',
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'ugv_bot',
            '-file', sdf,
            '-x', x,
            '-y', y,
            '-z', '0.09',
            '-Y', yaw,
        ],
        output='screen',
    )

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge}'],
        output='screen',
    )

    image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=[
            '/stereo/left/image_raw',
            '/stereo/right/image_raw',
            '/camera/image_raw',
            '/camera/depth/image_raw',
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('x', default_value='-7.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        set_resources,
        gz,
        rsp,
        spawn,
        bridge_node,
        image_bridge,
    ])
