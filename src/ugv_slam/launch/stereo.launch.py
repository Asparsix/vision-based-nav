#!/usr/bin/env python3
"""Stereo processing bringup.

Publishes:
  /stereo/depth ← Gazebo depth (Depth block for VSLAM)
  /stereo/points ← XYZ cloud from depth
"""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    opt = Path.home() / 'ugv_vision_nav_ws' / '.opt' / 'opt' / 'ros' / 'jazzy'
    ld = f"{opt}/lib:{opt}/lib/x86_64-linux-gnu:{os.environ.get('LD_LIBRARY_PATH', '')}"
    ament = f"{opt}{os.pathsep}{os.environ.get('AMENT_PREFIX_PATH', '')}"

    depth_relay = Node(
        package='ugv_slam',
        executable='topic_relay.py',
        name='stereo_depth_relay',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_topic': '/camera/depth/image_raw',
            'output_topic': '/stereo/depth',
            'msg_type': 'sensor_msgs/msg/Image',
        }],
    )
    info_relay = Node(
        package='ugv_slam',
        executable='topic_relay.py',
        name='stereo_depth_info_relay',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_topic': '/camera/depth/camera_info',
            'output_topic': '/stereo/depth/camera_info',
            'msg_type': 'sensor_msgs/msg/CameraInfo',
        }],
    )

    cloud = Node(
        package='rtabmap_util',
        executable='point_cloud_xyz',
        name='stereo_point_cloud',
        output='screen',
        additional_env={'LD_LIBRARY_PATH': ld},
        parameters=[{
            'use_sim_time': use_sim_time,
            'decimation': 2,
            'max_depth': 12.0,
            'voxel_size': 0.05,
        }],
        remappings=[
            ('depth/image', '/camera/depth/image_raw'),
            ('depth/camera_info', '/camera/depth/camera_info'),
            ('cloud', '/stereo/points'),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        SetEnvironmentVariable(name='AMENT_PREFIX_PATH', value=ament),
        SetEnvironmentVariable(name='LD_LIBRARY_PATH', value=ld),
        LogInfo(msg='[ugv_slam] stereo: left/right from Gazebo; depth+cloud from depth camera'),
        depth_relay,
        info_relay,
        cloud,
    ])
