#!/usr/bin/env python3
"""Fused odometry: wheel + IMU + RGB-D VO → /odometry/filtered (+ odom TF).

  /wheel/odom  (Gazebo DiffDrive)
  /imu
  /vo/odom     (rtabmap rgbd_odometry, no TF)
       └─► robot_localization ekf_node
             └─► /odometry/filtered
             └─► TF odom → base_footprint
"""

from __future__ import annotations

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('ugv_slam')
    ekf_yaml = os.path.join(pkg, 'config', 'ekf.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_vo = LaunchConfiguration('use_vo')

    opt = Path.home() / 'ugv_vision_nav_ws' / '.opt' / 'opt' / 'ros' / 'jazzy'
    # Only RTAB binaries need the .opt libs — do NOT export this globally or
    # robot_localization (system) dies with symbol lookup errors.
    rtab_ld = f"{opt}/lib:{opt}/lib/x86_64-linux-gnu:{os.environ.get('LD_LIBRARY_PATH', '')}"
    # Force system libs for EKF (ignore parent launch LD_LIBRARY_PATH pollution)
    ekf_ld = '/opt/ros/jazzy/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu'

    rgbd_odom = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',
        condition=IfCondition(use_vo),
        additional_env={'LD_LIBRARY_PATH': rtab_ld},
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'publish_tf': False,  # EKF owns odom TF
            'approx_sync': True,
            'queue_size': 30,
            'wait_imu_to_init': False,
            'Reg/Force3DoF': 'true',
            'Odom/Strategy': '0',
            'Vis/MinInliers': '8',
        }],
        remappings=[
            ('rgb/image', '/camera/image_raw'),
            ('rgb/camera_info', '/camera/camera_info'),
            ('depth/image', '/camera/depth/image_raw'),
            ('imu', '/imu'),
            ('odom', '/vo/odom'),
        ],
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        # Clean env: system robot_localization + diagnostic_updater
        additional_env={
            'LD_LIBRARY_PATH': ekf_ld,
        },
        parameters=[ekf_yaml, {'use_sim_time': use_sim_time}],
        remappings=[
            ('odometry/filtered', '/odometry/filtered'),
        ],
    )

    # Compatibility alias: many nodes still expect /odom
    odom_relay = Node(
        package='ugv_slam',
        executable='topic_relay.py',
        name='filtered_odom_alias',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_topic': '/odometry/filtered',
            'output_topic': '/odom',
            'msg_type': 'nav_msgs/msg/Odometry',
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'use_vo',
            default_value='true',
            description='Fuse rtabmap RGB-D visual odometry into EKF',
        ),
        LogInfo(msg='[ugv_slam] EKF: wheel/odom + imu + vo/odom → /odometry/filtered'),
        rgbd_odom,
        ekf,
        odom_relay,
    ])
