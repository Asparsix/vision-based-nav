#!/usr/bin/env python3
"""RTAB-Map RGB-D mapping / localization for ugv_vision_nav_ws.

Uses left RGB + Gazebo depth + fused odom (/odometry/filtered from EKF).
Requires workspace .opt RTAB overlay: source ~/ugv_vision_nav_ws/scripts/setup_env.sh
"""

from __future__ import annotations

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _opt_prefix() -> Path:
    return Path.home() / 'ugv_vision_nav_ws' / '.opt' / 'opt' / 'ros' / 'jazzy'


def _lib_env() -> dict:
    opt = _opt_prefix()
    dirs = [str(opt / 'lib'), str(opt / 'lib' / 'x86_64-linux-gnu')]
    dirs = [d for d in dirs if os.path.isdir(d)]
    old = os.environ.get('LD_LIBRARY_PATH', '')
    env = {}
    if dirs:
        env['LD_LIBRARY_PATH'] = ':'.join(dirs + ([old] if old else []))
    return env


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time')
    localization = LaunchConfiguration('localization')
    database_path = LaunchConfiguration('database_path').perform(context)
    delete_db = LaunchConfiguration('delete_db_on_start').perform(context).lower() in (
        'true',
        '1',
    )
    lib_env = _lib_env()

    parameters = {
        'frame_id': 'base_footprint',
        'odom_frame_id': 'odom',
        'map_frame_id': 'map',
        'use_sim_time': use_sim_time,
        'subscribe_depth': True,
        'subscribe_rgb': True,
        'subscribe_scan': False,
        'subscribe_odom_info': False,
        'approx_sync': True,
        'sync_queue_size': 30,
        'database_path': database_path,
        'Reg/Force3DoF': 'true',
        'Grid/RayTracing': 'true',
        'Grid/3D': 'false',
        'Grid/RangeMax': '12',
        'Grid/MaxGroundHeight': '0.05',
        'Grid/MaxObstacleHeight': '1.5',
        'Optimizer/GravitySigma': '0',
        'RGBD/NeighborLinkRefining': 'true',
        'RGBD/ProximityBySpace': 'true',
        'Vis/MinInliers': '10',
    }

    remappings = [
        ('rgb/image', '/camera/image_raw'),
        ('rgb/camera_info', '/camera/camera_info'),
        ('depth/image', '/camera/depth/image_raw'),
        # Prefer EKF output; /odom is an alias of /odometry/filtered
        ('odom', '/odometry/filtered'),
        ('imu', '/imu'),
    ]

    mapping_args = ['-d'] if delete_db else []

    def node(**kw):
        return Node(additional_env=lib_env, **kw)

    nodes = [
        node(
            condition=UnlessCondition(localization),
            package='rtabmap_slam',
            executable='rtabmap',
            output='screen',
            parameters=[parameters],
            remappings=remappings,
            arguments=mapping_args,
        ),
        node(
            condition=IfCondition(localization),
            package='rtabmap_slam',
            executable='rtabmap',
            output='screen',
            parameters=[
                parameters,
                {
                    'Mem/IncrementalMemory': 'False',
                    'Mem/InitWMWithAllNodes': 'True',
                },
            ],
            remappings=remappings,
        ),
        node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            output='screen',
            parameters=[parameters],
            remappings=remappings,
            condition=IfCondition(LaunchConfiguration('use_rtabmap_viz')),
        ),
        node(
            package='rtabmap_util',
            executable='point_cloud_xyz',
            output='screen',
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
        ),
    ]
    return nodes


def generate_launch_description():
    opt = str(_opt_prefix())
    ament = opt + os.pathsep + os.environ.get('AMENT_PREFIX_PATH', '')
    return LaunchDescription([
        SetEnvironmentVariable(name='AMENT_PREFIX_PATH', value=ament),
        SetEnvironmentVariable(
            name='LD_LIBRARY_PATH',
            value=f"{opt}/lib:{opt}/lib/x86_64-linux-gnu:"
            + os.environ.get('LD_LIBRARY_PATH', ''),
        ),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('localization', default_value='false'),
        DeclareLaunchArgument(
            'database_path',
            default_value=str(Path.home() / 'ugv_vision_nav_ws/data/maps/rtabmap.db'),
        ),
        DeclareLaunchArgument('delete_db_on_start', default_value='true'),
        DeclareLaunchArgument('use_rtabmap_viz', default_value='true'),
        LogInfo(msg='[ugv_slam] RTAB-Map RGB-D + EKF odom. Source scripts/setup_env.sh first.'),
        OpaqueFunction(function=launch_setup),
    ])
