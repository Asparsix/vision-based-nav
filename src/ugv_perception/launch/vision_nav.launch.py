#!/usr/bin/env python3
"""Vision nav stack: segmenter + costmap + local driver."""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    ws = Path.home() / 'ugv_vision_nav_ws'
    venv_site = ws / '.venv' / 'lib' / 'python3.12' / 'site-packages'
    weights_default = str(ws / 'data' / 'models' / 'unet_outdoor_best.pt')

    old = os.environ.get('PYTHONPATH', '')
    new_pp = str(venv_site)
    if old:
        new_pp = f'{new_pp}:{old}'

    return LaunchDescription([
        SetEnvironmentVariable(name='PYTHONPATH', value=new_pp),
        DeclareLaunchArgument('image_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('weights', default_value=weights_default),
        DeclareLaunchArgument('device', default_value='cpu'),
        DeclareLaunchArgument('skip_frames', default_value='1'),
        DeclareLaunchArgument('driver_enabled', default_value='true'),
        DeclareLaunchArgument('v_max', default_value='0.35'),
        DeclareLaunchArgument('w_max', default_value='0.85'),

        Node(
            package='ugv_perception',
            executable='segmenter',
            name='ugv_segmenter',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'image_topic': LaunchConfiguration('image_topic'),
                'weights': LaunchConfiguration('weights'),
                'device': LaunchConfiguration('device'),
                'publish_overlay': True,
                'overlay_alpha': 0.55,
                'skip_frames': ParameterValue(
                    LaunchConfiguration('skip_frames'), value_type=int
                ),
            }],
        ),
        Node(
            package='ugv_perception',
            executable='seg_costmap',
            name='ugv_seg_costmap',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'mask_topic': '/perception/segmentation',
                'output_topic': '/perception/local_costmap',
                'frame_id': 'base_link',
                'resolution': 0.05,
                'x_min': 0.30,
                'x_max': 6.0,
                'y_min': -2.5,
                'y_max': 2.5,
                'grass_cost': 45,
                'road_cost': 0,
                'tree_cost': 100,
                'tree_inflate_cells': 2,
                'stride': 2,
            }],
        ),
        Node(
            package='ugv_perception',
            executable='vision_local_planner',
            name='ugv_vision_driver',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'costmap_topic': '/perception/local_costmap',
                'cmd_vel_topic': '/cmd_vel',
                'enabled': ParameterValue(
                    LaunchConfiguration('driver_enabled'), value_type=bool
                ),
                'v_max': ParameterValue(LaunchConfiguration('v_max'), value_type=float),
                'w_max': ParameterValue(LaunchConfiguration('w_max'), value_type=float),
                'v_min': 0.08,
                'control_hz': 10.0,
            }],
        ),
    ])
