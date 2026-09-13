#!/usr/bin/env python3
"""Launch live UNet segmenter (adds workspace venv torch to PYTHONPATH)."""

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

    # Prepend venv so `import torch` works while keeping system rclpy/cv_bridge.
    old = os.environ.get('PYTHONPATH', '')
    new_pp = str(venv_site)
    if old:
        new_pp = f'{new_pp}:{old}'

    return LaunchDescription([
        SetEnvironmentVariable(name='PYTHONPATH', value=new_pp),
        DeclareLaunchArgument('image_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('weights', default_value=weights_default),
        DeclareLaunchArgument('device', default_value='cpu'),
        DeclareLaunchArgument(
            'skip_frames',
            default_value='1',
            description='Process 1 of every (skip_frames+1) images (CPU relief)',
        ),
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
    ])
