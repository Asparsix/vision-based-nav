#!/usr/bin/env python3
"""Launch image saver for Gazebo camera collection."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('image_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument(
            'output_dir',
            default_value='',
            description='Folder for images; empty = ~/ugv_vision_nav_ws/data/images_TIMESTAMP',
        ),
        DeclareLaunchArgument(
            'save_every_n',
            default_value='5',
            description='Save 1 of every N frames (reduce duplicates while driving)',
        ),
        DeclareLaunchArgument(
            'max_images',
            default_value='400',
            description='Stop saving after this many images (0 = unlimited)',
        ),
        Node(
            package='ugv_data_collect',
            executable='image_saver',
            name='image_saver',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'image_topic': LaunchConfiguration('image_topic'),
                'output_dir': LaunchConfiguration('output_dir'),
                'save_every_n': ParameterValue(
                    LaunchConfiguration('save_every_n'), value_type=int
                ),
                'max_images': ParameterValue(
                    LaunchConfiguration('max_images'), value_type=int
                ),
                'image_format': 'png',
            }],
        ),
    ])
