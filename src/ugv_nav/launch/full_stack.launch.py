#!/usr/bin/env python3
"""Full stereo + VIO + semantic Nav2 stack (sim must be started separately).

  Terminal 1: ros2 launch ugv_sim sim.launch.py
  Terminal 2: source scripts/setup_env.sh && ros2 launch ugv_nav full_stack.launch.py

Modes:
  mapping:=true   → RTAB mapping (teleop to build map/DB)
  mapping:=false  → RTAB localization + Nav2 (set goals in RViz)
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    ws = Path.home() / 'ugv_vision_nav_ws'
    opt = ws / '.opt' / 'opt' / 'ros' / 'jazzy'
    venv_site = ws / '.venv' / 'lib' / 'python3.12' / 'site-packages'
    weights = str(ws / 'data' / 'models' / 'unet_outdoor_best.pt')
    db_path = str(ws / 'data' / 'maps' / 'rtabmap.db')

    ugv_slam = get_package_share_directory('ugv_slam')
    ugv_nav = get_package_share_directory('ugv_nav')

    use_sim_time = LaunchConfiguration('use_sim_time')
    mapping = LaunchConfiguration('mapping')
    use_nav2 = LaunchConfiguration('use_nav2')
    use_rviz = LaunchConfiguration('use_rviz')

    # Ensure RTAB + torch are visible
    ament = f"{opt}{os.pathsep}{os.environ.get('AMENT_PREFIX_PATH', '')}"
    ld = f"{opt}/lib:{opt}/lib/x86_64-linux-gnu:{os.environ.get('LD_LIBRARY_PATH', '')}"
    py = f"{venv_site}{os.pathsep}{os.environ.get('PYTHONPATH', '')}"

    stereo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ugv_slam, 'launch', 'stereo.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    odometry = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ugv_slam, 'launch', 'odometry.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_vo': LaunchConfiguration('use_vo'),
        }.items(),
    )

    rtab_map = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ugv_slam, 'launch', 'rtabmap.launch.py')),
        condition=IfCondition(mapping),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'localization': 'false',
            'delete_db_on_start': 'true',
            'database_path': db_path,
            'use_rtabmap_viz': 'true',
        }.items(),
    )

    rtab_loc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ugv_slam, 'launch', 'rtabmap.launch.py')),
        condition=UnlessCondition(mapping),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'localization': 'true',
            'delete_db_on_start': 'false',
            'database_path': db_path,
            'use_rtabmap_viz': 'false',
        }.items(),
    )

    segmenter = Node(
        package='ugv_perception',
        executable='segmenter',
        name='ugv_segmenter',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'image_topic': '/camera/image_raw',
            'weights': weights,
            'device': 'cpu',
            'publish_overlay': True,
            'skip_frames': 1,
        }],
    )

    costmap = Node(
        package='ugv_perception',
        executable='seg_costmap',
        name='ugv_seg_costmap',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'mask_topic': '/perception/segmentation',
            'output_topic': '/perception/local_costmap',
            'obstacles_topic': '/perception/semantic_obstacles',
            'frame_id': 'base_link',
            'cam_y': 0.04,
        }],
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ugv_nav, 'launch', 'nav2.launch.py')),
        condition=IfCondition(
            PythonExpression(["'", use_nav2, "' == 'true' and '", mapping, "' == 'false'"])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_rviz': use_rviz,
        }.items(),
    )

    return LaunchDescription([
        SetEnvironmentVariable(name='AMENT_PREFIX_PATH', value=ament),
        SetEnvironmentVariable(name='LD_LIBRARY_PATH', value=ld),
        SetEnvironmentVariable(name='PYTHONPATH', value=py),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'mapping',
            default_value='false',
            description='true=RTAB mapping (teleop); false=localize + Nav2',
        ),
        DeclareLaunchArgument('use_nav2', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'use_vo',
            default_value='true',
            description='Fuse RGB-D visual odometry into EKF',
        ),
        LogInfo(msg='[ugv_nav] full stack: stereo + EKF(odom+imu+VO) + seg + RTAB (+ Nav2)'),
        stereo,
        odometry,
        segmenter,
        costmap,
        rtab_map,
        rtab_loc,
        nav2,
    ])
