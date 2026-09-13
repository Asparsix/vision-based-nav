#!/usr/bin/env python3
"""Mapping bringup: depth → /scan → slam_toolbox → /map.

Prereq: ugv_sim already running (Gazebo + robot + RGB-D bridge).

Drive slowly with teleop to build the map, then save:
  ros2 run nav2_map_server map_saver_cli -f ~/ugv_vision_nav_ws/data/maps/outdoor_path
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import AndSubstitution, LaunchConfiguration, NotSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    pkg = get_package_share_directory('ugv_slam')
    mapper_params = os.path.join(pkg, 'config', 'mapper_params.yaml')
    depth_params = os.path.join(pkg, 'config', 'depthimage_to_laserscan.yaml')
    rviz_cfg = os.path.join(pkg, 'rviz', 'mapping.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_lifecycle_manager = LaunchConfiguration('use_lifecycle_manager')
    use_rviz = LaunchConfiguration('use_rviz')

    depth_to_scan = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        output='screen',
        parameters=[depth_params, {'use_sim_time': use_sim_time}],
        remappings=[
            ('depth', '/camera/depth/image_raw'),
            ('depth_camera_info', '/camera/depth/camera_info'),
            ('scan', '/scan'),
        ],
    )

    slam = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        namespace='',
        parameters=[
            mapper_params,
            {
                'use_sim_time': use_sim_time,
                'use_lifecycle_manager': use_lifecycle_manager,
            },
        ],
    )

    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(
            AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager))
        ),
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                LogInfo(msg='[ugv_slam] activating slam_toolbox'),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        ),
        condition=IfCondition(
            AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager))
        ),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('use_lifecycle_manager', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        LogInfo(msg='[ugv_slam] depth→scan→slam_toolbox mapping (drive with teleop)'),
        depth_to_scan,
        slam,
        configure_event,
        activate_event,
        rviz,
    ])
