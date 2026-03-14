#!/usr/bin/env python3
"""Nav2 navigation stack launch — ROS 2 Humble
   Replaces original move_base.launch XML.
   Loads all original param files (DWA planner, costmaps, etc.).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('handyman')

    args = [
        DeclareLaunchArgument('cmd_vel_topic',
            default_value='/hsrb/command_velocity'),
    ]

    nav2_params = PathJoinSubstitution([pkg, 'param', 'nav2_params.yaml'])

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params],
        remappings=[('cmd_vel', LaunchConfiguration('cmd_vel_topic'))],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params],
    )

    recoveries_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time':  False,
            'autostart':     True,
            'node_names':    ['controller_server','planner_server',
                              'bt_navigator','behavior_server'],
        }],
    )

    return LaunchDescription(
        args + [controller_server, planner_server, bt_navigator,
                recoveries_server, lifecycle_manager]
    )
