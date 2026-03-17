#!/usr/bin/env python3
"""
Full-stack launch for Interactive Cleanup:
  - interactive_cleanup_sample  (C++ controller)
  - cleanup_detection_node      (Python vision: YOLO + PoseLandmarker)
  - sigverse_ros_bridge         (SIGVerse ↔ ROS 2)
  - rosbridge_websocket         (WebSocket bridge)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            'sigverse_ros_bridge_port', default_value='50001'),
        DeclareLaunchArgument(
            'ros_bridge_port', default_value='9090'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true'),
    ]

    cleanup_node = Node(
        package='interactive_cleanup',
        executable='interactive_cleanup_sample',
        name='interactive_cleanup_sample',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    vision_node = Node(
        package='cleanup_vision_ros2',
        executable='cleanup_detection_node',
        name='cleanup_detection_node',
        output='screen',
    )

    sigverse_bridge = Node(
        package='sigverse_ros_bridge',
        executable='sigverse_ros_bridge',
        arguments=[LaunchConfiguration('sigverse_ros_bridge_port')],
    )

    rosbridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('rosbridge_server'),
                'launch', 'rosbridge_websocket_launch.xml',
            ])
        ),
        launch_arguments={
            'port': LaunchConfiguration('ros_bridge_port'),
            'default_call_service_timeout': '5.0',
            'call_services_in_new_thread': 'true',
            'send_action_goals_in_new_thread': 'true',
        }.items(),
    )

    return LaunchDescription(
        args + [cleanup_node, vision_node, sigverse_bridge, rosbridge]
    )
