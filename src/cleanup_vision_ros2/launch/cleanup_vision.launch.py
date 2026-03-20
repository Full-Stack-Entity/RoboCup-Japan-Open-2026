#!/usr/bin/env python3
"""Launch the cleanup vision detection node standalone."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cleanup_vision_ros2',
            executable='cleanup_detection_node',
            name='cleanup_detection_node',
            output='screen',
        ),
    ])
