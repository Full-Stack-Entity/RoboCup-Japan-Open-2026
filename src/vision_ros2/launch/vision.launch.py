#!/usr/bin/env python3
"""Vision package launch file — ROS 2 Humble"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    vision_node = Node(
        package='vision_ros2',
        executable='object_detection_node',
        name='object_detection_node',
        output='screen',
    )
    return LaunchDescription([vision_node])
