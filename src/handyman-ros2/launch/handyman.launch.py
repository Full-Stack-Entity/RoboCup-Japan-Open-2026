#!/usr/bin/env python3
"""Main handyman launch file — ROS 2 Humble
   Replaces original sample.launch XML.
   All original args preserved including sigverse_ros_bridge_port.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    args = [
        DeclareLaunchArgument('sub_msg_to_robot_topic_name',
            default_value='/handyman/message/to_robot'),
        DeclareLaunchArgument('pub_msg_to_moderator_topic_name',
            default_value='/handyman/message/to_moderator'),
        DeclareLaunchArgument('pub_base_twist_topic_name',
            default_value='/hsrb/command_velocity'),
        DeclareLaunchArgument('pub_arm_trajectory_topic_name',
            default_value='/hsrb/arm_trajectory_controller/command'),
        DeclareLaunchArgument('pub_gripper_trajectory_topic_name',
            default_value='/hsrb/gripper_controller/command'),
        DeclareLaunchArgument('sigverse_ros_bridge_port', default_value='50001'),
        DeclareLaunchArgument('sync_time_num',             default_value='1'),
        DeclareLaunchArgument('ros_bridge_port',           default_value='9090'),
    ]

    handyman_node = Node(
        package='handyman',
        executable='handyman_sample',
        name='handyman_sample',
        output='screen',
        parameters=[{
            'sub_msg_to_robot_topic_name':
                LaunchConfiguration('sub_msg_to_robot_topic_name'),
            'pub_msg_to_moderator_topic_name':
                LaunchConfiguration('pub_msg_to_moderator_topic_name'),
            'pub_base_twist_topic_name':
                LaunchConfiguration('pub_base_twist_topic_name'),
            'pub_arm_trajectory_topic_name':
                LaunchConfiguration('pub_arm_trajectory_topic_name'),
            'pub_gripper_trajectory_topic_name':
                LaunchConfiguration('pub_gripper_trajectory_topic_name'),
        }],
    )

    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[{'port': LaunchConfiguration('ros_bridge_port')}],
    )

    return LaunchDescription(args + [handyman_node, rosbridge_node])
