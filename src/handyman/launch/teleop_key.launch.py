#!/usr/bin/env python3
"""Teleop key launch file — ROS 2 Humble
   Replaces original teleop_key.launch XML.
   Preserves all original args including sigverse_ros_bridge_port.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('sub_msg_to_robot_topic_name',
            default_value='/handyman/message/to_robot'),
        DeclareLaunchArgument('pub_msg_to_moderator_topic_name',
            default_value='/handyman/message/to_moderator'),
        DeclareLaunchArgument('sub_joint_state_topic_name',
            default_value='/hsrb/joint_states'),
        DeclareLaunchArgument('pub_base_twist_topic_name',
            default_value='/hsrb/command_velocity'),
        DeclareLaunchArgument('pub_base_trajectory_topic_name',
            default_value='/hsrb/omni_base_controller/command'),
        DeclareLaunchArgument('pub_arm_trajectory_topic_name',
            default_value='/hsrb/arm_trajectory_controller/command'),
        DeclareLaunchArgument('pub_gripper_trajectory_topic_name',
            default_value='/hsrb/gripper_controller/command'),
        # sigverse_ros_bridge_port: ROS1原稿中有此参数，ROS2中sigverse bridge已停用，
        # 保留参数声明以保持接口兼容
        DeclareLaunchArgument('sigverse_ros_bridge_port', default_value='50001'),
        DeclareLaunchArgument('sync_time_num',             default_value='1'),
        DeclareLaunchArgument('ros_bridge_port',           default_value='9090'),
    ]

    teleop_node = Node(
        package='handyman',
        executable='teleop_key_handyman',
        name='teleop_key_handyman',
        output='screen',
        # gnome-terminal相当于原稿 launch-prefix="gnome-terminal ..."
        prefix='xterm -e',
        parameters=[{
            'sub_msg_to_robot_topic_name':
                LaunchConfiguration('sub_msg_to_robot_topic_name'),
            'pub_msg_to_moderator_topic_name':
                LaunchConfiguration('pub_msg_to_moderator_topic_name'),
            'sub_joint_state_topic_name':
                LaunchConfiguration('sub_joint_state_topic_name'),
            'pub_base_twist_topic_name':
                LaunchConfiguration('pub_base_twist_topic_name'),
            'pub_base_trajectory_topic_name':
                LaunchConfiguration('pub_base_trajectory_topic_name'),
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

    return LaunchDescription(args + [teleop_node, rosbridge_node])
