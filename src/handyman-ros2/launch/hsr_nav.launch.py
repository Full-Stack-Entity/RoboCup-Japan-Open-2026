#!/usr/bin/env python3
"""HSR Navigation launch (handyman_sample + vision + rviz2) — ROS 2 Humble
   Replaces the original hsr_nav.launch XML.
   Mirrors original: launches handyman_sample, object_detection_node, rviz2, rosbridge.
   All original args preserved including sub_laser_scan_topic_name, sigverse_ros_bridge_port.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    handyman_pkg = FindPackageShare('handyman')

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
        DeclareLaunchArgument('sub_laser_scan_topic_name',
            default_value='/hsrb/base_scan'),
        DeclareLaunchArgument('rgbd_camera',
            default_value='head_rgbd_sensor'),
        DeclareLaunchArgument('sigverse_ros_bridge_port', default_value='50001'),
        DeclareLaunchArgument('ros_bridge_port',          default_value='9090'),
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

    vision_node = Node(
        package='rcup_vision',
        executable='object_detection_node',
        name='vision',
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([handyman_pkg, 'launch', 'hsr.rviz'])],
    )

    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[{'port': LaunchConfiguration('ros_bridge_port')}],
    )

    return LaunchDescription(args + [handyman_node, vision_node, rviz_node, rosbridge_node])
