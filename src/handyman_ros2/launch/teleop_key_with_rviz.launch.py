#!/usr/bin/env python3
"""teleop_key_with_rviz launch — ROS 2 Humble
   Replaces original teleop_key_with_rviz.launch XML.
   Launches: teleop_key_handyman + object_detection_node + rviz2 + rosbridge
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    handyman_pkg = FindPackageShare('handyman_ros2')

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
        DeclareLaunchArgument('sigverse_ros_bridge_port', default_value='50001'),
        DeclareLaunchArgument('sync_time_num',             default_value='1'),
        DeclareLaunchArgument('ros_bridge_port',           default_value='9090'),
    ]

    teleop_node = Node(
        package='handyman_ros2',
        executable='teleop_key_handyman',
        name='teleop_key_handyman',
        output='screen',
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

    return LaunchDescription(args + [teleop_node, vision_node, rviz_node, rosbridge_node])
