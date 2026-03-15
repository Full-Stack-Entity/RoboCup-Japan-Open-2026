#!/usr/bin/env python3
"""Map-making launch (slam_toolbox / teleop_key) — ROS 2 Humble
   Replaces original make_map.launch (gmapping).
   Uses slam_toolbox (the ROS2 SLAM standard) instead of gmapping.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    args = [
        DeclareLaunchArgument('sub_msg_to_robot_topic_name',
            default_value='/handyman/message/to_robot'),
        DeclareLaunchArgument('pub_msg_to_moderator_topic_name',
            default_value='/handyman/message/to_moderator'),
        DeclareLaunchArgument('sub_joint_state_topic_name',
            default_value='/hsrb/joint_states'),
        DeclareLaunchArgument('sub_laser_scan_topic_name',
            default_value='/hsrb/base_scan'),
        DeclareLaunchArgument('rgbd_camera',
            default_value='head_rgbd_sensor'),
        DeclareLaunchArgument('pub_base_twist_topic_name',
            default_value='/hsrb/opt_command_velocity'),
        DeclareLaunchArgument('pub_base_trajectory_topic_name',
            default_value='/hsrb/omni_base_controller/command'),
        DeclareLaunchArgument('pub_arm_trajectory_topic_name',
            default_value='/hsrb/arm_trajectory_controller/command'),
        DeclareLaunchArgument('pub_gripper_trajectory_topic_name',
            default_value='/hsrb/gripper_trajectory_controller/command'),
        DeclareLaunchArgument('sigverse_ros_bridge_port', default_value='50001'),
        DeclareLaunchArgument('sync_time_num',             default_value='1'),
        DeclareLaunchArgument('ros_bridge_port',           default_value='9090'),
    ]

    teleop_node = Node(
        package='handyman',
        executable='teleop_key_handyman',
        name='teleop_key_handyman',
        output='screen',
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

    # slam_toolbox online async mapper (replaces gmapping)
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'base_frame':           'base_footprint',
            'odom_frame':           'odom',
            'map_update_interval':  0.1,
            'max_laser_range':      4.0,
            'minimum_travel_distance': 0.2,
            'minimum_travel_heading':  0.2,
            'scan_buffer_size':     10,
            'use_sim_time':         False,
        }],
        remappings=[('scan', LaunchConfiguration('sub_laser_scan_topic_name'))],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d',
            PathJoinSubstitution(
                [FindPackageShare('handyman'), 'launch', 'hsr.rviz'])],
    )

    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[{'port': LaunchConfiguration('ros_bridge_port')}],
    )

    return LaunchDescription(args + [teleop_node, slam_node, rviz_node, rosbridge_node])
