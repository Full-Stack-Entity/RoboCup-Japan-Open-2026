#!/usr/bin/env python3
"""AMCL launch — ROS 2 Humble
   Replaces original amcl.launch XML.
   All original AMCL parameters preserved.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('scan_topic',     default_value='/hsrb/base_scan'),
        DeclareLaunchArgument('initial_pose_x', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_y', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_a', default_value='0.0'),
    ]

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{
            'min_particles':             500,
            'max_particles':             3000,
            'kld_err':                   0.02,
            'update_min_d':              0.20,
            'update_min_a':              0.20,
            'resample_interval':         2,
            'transform_tolerance':       0.5,
            'recovery_alpha_slow':       0.001,
            'recovery_alpha_fast':       0.1,
            'initial_pose_x':            LaunchConfiguration('initial_pose_x'),
            'initial_pose_y':            LaunchConfiguration('initial_pose_y'),
            'initial_pose_a':            LaunchConfiguration('initial_pose_a'),
            'gui_publish_rate':          50.0,
            'laser_max_range':           3.5,
            'laser_max_beams':           180,
            'laser_z_hit':               0.95,
            'laser_z_short':             0.05,
            'laser_z_max':               0.05,
            'laser_z_rand':              0.05,
            'laser_sigma_hit':           0.2,
            'laser_lambda_short':        0.1,
            'laser_likelihood_max_dist': 2.0,
            'laser_model_type':          'likelihood_field',
            'odom_model_type':           'diff',
            'odom_alpha1':               0.1,
            'odom_alpha2':               0.1,
            'odom_alpha3':               0.1,
            'odom_alpha4':               0.1,
            'odom_frame_id':             'odom',
            'base_frame_id':             'base_footprint',
            'global_frame_id':           'map',
            'use_sim_time':              False,
        }],
        remappings=[('scan', LaunchConfiguration('scan_topic'))],
    )

    return LaunchDescription(args + [amcl_node])
