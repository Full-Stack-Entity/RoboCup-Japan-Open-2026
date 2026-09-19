"""Explicitly armed control stack for an ALREADY running map/localizer/planner.

No coordinator, request sender or goals. With arm_control=true these nodes can
move the robot when a navigation client sends a goal. Default starts nothing.
"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


SEARCH_GOAL_OVERRIDES={
    'general_goal_checker.xy_goal_tolerance':0.10,
    'general_goal_checker.yaw_goal_tolerance':0.10,
    'FollowPath.xy_goal_tolerance':0.10,
}


def setup(context):
    value=LaunchConfiguration('arm_control').perform(context).lower()
    if value=='false':return []
    if value!='true':raise RuntimeError('arm_control must be true or false')
    share=Path(get_package_share_directory('handyman_ros2'))
    params=str(share/'param/nav2_params.yaml')
    actions=[]
    for package,name in [('nav2_controller','controller_server'),('nav2_behaviors','behavior_server')]:
        node_params=[params,SEARCH_GOAL_OVERRIDES] if name=='controller_server' else [params]
        actions.append(Node(package=package,executable=name,name=name,output='screen',
                            parameters=node_params,remappings=[('cmd_vel','/hsrb/command_velocity')]))
    actions.append(Node(package='nav2_bt_navigator',executable='bt_navigator',name='bt_navigator',output='screen',
        parameters=[params,{'default_nav_to_pose_bt_xml':str(share/'behavior_trees/navigate_w_recovery.xml')}]))
    actions.append(Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='lifecycle_manager_search_control',
        output='screen',parameters=[{'autostart':True,'node_names':['controller_server','behavior_server','bt_navigator']}]))
    return actions


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('arm_control',default_value='false'),OpaqueFunction(function=setup)])
