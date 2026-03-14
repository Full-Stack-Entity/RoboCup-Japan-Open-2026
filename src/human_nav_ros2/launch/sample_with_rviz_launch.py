# 原 ROS1: launch/sample_with_rviz.launch -> ROS2: launch/sample_with_rviz_launch.py
# 等价迁移：启动 human_navigation_sample、hsr_key_teleop 与 RViz2
# 按照官方 competition_test_tools 标准，包含 sigverse_ros_bridge 和 rosbridge_websocket

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # 官方标准端口参数
    sigverse_ros_bridge_port = LaunchConfiguration(
        "sigverse_ros_bridge_port", default="50001"
    )
    ros_bridge_port = LaunchConfiguration("ros_bridge_port", default="9090")

    # 话题参数
    sub_joint_state_topic_name = LaunchConfiguration(
        "sub_joint_state_topic_name", default="/hsrb/joint_states"
    )
    pub_base_twist_topic_name = LaunchConfiguration(
        "pub_base_twist_topic_name", default="/hsrb/command_velocity"
    )
    pub_arm_trajectory_topic_name = LaunchConfiguration(
        "pub_arm_trajectory_topic_name", default="/hsrb/arm_trajectory_controller/command"
    )
    pub_gripper_trajectory_topic_name = LaunchConfiguration(
        "pub_gripper_trajectory_topic_name", default="/hsrb/gripper_controller/command"
    )

    # human_navigation_sample 节点
    human_navigation_sample_node = Node(
        package="human_nav_ros2",
        executable="human_navigation_sample",
        name="human_navigation_sample",
        output="screen",
    )

    # hsr_key_teleop 节点
    hsr_key_teleop_node = Node(
        package="human_nav_ros2",
        executable="hsr_key_teleop",
        name="human_navigation_hsr_key_teleop",
        output="screen",
        parameters=[{
            "sub_joint_state_topic_name": sub_joint_state_topic_name,
            "pub_base_twist_topic_name": pub_base_twist_topic_name,
            "pub_arm_trajectory_topic_name": pub_arm_trajectory_topic_name,
            "pub_gripper_trajectory_topic_name": pub_gripper_trajectory_topic_name,
        }],
    )

    # sigverse_ros_bridge 节点（官方标准必需组件）
    sigverse_ros_bridge_node = Node(
        package="sigverse_ros_bridge",
        executable="sigverse_ros_bridge",
        name="sigverse_ros_bridge",
        arguments=[sigverse_ros_bridge_port],
        output="screen",
    )

    # rosbridge_websocket（使用官方 launch.xml 并带标准参数）
    rosbridge_websocket = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            FindPackageShare("rosbridge_server").launch("rosbridge_websocket_launch.xml")
        ),
        launch_arguments={
            "port": ros_bridge_port,
            "default_call_service_timeout": "5.0",
            "call_services_in_new_thread": "true",
            "send_action_goals_in_new_thread": "true",
        }.items(),
    )

    # RViz2 节点
    rviz_config_path = os.path.join(
        get_package_share_directory("human_nav_ros2"), "rviz", "hsr.rviz"
    )
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        arguments=["-d", rviz_config_path],
        output="screen",
    )

    ld = LaunchDescription()

    # 声明参数
    ld.add_action(
        DeclareLaunchArgument(
            "sigverse_ros_bridge_port",
            default_value="50001",
            description="Port for sigverse_ros_bridge.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "ros_bridge_port",
            default_value="9090",
            description="Port for rosbridge websocket.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "sub_joint_state_topic_name",
            default_value="/hsrb/joint_states",
            description="Topic name for joint states (hsr_key_teleop).",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "pub_base_twist_topic_name",
            default_value="/hsrb/command_velocity",
            description="Topic name for base twist command.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "pub_arm_trajectory_topic_name",
            default_value="/hsrb/arm_trajectory_controller/command",
            description="Topic name for arm trajectory command.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "pub_gripper_trajectory_topic_name",
            default_value="/hsrb/gripper_controller/command",
            description="Topic name for gripper trajectory command.",
        )
    )

    # 添加节点（按官方标准顺序）
    ld.add_action(rosbridge_websocket)
    ld.add_action(sigverse_ros_bridge_node)
    ld.add_action(human_navigation_sample_node)
    ld.add_action(hsr_key_teleop_node)
    ld.add_action(rviz2_node)

    return ld
