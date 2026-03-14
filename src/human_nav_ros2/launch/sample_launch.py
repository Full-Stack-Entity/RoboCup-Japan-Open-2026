# 原 ROS1: launch/sample.launch -> ROS2: launch/sample_launch.py
# 等价迁移：启动 human_navigation_sample 节点
# 按照官方 competition_test_tools 标准，包含 sigverse_ros_bridge 和 rosbridge_websocket

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

    # human_navigation_sample 节点
    human_navigation_sample_node = Node(
        package="human_nav_ros2",
        executable="human_navigation_sample",
        name="human_navigation_sample",
        output="screen",
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

    # 添加节点（按官方标准顺序）
    ld.add_action(rosbridge_websocket)
    ld.add_action(sigverse_ros_bridge_node)
    ld.add_action(human_navigation_sample_node)

    return ld
