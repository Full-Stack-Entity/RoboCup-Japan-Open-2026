# 原 ROS1: launch/sample.launch -> ROS2: launch/sample_launch.py
# 等价迁移：启动 human_navigation_sample 节点
# 按照官方 competition_test_tools 标准，包含 sigverse_ros_bridge 和 rosbridge_websocket
# 子任务 D：可选启动 human_nav_llm_ros2/rewrite_guidance_node，并向 human_navigation_sample 传参

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_setup(context, *args, **kwargs):
    sigverse_ros_bridge_port = LaunchConfiguration("sigverse_ros_bridge_port").perform(context)
    ros_bridge_port = LaunchConfiguration("ros_bridge_port").perform(context)

    enable_llm = LaunchConfiguration("enable_llm_rewrite").perform(context).lower() in (
        "true",
        "1",
        "yes",
    )
    llm_model = LaunchConfiguration("llm_ollama_model").perform(context)
    llm_http_timeout = float(LaunchConfiguration("llm_http_timeout_sec").perform(context))
    llm_client_timeout = float(LaunchConfiguration("llm_client_timeout_sec").perform(context))
    llm_service = LaunchConfiguration("llm_service_name").perform(context)
    direction_hint_interval = float(LaunchConfiguration("direction_hint_interval_sec").perform(context))
    strict_template = LaunchConfiguration("strict_template_mode").perform(context).lower() in (
        "true",
        "1",
        "yes",
    )

    rosbridge_websocket = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("rosbridge_server"),
                "launch",
                "rosbridge_websocket_launch.xml",
            ])
        ),
        launch_arguments={
            "port": ros_bridge_port,
            "default_call_service_timeout": "5.0",
            "call_services_in_new_thread": "true",
            "send_action_goals_in_new_thread": "true",
        }.items(),
    )

    sigverse_ros_bridge_node = Node(
        package="sigverse_ros_bridge",
        executable="sigverse_ros_bridge",
        name="sigverse_ros_bridge",
        arguments=[sigverse_ros_bridge_port],
        output="screen",
    )

    actions = [
        rosbridge_websocket,
        sigverse_ros_bridge_node,
    ]

    if enable_llm:
        actions.append(
            Node(
                package="human_nav_llm_ros2",
                executable="rewrite_guidance_node",
                name="rewrite_guidance_node",
                output="screen",
                parameters=[
                    {
                        "model": llm_model,
                        "request_timeout_sec": llm_http_timeout,
                    }
                ],
            )
        )

    actions.append(
        Node(
            package="human_nav_ros2",
            executable="human_navigation_sample",
            name="human_navigation_sample",
            output="screen",
            parameters=[
                {
                    "use_llm_rewrite": enable_llm,
                    "llm_timeout_sec": llm_client_timeout,
                    "llm_service_name": llm_service,
                    "direction_hint_interval_sec": direction_hint_interval,
                    "strict_template_mode": strict_template,
                }
            ],
        )
    )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "sigverse_ros_bridge_port",
                default_value="50001",
                description="Port for sigverse_ros_bridge.",
            ),
            DeclareLaunchArgument(
                "ros_bridge_port",
                default_value="9090",
                description="Port for rosbridge websocket.",
            ),
            DeclareLaunchArgument(
                "enable_llm_rewrite",
                default_value="false",
                description="If true, start rewrite_guidance_node and set use_llm_rewrite on human_navigation_sample.",
            ),
            DeclareLaunchArgument(
                "llm_ollama_model",
                default_value="llama3.2:3b",
                description="Ollama model name for rewrite_guidance_node.",
            ),
            DeclareLaunchArgument(
                "llm_http_timeout_sec",
                default_value="30.0",
                description="HTTP timeout (seconds) for Python node calling Ollama.",
            ),
            DeclareLaunchArgument(
                "llm_client_timeout_sec",
                default_value="2.0",
                description="spin_until_future_complete timeout (seconds) in human_navigation_sample.",
            ),
            DeclareLaunchArgument(
                "llm_service_name",
                default_value="/rewrite_guidance",
                description="RewriteGuidance service name.",
            ),
            DeclareLaunchArgument(
                "direction_hint_interval_sec",
                default_value="7.0",
                description="Interval (seconds) between directional hints (Forward/Left/Right/Backward).",
            ),
            DeclareLaunchArgument(
                "strict_template_mode",
                default_value="true",
                description="If true, skeleton guidance uses strict C++ template only (no LLM rewrite).",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
