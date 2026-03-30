/**
 * @file task_orchestrator_main.cpp
 * @brief task_orchestrator 可执行文件入口
 *
 * 替代 handyman_sample.cpp 的模块化版本入口。
 * 持有 TaskOrchestrator（协调 ProtocolHandler + MapNavController + VisionManipulationController）。
 */

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include "handyman_ros2/task_orchestrator.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("task_orchestrator");

    RCLCPP_INFO(node->get_logger(), "Starting task_orchestrator node (modular version)");

    handyman_ros2::TaskOrchestrator orchestrator(node);

    int result = orchestrator.run(argc, argv);

    rclcpp::shutdown();
    return result;
}
