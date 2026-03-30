#pragma once

/**
 * @file task_orchestrator.hpp
 * @brief 任务编排层头文件
 *
 * 模块4：任务编排层
 * 功能范围：
 * - 持有 ProtocolHandler、MapNavController、VisionManipulationController 实例
 * - 实现状态机主循环
 * - 串联各模块调用（协议 ↔ 导航 ↔ 视觉 ↔ 机械臂）
 * - 持有任务上下文（当前环境、房间、物体、目标）
 *
 * 这是整个系统的"大脑"，负责状态转换决策。
 */

#ifndef HANDYMAN_ROS2__TASK_ORCHESTRATOR_HPP_
#define HANDYMAN_ROS2__TASK_ORCHESTRATOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_msgs/msg/string.hpp>

#include <memory>
#include <string>
#include <atomic>

// 必须包含完整定义（TaskContext 和 Listener 接口在 ProtocolHandler 中）
#include "handyman_ros2/protocol_handler.hpp"
#include "handyman_ros2/map_nav_controller.hpp"
#include "handyman_ros2/vision_manipulation_controller.hpp"

// 前向声明（仅用于不需要完整类型的场合）
namespace handyman_ros2 {
class MapNavController;
class VisionManipulationController;
}  // namespace handyman_ros2

namespace handyman_ros2 {

/**
 * @brief 任务编排器
 *
 * 持有并协调 ProtocolHandler、MapNavController、VisionManipulationController 三个模块。
 * 实现状态机主循环，串联各模块调用。
 *
 * 实现 ProtocolHandler::Listener 接口，接收来自 Avatar/Moderator 的消息。
 */
class TaskOrchestrator : public ProtocolHandler::Listener {
public:
    // ========================================================================
    // 常量
    // ========================================================================

    static constexpr bool TEST_MODE_ENABLED = true;         // true = 测试模式, false = 正式运行
    static constexpr double TEST_SCAN_DURATION_SEC = 5.0;  // 测试模式扫描超时
    static constexpr double TEST_DEST_THRESHOLD_M = 1.2;   // 测试模式到达阈值

    // ========================================================================
    // 状态枚举
    // ========================================================================

    enum class Step {
        Initialize,
        Ready,
        WaitForInstruction,
        GoToRoom1,
        MoveToInFrontOfTarget,
        Grasp,
        GoToRoom2,
        Release,
        ComeBack,
        TaskFinished,
        GiveUp
    };

    // ========================================================================
    // 构造与初始化
    // ========================================================================

    explicit TaskOrchestrator(rclcpp::Node::SharedPtr node);
    ~TaskOrchestrator() = default;

    // ========================================================================
    // 运行（主循环）
    // ========================================================================

    int run(int argc, char** argv);

    // ========================================================================
    // ProtocolHandler::Listener 实现
    // ========================================================================

    void onEnvironment(const std::string& unity_env) override;
    void onAreYouReady() override;
    void onInstruction(const std::string& detail) override;
    void onCorrectedInstruction(const std::string& detail) override;
    void onTaskSucceeded() override;
    void onTaskFailed(const std::string& detail) override;
    void onMissionComplete() override;

    // ========================================================================
    // 内部成员
    // ========================================================================

private:
    rclcpp::Node::SharedPtr node_;

    // 四个模块实例
    std::unique_ptr<ProtocolHandler> protocol_;
    std::unique_ptr<MapNavController> nav_;
    std::unique_ptr<VisionManipulationController> vision_arm_;

    // 状态机
    std::atomic<Step> step_{Step::Initialize};
    bool is_started_ = false;
    bool is_finished_ = false;
    bool is_failed_ = false;
    bool give_up_sent_ = false;
    rclcpp::Time give_up_send_time_;
    static constexpr double GIVE_UP_REPLY_TIMEOUT_SEC = 30.0;

    // 任务上下文
    ProtocolHandler::TaskContext task_;

    // 放置目标
    geometry_msgs::msg::PoseStamped dest_pose_;

    // 时间戳
    rclcpp::Time waiting_start_time_;
    rclcpp::Time test_mode_scan_start_time_;
    rclcpp::Time test_mode_target_time_;
    bool test_mode_target_valid_ = false;

    // 测试模式目标位置
    double test_target_x_ = 0.0;
    double test_target_y_ = 0.0;

    // 发布者
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_base_twist_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_arm_trajectory_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_gripper_trajectory_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_detection_target_;

    // 辅助方法
    void resetTaskContext();
    void stopBase();
    void moveBase(double linear_x, double linear_y, double angular_z);

    // 状态处理
    void handleInitialize();
    void handleReady();
    void handleWaitForInstruction();
    void handleGoToRoom1();
    void handleMoveToInFrontOfTarget();
    void handleGrasp();
    void handleGoToRoom2();
    void handleRelease();
    void handleTaskFinished();
    void handleGiveUp();

    // 发布目标物体名给视觉节点
    void publishDetectionTarget();
};

}  // namespace handyman_ros2

#endif  // HANDYMAN_ROS2__TASK_ORCHESTRATOR_HPP_
