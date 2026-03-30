#pragma once

/**
 * @file map_nav_controller.hpp
 * @brief 地图加载与导航控制层头文件
 *
 * 模块2：地图加载与导航控制层
 * 功能范围：
 * - 环境地图加载与切换（LoadMapManager）
 * - Nav2 导航封装（导航目标发送、结果等待、取消）
 * - 房间 patrol waypoint 管理与距离判断
 * - 墙壁地图预检
 * - 走廊 fallback 机制
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "handyman_ros2/load_map_manager.hpp"

namespace handyman_ros2 {

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;

std::string lower(const std::string& str);

/**
 * @brief 房间距离判断的静态实现函数
 *
 * 供 TaskOrchestrator 直接调用，不依赖 MapNavController 实例。
 */
std::pair<bool, double> getRobotRoomDistanceImpl(
    const std::shared_ptr<tf2_ros::Buffer>& tf_buffer,
    const rclcpp::Node::SharedPtr& node,
    const std::string& environment,
    const std::string& target_room);

/**
 * @brief 地图加载与导航控制器
 *
 * 职责：
 * - 管理 LoadMapManager，加载/切换环境地图
 * - 封装 Nav2 navigate_to_pose action client
 * - 提供房间 patrol 导航（含走廊 fallback）
 * - 提供目标放置点导航
 * - 提供房间到达判断（矩形区域）
 * - 提供墙壁地图预检
 *
 * 被 TaskOrchestrator 持有和调用。
 * 不包含任务逻辑，只负责"去指定坐标"和"判断是否到达"。
 */
class MapNavController {
public:
    // ========================================================================
    // 常量
    // ========================================================================

    static constexpr double NAV_GOAL_TIMEOUT_SEC = 60.0;
    static constexpr int MAX_WAYPOINT_RETRIES = 2;

    // 房间距离检查间隔（秒）
    static constexpr double ROOM_CHECK_INTERVAL_FAR = 6.0;
    static constexpr double ROOM_CHECK_INTERVAL_MID = 4.0;
    static constexpr double ROOM_CHECK_INTERVAL_NEAR = 2.0;
    static constexpr double ROOM_DIST_FAR_THRESHOLD = 5.0;
    static constexpr double ROOM_DIST_NEAR_THRESHOLD = 2.0;

    // ========================================================================
    // 枚举
    // ========================================================================

    enum class NavResult {
        PENDING,   // 导航进行中
        SUCCESS,   // 导航成功到达
        FAILED,    // 导航失败（ABORTED）
        TIMEOUT,   // 导航超时
        CANCELLED  // 导航被取消
    };

    // ========================================================================
    // 构造与初始化
    // ========================================================================

    explicit MapNavController(rclcpp::Node::SharedPtr node);

    // ========================================================================
    // 任务级重置
    // ========================================================================

    void resetTaskState();

    // ========================================================================
    // 环境加载
    // ========================================================================

    bool loadEnvironment(const std::string& mapped_environment);
    bool waitForNav2Ready(int max_wait_attempts = 12);

    // ========================================================================
    // 基础导航控制
    // ========================================================================

    void sendNavGoal(const geometry_msgs::msg::PoseStamped& target_pose);
    void cancelNavGoal();
    bool isGoalActive() const;
    NavResult getNavResult();

    // ========================================================================
    // 房间 patrol 导航
    // ========================================================================

    // 开始巡逻（设置目标房间，初始化状态）
    void startRoomPatrol(const std::string& room);

    // 每循环调用一次，返回 true = patrol 全部完成
    bool updateRoomPatrol();

    // 查询是否已到达目标房间
    bool isRoomReached() const;

    // 查询机器人到目标房间的距离
    std::pair<bool, double> getRobotRoomDistance(const std::string& target_room);

    // 判断机器人是否在目标房间内
    bool isRobotInTargetRoom(const std::string& target_room);

    // ========================================================================
    // 目标放置点导航
    // ========================================================================

    void navigateToDestination(const std::string& destination, const std::string& room);

    // 设置目标位置用于实时距离检测
    void setDestinationForDistanceCheck(double x, double y);

    // 检查是否在目标阈值范围内（用于测试模式等场景）
    bool checkDestinationReached(double threshold = 1.2);

    NavResult getDestinationNavResult();

    // ========================================================================
    // 墙壁预检
    // ========================================================================

    bool isPathBlockedByWall(const geometry_msgs::msg::PoseStamped& target);

    // ========================================================================
    // TF / Pose 查询
    // ========================================================================

    std::shared_ptr<tf2_ros::Buffer> getTfBuffer() const { return tf_buffer_; }

    // ========================================================================
    // 静态工具方法（供外部调用，不依赖实例）
    // ========================================================================

    int roomLocationMaxPatrol(const std::string& environment, const std::string& room);
    geometry_msgs::msg::PoseStamped roomLocationByStep(const std::string& room, int variation);
    geometry_msgs::msg::PoseStamped getCorridorWaypoint(const std::string& room);
    geometry_msgs::msg::PoseStamped destLocation(const std::string& dest, const std::string& room);

    // ========================================================================
    // 内部成员
    // ========================================================================

private:
    rclcpp::Node::SharedPtr node_;

    // TF
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Nav2 action client
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_action_client_;
    GoalHandleNavigate::SharedPtr nav_goal_handle_;
    std::atomic<bool> nav_goal_active_{false};
    std::atomic<bool> nav_goal_cancelled_{false};
    std::atomic<bool> nav_goal_reached_{false};
    std::atomic<bool> nav_goal_failed_{false};
    std::atomic<bool> nav_goal_accepted_{false};
    std::atomic<bool> nav_goal_sent_{false};
    rclcpp::Time nav_goal_send_time_;

    // 目标放置点距离检测
    double last_destination_x_ = -1e10;
    double last_destination_y_ = -1e10;

    // LoadMapManager
    std::unique_ptr<LoadMapManager> map_manager_;
    std::string current_environment_;

    // 房间 patrol 状态
    std::string current_target_room_;
    int patrol_step_ = 0;
    bool room_reached_ = false;
    int waypoint_retry_count_ = 0;
    bool using_corridor_fallback_ = false;

    mutable std::mutex state_mutex_;

    // ========================================================================
    // 内部工具方法
    // ========================================================================

    bool waitForMapTf(double timeout_sec);
    void sendNavGoalInternalUnlocked(const geometry_msgs::msg::PoseStamped& target_pose);
    void cancelNavGoalInternalUnlocked();
    geometry_msgs::msg::PoseStamped makePoseStamped(double x, double y, double yaw_rad);
};

}  // namespace handyman_ros2
