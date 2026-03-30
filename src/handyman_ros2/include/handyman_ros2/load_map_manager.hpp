#pragma once

/**
 * @file load_map_manager.hpp
 * @brief 地图加载管理器头文件
 *
 * 负责在环境切换时：
 * - 启动/停止 map_server
 * - 启动/停止 AMCL
 * - 启动/停止 Nav2
 * - 发布初始位姿 /initialpose
 * - 清空 costmap
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_srvs/srv/empty.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <map>
#include <mutex>
#include <string>
#include <condition_variable>

namespace handyman_ros2 {

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;

/**
 * @brief 地图加载管理器
 *
 * 负责在环境切换时重建导航系统。
 * 包含 map_server、AMCL、Nav2 的启动/停止逻辑。
 * 实现为 header-only（所有方法内联），避免链接依赖。
 */
class LoadMapManager {
private:
    rclcpp::Node::SharedPtr node_;
    std::string current_environment_;
    std::map<std::string, std::string> environment_to_map_;
    std::map<std::string, geometry_msgs::msg::Pose> environment_to_initial_pose_;

    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr global_costmap_client_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr local_costmap_client_;

    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_action_client_;
    bool move_base_active_;
    bool navigation_cancelled_;
    rclcpp::Time last_feedback_time_;
    std::mutex state_mutex_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_msg_;
    std::mutex map_mutex_;
    std::condition_variable map_cv_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::string mapEnvironmentFormat(const std::string& env);
    bool gracefulStopNavigation();
    bool startMapServer(const std::string& map_file);
    bool startAMCL(const geometry_msgs::msg::Pose& initial_pose);
    bool startNav2();
    bool forceClearCostmaps();
    bool waitForSystemReady();
    bool checkTFReady();
    bool waitForMapService();
    bool publishInitialPose(const geometry_msgs::msg::Pose& initial_pose);
    bool stopMapServer();
    bool stopAMCL();
    bool stopNav2();
    void registerDefaultEnvironments();

public:
    explicit LoadMapManager(rclcpp::Node::SharedPtr node);

    bool isMoveBaseActive();
    bool cancelCurrentNavigationGoal();
    bool waitForNavigationStop(double timeout_seconds = 10.0);
    bool switchEnvironment(const std::string& environment);
    bool loadMap(const std::string& map_file);
    bool resetAMCL(const geometry_msgs::msg::Pose& initial_pose);

    void registerEnvironment(const std::string& env,
                             const std::string& map_file,
                             const geometry_msgs::msg::Pose& initial_pose);
};

}  // namespace handyman_ros2
