#pragma once

#include <chrono>
#include <cstddef>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/srv/clear_entire_costmap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "handyman_rebuild_ros2/environment_config.hpp"
#include "handyman_rebuild_ros2/navigation_plan.hpp"

namespace handyman_rebuild_ros2
{

enum class NavigationTarget
{
  kRoom,
  kDestination,
};

struct NavigationOutcome
{
  bool success{false};
  NavigationTarget target{NavigationTarget::kRoom};
  std::size_t attempts{0};
  Pose2D robot_pose{};
  std::string reason;
};

struct NavigationSettings
{
  std::size_t maximum_attempts{3};
  double goal_timeout_sec{60.0};
  double server_wait_timeout_sec{15.0};
  double room_boundary_tolerance_m{0.20};
  double destination_tolerance_m{0.75};
  std::string map_frame{"map"};
  std::string robot_frame{"base_footprint"};
  std::string action_name{"navigate_to_pose"};
};

class NavigationExecutor
{
public:
  using CompletionCallback = std::function<void(const NavigationOutcome &)>;

  NavigationExecutor(
    rclcpp::Node * node,
    const EnvironmentCatalog * catalog,
    NavigationSettings settings = {});

  bool navigateToRoom(
    const std::string & environment,
    const std::string & room,
    CompletionCallback completion);

  bool navigateToDestination(
    const std::string & environment,
    const std::string & destination,
    const std::string & destination_room,
    CompletionCallback completion);

  void cancel();
  bool active() const noexcept;

private:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  bool begin(
    const EnvironmentConfig & environment,
    NavigationPlan plan,
    NavigationTarget target,
    CompletionCallback completion);
  void waitForServer();
  void sendCurrentGoal();
  void handleGoalRejected(std::uint64_t token);
  void failAttempt(std::uint64_t token, const std::string & reason);
  void verifyResult(std::uint64_t token);
  void finish(bool success, const std::string & reason, const Pose2D & pose = {});
  void clearCostmaps();
  std::optional<Pose2D> lookupRobotPose() const;
  void cancelTimers();

  rclcpp::Node * node_;
  const EnvironmentCatalog * catalog_;
  NavigationSettings settings_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_global_costmap_;
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_local_costmap_;
  mutable tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::optional<NavigationPlan> plan_;
  const EnvironmentConfig * environment_{nullptr};
  NavigationTarget target_{NavigationTarget::kRoom};
  CompletionCallback completion_;
  GoalHandle::SharedPtr goal_handle_;
  rclcpp::TimerBase::SharedPtr server_timer_;
  rclcpp::TimerBase::SharedPtr goal_timer_;
  rclcpp::TimerBase::SharedPtr retry_timer_;
  std::chrono::steady_clock::time_point server_wait_started_;
  std::uint64_t generation_{0};
  std::size_t route_waypoint_index_{0};
};

}  // namespace handyman_rebuild_ros2
