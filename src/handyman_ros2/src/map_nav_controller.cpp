/**
 * @file map_nav_controller.cpp
 * @brief 地图加载与导航控制层实现
 *
 * 模块2：地图加载与导航控制层
 * 功能范围：
 * - 环境地图加载与切换（LoadMapManager）
 * - Nav2 导航封装（导航目标发送、结果等待、取消）
 * - 墙壁地图预检（WallMapAnalyzer）
 * - 房间 patrol waypoint 管理与距离判断
 * - 走廊 fallback 机制
 *
 * 被 TaskOrchestrator 调用，不包含任务逻辑。
 */

#include "handyman_ros2/map_nav_controller.hpp"

#include <algorithm>
#include <cctype>

namespace handyman_ros2 {

// ---------------------------------------------------------------------------
// 静态工具函数
// ---------------------------------------------------------------------------

std::string lower(const std::string& str) {
    std::string lower_str = str;
    std::transform(lower_str.begin(), lower_str.end(), lower_str.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    return lower_str;
}

// ---------------------------------------------------------------------------
// MapNavController::MapNavController
// ---------------------------------------------------------------------------

MapNavController::MapNavController(rclcpp::Node::SharedPtr node)
  : node_(node)
{
    // 初始化 TF buffer 和 listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 初始化 Nav2 action client
    nav_action_client_ = rclcpp_action::create_client<NavigateToPose>(node_, "navigate_to_pose");

    // 初始化 LoadMapManager
    map_manager_ = std::make_unique<LoadMapManager>(node_);

    RCLCPP_INFO(node_->get_logger(), "MapNavController initialized");
}

void MapNavController::resetTaskState()
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    nav_goal_sent_ = false;
    nav_goal_failed_ = false;
    nav_goal_reached_ = false;
    nav_goal_active_ = false;
    nav_goal_cancelled_ = false;
    nav_goal_accepted_ = false;
    nav_goal_handle_.reset();
    patrol_step_ = 0;
    waypoint_retry_count_ = 0;
    using_corridor_fallback_ = false;
    room_reached_ = false;
    current_target_room_.clear();
}

bool MapNavController::loadEnvironment(const std::string& mapped_environment)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    RCLCPP_INFO(node_->get_logger(), "MapNavController: Loading environment '%s'", mapped_environment.c_str());

    if (!map_manager_->switchEnvironment(mapped_environment)) {
        RCLCPP_ERROR(node_->get_logger(), "MapNavController: Failed to switch environment");
        return false;
    }

    current_environment_ = mapped_environment;
    return true;
}

bool MapNavController::waitForNav2Ready(int max_wait_attempts)
{
    RCLCPP_INFO(node_->get_logger(), "MapNavController: Waiting for navigate_to_pose action server...");

    for (int attempt = 0; attempt < max_wait_attempts; ++attempt) {
        if (nav_action_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_INFO(node_->get_logger(), "MapNavController: navigate_to_pose action server is ready");
            return true;
        }
        RCLCPP_WARN(node_->get_logger(),
                    "MapNavController: navigate_to_pose not ready, retrying... (attempt %d/%d)",
                    attempt + 1, max_wait_attempts);
        rclcpp::spin_some(node_);
    }

    RCLCPP_ERROR(node_->get_logger(), "MapNavController: Failed to connect to navigate_to_pose after %ds",
                 max_wait_attempts * 5);
    return false;
}

// ---------------------------------------------------------------------------
// 导航控制
// ---------------------------------------------------------------------------

void MapNavController::sendNavGoal(const geometry_msgs::msg::PoseStamped& target_pose)
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    if (!nav_action_client_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(node_->get_logger(), "MapNavController: Nav2 action server not available");
        nav_goal_failed_ = true;
        return;
    }

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = target_pose;

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    send_goal_options.goal_response_callback =
        [this](const GoalHandleNavigate::SharedPtr& goal_handle) {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (!goal_handle) {
                RCLCPP_ERROR(node_->get_logger(), "MapNavController: Navigation goal REJECTED");
                nav_goal_active_ = false;
                nav_goal_accepted_ = false;
                nav_goal_failed_ = true;
                nav_goal_sent_ = false;
                nav_goal_handle_.reset();
            } else {
                RCLCPP_INFO(node_->get_logger(), "MapNavController: Navigation goal ACCEPTED");
                nav_goal_accepted_ = true;
                nav_goal_handle_ = goal_handle;
            }
        };

    send_goal_options.result_callback =
        [this](const GoalHandleNavigate::WrappedResult& result) {
            std::lock_guard<std::mutex> lock(state_mutex_);
            nav_goal_active_ = false;
            switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    if (!nav_goal_cancelled_) {
                        RCLCPP_INFO(node_->get_logger(), "MapNavController: Navigation SUCCEEDED");
                        nav_goal_reached_ = true;
                    }
                    nav_goal_handle_.reset();
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(node_->get_logger(), "MapNavController: Navigation ABORTED");
                    nav_goal_failed_ = true;
                    nav_goal_sent_ = false;
                    nav_goal_handle_.reset();
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_WARN(node_->get_logger(), "MapNavController: Navigation CANCELED");
                    nav_goal_failed_ = true;
                    nav_goal_sent_ = false;
                    nav_goal_handle_.reset();
                    break;
                default:
                    RCLCPP_ERROR(node_->get_logger(), "MapNavController: Navigation UNKNOWN result");
                    nav_goal_failed_ = true;
                    nav_goal_sent_ = false;
                    nav_goal_handle_.reset();
                    break;
            }
        };

    send_goal_options.feedback_callback =
        [](GoalHandleNavigate::SharedPtr,
           const std::shared_ptr<const NavigateToPose::Feedback>& feedback) {
            // 反馈信息已在 Nav2 侧打印，此处可按需记录
        };

    nav_goal_active_ = true;
    nav_goal_reached_ = false;
    nav_goal_failed_ = false;
    nav_goal_cancelled_ = false;
    nav_goal_accepted_ = false;
    nav_goal_send_time_ = node_->now();
    nav_action_client_->async_send_goal(goal_msg, send_goal_options);

    RCLCPP_INFO(node_->get_logger(), "MapNavController: Nav goal sent to (%.2f, %.2f)",
                target_pose.pose.position.x, target_pose.pose.position.y);
}

void MapNavController::cancelNavGoal()
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    if (nav_goal_handle_) {
        nav_action_client_->async_cancel_goal(nav_goal_handle_);
        nav_goal_handle_.reset();
    }
    nav_action_client_->async_cancel_all_goals();
    nav_goal_active_ = false;
    nav_goal_cancelled_ = true;
    nav_goal_reached_ = false;
    nav_goal_failed_ = false;
    nav_goal_sent_ = false;

    RCLCPP_INFO(node_->get_logger(), "MapNavController: Navigation cancelled");
}

// ---------------------------------------------------------------------------
// 导航状态查询
// ---------------------------------------------------------------------------

bool MapNavController::isGoalActive() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!nav_goal_active_) return false;

    // 检查是否超时
    auto elapsed = node_->now() - nav_goal_send_time_;
    if (elapsed.seconds() > NAV_GOAL_TIMEOUT_SEC) {
        RCLCPP_WARN(node_->get_logger(), "MapNavController: Goal timeout detected (%.0f s > %.0f s)",
                    elapsed.seconds(), NAV_GOAL_TIMEOUT_SEC);
        return false;
    }
    return true;
}

MapNavController::NavResult MapNavController::getNavResult()
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    if (nav_goal_reached_) {
        nav_goal_reached_ = false;
        nav_goal_sent_ = false;
        return NavResult::SUCCESS;
    }
    if (nav_goal_failed_) {
        nav_goal_failed_ = false;
        nav_goal_sent_ = false;
        return NavResult::FAILED;
    }

    // 检查超时
    if (nav_goal_sent_ && !nav_goal_active_) {
        if ((node_->now() - nav_goal_send_time_).seconds() > NAV_GOAL_TIMEOUT_SEC) {
            RCLCPP_WARN(node_->get_logger(), "MapNavController: Navigation timed out");
            nav_goal_sent_ = false;
            nav_goal_cancelled_ = true;
            return NavResult::TIMEOUT;
        }
    }

    return NavResult::PENDING;
}

// ---------------------------------------------------------------------------
// 房间 patrol 导航
// ---------------------------------------------------------------------------

void MapNavController::startRoomPatrol(const std::string& room)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_target_room_ = room;
    patrol_step_ = 0;
    room_reached_ = false;
    waypoint_retry_count_ = 0;
    using_corridor_fallback_ = false;

    // 等待 TF 可用
    if (!waitForMapTf(30.0)) {
        RCLCPP_ERROR(node_->get_logger(), "MapNavController: Map TF not available, cannot start patrol");
    }
}

bool MapNavController::updateRoomPatrol()
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    if (current_target_room_.empty()) {
        RCLCPP_WARN(node_->get_logger(), "MapNavController: No target room set for patrol");
        return false;
    }

    // 检查是否已在目标房间
    if (isRobotInTargetRoom(current_target_room_)) {
        if (!room_reached_) {
            RCLCPP_INFO(node_->get_logger(),
                        "MapNavController: Robot already in room '%s', marking room_reached",
                        current_target_room_.c_str());
            room_reached_ = true;
        }
        return true;
    }

    // 获取当前环境的最大 patrol 数
    int max_patrol = roomLocationMaxPatrol(current_environment_, current_target_room_);

    // 需要发送新目标
    if (!nav_goal_sent_ && patrol_step_ < max_patrol) {
        geometry_msgs::msg::PoseStamped goal_pose;

        if (using_corridor_fallback_) {
            goal_pose = getCorridorWaypoint(current_target_room_);
        } else {
            goal_pose = roomLocationByStep(current_target_room_, patrol_step_);
            // 墙壁预检
            if (isPathBlockedByWall(goal_pose)) {
                RCLCPP_WARN(node_->get_logger(),
                            "MapNavController: Path to patrol %d blocked by wall, using corridor fallback",
                            patrol_step_ + 1);
                goal_pose = getCorridorWaypoint(current_target_room_);
                using_corridor_fallback_ = true;
                waypoint_retry_count_ = 0;
            }
        }

        goal_pose.header.frame_id = "map";
        goal_pose.header.stamp = node_->now();
        RCLCPP_INFO(node_->get_logger(),
                    "MapNavController: Sending patrol goal %d/%d to (%.2f, %.2f)%s",
                    patrol_step_ + 1, max_patrol,
                    goal_pose.pose.position.x, goal_pose.pose.position.y,
                    using_corridor_fallback_ ? " [CORRIDOR]" : "");
        sendNavGoalInternalUnlocked(goal_pose);
        nav_goal_sent_ = true;
        return false;
    }

    // 走廊 fallback 到达中转点后切回原始 waypoint
    if (using_corridor_fallback_ && nav_goal_reached_) {
        RCLCPP_INFO(node_->get_logger(), "MapNavController: Corridor reached, switching back to patrol waypoint");
        nav_goal_reached_ = false;
        nav_goal_sent_ = false;
        using_corridor_fallback_ = false;
        waypoint_retry_count_ = 0;
        return false;
    }

    // 导航成功
    if (nav_goal_reached_) {
        nav_goal_reached_ = false;
        nav_goal_sent_ = false;
        waypoint_retry_count_ = 0;

        if (!room_reached_ && isRobotInTargetRoom(current_target_room_)) {
            RCLCPP_INFO(node_->get_logger(), "MapNavController: Robot entered target room!");
            room_reached_ = true;
        }

        patrol_step_++;
        int max_patrol = roomLocationMaxPatrol(current_environment_, current_target_room_);

        if (patrol_step_ >= max_patrol) {
            if (!room_reached_) {
                if (isRobotInTargetRoom(current_target_room_)) {
                    room_reached_ = true;
                }
            }
            RCLCPP_INFO(node_->get_logger(), "MapNavController: All patrol waypoints done");
            return true;  // 全部完成
        }
        // 继续下一个 waypoint
        return false;
    }

    // 导航失败（重试逻辑）
    if (nav_goal_failed_) {
        nav_goal_failed_ = false;
        nav_goal_sent_ = false;
        waypoint_retry_count_++;

        if (using_corridor_fallback_) {
            using_corridor_fallback_ = false;
            patrol_step_++;
            int max_patrol = roomLocationMaxPatrol(current_environment_, current_target_room_);
            if (patrol_step_ >= max_patrol) {
                if (isRobotInTargetRoom(current_target_room_)) room_reached_ = true;
                return true;
            }
        } else if (waypoint_retry_count_ >= MAX_WAYPOINT_RETRIES) {
            RCLCPP_WARN(node_->get_logger(), "MapNavController: Waypoint %d failed %d times, corridor fallback",
                        patrol_step_ + 1, waypoint_retry_count_);
            waypoint_retry_count_ = 0;
            using_corridor_fallback_ = true;
        }
        return false;
    }

    // 导航超时
    if (nav_goal_sent_ && !nav_goal_active_) {
        if ((node_->now() - nav_goal_send_time_).seconds() > NAV_GOAL_TIMEOUT_SEC) {
            RCLCPP_WARN(node_->get_logger(), "MapNavController: Patrol goal timed out");
            cancelNavGoalInternalUnlocked();
            waypoint_retry_count_++;

            if (using_corridor_fallback_) {
                using_corridor_fallback_ = false;
                patrol_step_++;
                int max_patrol = roomLocationMaxPatrol(current_environment_, current_target_room_);
                if (patrol_step_ >= max_patrol) {
                    if (isRobotInTargetRoom(current_target_room_)) room_reached_ = true;
                    return true;
                }
            } else if (waypoint_retry_count_ >= MAX_WAYPOINT_RETRIES) {
                RCLCPP_WARN(node_->get_logger(), "MapNavController: Waypoint %d timed out %d times, corridor fallback",
                            patrol_step_ + 1, waypoint_retry_count_);
                waypoint_retry_count_ = 0;
                using_corridor_fallback_ = true;
            }
        }
    }

    return false;  // 继续等待
}

bool MapNavController::isRoomReached() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return room_reached_;
}

void MapNavController::navigateToDestination(const std::string& destination, const std::string& room)
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    if (nav_goal_sent_) {
        RCLCPP_WARN(node_->get_logger(), "MapNavController: Goal already sent for destination");
        return;
    }

    geometry_msgs::msg::PoseStamped goal_pose = destLocation(destination, room);
    goal_pose.header.frame_id = "map";
    goal_pose.header.stamp = node_->now();

    RCLCPP_INFO(node_->get_logger(), "MapNavController: Sending destination goal '%s' in room '%s' to (%.2f, %.2f)",
                destination.c_str(), room.c_str(),
                goal_pose.pose.position.x, goal_pose.pose.position.y);

    sendNavGoalInternalUnlocked(goal_pose);
    nav_goal_sent_ = true;
}

bool MapNavController::checkDestinationReached(double threshold)
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    if (!isGoalActive() && nav_goal_sent_) {
        // 目标已停止，检查是否是成功到达
        return false;  // 由 getNavResult() 判断
    }

    if (!nav_goal_sent_ || !isGoalActive()) {
        return false;
    }

    // 通过 TF 计算实时距离
    try {
        geometry_msgs::msg::TransformStamped transform =
            tf_buffer_->lookupTransform("map", "base_footprint", node_->now());

        if (last_destination_x_ < -1e9) {
            last_destination_x_ = transform.transform.translation.x;
            last_destination_y_ = transform.transform.translation.y;
            return false;
        }

        double dx = transform.transform.translation.x - last_destination_x_;
        double dy = transform.transform.translation.y - last_destination_y_;
        double dist = std::sqrt(dx * dx + dy * dy);

        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                             "MapNavController: Distance to destination: %.2f m", dist);

        return dist < threshold;
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(node_->get_logger(), "MapNavController: TF error in distance check: %s", ex.what());
        return false;
    }
}

void MapNavController::setDestinationForDistanceCheck(double x, double y)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_destination_x_ = x;
    last_destination_y_ = y;
}

MapNavController::NavResult MapNavController::getDestinationNavResult()
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    if (nav_goal_reached_) {
        nav_goal_reached_ = false;
        nav_goal_sent_ = false;
        nav_goal_handle_.reset();
        return NavResult::SUCCESS;
    }
    if (nav_goal_failed_) {
        nav_goal_failed_ = false;
        nav_goal_sent_ = false;
        nav_goal_handle_.reset();
        return NavResult::FAILED;
    }
    if (nav_goal_sent_ && !nav_goal_active_) {
        if ((node_->now() - nav_goal_send_time_).seconds() > NAV_GOAL_TIMEOUT_SEC) {
            RCLCPP_WARN(node_->get_logger(), "MapNavController: Destination nav timed out");
            cancelNavGoalInternalUnlocked();
            return NavResult::TIMEOUT;
        }
    }
    return NavResult::PENDING;
}

// ---------------------------------------------------------------------------
// 墙壁预检（WallMapAnalyzer 功能暂未实现，保留接口待补充）
bool MapNavController::isPathBlockedByWall(const geometry_msgs::msg::PoseStamped& /*target*/)
{
    // TODO: WallMapAnalyzer 功能（见 .cursorrules "[墙壁地图修复] 2026-03-28"）
    // 原 LoadMapManager 不包含此功能，暂时返回 false
    return false;
}

// ---------------------------------------------------------------------------
// 房间判断
// ---------------------------------------------------------------------------

std::pair<bool, double> MapNavController::getRobotRoomDistance(const std::string& target_room)
{
    return handyman_ros2::getRobotRoomDistanceImpl(
        tf_buffer_, node_,
        current_environment_, target_room);
}

bool MapNavController::isRobotInTargetRoom(const std::string& target_room)
{
    auto [in_room, dist] = getRobotRoomDistance(target_room);
    return in_room;
}

// ---------------------------------------------------------------------------
// 内部工具方法
// ---------------------------------------------------------------------------

bool MapNavController::waitForMapTf(double timeout_sec)
{
    auto start = node_->now();
    while (rclcpp::ok() && (node_->now() - start).seconds() < timeout_sec) {
        try {
            tf_buffer_->lookupTransform("map", "base_footprint",
                                          node_->now(), rclcpp::Duration::from_seconds(1.0));
            return true;
        } catch (const tf2::TransformException&) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            rclcpp::spin_some(node_);
        }
    }
    return false;
}

void MapNavController::sendNavGoalInternalUnlocked(const geometry_msgs::msg::PoseStamped& target_pose)
{
    if (!nav_action_client_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(node_->get_logger(), "MapNavController: Nav2 action server not available");
        nav_goal_failed_ = true;
        return;
    }

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = target_pose;

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    send_goal_options.goal_response_callback =
        [this](const GoalHandleNavigate::SharedPtr& goal_handle) {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (!goal_handle) {
                nav_goal_active_ = false;
                nav_goal_accepted_ = false;
                nav_goal_failed_ = true;
                nav_goal_sent_ = false;
                nav_goal_handle_.reset();
            } else {
                nav_goal_accepted_ = true;
                nav_goal_handle_ = goal_handle;
            }
        };

    send_goal_options.result_callback =
        [this](const GoalHandleNavigate::WrappedResult& result) {
            std::lock_guard<std::mutex> lock(state_mutex_);
            nav_goal_active_ = false;
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                if (!nav_goal_cancelled_) nav_goal_reached_ = true;
                nav_goal_handle_.reset();
            } else if (result.code == rclcpp_action::ResultCode::ABORTED) {
                nav_goal_failed_ = true;
                nav_goal_sent_ = false;
                nav_goal_handle_.reset();
            } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
                nav_goal_failed_ = true;
                nav_goal_sent_ = false;
                nav_goal_handle_.reset();
            } else {
                nav_goal_failed_ = true;
                nav_goal_sent_ = false;
                nav_goal_handle_.reset();
            }
        };

    send_goal_options.feedback_callback = nullptr;

    nav_goal_active_ = true;
    nav_goal_reached_ = false;
    nav_goal_failed_ = false;
    nav_goal_cancelled_ = false;
    nav_goal_accepted_ = false;
    nav_goal_send_time_ = node_->now();
    nav_action_client_->async_send_goal(goal_msg, send_goal_options);

    RCLCPP_INFO(node_->get_logger(), "MapNavController: Nav goal sent to (%.2f, %.2f)",
                target_pose.pose.position.x, target_pose.pose.position.y);
}

void MapNavController::cancelNavGoalInternalUnlocked()
{
    if (nav_goal_handle_) {
        nav_action_client_->async_cancel_goal(nav_goal_handle_);
        nav_goal_handle_.reset();
    }
    nav_action_client_->async_cancel_all_goals();
    nav_goal_active_ = false;
    nav_goal_cancelled_ = true;
    nav_goal_reached_ = false;
    nav_goal_failed_ = false;
    nav_goal_sent_ = false;
}

// ---------------------------------------------------------------------------
// 静态工具方法（房间 patrol waypoint）
// ---------------------------------------------------------------------------

int MapNavController::roomLocationMaxPatrol(const std::string& environment,
                                           const std::string& room)
{
    std::string env_lower = lower(environment);
    if (env_lower == "layout2019hm01") {
        if (room == "living")   return 2;
        if (room == "bedroom")  return 2;
        if (room == "lobby")    return 2;
        if (room == "kitchen")  return 1;
    } else if (env_lower == "layout2019hm02") {
        if (room == "living")   return 2;
        if (room == "lobby")    return 2;
        if (room == "kitchen")  return 2;
    } else if (env_lower == "layout2020hm01") {
        if (room == "living")   return 4;
        if (room == "bedroom")  return 2;
        if (room == "kitchen")  return 3;
    } else if (env_lower == "layout2021hm01") {
        if (room == "living")   return 2;
        if (room == "bedroom")  return 2;
        if (room == "lobby")    return 2;
    }
    return 1;  // 默认
}

geometry_msgs::msg::PoseStamped MapNavController::roomLocationByStep(
    const std::string& room, int variation)
{
    std::string env_lower = lower(current_environment_);

    // LayoutA
    if (env_lower == "layout2019hm01") {
        if (room == "living") {
            if (variation == 0) return makePoseStamped(2.0, 1.0, 1.57);
            if (variation == 1) return makePoseStamped(2.5, 4.08, 3.14);
        } else if (room == "bedroom") {
            if (variation == 0) return makePoseStamped(2.57, -4.31, 0);
            if (variation == 1) return makePoseStamped(8.64, -5.7, 0);
        } else if (room == "lobby") {
            if (variation == 0) return makePoseStamped(1.2, -6.16, -1.57);
            if (variation == 1) return makePoseStamped(1.0, -3.6, 0);
        } else if (room == "kitchen") {
            if (variation == 0) return makePoseStamped(8.5, 2.8, 3.14);
        }
    }
    // LayoutB
    else if (env_lower == "layout2019hm02") {
        if (room == "living") {
            if (variation == 0) return makePoseStamped(3.5, 9.6, 2.4);
            if (variation == 1) return makePoseStamped(1.84, 10.2, 0);
        } else if (room == "lobby") {
            if (variation == 0) return makePoseStamped(1.0, 0, 0);
            if (variation == 1) return makePoseStamped(2.5, 2.0, 0);
        } else if (room == "kitchen") {
            if (variation == 0) return makePoseStamped(5.5, -1.13, 0);
            if (variation == 1) return makePoseStamped(8.42, -1.13, 0);
        }
    }
    // LayoutC
    else if (env_lower == "layout2020hm01") {
        if (room == "living") {
            if (variation == 0) return makePoseStamped(0.5, 2.0, 1.57);
            if (variation == 1) return makePoseStamped(0.42, 3.48, 2.355);
            if (variation == 2) return makePoseStamped(4.5, 3.48, 0.0);
            if (variation == 3) return makePoseStamped(4.5, -0.65, 0.0);
        } else if (room == "bedroom") {
            if (variation == 0) return makePoseStamped(0.1, 6.9, 0.0);
            if (variation == 1) return makePoseStamped(3.0, 8.0, 0.0);
        } else if (room == "kitchen") {
            if (variation == 0) return makePoseStamped(6.5, -1.2, 0);
            if (variation == 1) return makePoseStamped(7.8, 1.2, 0);
            if (variation == 2) return makePoseStamped(6.5, 3.9, 0);
        }
    }
    // LayoutD
    else if (env_lower == "layout2021hm01") {
        if (room == "living") {
            if (variation == 0) return makePoseStamped(1.0, 0.0, 0);
            if (variation == 1) return makePoseStamped(3.5, 0.0, 0);
        } else if (room == "bedroom") {
            if (variation == 0) return makePoseStamped(4.0, -8.5, 0);
            if (variation == 1) return makePoseStamped(1.69, -8.0, 0.0);
        } else if (room == "lobby") {
            if (variation == 0) return makePoseStamped(-1.86, -8.38, 0.0);
            if (variation == 1) return makePoseStamped(-4.86, -8.7, 0.0);
        }
    }

    // Fallback
    return makePoseStamped(0.5, 2.0, 1.57);
}

geometry_msgs::msg::PoseStamped MapNavController::getCorridorWaypoint(const std::string& room)
{
    geometry_msgs::msg::PoseStamped corridor;
    std::string env_lower = lower(current_environment_);

    if (env_lower == "layout2019hm01") {
        if (room == "living")     corridor = makePoseStamped(1.8,  0.0,   1.57);
        else if (room == "bedroom") corridor = makePoseStamped(3.5,  -3.5,  0);
        else if (room == "lobby")   corridor = makePoseStamped(0.0,  -3.0,  0);
        else if (room == "kitchen") corridor = makePoseStamped(6.0,   1.0,  0);
        else corridor = makePoseStamped(1.8, 0.0, 1.57);
    } else if (env_lower == "layout2019hm02") {
        if (room == "living")     corridor = makePoseStamped(2.0,   8.5,  2.4);
        else if (room == "lobby")   corridor = makePoseStamped(0.0,   0.0,  0);
        else if (room == "kitchen") corridor = makePoseStamped(4.5,  -0.5,  0);
        else corridor = makePoseStamped(2.0, 8.5, 2.4);
    } else if (env_lower == "layout2020hm01") {
        if (room == "living")     corridor = makePoseStamped(0.5,   0.5,  1.57);
        else if (room == "bedroom") corridor = makePoseStamped(1.5,   5.5,  0);
        else if (room == "kitchen") corridor = makePoseStamped(5.0,  -0.5,  0);
        else corridor = makePoseStamped(0.5, 0.5, 1.57);
    } else if (env_lower == "layout2021hm01") {
        if (room == "living")     corridor = makePoseStamped(0.5,  -1.0,  0);
        else if (room == "bedroom") corridor = makePoseStamped(2.5,  -6.5,  0);
        else if (room == "lobby")   corridor = makePoseStamped(-2.0, -5.5,  0);
        else corridor = makePoseStamped(0.5, -1.0, 0);
    } else {
        corridor = makePoseStamped(1.0, 0.0, 0);
    }

    RCLCPP_INFO(node_->get_logger(), "MapNavController: Corridor waypoint for room '%s': (%.2f, %.2f)",
                room.c_str(), corridor.pose.position.x, corridor.pose.position.y);
    return corridor;
}

// ---------------------------------------------------------------------------
// destLocation
// ---------------------------------------------------------------------------

geometry_msgs::msg::PoseStamped MapNavController::destLocation(const std::string& dest,
                                                              const std::string& room)
{
    // LayoutA
    auto A_ws_lv  = makePoseStamped(2.13,  -0.003, -1.57);
    auto A_ws_lb  = makePoseStamped(9.3,   -6.4,    0);
    auto A_ws_lo  = makePoseStamped(1.3,   -6.1,   -1.57);
    auto A_corner = makePoseStamped(-0.15, -6.0,   -1.57);
    auto A_arm    = makePoseStamped(2.08,  -3.2916, 1.57);
    auto A_recycle = makePoseStamped(-0.5,   1.28,   3.14);
    auto A_burnable = makePoseStamped(-0.5,  2.4,    3.14);
    auto A_bottle  = makePoseStamped(-0.8,  -2.0,    3.14);
    auto A_sq_low  = makePoseStamped(0.774, 2.79,    0.8);
    auto A_dining  = makePoseStamped(7.8,   3.07,    0);
    auto A_ws_side = makePoseStamped(4.8,   1.9,    -1.8);
    auto A_bed     = makePoseStamped(6.8,  -4.9,     1.57);
    auto A_wagon   = makePoseStamped(8.12, -2.7,     1.57);
    auto A_card    = makePoseStamped(9.2,  -4.85,    0);

    // LayoutB
    auto B_ws_lv  = makePoseStamped(1.45,  7.6,  -1.57);
    auto B_ws_lo  = makePoseStamped(1.4,  -0.5, -1.57);
    auto B_arm    = makePoseStamped(-0.19, 4.45, 3.14);
    auto B_recycle = makePoseStamped(6.7,  3.8,  1.57);
    auto B_bottle  = makePoseStamped(7.56, 3.7,  1.57);
    auto B_sq_low  = makePoseStamped(1.28, 10.5, 1.57);
    auto B_round   = makePoseStamped(1.9,   8.3,  1.57);
    auto B_din_k   = makePoseStamped(7.85, -1.1,  3.14);
    auto B_din_l   = makePoseStamped(2.2,   2.0,  3.14);
    auto B_ws_side_l = makePoseStamped(1.0,  5.25, 1.57);
    auto B_ws_side_k = makePoseStamped(9.0,  3.0,  0);
    auto B_shelf   = makePoseStamped(2.0,   5.0,  1.57);
    auto B_wagon   = makePoseStamped(3.14,  0.75, 0);
    auto B_card    = makePoseStamped(2.9,   7.6, -1.57);

    // LayoutC
    auto C_recycle  = makePoseStamped(7.2,   4.55,  1.57);
    auto C_burn_l   = makePoseStamped(3.8,  -1.5,  -1.57);
    auto C_burn_k   = makePoseStamped(8.3,   2.3,   0);
    auto C_bottle   = makePoseStamped(6.4587, 4.6449, 1.57);
    auto C_ws_lv    = makePoseStamped(-0.4,  3.5,   3.14);
    auto C_ws_bed   = makePoseStamped(2.8,   7.2,  -1.57);
    auto C_dining   = makePoseStamped(3.2,   0.1,   1.57);
    auto C_ws_side  = makePoseStamped(2.7,  -1.6,  -1.57);
    auto C_iron_bed = makePoseStamped(3.24,  8.2,    0);
    auto C_shelf    = makePoseStamped(2.4,   4.8,   1.57);
    auto C_card     = makePoseStamped(0.95,  8.5,   1.57);

    // LayoutD
    auto D_ws_lv   = makePoseStamped(2.15,  0.7,   1.57);
    auto D_corner  = makePoseStamped(-5.3, -10.2, -1.57);
    auto D_arm     = makePoseStamped(4.0,  -9.86, -1.57);
    auto D_burn_l  = makePoseStamped(-1.58, -1.05,  3.14);
    auto D_burn_lo = makePoseStamped(-3.17, -4.8,   1.57);
    auto D_ws_lo   = makePoseStamped(-3.7, -10.3,  -1.57);
    auto D_din_l   = makePoseStamped(1.5,  -2.2,    0);
    auto D_din_lo  = makePoseStamped(-2.4, -8.4,    3.14);
    auto D_ws_bed  = makePoseStamped(1.3,  -7.5,    1.57);
    auto D_bed     = makePoseStamped(1.8, -10.6,    3.14);
    auto D_shelf   = makePoseStamped(-4.13, -4.7,   1.57);
    auto D_wag_lv  = makePoseStamped(-1.6,  -2.35,  3.14);
    auto D_wag_bed = makePoseStamped(0.27,  -9.06,   0);
    auto D_card    = makePoseStamped(-6.2965, -6.9433, 3.14);

    geometry_msgs::msg::PoseStamped location;

    if (current_environment_ == "Layout2019HM01") {
        if (dest == "white_side_table") {
            if (room == "living") location = A_ws_lv;
            else if (room == "lobby") location = A_ws_lo;
            else location = A_ws_lb;
        } else if (dest == "corner_sofa")    location = A_corner;
        else if (dest == "armchair")         location = A_arm;
        else if (dest == "trash_box_for_recycle")  location = A_recycle;
        else if (dest == "trash_box_for_burnable") location = A_burnable;
        else if (dest == "trash_box_for_bottle_can") location = A_bottle;
        else if (dest == "square_low_table") location = A_sq_low;
        else if (dest == "dining_table")     location = A_dining;
        else if (dest == "wooden_side_table") location = A_ws_side;
        else if (dest == "wooden_bed")       location = A_bed;
        else if (dest == "wagon")            location = A_wagon;
        else if (dest == "cardboard_box")     location = A_card;
    } else if (current_environment_ == "Layout2019HM02") {
        if (dest == "white_side_table") {
            if (room == "living") location = B_ws_lv;
            else if (room == "lobby") location = B_ws_lo;
        } else if (dest == "armchair")         location = B_arm;
        else if (dest == "trash_box_for_recycle")  location = B_recycle;
        else if (dest == "round_low_table")    location = B_round;
        else if (dest == "trash_box_for_bottle_can") location = B_bottle;
        else if (dest == "square_low_table")   location = B_sq_low;
        else if (dest == "dining_table") {
            if (room == "kitchen") location = B_din_k;
            else if (room == "lobby") location = B_din_l;
        } else if (dest == "wooden_side_table") {
            if (room == "kitchen") location = B_ws_side_k;
            else if (room == "lobby") location = B_ws_side_l;
        } else if (dest == "wagon")        location = B_wagon;
        else if (dest == "cardboard_box")  location = B_card;
        else if (dest == "wooden_shelf")   location = B_shelf;
    } else if (current_environment_ == "Layout2020HM01") {
        if (dest == "white_side_table") {
            if (room == "living") location = C_ws_lv;
            else if (room == "bedroom") location = C_ws_bed;
        } else if (dest == "trash_box_for_recycle") location = C_recycle;
        else if (dest == "trash_box_for_burnable") {
            if (room == "kitchen") location = C_burn_k;
            else if (room == "living") location = C_burn_l;
        } else if (dest == "trash_box_for_bottle_can") location = C_bottle;
        else if (dest == "dining_table")  location = C_dining;
        else if (dest == "wooden_side_table") location = C_ws_side;
        else if (dest == "iron_bed")      location = C_iron_bed;
        else if (dest == "cardboard_box")  location = C_card;
        else if (dest == "wooden_shelf")   location = C_shelf;
    } else if (current_environment_ == "Layout2021HM01") {
        if (dest == "white_side_table") {
            if (room == "living") location = D_ws_lv;
            else if (room == "bedroom") location = D_ws_bed;
            else if (room == "lobby") location = D_ws_lo;
        } else if (dest == "corner_sofa")    location = D_corner;
        else if (dest == "armchair")         location = D_arm;
        else if (dest == "trash_box_for_burnable") {
            if (room == "living") location = D_burn_l;
            else if (room == "lobby") location = D_burn_lo;
        } else if (dest == "wooden_shelf")   location = D_shelf;
        else if (dest == "dining_table") {
            if (room == "living") location = D_din_l;
            else if (room == "lobby") location = D_din_lo;
        } else if (dest == "wooden_bed")     location = D_bed;
        else if (dest == "wagon") {
            if (room == "living") location = D_wag_lv;
            else if (room == "bedroom") location = D_wag_bed;
        } else if (dest == "cardboard_box")  location = D_card;
    }

    RCLCPP_INFO(node_->get_logger(), "MapNavController: destLocation('%s', '%s') -> (%.2f, %.2f)",
                 dest.c_str(), room.c_str(),
                 location.pose.position.x, location.pose.position.y);
    return location;
}

geometry_msgs::msg::PoseStamped MapNavController::makePoseStamped(double x, double y, double yaw_rad)
{
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = "map";
    ps.header.stamp = node_->now();
    ps.pose.position.x = x;
    ps.pose.position.y = y;
    ps.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw_rad);
    q.normalize();
    ps.pose.orientation = tf2::toMsg(q);
    return ps;
}

// ---------------------------------------------------------------------------
// getRobotRoomDistance 静态实现（房间矩形区域判断）
// ---------------------------------------------------------------------------

std::pair<bool, double> getRobotRoomDistanceImpl(
    const std::shared_ptr<tf2_ros::Buffer>& tf_buffer,
    const rclcpp::Node::SharedPtr& node,
    const std::string& environment,
    const std::string& target_room)
{
    static constexpr double ROOM_HALF_SIZE_M = 1.0;

    auto distToRect = [](double px, double py, double cx, double cy, double h) -> double {
        if (px >= cx - h && px <= cx + h && py >= cy - h && py <= cy + h) {
            return 0.0;
        }
        double dx = 0.0, dy = 0.0;
        if (px < cx - h) dx = (cx - h) - px;
        else if (px > cx + h) dx = px - (cx + h);
        if (py < cy - h) dy = (cy - h) - py;
        else if (py > cy + h) dy = py - (cy + h);
        return std::sqrt(dx * dx + dy * dy);
    };

    try {
        geometry_msgs::msg::TransformStamped tf_stamped =
            tf_buffer->lookupTransform("map", "base_footprint",
                node->now(), rclcpp::Duration::from_seconds(1.0));
        double robot_x = tf_stamped.transform.translation.x;
        double robot_y = tf_stamped.transform.translation.y;

        // 各环境各房间的 waypoint 中心坐标
        std::vector<std::pair<double, double>> waypoints;
        std::string env_lower = lower(environment);

        if (env_lower == "layout2019hm01") {
            if (target_room == "living")   waypoints = {{2.0, 1.0}, {2.5, 4.08}};
            else if (target_room == "bedroom")  waypoints = {{2.57, -4.31}, {8.64, -5.7}};
            else if (target_room == "lobby")    waypoints = {{1.2, -6.16}, {1.0, -3.6}};
            else if (target_room == "kitchen")  waypoints = {{8.5, 2.8}, {5.0, 4.2}};
        } else if (env_lower == "layout2019hm02") {
            if (target_room == "living")    waypoints = {{3.5, 9.6}, {1.84, 10.2}};
            else if (target_room == "lobby")    waypoints = {{1.0, 0.0}, {2.5, 2.0}};
            else if (target_room == "kitchen")  waypoints = {{5.5, -1.13}, {8.42, -1.13}};
        } else if (env_lower == "layout2020hm01") {
            if (target_room == "living")    waypoints = {{0.5, 2.0}, {0.42, 3.48}, {4.5, 3.48}, {4.5, -0.65}};
            else if (target_room == "bedroom")  waypoints = {{0.1, 6.9}, {3.0, 8.0}};
            else if (target_room == "kitchen")  waypoints = {{6.5, -1.2}, {7.8, 1.2}, {6.5, 3.9}};
        } else if (env_lower == "layout2021hm01") {
            if (target_room == "living")    waypoints = {{1.0, 0.0}, {3.5, 0.0}};
            else if (target_room == "bedroom")  waypoints = {{4.0, -8.5}, {1.69, -8.0}};
            else if (target_room == "lobby")    waypoints = {{-1.86, -8.38}, {-4.86, -8.7}};
        }

        if (waypoints.empty()) return {false, 0.0};

        double min_dist = 1e9;
        bool in_room = false;
        for (const auto& wp : waypoints) {
            double dist = distToRect(robot_x, robot_y, wp.first, wp.second, ROOM_HALF_SIZE_M);
            if (dist < 0.01) {
                return {true, 0.0};
            }
            min_dist = std::min(min_dist, dist);
        }
        return {false, min_dist};

    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(node->get_logger(), "getRobotRoomDistance: TF error: %s", ex.what());
        return {false, -1.0};
    }
}

}  // namespace handyman_ros2
