/**
 * handyman_sample.cpp  —  ROS 2 Humble
 * Complete migration from ROS1. All functionality preserved.
 */
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <handyman_ros2/msg/handyman_msg.hpp>
#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstring>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav  = rclcpp_action::ClientGoalHandle<NavigateToPose>;
using HandymanMsg    = handyman_ros2::msg::HandymanMsg;

std::string lower(const std::string &str) {
  std::string s = str;
  std::transform(s.begin(), s.end(), s.begin(), ::tolower);
  return s;
}

// ==========================================================================
// LoadMapManager
// ==========================================================================
class LoadMapManager {
public:
  explicit LoadMapManager(rclcpp::Node::SharedPtr node)
  : node_(node), current_environment_("None"),
    move_base_active_(false), navigation_cancelled_(false)
  {
    global_costmap_client_ = node_->create_client<std_srvs::srv::Empty>(
      "/global_costmap/clear_entirely_global_costmap");
    local_costmap_client_  = node_->create_client<std_srvs::srv::Empty>(
      "/local_costmap/clear_entirely_local_costmap");
    last_feedback_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    tf_buffer_      = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_lmm_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    nav_action_client_lmm_ = rclcpp_action::create_client<NavigateToPose>(node_, "navigate_to_pose");
    // 订阅Nav2 feedback更新导航活跃状态（对应原稿 move_base/feedback 订阅）
    using NavFeedback = nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage;
    nav_feedback_sub_ = node_->create_subscription<NavFeedback>(
      "/navigate_to_pose/_action/feedback", 1,
      [this](const NavFeedback::SharedPtr) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        last_feedback_time_ = node_->now();
        move_base_active_ = true;
      });
    registerDefaultEnvironments();
  }

  bool switchEnvironment(const std::string &environment) {
    RCLCPP_INFO(node_->get_logger(), "Switching to environment: %s", environment.c_str());
    std::string mapped_env = mapEnvironmentFormat(environment);
    if (environment_to_map_.find(mapped_env) == environment_to_map_.end()) {
      RCLCPP_ERROR(node_->get_logger(), "Environment %s not registered", mapped_env.c_str());
      return false;
    }
    // 优雅停止（取消导航 -> 等待停止 -> 停止所有节点）
    // 对应原稿：gracefulStopNavigation 失败时 return false
    if (isNavigationActive()) {
      RCLCPP_INFO(node_->get_logger(), "Navigation active, cancelling before switch...");
      cancelCurrentNavigationGoal();
      waitForNavigationStop(15.0);
    }
    if (!gracefulStopNavigation()) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to gracefully stop navigation");
      return false;
    }

    const std::string &map_file = environment_to_map_[mapped_env];
    if (!startMapServer(map_file)) return false;

    // 基础地图等待
    if (!waitForMapTopic(30.0)) return false;

    // 环境切换后带时间戳验证的地图检测
    if (!waitForMapAfterEnvironmentSwitch(mapped_env)) return false;

    geometry_msgs::msg::Pose initial_pose = environment_to_initial_pose_[mapped_env];
    if (!startAMCL(initial_pose)) return false;
    if (!startNav2()) return false;

    forceClearCostmaps();

    // 等待所有导航服务+TF树就绪（对应原稿 waitForSystemReady 失败时 return false）
    if (!waitForSystemReady()) {
      RCLCPP_ERROR(node_->get_logger(), "System not ready after switching");
      return false;
    }

    current_environment_ = mapped_env;
    RCLCPP_INFO(node_->get_logger(), "Successfully switched to: %s", mapped_env.c_str());
    return true;
  }

  bool resetAMCL(const geometry_msgs::msg::Pose &initial_pose) { return startAMCL(initial_pose); }

  void registerEnvironment(const std::string &env, const std::string &map_file,
                           const geometry_msgs::msg::Pose &initial_pose) {
    environment_to_map_[env]          = map_file;
    environment_to_initial_pose_[env] = initial_pose;
    RCLCPP_INFO(node_->get_logger(), "Registered: %s -> %s", env.c_str(), map_file.c_str());
  }

  std::string mapEnvironmentFormat(const std::string &input_env) {
    if (input_env.find("Layout") == 0) return input_env.substr(6);
    return input_env;
  }

  std::string getCurrentEnvironment() const { return current_environment_; }

  // 取消当前导航目标（对应原稿 cancelCurrentNavigationGoal，使用Nav2 action client）
  bool cancelCurrentNavigationGoal() {
    RCLCPP_INFO(node_->get_logger(), "Cancelling current navigation goal...");
    if (!nav_action_client_lmm_) { navigation_cancelled_ = true; return true; }
    if (!nav_action_client_lmm_->wait_for_action_server(std::chrono::seconds(1))) {
      RCLCPP_WARN(node_->get_logger(), "NavigateToPose server not connected, cannot cancel");
      return false;
    }
    nav_action_client_lmm_->async_cancel_all_goals();
    navigation_cancelled_ = true;
    RCLCPP_INFO(node_->get_logger(), "Navigation goal cancelled successfully");
    return true;
  }

  // 地图独立加载接口（对应原稿 loadMap(map_file)）
  bool loadMap(const std::string &map_file) {
    RCLCPP_INFO(node_->get_logger(), "Loading map: %s", map_file.c_str());
    return startMapServer(map_file);
  }

  // 停止AMCL（对应原稿 stopAMCL）
  bool stopAMCL() {
    RCLCPP_INFO(node_->get_logger(), "Stopping AMCL...");
    system("pkill -f 'amcl' 2>/dev/null || true");
    rclcpp::sleep_for(1s);
    return true;
  }

  // 状态检查（对应原稿 isMapServerReady / isAMCLReady）
  bool isMapServerReady() {
    return node_->count_publishers("/map") > 0;
  }
  bool isAMCLReady() {
    return node_->count_publishers("/amcl_pose") > 0;
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::string current_environment_;
  std::map<std::string, std::string>              environment_to_map_;
  std::map<std::string, geometry_msgs::msg::Pose> environment_to_initial_pose_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr global_costmap_client_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr local_costmap_client_;
  bool move_base_active_;
  bool navigation_cancelled_;
  std::mutex state_mutex_;
  rclcpp::Time last_feedback_time_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_lmm_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_action_client_lmm_;
  // feedback subscriber (对应原稿 move_base_feedback_sub_)
  rclcpp::Subscription<nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage>::SharedPtr nav_feedback_sub_;

  void registerDefaultEnvironments() {
    geometry_msgs::msg::Pose p;
    p.orientation.w = 1.0;
    // Bug fix: 用 ament_index 动态查找包的 share 路径，避免硬编码安装路径
    std::string base;
    try {
      base = ament_index_cpp::get_package_share_directory("handyman_ros2") + "/maps/";
    } catch (const std::exception &e) {
      RCLCPP_ERROR(node_->get_logger(),
        "Failed to find handyman package share directory: %s", e.what());
      base = "/home/robot/ros2_ws/install/handyman_ros2/share/handyman_ros2/maps/"; // fallback
    }
    registerEnvironment("2019HM01", base + "2019HM01.yaml", p);
    registerEnvironment("2019HM02", base + "2019HM02.yaml", p);
    registerEnvironment("2020HM01", base + "2020HM01.yaml", p);
    registerEnvironment("2021HM01", base + "2021HM01.yaml", p);
  }

  bool stopMapServer() {
    system("pkill -f 'map_server' 2>/dev/null || true");
    std::this_thread::sleep_for(1500ms);
    int r = system("pgrep -f 'map_server' > /dev/null 2>&1");
    if (r == 0) { system("pkill -9 -f 'map_server' 2>/dev/null || true"); std::this_thread::sleep_for(500ms); }
    return true;
  }

  bool startMapServer(const std::string &map_file) {
    RCLCPP_INFO(node_->get_logger(), "Starting map_server: %s", map_file.c_str());
    stopMapServer();
    // Bug fix: system() 对后台命令(&)总返回0，无法用返回值判断启动是否成功
    // 改为启动后等待 /map publisher 出现来验证
    std::string cmd = "ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=" + map_file + " &";
    system(cmd.c_str());
    std::this_thread::sleep_for(2s);
    // 验证 map_server 是否真正启动（检查 /map topic 是否有 publisher）
    int retries = 5;
    while (retries-- > 0) {
      if (node_->count_publishers("/map") > 0) {
        RCLCPP_INFO(node_->get_logger(), "map_server started successfully");
        return true;
      }
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(1s);
    }
    RCLCPP_ERROR(node_->get_logger(), "map_server did not publish /map within timeout");
    return false;
  }

  bool startAMCL(const geometry_msgs::msg::Pose &initial_pose) {
    RCLCPP_INFO(node_->get_logger(), "Starting AMCL...");
    system("pkill -f 'amcl' 2>/dev/null || true");
    std::this_thread::sleep_for(1s);
    // Bug fix: system() 对后台命令(&)总返回0，改为验证 /amcl_pose publisher 出现
    system("ros2 run nav2_amcl amcl --ros-args -p use_sim_time:=false &");
    std::this_thread::sleep_for(2s);
    int retries = 5;
    while (retries-- > 0) {
      if (node_->count_publishers("/amcl_pose") > 0) {
        RCLCPP_INFO(node_->get_logger(), "AMCL started successfully");
        publishInitialPose(initial_pose);
        return true;
      }
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(1s);
    }
    RCLCPP_ERROR(node_->get_logger(), "AMCL did not publish /amcl_pose within timeout");
    return false;
  }

  bool startNav2() {
    RCLCPP_INFO(node_->get_logger(), "Starting Nav2...");
    int r = system("ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false &");
    if (r != 0) RCLCPP_WARN(node_->get_logger(), "Nav2 launch non-zero; may already be running");
    std::this_thread::sleep_for(3s);
    // 等待 navigate_to_pose action server 就绪
    // 对应原稿 waitForService("/move_base/make_plan", ros::Duration(5.0))
    auto deadline = node_->now() + rclcpp::Duration::from_seconds(15.0);
    while (rclcpp::ok()) {
      rclcpp::spin_some(node_);
      if (nav_action_client_lmm_->wait_for_action_server(std::chrono::milliseconds(500))) {
        RCLCPP_INFO(node_->get_logger(), "Nav2 navigate_to_pose action server is ready");
        return true;
      }
      if (node_->now() > deadline) {
        RCLCPP_ERROR(node_->get_logger(), "Timeout waiting for Nav2 action server");
        return false;
      }
      RCLCPP_INFO(node_->get_logger(), "Waiting for navigate_to_pose action server...");
    }
    return false;
  }

  bool gracefulStopNavigation() {
    RCLCPP_INFO(node_->get_logger(), "Gracefully stopping navigation system...");

    // 步骤1: 检查Nav2是否活跃
    if (isNavigationActive()) {
      RCLCPP_INFO(node_->get_logger(), "Navigation active, cancelling current goal...");
      // 步骤2: 取消当前导航目标
      if (!cancelCurrentNavigationGoal()) {
        RCLCPP_WARN(node_->get_logger(), "Failed to cancel navigation goal, continuing");
      }
      // 步骤3: 等待导航停止
      if (!waitForNavigationStop(15.0)) {
        RCLCPP_WARN(node_->get_logger(), "Navigation did not stop gracefully, forcing shutdown");
      }
    } else {
      RCLCPP_INFO(node_->get_logger(), "Navigation not active, proceeding with shutdown");
    }

    // 步骤4: 按正确顺序停止节点
    RCLCPP_INFO(node_->get_logger(), "Stopping navigation nodes in sequence...");
    // 先停 bt_navigator/controller (防止继续发速度命令)
    system("pkill -f 'bt_navigator' 2>/dev/null || true");
    system("pkill -f 'controller_server' 2>/dev/null || true");
    system("pkill -f 'planner_server' 2>/dev/null || true");
    system("pkill -f 'behavior_server' 2>/dev/null || true");
    system("pkill -f 'waypoint_follower' 2>/dev/null || true");
    rclcpp::sleep_for(2s);
    // 再停 AMCL
    if (!stopAMCL()) {
      RCLCPP_WARN(node_->get_logger(), "Failed to stop AMCL, continuing anyway");
    }
    // 最后停 map_server
    if (!stopMapServer()) {
      RCLCPP_WARN(node_->get_logger(), "Failed to stop map server, continuing anyway");
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    move_base_active_ = false; navigation_cancelled_ = false;
    RCLCPP_INFO(node_->get_logger(), "Navigation system stopped gracefully");
    return true;
  }

  bool publishInitialPose(const geometry_msgs::msg::Pose &initial_pose) {
    auto pub = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 1);
    // Bug fix: publisher discovery 需要时间，必须等待至少一个 subscriber 连接后再发布
    // 否则 AMCL 尚未订阅时消息被丢弃，初始位姿设置失败
    int wait_ms = 0;
    while (pub->get_subscription_count() == 0 && wait_ms < 3000) {
      rclcpp::sleep_for(100ms);
      rclcpp::spin_some(node_);
      wait_ms += 100;
    }
    if (pub->get_subscription_count() == 0) {
      RCLCPP_WARN(node_->get_logger(), "No subscriber on /initialpose after 3s, publishing anyway");
    }
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.stamp = node_->now(); msg.header.frame_id = "map";
    msg.pose.pose    = initial_pose;
    msg.pose.covariance[0] = 0.05; msg.pose.covariance[7] = 0.05; msg.pose.covariance[35] = 0.01;
    for (int i = 0; i < 3; ++i) { pub->publish(msg); rclcpp::sleep_for(100ms); }
    return true;
  }

  // 检查Nav2是否正在执行导航（对应原稿 isMoveBaseActive，双重检测逻辑）
  bool isNavigationActive() {
    // Bug fix: wait_for_action_server 不能在持有 state_mutex_ 时调用，否则与 feedback 回调死锁
    // 先在锁外检查 action server 连通性
    bool server_connected = nav_action_client_lmm_ &&
      nav_action_client_lmm_->wait_for_action_server(std::chrono::milliseconds(100));

    std::lock_guard<std::mutex> lock(state_mutex_);
    // 检查最近是否有反馈（2秒内无反馈则认为不活跃）
    if (last_feedback_time_.nanoseconds() > 0) {
      double secs = (node_->now() - last_feedback_time_).seconds();
      if (secs > 2.0) move_base_active_ = false;
    }
    if (!server_connected) {
      move_base_active_ = false;
    }
    return move_base_active_;
  }

  // 等待导航停止
  bool waitForNavigationStop(double timeout_seconds = 10.0) {
    RCLCPP_INFO(node_->get_logger(), "Waiting for navigation to stop...");
    auto start = node_->now();
    rclcpp::Rate rate(10.0);
    while (rclcpp::ok()) {
      // 必须 spin_some 才能让 feedback 回调更新 move_base_active_
      rclcpp::spin_some(node_);
      if (!isNavigationActive()) {
        RCLCPP_INFO(node_->get_logger(), "Navigation has stopped"); return true;
      }
      if ((node_->now() - start).seconds() > timeout_seconds) {
        RCLCPP_WARN(node_->get_logger(), "Timeout waiting for navigation to stop"); return false;
      }
      rate.sleep();
    }
    return false;
  }

  // 等待/map topic有效数据（含新鲜度检测，对应原稿 waitForMapService）
  // ROS2中用subscription+promise代替 ros::topic::waitForMessage
  bool waitForMapTopic(double timeout_sec) {
    RCLCPP_INFO(node_->get_logger(), "Waiting for /map topic with valid data...");
    const double max_freshness_age = 5.0;
    auto start = node_->now();
    rclcpp::Rate rate(2.0);

    // ROS2: subscription回调只在spin时触发，用executor在等待循环中驱动
    nav_msgs::msg::OccupancyGrid::SharedPtr received_map;
    std::mutex map_mutex;
    auto sub = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", rclcpp::QoS(1).transient_local(),
      [&](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(map_mutex);
        received_map = msg;
      });

    while (rclcpp::ok()) {
      if ((node_->now() - start).seconds() > timeout_sec) {
        RCLCPP_ERROR(node_->get_logger(),
          "Failed to get valid fresh map within %.2f seconds", timeout_sec);
        return false;
      }
      // 驱动回调
      rclcpp::spin_some(node_);
      nav_msgs::msg::OccupancyGrid::SharedPtr snap;
      { std::lock_guard<std::mutex> lk(map_mutex); snap = received_map; received_map = nullptr; }
      if (snap) {
        auto &m = *snap;
        double age = (node_->now() - rclcpp::Time(m.header.stamp)).seconds();
        if (m.info.width > 0 && m.info.height > 0 && !m.data.empty()) {
          if (age < max_freshness_age) {
            RCLCPP_INFO(node_->get_logger(),
              "Valid fresh map received! Size: %dx%d, Age: %.2f s",
              m.info.width, m.info.height, age);
            return true;
          } else {
            RCLCPP_WARN(node_->get_logger(),
              "Map not fresh enough (age: %.2f s, max: %.2f)", age, max_freshness_age);
          }
        } else {
          RCLCPP_WARN(node_->get_logger(),
            "Map data invalid (size: %dx%d, data len: %zu)",
            m.info.width, m.info.height, m.data.size());
        }
      }
      rate.sleep();
    }
    return false;
  }

  // 环境切换后的地图检测（含时间戳验证，对应原稿 waitForMapAfterEnvironmentSwitch）
  bool waitForMapAfterEnvironmentSwitch(const std::string &environment_name) {
    RCLCPP_INFO(node_->get_logger(),
      "Waiting for fresh map after switching to: %s", environment_name.c_str());
    const double max_wait_time     = 30.0;
    const double max_freshness_age = 10.0;
    auto switch_start = node_->now();
    rclcpp::Rate rate(2.0);

    nav_msgs::msg::OccupancyGrid::SharedPtr received_map;
    std::mutex map_mutex;
    auto sub = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", rclcpp::QoS(1).transient_local(),
      [&](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(map_mutex);
        received_map = msg;
      });

    while (rclcpp::ok()) {
      if ((node_->now() - switch_start).seconds() > max_wait_time) {
        RCLCPP_ERROR(node_->get_logger(),
          "Failed to get fresh map for %s within %.1f s",
          environment_name.c_str(), max_wait_time);
        return false;
      }
      // 驱动回调
      rclcpp::spin_some(node_);
      nav_msgs::msg::OccupancyGrid::SharedPtr snap;
      { std::lock_guard<std::mutex> lk(map_mutex); snap = received_map; received_map = nullptr; }
      if (snap) {
        auto &m = *snap;
        double age = (node_->now() - rclcpp::Time(m.header.stamp)).seconds();
        double stamp_sec = rclcpp::Time(m.header.stamp).seconds();
        double switch_sec = switch_start.seconds();
        if (m.info.width > 0 && m.info.height > 0 && !m.data.empty()) {
          if (stamp_sec >= switch_sec - 2.0) {
            if (age < max_freshness_age) {
              RCLCPP_INFO(node_->get_logger(),
                "Fresh map received after environment switch! Size: %dx%d, Age: %.2f s",
                m.info.width, m.info.height, age);
              return true;
            } else {
              RCLCPP_WARN(node_->get_logger(),
                "Map received after switch but not fresh enough (age: %.2f s)", age);
            }
          } else {
            RCLCPP_WARN(node_->get_logger(),
              "Map timestamp (%.2f) is before switch time (%.2f)",
              stamp_sec, switch_sec);
          }
        } else {
          RCLCPP_WARN(node_->get_logger(),
            "Map data invalid (size: %dx%d, data len: %zu)",
            m.info.width, m.info.height, m.data.size());
        }
      }
      rate.sleep();
    }
    return false;
  }

  // 等待所有导航服务就绪 + TF树检查（对应原稿 waitForSystemReady + checkTFReady）
  bool waitForSystemReady() {
    RCLCPP_INFO(node_->get_logger(), "Waiting for navigation system to be ready...");
    auto start = node_->now();
    const double timeout = 20.0;
    rclcpp::Rate rate(2.0);
    while (rclcpp::ok()) {
      // Bug fix: 必须 spin_some 才能让 TF / feedback 回调更新
      rclcpp::spin_some(node_);
      bool all_ready = true;

      // 检查map服务（对应原稿 /static_map 服务检查）
      if (node_->count_publishers("/map") == 0) {
        RCLCPP_WARN(node_->get_logger(), "Map server not ready"); all_ready = false;
      }
      // 检查AMCL（对应原稿 /amcl/set_parameters 服务检查）
      if (node_->count_publishers("/amcl_pose") == 0) {
        RCLCPP_WARN(node_->get_logger(), "AMCL not ready"); all_ready = false;
      }
      // 检查Nav2 navigate_to_pose（对应原稿 /move_base/make_plan 服务检查）
      if (!nav_action_client_lmm_ ||
          !nav_action_client_lmm_->wait_for_action_server(std::chrono::milliseconds(100))) {
        RCLCPP_WARN(node_->get_logger(), "Nav2 navigate_to_pose not ready"); all_ready = false;
      }
      // 检查TF: map->base_footprint
      if (!checkTFReady()) {
        RCLCPP_WARN(node_->get_logger(), "TF tree not ready"); all_ready = false;
      }
      if (all_ready) {
        RCLCPP_INFO(node_->get_logger(), "All navigation services are ready"); return true;
      }
      if ((node_->now() - start).seconds() > timeout) {
        RCLCPP_ERROR(node_->get_logger(), "Timeout waiting for system ready"); return false;
      }
      rate.sleep();
    }
    return false;
  }

  // TF树就绪检查（对应原稿 checkTFReady）
  bool checkTFReady() {
    if (!tf_buffer_) return false;
    std::string err;
    // 尝试 map->base_footprint
    if (tf_buffer_->canTransform("map", "base_footprint", tf2::TimePointZero, &err)) {
      return true;
    }
    // fallback: odom->base_footprint
    if (tf_buffer_->canTransform("odom", "base_footprint", tf2::TimePointZero, &err)) {
      return true;
    }
    return false;
  }

  bool forceClearCostmaps() {
    RCLCPP_INFO(node_->get_logger(), "Force clearing costmaps...");
    auto req = std::make_shared<std_srvs::srv::Empty::Request>();
    bool global_cleared = false;
    bool local_cleared  = false;

    // 第一次清除
    if (global_costmap_client_->service_is_ready()) {
      global_costmap_client_->async_send_request(req);
      global_cleared = true;
      RCLCPP_INFO(node_->get_logger(), "Global costmap cleared successfully");
    } else {
      RCLCPP_WARN(node_->get_logger(), "Global costmap service not available");
    }
    if (local_costmap_client_->service_is_ready()) {
      local_costmap_client_->async_send_request(req);
      local_cleared = true;
      RCLCPP_INFO(node_->get_logger(), "Local costmap cleared successfully");
    } else {
      RCLCPP_WARN(node_->get_logger(), "Local costmap service not available");
    }

    rclcpp::sleep_for(1s);

    // 第二次清除确保完全清除（与原稿一致）
    if (global_costmap_client_->service_is_ready()) global_costmap_client_->async_send_request(req);
    if (local_costmap_client_->service_is_ready())  local_costmap_client_->async_send_request(req);

    RCLCPP_INFO(node_->get_logger(), "Costmap clearing completed - Global: %s, Local: %s",
                global_cleared ? "SUCCESS" : "FAILED",
                local_cleared  ? "SUCCESS" : "FAILED");
    return global_cleared && local_cleared;
  }
};
// END LoadMapManager

// ==========================================================================
// HandymanSample
// ==========================================================================
class HandymanSample : public rclcpp::Node
{
public:
  HandymanSample() : rclcpp::Node("handyman_sample") {}
  int run();

private:
  enum Step {
    Initialize, Ready, WaitForInstruction, GoToRoom1, GoToRoom2,
    MoveToInFrontOfTarget, MoveToInFrontOfDest, Grasp, Release, ComeBack, TaskFinished
  };
  const std::string MSG_ARE_YOU_READY    = "Are_you_ready?";
  const std::string MSG_ENVIRONMENT      = "Environment";
  const std::string MSG_INSTRUCTION      = "Instruction";
  const std::string MSG_TASK_SUCCEEDED   = "Task_succeeded";
  const std::string MSG_TASK_FAILED      = "Task_failed";
  const std::string MSG_MISSION_COMPLETE = "Mission_complete";
  const std::string MSG_I_AM_READY       = "I_am_ready";
  const std::string MSG_ROOM_REACHED     = "Room_reached";
  const std::string MSG_OBJECT_GRASPED   = "Object_grasped";
  const std::string MSG_TASK_FINISHED    = "Task_finished";

  std::vector<std::string> rooms   = {"living","bedroom","lobby","kitchen"};
  std::vector<std::string> objects = {
    "apple","toy_penguin","rabbit_doll","bear_doll","dog_doll","canned_juice","sugar",
    "soysauce","sauce","ketchup","tumbler","white_cup","pink_cup","empty_ketchup",
    "filled_ketchup","ground_pepper","salt","empty_plastic_bottle","filled_plastic_bottle",
    "cubic_clock","toy_car","toy_duck","nursing_bottle","cigarette","hourglass","camera",
    "rubik's_cube","spray_bottle","matryoshka","game_controller","piggy_bank"};
  std::vector<std::string> dests = {
    "white_side_table","corner_sofa","round_low_table","square_low_table","wooden_shelf",
    "armchair","dining_table","wooden_side_table","wooden_bed","iron_bed","wagon",
    "trash_box_for_recycle","trash_box_for_burnable","trash_box_for_bottle_can",
    "cardboard_box","Avatar"};

  std::string ENVIRONMENT = "None";
  std::unique_ptr<LoadMapManager> map_manager_;
  trajectory_msgs::msg::JointTrajectory arm_joint_trajectory_;

  int    step_;
  int    patrol_step_;
  int    max_patrol_;
  bool   room_reached_;
  bool   found_object_, found_dest_;
  bool   ready_to_grasp_;
  double arm_height_, body_height_;
  bool   aligned_x_, aligned_y_;
  geometry_msgs::msg::PoseStamped object_pose_;
  geometry_msgs::msg::Pose dest_pose_;
  double init_yaw_;
  bool   extend_before_release_;
  std::string instruction_msg_;
  bool   is_started_, is_finished_, is_failed_;
  rclcpp::Time last_detect_;
  double tol_multiplier_;
  double x_adjust_, y_adjust_;
  double x_det_, y_det_;
  rclcpp::Time waiting_start_time_;
  std::vector<std::string> object_list_, room_list_, dest_list_;

  rclcpp_action::Client<NavigateToPose>::SharedPtr            nav_client_;
  rclcpp::Subscription<HandymanMsg>::SharedPtr                sub_msg_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_vision_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr  sub_hand_;
  rclcpp::Publisher<HandymanMsg>::SharedPtr                   pub_msg_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr     pub_base_twist_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_arm_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_gripper_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr         pub_target_object_;
  std::shared_ptr<tf2_ros::Buffer>             tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener>  tf_listener_;
  // 消息去重机制（对应原稿 last_are_you_ready_time_）
  rclcpp::Time last_are_you_ready_time_;
  const double MESSAGE_DEDUPE_INTERVAL = 0.1;

  void init();
  void reset();
  void visionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose);
  void graspVisionCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
  void messageCallback(const HandymanMsg::SharedPtr message);
  void sendMessage(const std::string &message);
  void tokenize(const std::string &str, char delim, std::vector<std::string> &out);
  void extractInfo(const std::string &msg, std::vector<std::string> &room_arr,
                   std::vector<std::string> &object_arr, std::vector<std::string> &dest_arr);
  NavigateToPose::Goal destLocation(const std::string &dest, const std::string &room);
  NavigateToPose::Goal roomLocation(const std::string &room, int variation);
  void sendNavGoal(const NavigateToPose::Goal &goal);
  void moveBase(double linear_x, double linear_y, double angular_z);
  void stopBase();
  geometry_msgs::msg::TransformStamped getTfBase();
  void resetAMCL();
  void moveArm(const std::vector<double> &positions, rclcpp::Duration &duration);
  void operateHand(bool should_grasp);
  bool loadMap();
};
// END class HandymanSample declaration

// --------------------------------------------------------------------------
// HandymanSample::init
// --------------------------------------------------------------------------
void HandymanSample::init()
{
  map_manager_ = std::make_unique<LoadMapManager>(shared_from_this());

  arm_joint_trajectory_.joint_names = {
    "arm_lift_joint","arm_flex_joint","arm_roll_joint","wrist_flex_joint","wrist_roll_joint"};
  trajectory_msgs::msg::JointTrajectoryPoint pt;
  pt.positions = {0.0,0.0,0.0,0.0,0.0};
  arm_joint_trajectory_.points.push_back(pt);

  step_ = Initialize;
  last_detect_ = rclcpp::Time(0,0,RCL_ROS_TIME);
  last_are_you_ready_time_ = rclcpp::Time(0,0,RCL_ROS_TIME);
  reset();

  // 对应原稿 node_handle_.param<std::string>(...) — 从launch文件参数读取topic名
  this->declare_parameter<std::string>("sub_msg_to_robot_topic_name",       "/handyman/message/to_robot");
  this->declare_parameter<std::string>("pub_msg_to_moderator_topic_name",   "/handyman/message/to_moderator");
  this->declare_parameter<std::string>("pub_base_twist_topic_name",         "/hsrb/command_velocity");
  this->declare_parameter<std::string>("pub_arm_trajectory_topic_name",     "/hsrb/arm_trajectory_controller/command");
  this->declare_parameter<std::string>("pub_gripper_trajectory_topic_name", "/hsrb/gripper_controller/command");

  const auto sub_msg_topic  = this->get_parameter("sub_msg_to_robot_topic_name").as_string();
  const auto pub_msg_topic  = this->get_parameter("pub_msg_to_moderator_topic_name").as_string();
  const auto pub_twist_topic= this->get_parameter("pub_base_twist_topic_name").as_string();
  const auto pub_arm_topic  = this->get_parameter("pub_arm_trajectory_topic_name").as_string();
  const auto pub_grip_topic = this->get_parameter("pub_gripper_trajectory_topic_name").as_string();

  pub_msg_           = create_publisher<HandymanMsg>(pub_msg_topic, 10);
  pub_base_twist_    = create_publisher<geometry_msgs::msg::Twist>(pub_twist_topic, 10);
  pub_arm_           = create_publisher<trajectory_msgs::msg::JointTrajectory>(pub_arm_topic, 10);
  pub_gripper_       = create_publisher<trajectory_msgs::msg::JointTrajectory>(pub_grip_topic, 10);
  pub_target_object_ = create_publisher<std_msgs::msg::String>("/detection_target", 10);

  sub_msg_ = create_subscription<HandymanMsg>(
    sub_msg_topic, 100,
    std::bind(&HandymanSample::messageCallback, this, std::placeholders::_1));
  sub_vision_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/vision",1000,
    std::bind(&HandymanSample::visionCallback,this,std::placeholders::_1));
  sub_hand_ = create_subscription<std_msgs::msg::Int32MultiArray>(
    "/hand_detection",1000,
    std::bind(&HandymanSample::graspVisionCallback,this,std::placeholders::_1));

  nav_client_ = rclcpp_action::create_client<NavigateToPose>(this,"navigate_to_pose");

  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

// --------------------------------------------------------------------------
// HandymanSample::reset
// --------------------------------------------------------------------------
void HandymanSample::reset()
{
  instruction_msg_ = "";
  ENVIRONMENT      = "None";
  is_started_  = false;
  is_finished_ = false;
  is_failed_   = false;
  tol_multiplier_ = 1.0;
  object_list_.clear(); room_list_.clear(); dest_list_.clear();
  if (!arm_joint_trajectory_.points.empty())
    arm_joint_trajectory_.points[0].positions = {0.0,0.0,0.0,0.0,0.0};
}

// --------------------------------------------------------------------------
// Callbacks
// --------------------------------------------------------------------------
void HandymanSample::visionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose)
{
  object_pose_ = *pose;
}

void HandymanSample::graspVisionCallback(const std_msgs::msg::Int32MultiArray::SharedPtr posearray)
{
  // Bug fix: 访问 data[] 前检查长度，防止越界崩溃
  if (posearray->data.size() < 4) {
    RCLCPP_WARN(get_logger(), "graspVisionCallback: data size %zu < 4, ignoring",
                posearray->data.size());
    return;
  }
  if (!found_object_) found_object_ = true;
  x_det_ = posearray->data[0];
  y_det_ = posearray->data[1];
  std::cout << "X=" << x_det_ << " Y=" << y_det_
            << " W=" << posearray->data[2] << " H=" << posearray->data[3] << std::endl;
  if (posearray->data[2] >= 450) {
    ready_to_grasp_ = true;
    tol_multiplier_ = 1.5;
  } else {
    ready_to_grasp_ = false;
    tol_multiplier_ = 1.0;
  }
  last_detect_ = now();
}

void HandymanSample::messageCallback(const HandymanMsg::SharedPtr message)
{
  RCLCPP_INFO(get_logger(),"Subscribe message:%s, %s",
              message->message.c_str(), message->detail.c_str());

  if (message->message == MSG_ENVIRONMENT) {
    ENVIRONMENT = message->detail;
  }
  RCLCPP_INFO(get_logger(),"###### Environment is %s", ENVIRONMENT.c_str());

  if (ENVIRONMENT != "None") {
    if (message->message == MSG_ARE_YOU_READY) {
      // Bug fix: 使用去重机制防止重复的 Are_you_ready? 消息触发多次重置
      auto now_time = now();
      if ((now_time - last_are_you_ready_time_).seconds() < MESSAGE_DEDUPE_INTERVAL) {
        RCLCPP_DEBUG(get_logger(), "Ignoring duplicate Are_you_ready? (dedupe interval)");
      } else {
        last_are_you_ready_time_ = now_time;
        if (step_ == Ready) {
          is_started_ = true;
        } else if (step_ != Initialize) {
          if (step_==GoToRoom2||step_==GoToRoom1||step_==MoveToInFrontOfTarget||
              step_==Grasp||step_==Release) {
            RCLCPP_WARN(get_logger(),"Critical state interrupted by Are_you_ready? step=%d",step_);
            step_ = Initialize;
          } else if (step_==WaitForInstruction) {
            RCLCPP_DEBUG(get_logger(),"Ignoring Are_you_ready in WaitForInstruction");
          } else if (step_==ComeBack||step_==TaskFinished) {
            step_ = Initialize;
          }
        }
      }
    }
    if (message->message == MSG_INSTRUCTION) {
      if (step_ == WaitForInstruction) {
        instruction_msg_ = message->detail;
      }
    }
  }

  if (message->message == MSG_TASK_SUCCEEDED) {
    if (step_ == TaskFinished) is_finished_ = true;
  }
  if (message->message == MSG_TASK_FAILED)      { is_failed_ = true; }
  if (message->message == MSG_MISSION_COMPLETE) { rclcpp::shutdown(); }
}

void HandymanSample::sendMessage(const std::string &message)
{
  RCLCPP_INFO(get_logger(),"Send message:%s",message.c_str());
  HandymanMsg msg;
  msg.message = message;
  pub_msg_->publish(msg);
}
// END init/reset/callbacks

// --------------------------------------------------------------------------
// tokenize / extractInfo
// --------------------------------------------------------------------------
void HandymanSample::tokenize(const std::string &str, char delim, std::vector<std::string> &out)
{
  std::stringstream ss(str);
  std::string s;
  while (std::getline(ss, s, delim)) out.push_back(s);
}

void HandymanSample::extractInfo(const std::string &msg,
  std::vector<std::string> &room_arr,
  std::vector<std::string> &object_arr,
  std::vector<std::string> &dest_arr)
{
  std::vector<std::string> info;
  tokenize(msg, ' ', info);
  for (auto &token : info) {
    for (auto &r : rooms)   { if (token.find(r)   != std::string::npos) room_arr.push_back(r);   }
    for (auto &o : objects) { if (token.find(o)   != std::string::npos) object_arr.push_back(o); }
    for (auto &d : dests)   { if (token.find(d)   != std::string::npos) dest_arr.push_back(d);   }
  }
}

// --------------------------------------------------------------------------
// moveBase / moveArm / operateHand / sendNavGoal / loadMap
// --------------------------------------------------------------------------
void HandymanSample::moveBase(double linear_x, double linear_y, double angular_z)
{
  geometry_msgs::msg::Twist twist;
  twist.linear.x  = linear_x;
  twist.linear.y  = linear_y;
  twist.angular.z = angular_z;
  pub_base_twist_->publish(twist);
}

void HandymanSample::moveArm(const std::vector<double> &positions, rclcpp::Duration &duration)
{
  arm_joint_trajectory_.points[0].positions = positions;
  arm_joint_trajectory_.points[0].time_from_start = duration;
  pub_arm_->publish(arm_joint_trajectory_);
}

void HandymanSample::operateHand(bool should_grasp)
{
  trajectory_msgs::msg::JointTrajectory jt;
  jt.joint_names = {"hand_motor_joint"};
  trajectory_msgs::msg::JointTrajectoryPoint pt;
  pt.positions = {should_grasp ? -0.105 : +1.239};
  pt.time_from_start = rclcpp::Duration::from_seconds(2.0);
  jt.points.push_back(pt);
  pub_gripper_->publish(jt);
}

// stopBase（对应原稿 stopBase）
void HandymanSample::stopBase()
{
  moveBase(0.0, 0.0, 0.0);
}

// getTfBase — 获取odom->base_footprint变换（对应原稿 getTfBase）
geometry_msgs::msg::TransformStamped HandymanSample::getTfBase()
{
  geometry_msgs::msg::TransformStamped tf_transform;
  try {
    tf_transform = tf_buffer_->lookupTransform("odom", "base_footprint", tf2::TimePointZero);
  } catch (const tf2::TransformException &ex) {
    RCLCPP_ERROR(get_logger(), "getTfBase: %s", ex.what());
  }
  return tf_transform;
}

// resetAMCL（对应原稿 resetAMCL，使用LoadMapManager）
void HandymanSample::resetAMCL()
{
  RCLCPP_INFO(get_logger(), "Resetting AMCL for new map...");
  if (!map_manager_) {
    RCLCPP_ERROR(get_logger(), "LoadMapManager not initialized");
    return;
  }
  geometry_msgs::msg::Pose initial_pose;
  initial_pose.position.x    = 0.0;
  initial_pose.position.y    = 0.0;
  initial_pose.position.z    = 0.0;
  initial_pose.orientation.x = 0.0;
  initial_pose.orientation.y = 0.0;
  initial_pose.orientation.z = 0.0;
  initial_pose.orientation.w = 1.0;
  if (map_manager_->resetAMCL(initial_pose)) {
    RCLCPP_INFO(get_logger(), "AMCL reset completed successfully.");
  } else {
    RCLCPP_ERROR(get_logger(), "AMCL reset failed.");
  }
}

void HandymanSample::sendNavGoal(const NavigateToPose::Goal &goal)
{
  if (!nav_client_->wait_for_action_server(1s)) {
    RCLCPP_WARN(get_logger(), "NavigateToPose server not available");
    return;
  }
  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  send_goal_options.result_callback =
    [this](const GoalHandleNav::WrappedResult &result) {
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
        RCLCPP_INFO(get_logger(), "Navigation goal succeeded");
      else
        RCLCPP_WARN(get_logger(), "Navigation goal failed/cancelled, code=%d",
                    static_cast<int>(result.code));
    };
  nav_client_->async_send_goal(goal, send_goal_options);
}

bool HandymanSample::loadMap()
{
  RCLCPP_INFO(get_logger(), "Loading map for: %s", ENVIRONMENT.c_str());
  if (!map_manager_) { RCLCPP_ERROR(get_logger(), "map_manager_ not initialized"); return false; }
  if (map_manager_->switchEnvironment(ENVIRONMENT)) {
    RCLCPP_INFO(get_logger(), "Switched to: %s", ENVIRONMENT.c_str());
    return true;
  }
  RCLCPP_ERROR(get_logger(), "Failed to switch to: %s", ENVIRONMENT.c_str());
  return false;
}
// END helpers

// --------------------------------------------------------------------------
// destLocation  (all 4 layouts, complete)
// --------------------------------------------------------------------------
NavigateToPose::Goal HandymanSample::destLocation(const std::string &dest, const std::string &room)
{
  RCLCPP_INFO(get_logger(),"destLocation dest=%s room=%s env=%s",
              dest.c_str(),room.c_str(),ENVIRONMENT.c_str());
  tf2::Quaternion q;
  NavigateToPose::Goal g;
  auto makeGoal=[&](double x,double y,double yaw)->NavigateToPose::Goal{
    NavigateToPose::Goal goal;
    goal.pose.pose.position.x=x; goal.pose.pose.position.y=y;
    q.setRPY(0,0,yaw); q.normalize();
    goal.pose.pose.orientation=tf2::toMsg(q);
    return goal;
  };

  if(ENVIRONMENT=="Layout2019HM01"||ENVIRONMENT=="2019HM01"){
    if(dest=="white_side_table"){
      if(room=="living")      g=makeGoal(2.13,-0.003,-1.57);
      else if(room=="lobby")  g=makeGoal(1.3,-6.1,-1.57);
      else                    g=makeGoal(9.3,-6.4,0);
    }else if(dest=="corner_sofa")             g=makeGoal(-0.15,-6.0,-1.57);
    else if(dest=="armchair")                 g=makeGoal(2.08,-3.2916,1.57);
    else if(dest=="trash_box_for_recycle")    g=makeGoal(-0.5,1.28,3.14);
    else if(dest=="trash_box_for_burnable")   g=makeGoal(-0.5,2.4,3.14);
    else if(dest=="trash_box_for_bottle_can") g=makeGoal(-0.8,-2.0,3.14);
    else if(dest=="square_low_table")         g=makeGoal(0.774,2.79,0.8);
    else if(dest=="dining_table")             g=makeGoal(7.8,3.07,0);
    else if(dest=="wooden_side_table")        g=makeGoal(4.8,1.9,-1.8);
    else if(dest=="wooden_bed")            g=makeGoal(6.8,-4.9,1.57);else if(dest=="wagon")                   g=makeGoal(8.12,-2.7,1.57);
    else if(dest=="cardboard_box")            g=makeGoal(9.2,-4.85,0);
  }
  else if(ENVIRONMENT=="Layout2019HM02"||ENVIRONMENT=="2019HM02"){
    if(dest=="white_side_table"){
      if(room=="living")      g=makeGoal(1.45,7.6,-1.57);
      else if(room=="lobby")  g=makeGoal(1.4,-0.5,-1.57);
    }else if(dest=="armchair")                g=makeGoal(-0.19,4.45,3.14);
    else if(dest=="trash_box_for_recycle")    g=makeGoal(6.7,3.8,1.57);
    else if(dest=="trash_box_for_bottle_can") g=makeGoal(7.56,3.7,1.57);
    else if(dest=="square_low_table")         g=makeGoal(1.28,10.5,1.57);
    else if(dest=="round_low_table")          g=makeGoal(1.9,8.3,1.57);
    else if(dest=="dining_table"){
      if(room=="kitchen")     g=makeGoal(7.85,-1.1,3.14);
      else                    g=makeGoal(2.2,2.0,3.14);
    }else if(dest=="wooden_side_table"){
      if(room=="kitchen")     g=makeGoal(9.0,3.0,0);
      else                    g=makeGoal(1.00,5.25,1.57);
    }else if(dest=="wagon")                   g=makeGoal(3.14,0.75,0);
    else if(dest=="cardboard_box")            g=makeGoal(2.9,7.6,-1.57);
    else if(dest=="wooden_shelf")             g=makeGoal(2.0,5.0,1.57);
  }
  else if(ENVIRONMENT=="Layout2020HM01"||ENVIRONMENT=="2020HM01"){
    if(dest=="white_side_table"){
      if(room=="living")      g=makeGoal(-0.4,3.5,3.14);
      else if(room=="bedroom")g=makeGoal(2.8,7.2,-1.57);
    }else if(dest=="trash_box_for_recycle")   g=makeGoal(7.2,4.55,1.57);
    else if(dest=="trash_box_for_burnable"){
      if(room=="kitchen")     g=makeGoal(8.3,2.3,0);
      else                    g=makeGoal(3.8,-1.5,-1.57);
    }else if(dest=="trash_box_for_bottle_can")g=makeGoal(6.4587,4.6449,1.57);
    else if(dest=="round_low_table")          g=makeGoal(-0.2,8.4,1.57);
    else if(dest=="dining_table")             g=makeGoal(3.2,0.1,1.57);
    else if(dest=="wooden_side_table")        g=makeGoal(2.7,-1.6,-1.57);
    else if(dest=="iron_bed")                 g=makeGoal(3.24,8.2,0);
    else if(dest=="cardboard_box")            g=makeGoal(0.95,8.5,1.57);
    else if(dest=="wooden_shelf")             g=makeGoal(2.4,4.8,1.57);
  }
  else if(ENVIRONMENT=="Layout2021HM01"||ENVIRONMENT=="2021HM01"){
    if(dest=="white_side_table"){
      if(room=="living")      g=makeGoal(2.15,0.7,1.57);
      else if(room=="bedroom")g=makeGoal(1.3,-7.5,1.57);
      else if(room=="lobby")  g=makeGoal(-3.7,-10.3,-1.57);
    }else if(dest=="corner_sofa")             g=makeGoal(-5.3,-10.2,-1.57);
    else if(dest=="armchair")                 g=makeGoal(4.0,-9.86,-1.57);
    else if(dest=="trash_box_for_burnable"){
      if(room=="living")      g=makeGoal(-1.58,-1.05,3.14);
      else                    g=makeGoal(-3.17,-4.8,1.57);
    }else if(dest=="wooden_shelf")            g=makeGoal(-4.13,-4.7,1.57);
    else if(dest=="dining_table"){
      if(room=="living")      g=makeGoal(1.5,-2.2,0);
      else                    g=makeGoal(-2.4,-8.4,3.14);
    }else if(dest=="wooden_bed")              g=makeGoal(1.8,-10.6,3.14);
    else if(dest=="wagon"){
      if(room=="living")      g=makeGoal(-1.6,-2.35,3.14);
      else                    g=makeGoal(0.27,-9.06,0);
    }else if(dest=="cardboard_box")           g=makeGoal(-6.2965,-6.9433,3.14);
  }

  if(g.pose.pose.orientation.w==0.0&&g.pose.pose.orientation.x==0.0&&
     g.pose.pose.orientation.y==0.0&&g.pose.pose.orientation.z==0.0)
    RCLCPP_ERROR(get_logger(),"Invalid quaternion for dest=%s room=%s",dest.c_str(),room.c_str());
  return g;
}
// END destLocation

// --------------------------------------------------------------------------
// roomLocation  (all 4 layouts, complete)
// --------------------------------------------------------------------------
NavigateToPose::Goal HandymanSample::roomLocation(const std::string &room, int variation)
{
  tf2::Quaternion q;
  NavigateToPose::Goal g;
  auto makeGoal=[&](double x,double y,double yaw)->NavigateToPose::Goal{
    NavigateToPose::Goal goal;
    goal.pose.pose.position.x=x; goal.pose.pose.position.y=y;
    q.setRPY(0,0,yaw); q.normalize();
    goal.pose.pose.orientation=tf2::toMsg(q);
    return goal;
  };

  std::string env_l = lower(ENVIRONMENT);

  if(env_l=="layout2019hm01"||env_l=="2019hm01"){
    if(room=="living"){  max_patrol_=2; g=(variation==0)?makeGoal(2.0,1,1.57):makeGoal(2.5,4.08,3.14); }
    else if(room=="bedroom"){ max_patrol_=2; g=(variation==0)?makeGoal(2.57,-4.31,0):makeGoal(8.64,-5.7,0); }
    else if(room=="lobby"){ max_patrol_=2; g=(variation==0)?makeGoal(1.2,-6.16,-1.57):makeGoal(1.0,-3.6,0); }
    else if(room=="kitchen"){ max_patrol_=1; g=makeGoal(8.5,2.8,3.14); }
  }
  else if(env_l=="layout2019hm02"||env_l=="2019hm02"){
    if(room=="living"){ max_patrol_=2; g=(variation==0)?makeGoal(3.5,9.6,2.4):makeGoal(1.84,10.2,0); }
    else if(room=="lobby"){ max_patrol_=2; g=(variation==0)?makeGoal(1.0,0,0):makeGoal(2.5,2.0,0); }
    else if(room=="kitchen"){ max_patrol_=2; g=(variation==0)?makeGoal(5.5,-1.13,0):makeGoal(8.42,-1.13,0); }
  }
  else if(env_l=="layout2020hm01"||env_l=="2020hm01"){
    if(room=="living"){
      max_patrol_=4;
      if(variation==0)      g=makeGoal(0.5,2.0,1.57);
      else if(variation==1) g=makeGoal(0.42,3.48,2.355);
      else if(variation==2) g=makeGoal(4.5,3.48,0.0);
      else                  g=makeGoal(4.5,-0.65,0.0);
    }else if(room=="bedroom"){ max_patrol_=2; g=(variation==0)?makeGoal(0.1,6.9,0.0):makeGoal(3.0,8.0,0.0); }
    else if(room=="kitchen"){
      max_patrol_=3;
      if(variation==0)      g=makeGoal(6.5,-1.2,0);
      else if(variation==1) g=makeGoal(7.8,1.2,0);
      else                  g=makeGoal(6.5,3.9,0);
    }
  }
  else if(env_l=="layout2021hm01"||env_l=="2021hm01"){
    if(room=="living"){ max_patrol_=2; g=(variation==0)?makeGoal(1.0,0.0,0):makeGoal(3.5,0.0,0); }
    else if(room=="bedroom"){ max_patrol_=2; g=(variation==0)?makeGoal(4.0,-8.5,0):makeGoal(1.69,-8.0,0); }
    else if(room=="lobby"){ max_patrol_=2; g=(variation==0)?makeGoal(-1.86,-8.38,0):makeGoal(-4.86,-8.7,0); }
  }
  else{
    RCLCPP_ERROR(get_logger(),"Unknown environment for roomLocation: %s",ENVIRONMENT.c_str());
    max_patrol_=1;
    g=makeGoal(0.5,2.0,1.57);
  }
  return g;
}
// END roomLocation

// --------------------------------------------------------------------------
// HandymanSample::run
// --------------------------------------------------------------------------
int HandymanSample::run()
{
  // Bug fix: shared_from_this() 只能在构造完成后使用
  // LoadMapManager 在此处初始化而非构造函数
  init();

  rclcpp::Rate loop_rate(10);
  RCLCPP_INFO(get_logger(), "Handyman sample start!");

  while (rclcpp::ok()) {
    if (is_failed_) {
      RCLCPP_INFO(get_logger(), "Task failed!");
      step_      = Initialize;
      is_failed_ = false;
    }

    switch (step_) {
      case Initialize: {
        reset();
        ENVIRONMENT = "None";
        step_++;
        break;
      }
      case Ready: {
        if (is_started_) {
          if (ENVIRONMENT == "None") {
            RCLCPP_WARN(get_logger(), "Environment not set, waiting...");
            break;
          }
          RCLCPP_INFO(get_logger(), "Before loadmap: %s", ENVIRONMENT.c_str());
          if (!loadMap()) {
            RCLCPP_ERROR(get_logger(), "Failed to load map");
            step_ = Initialize;
            break;
          }
          while (!nav_client_->wait_for_action_server(1s)) {
            RCLCPP_INFO(get_logger(), "Waiting for NavigateToPose action server...");
            if (!rclcpp::ok()) return EXIT_FAILURE;
          }
          RCLCPP_INFO(get_logger(), "System ready, sending I_am_ready");
          sendMessage(MSG_I_AM_READY);
          step_++;
        }
        break;
      }
      case WaitForInstruction: {
        if (instruction_msg_ != "") {
          RCLCPP_INFO(get_logger(), "%s", instruction_msg_.c_str());
          extractInfo(instruction_msg_, room_list_, object_list_, dest_list_);
          std_msgs::msg::String target_obj;
          target_obj.data = object_list_[0];
          pub_target_object_->publish(target_obj);
          std::vector<double> pos{0.1, 0.0, 0.0, -1.57, 0.0};
          rclcpp::Duration dur = rclcpp::Duration::from_seconds(1.0);
          moveArm(pos, dur);
          operateHand(false);
          room_reached_ = false;
          found_object_ = false;
          patrol_step_  = 0;
          step_         = GoToRoom1;
        }
        break;
      }
      case GoToRoom1: {
        if (!found_object_ || !room_reached_) {
          auto goal = roomLocation(room_list_[0], patrol_step_);
          goal.pose.header.frame_id = "map";
          goal.pose.header.stamp    = now();
          RCLCPP_INFO(get_logger(), "GoToRoom1: room=%s patrol=%d x=%.3f y=%.3f",
            room_list_[0].c_str(), patrol_step_,
            goal.pose.pose.position.x, goal.pose.pose.position.y);
          // sendNavGoal是异步的：发送后等待action server返回结果再继续
          if (!nav_client_->wait_for_action_server(1s)) {
            RCLCPP_WARN(get_logger(), "NavigateToPose server not available in GoToRoom1");
            break;
          }
          // Bug fix: result_callback 由 action 线程池异步触发，nav_done 必须是
          // std::atomic<bool> 避免栈变量悬空引用和数据竞争
          auto nav_done = std::make_shared<std::atomic<bool>>(false);
          auto send_opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
          send_opts.result_callback =
            [this, nav_done](const GoalHandleNav::WrappedResult &result) {
              if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
                RCLCPP_INFO(get_logger(), "GoToRoom1: navigation succeeded");
              else
                RCLCPP_WARN(get_logger(), "GoToRoom1: navigation ended code=%d",
                            static_cast<int>(result.code));
              nav_done->store(true);
            };
          nav_client_->async_send_goal(goal, send_opts);
          // 等待到达（在等待期间持续 spin）
          rclcpp::Rate wait_rate(10);
          while (rclcpp::ok() && !nav_done->load()) {
            rclcpp::spin_some(shared_from_this());
            wait_rate.sleep();
          }
          patrol_step_++;
          if (patrol_step_ >= max_patrol_) patrol_step_ = 0;
          if (!room_reached_) {
            RCLCPP_INFO(get_logger(), "Room Reached");
            sendMessage(MSG_ROOM_REACHED);
            room_reached_ = true;
          }
        } else {
          ready_to_grasp_ = false;
          aligned_x_ = false; aligned_y_ = false;
          arm_height_ = 0.0;  x_adjust_  = 0.0;
          y_adjust_   = 0.0;  body_height_ = 0.0;
          step_ = MoveToInFrontOfTarget;
        }
        break;
      }
      case GoToRoom2: {
        if (!room_reached_) {
          NavigateToPose::Goal goal;
          if (room_list_.size() > 1)
            goal = destLocation(dest_list_[dest_list_.size()-1], room_list_[1]);
          else
            goal = destLocation(dest_list_[dest_list_.size()-1], room_list_[0]);
          std::cout << "dest = " << dest_list_[dest_list_.size()-1] << std::endl;
          RCLCPP_INFO(get_logger(), "GoToRoom2: dest=%s room=%s",
                      dest_list_[dest_list_.size()-1].c_str(), room_list_[0].c_str());
          goal.pose.header.frame_id = "map";
          goal.pose.header.stamp    = now();
          std::vector<double> pos{0.2, -0.3, 0.0, -1.57, 0.0};
          rclcpp::Duration dur = rclcpp::Duration::from_seconds(1.0);
          moveArm(pos, dur);
          // 等待导航完成再继续（对应原稿阻塞式 actionlib）
          if (!nav_client_->wait_for_action_server(1s)) {
            RCLCPP_WARN(get_logger(), "NavigateToPose server not available in GoToRoom2");
            break;
          }
          // Bug fix: result_callback 由 action 线程池异步触发，nav_done 必须是
          // std::atomic<bool> 避免栈变量悬空引用和数据竞争
          auto nav_done = std::make_shared<std::atomic<bool>>(false);
          auto send_opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
          send_opts.result_callback =
            [this, nav_done](const GoalHandleNav::WrappedResult &result) {
              if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
                RCLCPP_INFO(get_logger(), "GoToRoom2: navigation succeeded");
              else
                RCLCPP_WARN(get_logger(), "GoToRoom2: navigation ended code=%d",
                            static_cast<int>(result.code));
              nav_done->store(true);
            };
          nav_client_->async_send_goal(goal, send_opts);
          rclcpp::Rate wait_rate(10);
          while (rclcpp::ok() && !nav_done->load()) {
            rclcpp::spin_some(shared_from_this());
            wait_rate.sleep();
          }
          room_reached_ = true;
        } else {
          std::vector<double> pos{0.2, -0.8, 0.0, -1.0, 0.0};
          rclcpp::Duration dur = rclcpp::Duration::from_seconds(1.0);
          moveArm(pos, dur);
          waiting_start_time_    = now();
          extend_before_release_ = false;
          step_ = Release;
        }
        break;
      }
      case MoveToInFrontOfTarget: {
        if ((now() - last_detect_).seconds() < 1.0) {
          if (x_det_ < 240 - 25*tol_multiplier_) {
            x_adjust_ = 0.03; aligned_x_ = false;
          } else if (x_det_ > 240 + 25*tol_multiplier_) {
            x_adjust_ = -0.03; aligned_x_ = false;
          } else {
            x_adjust_ = 0.0; aligned_x_ = true;
          }
          if (y_det_ < 360 - 25*tol_multiplier_) {
            arm_height_ += 0.004;
            if (arm_height_ > 0.0) arm_height_ = 0.0;
            y_adjust_ = 0.01; aligned_y_ = false;
          } else if (y_det_ > 360 + 25*tol_multiplier_) {
            arm_height_ -= 0.004;
            y_adjust_ = -0.01; aligned_y_ = false;
          } else {
            y_adjust_ = 0.0; aligned_y_ = true;
          }
          std::vector<double> pos{0.2, arm_height_, 0.0, -1.57 - arm_height_*0.4, 0.0};
          rclcpp::Duration dur = rclcpp::Duration::from_seconds(1.0);
          moveArm(pos, dur);
          moveBase(y_adjust_, x_adjust_*0.5, x_adjust_);
        }
        if (aligned_x_ && aligned_y_) {
          std::cout << "ready = " << std::to_string(ready_to_grasp_) << std::endl;
          if (!ready_to_grasp_) {
            moveBase(0.02, 0.0, 0.0);
          } else {
            std::cout << "Gripping" << std::endl;
            moveBase(0.0, 0.0, 0.0);
            waiting_start_time_ = now();
            step_ = Grasp;
          }
        }
        break;
      }
      case Grasp: {
        // 注释保留的逆运动学计算代码（原稿保留，未启用）
        // geometry_msgs::msg::TransformStamped tf_transform = getTfBase();
        // float hyp = pow(pow(object_pose_.pose.position.x -
        //   tf_transform.transform.translation.x - 0.145, 2.0) + pow(0.15,2.0), 0.5);
        // float lower_angle_1 = atan2f(0.15,
        //   object_pose_.pose.position.x - tf_transform.transform.translation.x - 0.145);
        // float upper_angle = acos((pow(0.345,2.0) + pow(0.140,2.0) - pow(hyp,2.0))
        //                         /(2.0*0.345*0.140));
        // float lower_angle_2 = acos((pow(0.345,2) - pow(0.140,2) + pow(hyp,2.0))
        //                           /(2.0*0.345*hyp));
        // std::vector<double> ik_pos {
        //   object_pose_.pose.position.z - 0.15 - 0.34,
        //   -1.57-(lower_angle_1 + lower_angle_2),
        //   0.0, upper_angle - 1.57, 0.0 };
        // rclcpp::Duration ik_dur = rclcpp::Duration::from_seconds(3.0);
        // moveArm(ik_pos, ik_dur);
        operateHand(true);
        if ((now() - waiting_start_time_).seconds() > 8.0) {
          sendMessage(MSG_OBJECT_GRASPED);
          room_reached_ = false;
          found_object_ = false;
          step_ = GoToRoom2;
        }
        break;
      }
      case Release: {
        if (!extend_before_release_) {
          if ((now() - waiting_start_time_).seconds() > 4.0) {
            operateHand(false);
            waiting_start_time_    = now();
            extend_before_release_ = true;
          }
        } else {
          if ((now() - waiting_start_time_).seconds() > 8.0) {
            sendMessage(MSG_TASK_FINISHED);
            step_ = TaskFinished;
          }
        }
        break;
      }
      case ComeBack: {
        // reserved
        break;
      }
      case MoveToInFrontOfDest: {
        // reserved — 预留，对应原稿 MoveToInFrontOfDest case
        break;
      }
      case TaskFinished: {
        if (is_finished_) {
          RCLCPP_INFO(get_logger(), "Task finished!");
          step_ = Initialize;
        }
        break;
      }
    } // end switch

    rclcpp::spin_some(shared_from_this());
    loop_rate.sleep();
  }
  return EXIT_SUCCESS;
}

// --------------------------------------------------------------------------
// main
// --------------------------------------------------------------------------
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HandymanSample>();
  int ret = node->run();
  rclcpp::shutdown();
  return ret;
}

