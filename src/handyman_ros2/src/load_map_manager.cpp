#include <handyman_ros2/load_map_manager.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace handyman_ros2 {

LoadMapManager::LoadMapManager(rclcpp::Node::SharedPtr node)
    : node_(node), current_environment_("None"), move_base_active_(false), navigation_cancelled_(false),
      last_feedback_time_(0, 0, node->get_clock()->get_clock_type())
{
    global_costmap_client_ = node_->create_client<std_srvs::srv::Empty>("/global_costmap/clear_entirely_global_costmap");
    local_costmap_client_ = node_->create_client<std_srvs::srv::Empty>("/local_costmap/clear_entirely_local_costmap");
    nav_action_client_ = rclcpp_action::create_client<NavigateToPose>(node_, "navigate_to_pose");

    map_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10,
        [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(map_mutex_);
            latest_map_msg_ = msg;
            map_cv_.notify_all();
        });

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    registerDefaultEnvironments();
}

bool LoadMapManager::isMoveBaseActive() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (last_feedback_time_.nanoseconds() > 0) {
        auto time_since_feedback = node_->now() - last_feedback_time_;
        if (time_since_feedback > rclcpp::Duration::from_seconds(2.0)) {
            move_base_active_ = false;
        }
    }
    return move_base_active_;
}

bool LoadMapManager::cancelCurrentNavigationGoal() {
    RCLCPP_INFO(node_->get_logger(), "Cancelling current navigation goal...");
    if (!nav_action_client_->action_server_is_ready()) {
        RCLCPP_WARN(node_->get_logger(), "Nav2 action server not ready, cannot cancel goal");
        return false;
    }
    nav_action_client_->async_cancel_all_goals();
    navigation_cancelled_ = true;
    RCLCPP_INFO(node_->get_logger(), "Navigation goal cancelled successfully");
    return true;
}

bool LoadMapManager::waitForNavigationStop(double timeout_seconds) {
    RCLCPP_INFO(node_->get_logger(), "Waiting for navigation to stop...");
    auto start_time = node_->now();
    rclcpp::Rate check_rate(10.0);
    while (rclcpp::ok()) {
        if (!isMoveBaseActive()) {
            RCLCPP_INFO(node_->get_logger(), "Navigation has stopped");
            return true;
        }
        if ((node_->now() - start_time).seconds() > timeout_seconds) {
            RCLCPP_WARN(node_->get_logger(), "Timeout waiting for navigation to stop");
            return false;
        }
        check_rate.sleep();
        rclcpp::spin_some(node_);
    }
    return false;
}

std::string LoadMapManager::mapEnvironmentFormat(const std::string& input_env) {
    RCLCPP_INFO(node_->get_logger(), "Mapping environment format: %s", input_env.c_str());
    if (input_env.find("Layout") == 0) {
        std::string mapped_env = input_env.substr(6);
        RCLCPP_INFO(node_->get_logger(), "Environment format mapped: %s -> %s", input_env.c_str(), mapped_env.c_str());
        return mapped_env;
    }
    RCLCPP_INFO(node_->get_logger(), "Environment format already correct: %s", input_env.c_str());
    return input_env;
}

void LoadMapManager::registerEnvironment(const std::string& env,
                                       const std::string& map_file,
                                       const geometry_msgs::msg::Pose& initial_pose) {
    environment_to_map_[env] = map_file;
    environment_to_initial_pose_[env] = initial_pose;
    RCLCPP_INFO(node_->get_logger(), "Registered environment: %s -> %s", env.c_str(), map_file.c_str());
}

void LoadMapManager::registerDefaultEnvironments() {
    geometry_msgs::msg::Pose default_pose;
    default_pose.position.x = 0.0;
    default_pose.position.y = 0.0;
    default_pose.position.z = 0.0;
    default_pose.orientation.x = 0.0;
    default_pose.orientation.y = 0.0;
    default_pose.orientation.z = 0.0;
    default_pose.orientation.w = 1.0;

    std::string maps_dir = ament_index_cpp::get_package_share_directory("handyman_ros2") + "/maps/";
    registerEnvironment("2019HM01", maps_dir + "2019HM01.yaml", default_pose);
    registerEnvironment("2019HM02", maps_dir + "2019HM02.yaml", default_pose);
    registerEnvironment("2020HM01", maps_dir + "2020HM01.yaml", default_pose);
    registerEnvironment("2021HM01", maps_dir + "2021HM01.yaml", default_pose);
}

bool LoadMapManager::stopMapServer() {
    RCLCPP_INFO(node_->get_logger(), "Stopping all map server instances...");
    (void)system("pkill -f '[m]ap_server' 2>/dev/null || true");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    int result = system("pgrep -f '[m]ap_server' > /dev/null 2>&1");
    if (result == 0) {
        RCLCPP_WARN(node_->get_logger(), "Some map_server processes are still running, forcing cleanup...");
        (void)system("pkill -9 -f '[m]ap_server' 2>/dev/null || true");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    RCLCPP_INFO(node_->get_logger(), "All map server instances stopped");
    return true;
}

bool LoadMapManager::startMapServer(const std::string& map_file) {
    RCLCPP_INFO(node_->get_logger(), "Starting map server with map: %s", map_file.c_str());
    stopMapServer();
    if (!rclcpp::ok()) return false;

    std::string cmd = "ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=" + map_file + " &";
    int result = system(cmd.c_str());
    if (result != 0) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to start map server");
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    if (!rclcpp::ok()) return false;

    (void)system("ros2 lifecycle set /map_server configure 2>/dev/null || true");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    if (!rclcpp::ok()) return false;

    (void)system("ros2 lifecycle set /map_server activate 2>/dev/null || true");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    if (!rclcpp::ok()) return false;

    bool map_ready = waitForMapService();
    if (!map_ready) {
        RCLCPP_ERROR(node_->get_logger(), "Map service not ready after starting map server");
        return false;
    }
    RCLCPP_INFO(node_->get_logger(), "Map server started successfully");
    return true;
}

bool LoadMapManager::stopAMCL() {
    RCLCPP_INFO(node_->get_logger(), "Stopping AMCL...");
    (void)system("pkill -f \"amcl\" 2>/dev/null || true");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    return true;
}

bool LoadMapManager::stopNav2() {
    RCLCPP_INFO(node_->get_logger(), "Stopping Nav2 nodes...");
    (void)system("pkill -f \"controller_server\" 2>/dev/null || true");
    (void)system("pkill -f \"planner_server\" 2>/dev/null || true");
    (void)system("pkill -f \"bt_navigator\" 2>/dev/null || true");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    return true;
}

bool LoadMapManager::gracefulStopNavigation() {
    RCLCPP_INFO(node_->get_logger(), "Gracefully stopping navigation system...");
    if (isMoveBaseActive()) {
        RCLCPP_INFO(node_->get_logger(), "Nav2 is active, cancelling current goal...");
        if (!cancelCurrentNavigationGoal()) {
            RCLCPP_WARN(node_->get_logger(), "Failed to cancel navigation goal, continuing with shutdown");
        }
        if (!waitForNavigationStop(15.0)) {
            RCLCPP_WARN(node_->get_logger(), "Navigation did not stop gracefully, forcing shutdown");
        }
    } else {
        RCLCPP_INFO(node_->get_logger(), "Nav2 is not active, proceeding with shutdown");
    }

    RCLCPP_INFO(node_->get_logger(), "Stopping navigation nodes in sequence...");
    if (!stopNav2()) RCLCPP_WARN(node_->get_logger(), "Failed to stop Nav2, continuing anyway");
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    if (!stopAMCL()) RCLCPP_WARN(node_->get_logger(), "Failed to stop AMCL, continuing anyway");
    if (!stopMapServer()) RCLCPP_WARN(node_->get_logger(), "Failed to stop map server, continuing anyway");

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        move_base_active_ = false;
        navigation_cancelled_ = false;
    }
    RCLCPP_INFO(node_->get_logger(), "Navigation system stopped gracefully");
    return true;
}

bool LoadMapManager::startAMCL(const geometry_msgs::msg::Pose& initial_pose) {
    RCLCPP_INFO(node_->get_logger(), "Starting AMCL...");
    int result = system("ros2 launch handyman_ros2 amcl.launch.py &");
    if (result != 0) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to start AMCL");
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    if (!publishInitialPose(initial_pose)) {
        RCLCPP_WARN(node_->get_logger(), "Failed to publish initial pose, continuing anyway");
    }
    RCLCPP_INFO(node_->get_logger(), "AMCL started successfully");
    return true;
}

bool LoadMapManager::publishInitialPose(const geometry_msgs::msg::Pose& initial_pose) {
    RCLCPP_INFO(node_->get_logger(), "Publishing initial pose to AMCL...");
    try {
        auto initial_pose_pub = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 1);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        geometry_msgs::msg::PoseWithCovarianceStamped initial_pose_msg;
        initial_pose_msg.header.stamp = node_->now();
        initial_pose_msg.header.frame_id = "map";
        initial_pose_msg.pose.pose = initial_pose;
        initial_pose_msg.pose.covariance = {
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1
        };

        for (int i = 0; i < 5; i++) {
            initial_pose_pub->publish(initial_pose_msg);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
        RCLCPP_INFO(node_->get_logger(), "Waiting for AMCL to publish stable TF...");
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
        RCLCPP_INFO(node_->get_logger(), "Initial pose published successfully");
        return true;
    } catch (const std::exception& ex) {
        RCLCPP_ERROR(node_->get_logger(), "Exception while publishing initial pose: %s", ex.what());
        return false;
    }
}

bool LoadMapManager::startNav2() {
    RCLCPP_INFO(node_->get_logger(), "Starting Nav2...");
    int result = system("ros2 launch handyman_ros2 move_base.launch.py &");
    if (result != 0) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to start Nav2");
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    RCLCPP_INFO(node_->get_logger(), "Nav2 started successfully");
    return true;
}

bool LoadMapManager::forceClearCostmaps() {
    RCLCPP_INFO(node_->get_logger(), "Force clearing costmaps...");
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    bool global_cleared = false;
    bool local_cleared = false;

    if (global_costmap_client_->service_is_ready()) {
        auto future = global_costmap_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(5)) == rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(node_->get_logger(), "Global costmap cleared successfully");
            global_cleared = true;
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Failed to clear global costmap");
        }
    } else {
        RCLCPP_WARN(node_->get_logger(), "Global costmap service not available");
    }

    if (local_costmap_client_->service_is_ready()) {
        auto future = local_costmap_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(5)) == rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(node_->get_logger(), "Local costmap cleared successfully");
            local_cleared = true;
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Failed to clear local costmap");
        }
    } else {
        RCLCPP_WARN(node_->get_logger(), "Local costmap service not available");
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    RCLCPP_INFO(node_->get_logger(), "Costmap clearing completed - Global: %s, Local: %s",
             global_cleared ? "SUCCESS" : "FAILED",
             local_cleared ? "SUCCESS" : "FAILED");
    return global_cleared && local_cleared;
}

bool LoadMapManager::waitForMapService() {
    RCLCPP_INFO(node_->get_logger(), "Waiting for map topic with valid data...");
    const double max_wait_time = 30.0;
    auto start_time = node_->now();
    while (rclcpp::ok() && (node_->now() - start_time).seconds() < max_wait_time) {
        {
            std::unique_lock<std::mutex> lock(map_mutex_);
            map_cv_.wait_for(lock, std::chrono::seconds(2));
            if (latest_map_msg_ && latest_map_msg_->info.width > 0 &&
                latest_map_msg_->info.height > 0 && !latest_map_msg_->data.empty()) {
                RCLCPP_INFO(node_->get_logger(), "Valid map received! Size: %dx%d",
                           latest_map_msg_->info.width, latest_map_msg_->info.height);
                return true;
            }
        }
        rclcpp::spin_some(node_);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    RCLCPP_ERROR(node_->get_logger(), "Failed to get valid map within %.2f seconds", max_wait_time);
    return false;
}

bool LoadMapManager::waitForSystemReady() {
    RCLCPP_INFO(node_->get_logger(), "Waiting for navigation system to be ready...");
    auto start_time = node_->now();
    double timeout = 30.0;

    while (rclcpp::ok()) {
        bool all_ready = true;
        if (!nav_action_client_->action_server_is_ready()) {
            RCLCPP_WARN(node_->get_logger(), "Nav2 action server not ready");
            all_ready = false;
        }
        if (!checkTFReady()) {
            RCLCPP_WARN(node_->get_logger(), "TF tree not ready");
            all_ready = false;
        }
        if (all_ready) {
            RCLCPP_INFO(node_->get_logger(), "All navigation services are ready");
            return true;
        }
        if ((node_->now() - start_time).seconds() > timeout) {
            RCLCPP_ERROR(node_->get_logger(), "Timeout waiting for system to be ready");
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        rclcpp::spin_some(node_);
    }
    return false;
}

bool LoadMapManager::checkTFReady() {
    if (!tf_buffer_) {
        RCLCPP_ERROR(node_->get_logger(), "TF buffer not initialized");
        return false;
    }
    const int max_retries = 3;
    for (int retry = 0; retry < max_retries; retry++) {
        try {
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            if (tf_buffer_->canTransform("map", "base_footprint",
                node_->now(), rclcpp::Duration::from_seconds(2))) {
                RCLCPP_DEBUG(node_->get_logger(), "TF transform map->base_footprint is available");
                return true;
            }
            if (tf_buffer_->canTransform("odom", "base_footprint",
                node_->now(), rclcpp::Duration::from_seconds(2))) {
                RCLCPP_DEBUG(node_->get_logger(), "TF transform odom->base_footprint is available (fallback)");
                return true;
            }
            if (retry < max_retries - 1) {
                RCLCPP_WARN(node_->get_logger(), "TF not ready, retry %d/%d...", retry + 1, max_retries);
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            }
        } catch (const std::exception& ex) {
            RCLCPP_DEBUG(node_->get_logger(), "TF check exception: %s", ex.what());
        }
    }
    return false;
}

bool LoadMapManager::switchEnvironment(const std::string& environment) {
    RCLCPP_INFO(node_->get_logger(), "Switching to environment: %s", environment.c_str());
    std::string mapped_env = mapEnvironmentFormat(environment);
    if (environment_to_map_.find(mapped_env) == environment_to_map_.end()) {
        RCLCPP_ERROR(node_->get_logger(), "Environment %s (mapped to %s) not registered", environment.c_str(), mapped_env.c_str());
        return false;
    }
    if (!gracefulStopNavigation()) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to gracefully stop navigation");
        return false;
    }
    if (tf_buffer_) {
        tf_buffer_->clear();
        RCLCPP_INFO(node_->get_logger(), "Cleared TF buffer after navigation stop");
    }
    std::string map_file = environment_to_map_[mapped_env];
    if (!startMapServer(map_file)) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to start map server with map: %s", map_file.c_str());
        return false;
    }
    geometry_msgs::msg::Pose initial_pose = environment_to_initial_pose_[mapped_env];
    if (!startAMCL(initial_pose)) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to start AMCL");
        return false;
    }
    if (!startNav2()) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to start Nav2");
        return false;
    }
    if (!forceClearCostmaps()) {
        RCLCPP_WARN(node_->get_logger(), "Failed to clear costmaps, continuing anyway...");
    }
    if (!waitForSystemReady()) {
        RCLCPP_ERROR(node_->get_logger(), "System not ready after switching");
        return false;
    }
    current_environment_ = mapped_env;
    RCLCPP_INFO(node_->get_logger(), "Successfully switched to environment: %s (original: %s)", mapped_env.c_str(), environment.c_str());
    return true;
}

bool LoadMapManager::loadMap(const std::string& map_file) {
    RCLCPP_INFO(node_->get_logger(), "Loading map: %s", map_file.c_str());
    return startMapServer(map_file);
}

bool LoadMapManager::resetAMCL(const geometry_msgs::msg::Pose& initial_pose) {
    RCLCPP_INFO(node_->get_logger(), "Resetting AMCL with initial pose");
    return startAMCL(initial_pose);
}

}  // namespace handyman_ros2
