#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <action_msgs/msg/goal_status.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <handyman_msgs/msg/handyman_msg.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

// Enable M_PI constant from cmath
#define _USE_MATH_DEFINES
#include <cmath>

#include <iostream>
#include <cstring>
#include <cmath>
#include <vector>
#include <thread>
#include <algorithm>
#include <cctype>
#include <map>
#include <mutex>
#include <sstream>
#include <condition_variable>
#include <atomic>

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;

std::string lower(const std::string &str) {
  std::string lower_str = str;
  std::transform(lower_str.begin(), lower_str.end(), lower_str.begin(), ::tolower);
  return lower_str;
}


class LoadMapManager {
private:
    std::shared_ptr<rclcpp::Node> node_;
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

    // TF management - persistent buffer and listener for proper TF availability checking
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

public:
    LoadMapManager(std::shared_ptr<rclcpp::Node> node) :
        node_(node), current_environment_("None"), move_base_active_(false), navigation_cancelled_(false),
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

        // Initialize persistent TF buffer and listener
        // tf2_ros::Buffer requires Clock parameter, use unique_ptr for dynamic allocation
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        registerDefaultEnvironments();
    }

    bool isMoveBaseActive() {
        std::lock_guard<std::mutex> lock(state_mutex_);

        if (last_feedback_time_.nanoseconds() > 0) {
            auto time_since_feedback = node_->now() - last_feedback_time_;
            if (time_since_feedback > rclcpp::Duration::from_seconds(2.0)) {
                move_base_active_ = false;
            }
        }

        return move_base_active_;
    }

    bool cancelCurrentNavigationGoal() {
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

    bool waitForNavigationStop(double timeout_seconds = 10.0) {
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

    bool switchEnvironment(const std::string& environment) {
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

        // Clear TF buffer to prevent extrapolation errors from stale transforms
        // Old map TF timestamps can conflict with new map after environment switch
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

    bool loadMap(const std::string& map_file) {
        RCLCPP_INFO(node_->get_logger(), "Loading map: %s", map_file.c_str());
        return startMapServer(map_file);
    }

    bool resetAMCL(const geometry_msgs::msg::Pose& initial_pose) {
        RCLCPP_INFO(node_->get_logger(), "Resetting AMCL with initial pose");
        return startAMCL(initial_pose);
    }

    void registerEnvironment(const std::string& env,
                           const std::string& map_file,
                           const geometry_msgs::msg::Pose& initial_pose) {
        environment_to_map_[env] = map_file;
        environment_to_initial_pose_[env] = initial_pose;
        RCLCPP_INFO(node_->get_logger(), "Registered environment: %s -> %s", env.c_str(), map_file.c_str());
    }

    std::string mapEnvironmentFormat(const std::string& input_env) {
        RCLCPP_INFO(node_->get_logger(), "Mapping environment format: %s", input_env.c_str());

        if (input_env.find("Layout") == 0) {
            std::string mapped_env = input_env.substr(6);
            RCLCPP_INFO(node_->get_logger(), "Environment format mapped: %s -> %s", input_env.c_str(), mapped_env.c_str());
            return mapped_env;
        }

        RCLCPP_INFO(node_->get_logger(), "Environment format already correct: %s", input_env.c_str());
        return input_env;
    }

    std::string getCurrentEnvironment() const {
        return current_environment_;
    }

private:
    void registerDefaultEnvironments() {
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

    bool stopMapServer() {
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

    bool startMapServer(const std::string& map_file) {
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

        RCLCPP_INFO(node_->get_logger(), "Configuring map_server...");
        (void)system("ros2 lifecycle set /map_server configure 2>/dev/null || true");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        if (!rclcpp::ok()) return false;

        RCLCPP_INFO(node_->get_logger(), "Activating map_server...");
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

    bool stopAMCL() {
        RCLCPP_INFO(node_->get_logger(), "Stopping AMCL...");
        (void)system("pkill -f \"amcl\" 2>/dev/null || true");
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        return true;
    }

    bool gracefulStopNavigation() {
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

        if (!stopNav2()) {
            RCLCPP_WARN(node_->get_logger(), "Failed to stop Nav2, continuing anyway");
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(2000));

        if (!stopAMCL()) {
            RCLCPP_WARN(node_->get_logger(), "Failed to stop AMCL, continuing anyway");
        }

        if (!stopMapServer()) {
            RCLCPP_WARN(node_->get_logger(), "Failed to stop map server, continuing anyway");
        }

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            move_base_active_ = false;
            navigation_cancelled_ = false;
        }

        RCLCPP_INFO(node_->get_logger(), "Navigation system stopped gracefully");
        return true;
    }

    bool stopNav2() {
        RCLCPP_INFO(node_->get_logger(), "Stopping Nav2 nodes...");
        (void)system("pkill -f \"controller_server\" 2>/dev/null || true");
        (void)system("pkill -f \"planner_server\" 2>/dev/null || true");
        (void)system("pkill -f \"bt_navigator\" 2>/dev/null || true");
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        return true;
    }

    bool startAMCL(const geometry_msgs::msg::Pose& initial_pose) {
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

    bool publishInitialPose(const geometry_msgs::msg::Pose& initial_pose) {
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

            // Wait for AMCL to publish stable TF after receiving initial pose
            RCLCPP_INFO(node_->get_logger(), "Waiting for AMCL to publish stable TF...");
            std::this_thread::sleep_for(std::chrono::milliseconds(3000));

            RCLCPP_INFO(node_->get_logger(), "Initial pose published successfully");
            return true;

        } catch (const std::exception& ex) {
            RCLCPP_ERROR(node_->get_logger(), "Exception while publishing initial pose: %s", ex.what());
            return false;
        }
    }

    bool startNav2() {
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

    bool forceClearCostmaps() {
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

    bool waitForMapService() {
        RCLCPP_INFO(node_->get_logger(), "Waiting for map topic with valid data...");

        const double max_wait_time = 30.0;
        auto start_time = node_->now();

        while (rclcpp::ok() && (node_->now() - start_time).seconds() < max_wait_time) {
            {
                std::unique_lock<std::mutex> lock(map_mutex_);
                map_cv_.wait_for(lock, std::chrono::seconds(2));

                if (latest_map_msg_ &&
                    latest_map_msg_->info.width > 0 &&
                    latest_map_msg_->info.height > 0 &&
                    !latest_map_msg_->data.empty()) {
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

    bool waitForMapAfterEnvironmentSwitch(const std::string& environment_name) {
        RCLCPP_INFO(node_->get_logger(), "Waiting for map after switching to environment: %s", environment_name.c_str());

        latest_map_msg_ = nullptr;

        const double max_wait_time = 30.0;
        auto switch_start_time = node_->now();

        while (rclcpp::ok() && (node_->now() - switch_start_time).seconds() < max_wait_time) {
            {
                std::unique_lock<std::mutex> lock(map_mutex_);
                map_cv_.wait_for(lock, std::chrono::seconds(3));

                if (latest_map_msg_ &&
                    latest_map_msg_->info.width > 0 &&
                    latest_map_msg_->info.height > 0 &&
                    !latest_map_msg_->data.empty()) {
                    RCLCPP_INFO(node_->get_logger(), "Map received after environment switch! Size: %dx%d",
                            latest_map_msg_->info.width, latest_map_msg_->info.height);
                    return true;
                }
            }

            rclcpp::spin_some(node_);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }

        RCLCPP_ERROR(node_->get_logger(), "Failed to get map for environment %s within %.2f seconds",
                 environment_name.c_str(), max_wait_time);
        return false;
    }

    bool waitForSystemReady() {
        RCLCPP_INFO(node_->get_logger(), "Waiting for navigation system to be ready...");

        auto start_time = node_->now();
        double timeout = 30.0;  // Increased from 20s to allow more time for TF and Nav2 to stabilize

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

    bool checkTFReady() {
        if (!tf_buffer_) {
            RCLCPP_ERROR(node_->get_logger(), "TF buffer not initialized");
            return false;
        }
        
        const int max_retries = 3;
        for (int retry = 0; retry < max_retries; retry++) {
            try {
                // Wait for TF buffer to fill (TF listener needs time to receive messages)
                std::this_thread::sleep_for(std::chrono::milliseconds(2000));

                // Use persistent member variable tf_buffer_ instead of creating new one
                if (tf_buffer_->canTransform("map", "base_footprint", tf2::TimePointZero, tf2::durationFromSec(2.0))) {
                    RCLCPP_DEBUG(node_->get_logger(), "TF transform map->base_footprint is available");
                    return true;
                }

                if (tf_buffer_->canTransform("odom", "base_footprint", tf2::TimePointZero, tf2::durationFromSec(2.0))) {
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
};

class HandymanSample
{
private:
  enum Step
  {
    Initialize,
    Ready,
    WaitForInstruction,
    GoToRoom1,
    GoToRoom2,
    MoveToInFrontOfTarget,
    MoveToInFrontOfDest,
    Grasp,
    Release,
    ComeBack,
    TaskFinished,
  };

  const std::string MSG_ARE_YOU_READY    = "Are_you_ready?";
  const std::string MSG_ENVIRONMENT    = "Environment";
  const std::string MSG_INSTRUCTION      = "Instruction";
  const std::string MSG_TASK_SUCCEEDED   = "Task_succeeded";
  const std::string MSG_TASK_FAILED      = "Task_failed";
  const std::string MSG_MISSION_COMPLETE = "Mission_complete";

  const std::string MSG_I_AM_READY     = "I_am_ready";
  const std::string MSG_ROOM_REACHED   = "Room_reached";
  const std::string MSG_OBJECT_GRASPED = "Object_grasped";
  const std::string MSG_TASK_FINISHED  = "Task_finished";

  std::vector<std::string> rooms = {"living", "bedroom", "lobby", "kitchen"};
  std::vector<std::string> objects = {"apple", "toy_penguin", "rabbit_doll", "bear_doll", "dog_doll", "canned_juice", "sugar",
                          "soysauce", "sauce", "ketchup", "tumbler","white_cup","pink_cup","empty_ketchup","filled_ketchup","ground_pepper","salt","canned_juice","empty_plastic_bottle","filled_plastic_bottle","cubic_clock","toy_car","toy_duck","nursing_bottle","cigarette","hourglass","camera","rubik's_cube","spray_bottle","apple","matryoshka","game_controller","piggy_bank"};
  std::vector<std::string> dests = {"white_side_table","corner_sofa","round_low_table","square_low_table","wooden_shelf","armchair","dining_table","wooden_side_table","wooden_bed","iron_bed","wagon", "trash_box_for_recycle", "trash_box_for_burnable",
                        "trash_box_for_bottle_can","cardboard_box", "Avatar"};

  std::string ENVIRONMENT = "None";

  std::unique_ptr<LoadMapManager> map_manager_;

  trajectory_msgs::msg::JointTrajectory arm_joint_trajectory_;

  int step_;
  int patrol_step;
  int max_patrol = 0;  // FIX: Initialize to 0, roomLocation() will set correct value
  bool room_reached;
  bool found_object,found_dest;
  double init_yaw;
  double x_det, y_det;
  bool ready_to_grasp;
  double arm_height,body_height;
  bool aligned_x, aligned_y;
  geometry_msgs::msg::PoseStamped object_pose;
  geometry_msgs::msg::Pose dest_pose;
  bool extend_before_release;

  // Phase 1: Room scan state variables
  bool scan_started_;           // True after entering MoveToInFrontOfTarget from GoToRoom1
  double scan_total_rotation_;  // Total rotation angle during room scan
  rclcpp::Time scan_start_time_;  // When room scan started
  static constexpr double MAX_SCAN_ROTATION = 6.28;  // Max 360 degree scan before giving up
  static constexpr double SCAN_ROTATION_SPEED = 0.3;  // rad/s for room scan

  // Dynamic room distance check state
  rclcpp::Time last_room_check_time_;  // Last time we checked room distance
  static constexpr double ROOM_CHECK_INTERVAL_FAR = 6.0;   // Check every 6s when far (>5m)
  static constexpr double ROOM_CHECK_INTERVAL_MID = 4.0;    // Check every 4s when mid (2-5m)
  static constexpr double ROOM_CHECK_INTERVAL_NEAR = 2.0;   // Check every 2s when near (<2m)
  static constexpr double ROOM_DIST_FAR_THRESHOLD = 5.0;     // >5m = far
  static constexpr double ROOM_DIST_NEAR_THRESHOLD = 2.0;   // <2m = near

  std::string instruction_msg_;

  bool is_started_;
  bool is_finished_;
  bool is_failed_;
  rclcpp::Time last_detect_;
  double tol_multiplier;
  double x_adjust,y_adjust;

  std::vector<std::string> object_list,room_list,dest_list;

  rclcpp::Time last_are_you_ready_time_;
  const double MESSAGE_DEDUPE_INTERVAL = 0.1;

  std::shared_ptr<rclcpp::Node> node_;

  // TF buffer for checking robot position
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_action_client_;
  std::shared_future<GoalHandleNavigate::SharedPtr> nav_goal_future_;
  GoalHandleNavigate::SharedPtr nav_goal_handle_;
  std::atomic<bool> nav_goal_active_{false};
  std::atomic<bool> nav_goal_cancelled_{false};  // Flag to distinguish timeout-cancelled from normal completion
  std::atomic<bool> nav_goal_reached_{false};
  std::atomic<bool> nav_goal_failed_{false};
  std::atomic<bool> nav_goal_accepted_{false};
  bool nav_goal_sent_{false};
  rclcpp::Time nav_goal_send_time_;
  static constexpr double NAV_GOAL_TIMEOUT_SEC = 60.0;

  void init()
  {
    map_manager_ = std::make_unique<LoadMapManager>(node_);

    // Initialize TF buffer and listener for robot position checking
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    std::vector<std::string> arm_joint_names {"arm_lift_joint", "arm_flex_joint", "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"};

    trajectory_msgs::msg::JointTrajectoryPoint arm_joint_point;

    arm_joint_trajectory_.joint_names = arm_joint_names;
    arm_joint_trajectory_.points.push_back(arm_joint_point);

    step_ = Initialize;

    reset();
    last_are_you_ready_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
  }

  void reset()
  {
    instruction_msg_ = "";
    ENVIRONMENT = "None";
    is_started_  = false;
    is_finished_ = false;
    is_failed_   = false;

    tol_multiplier = 1.0;
    object_list.clear();
    room_list.clear();
    dest_list.clear();
    patrol_step = 0;  // FIX: must reset patrol_step between tasks to avoid stuck navigation
    nav_goal_sent_ = false;
    nav_goal_failed_ = false;
    nav_goal_reached_ = false;
    nav_goal_active_ = false;
    nav_goal_cancelled_ = false;  // Reset cancellation flag
    nav_goal_accepted_ = false;
    std::vector<double> arm_positions { 0.0, 0.0, 0.0, 0.0, 0.0 };
    arm_joint_trajectory_.points[0].positions = arm_positions;

    // Phase 1: Reset room scan state
    scan_started_ = false;
    scan_total_rotation_ = 0.0;
  }


  void visionCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr pose)
  {
    object_pose = *pose;
  }

  void graspvisioncallback(const std_msgs::msg::Int32MultiArray::ConstSharedPtr posearray){

    if(!found_object){
      found_object = true;
    }

    x_det = posearray->data[0];
    y_det = posearray->data[1];
    std::cout << "X = " << x_det << " Y = " << y_det << " W = " << posearray->data[2] << " H = " << posearray->data[3] << std::endl;
    if(posearray->data[2] >= 450){
      ready_to_grasp = true;
      tol_multiplier = 1.5;
    }else{
      ready_to_grasp = false;
      tol_multiplier = 1.0;
    }
    last_detect_ = node_->now();
  }

  void messageCallback(const handyman_msgs::msg::HandymanMsg::ConstSharedPtr message)
  {
    RCLCPP_INFO(node_->get_logger(), "Subscribe message:%s, %s", message->message.c_str(), message->detail.c_str());

    if(message->message.c_str()==MSG_ENVIRONMENT)
    {
      ENVIRONMENT = mapUnityEnvironmentName(message->detail);
      return;  // environment assignment must not fall through to Are_you_ready? handler
    }
    RCLCPP_INFO(node_->get_logger(), "######The environment is %s", ENVIRONMENT.c_str());
    if(ENVIRONMENT != "None"){
      if(message->message.c_str()==MSG_ARE_YOU_READY)
      {
        if(step_==Ready)
        {
          is_started_ = true;
        }else if(step_ != Initialize){
          if(step_ == GoToRoom2 || step_ == GoToRoom1 || step_ == MoveToInFrontOfTarget || step_ == Grasp || step_ == Release)
          {
            RCLCPP_WARN(node_->get_logger(), "Critical state interrupted by Are_you_ready? message. Current step: %d", step_);
            step_ = Initialize;
          }
          else if(step_ == WaitForInstruction)
          {
            RCLCPP_DEBUG(node_->get_logger(), "Ignoring Are_you_ready? message in WaitForInstruction state - robot is ready for instructions");
          }
          else if(step_ == ComeBack || step_ == TaskFinished)
          {
            RCLCPP_INFO(node_->get_logger(), "Conditional interrupt in state: %d", step_);
            step_ = Initialize;
          }
        }
      }

      if(message->message.c_str()==MSG_INSTRUCTION)
      {
        if(step_==WaitForInstruction)
        {
          instruction_msg_ = message->detail.c_str();
        }
      }
    }

    if(message->message.c_str()==MSG_TASK_SUCCEEDED)
    {
      if(step_==TaskFinished)
      {
        is_finished_ = true;
      }
    }
    if(message->message.c_str()==MSG_TASK_FAILED)
    {
      is_failed_ = true;
    }
    if(message->message.c_str()==MSG_MISSION_COMPLETE)
    {
      exit(EXIT_SUCCESS);
    }
  }

  void sendMessage(rclcpp::Publisher<handyman_msgs::msg::HandymanMsg>::SharedPtr &publisher, const std::string &message)
  {
    RCLCPP_INFO(node_->get_logger(), "Send message:%s", message.c_str());

    handyman_msgs::msg::HandymanMsg handyman_msg;
    handyman_msg.message = message;
    publisher->publish(handyman_msg);
  }

  void tokenize(std::string const &str, const char delim, std::vector<std::string> &out)
  {
    std::stringstream ss(str);

    std::string s;
    while (std::getline(ss, s, delim)) {
        out.push_back(s);
    }
  }

  std::vector<std::string> extractInfo(std::string msg,std::vector<std::string>& room_array,std::vector<std::string>& object_array,std::vector<std::string>& dest_array){
    std::vector<std::string> info;
    tokenize(msg,' ',info);

    for(size_t i=0; i<info.size();i++){

    for(size_t j=0;j<rooms.size();j++){
        size_t index = info[i].find(rooms[j]);
        if (index != std::string::npos){
            room_array.push_back(rooms[j]);
        }
    }

    for(size_t j=0;j<objects.size();j++){
        size_t index = info[i].find(objects[j]);
        if (index != std::string::npos){
            object_array.push_back(objects[j]);
        }
    }

    for(size_t j=0;j<dests.size();j++){
        size_t index = info[i].find(dests[j]);
        if (index != std::string::npos){
            dest_array.push_back(dests[j]);
        }
    }

    }

    return info;
  }

  std::string mapUnityEnvironmentName(const std::string& unity_env) {
    static const std::map<std::string, std::string> unity_to_internal = {
      {"LayoutA", "Layout2019HM01"},
      {"LayoutB", "Layout2019HM02"},
      {"LayoutC", "Layout2020HM01"},
      {"LayoutD", "Layout2021HM01"},
    };

    auto it = unity_to_internal.find(unity_env);
    if (it != unity_to_internal.end()) {
      RCLCPP_INFO(node_->get_logger(), "Mapped Unity environment: %s -> %s", unity_env.c_str(), it->second.c_str());
      return it->second;
    }

    RCLCPP_INFO(node_->get_logger(), "Environment name unchanged: %s", unity_env.c_str());
    return unity_env;
  }

  geometry_msgs::msg::PoseStamped makePoseStamped(double x, double y, double yaw_rad) {
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

  geometry_msgs::msg::PoseStamped destLocation(std::string dest,std::string room){

    RCLCPP_INFO(node_->get_logger(), "destLocation called with dest=%s, room=%s, ENVIRONMENT=%s",
      dest.c_str(), room.c_str(), ENVIRONMENT.c_str());

    // 2019HM01
    auto LayoutA_white_side_table_lobby = makePoseStamped(1.3, -6.1, -1.57);
    auto LayoutA_corner_sofa = makePoseStamped(-0.15, -6.0, -1.57);
    auto LayoutA_arm_chair = makePoseStamped(2.08, -3.2916, 1.57);
    auto LayoutA_trash_box_for_recycle = makePoseStamped(-0.5, 1.28, 3.14);
    auto LayoutA_trash_box_for_burnable = makePoseStamped(-0.5, 2.4, 3.14);
    auto LayoutA_trash_box_for_bottle_can = makePoseStamped(-0.8, -2.0, 3.14);
    auto LayoutA_white_side_table_living_room = makePoseStamped(2.13, -0.003, -1.57);
    auto LayoutA_square_low_table = makePoseStamped(0.774, 2.79, 0.8);
    auto LayoutA_dining_table = makePoseStamped(7.8, 3.07, 0);
    auto LayoutA_wooden_side_table = makePoseStamped(4.8, 1.9, -1.8);
    auto LayoutA_white_side_table_bedroom = makePoseStamped(9.3, -6.4, 0);
    auto LayoutA_white_side_table_bedroom_2 = makePoseStamped(9.37, -2.6, 1.57);
    auto LayoutA_wooden_bed = makePoseStamped(6.8, -4.9, 1.57);
    auto LayoutA_wagon = makePoseStamped(8.12, -2.7, 1.57);
    auto LayoutA_cardboard = makePoseStamped(9.2, -4.85, 0);

    //2019HM02
    auto LayoutB_white_side_table_living_room = makePoseStamped(1.45, 7.6, -1.57);
    auto LayoutB_arm_chair = makePoseStamped(-0.19, 4.45, 3.14);
    auto LayoutB_trash_box_for_recycle = makePoseStamped(6.7, 3.8, 1.57);
    auto LayoutB_trash_box_for_bottle_can = makePoseStamped(7.56, 3.7, 1.57);
    auto LayoutB_white_side_table_lobby = makePoseStamped(1.4, -0.5, -1.57);
    auto LayoutB_square_low_table = makePoseStamped(1.28, 10.5, 1.57);
    auto LayoutB_round_low_table = makePoseStamped(1.9, 8.3, 1.57);
    auto LayoutB_dining_table_kitchen = makePoseStamped(7.85, -1.1, 3.14);
    auto LayoutB_dining_table_lobby = makePoseStamped(2.2, 2.0, 3.14);
    auto LayoutB_wooden_side_table_lobby = makePoseStamped(1.00, 5.25, 1.57);
    auto LayoutB_wooden_side_table_kitchen = makePoseStamped(9.0, 3.0, 0);
    auto LayoutB_wooden_shelf = makePoseStamped(2.0, 5.0, 1.57);
    auto LayoutB_wagon = makePoseStamped(3.14, 0.75, 0);
    auto LayoutB_cardboard = makePoseStamped(2.9, 7.6, -1.57);

    //2020HM01
    auto LayoutC_trash_box_for_recycle = makePoseStamped(7.2, 4.55, 1.57);
    auto LayoutC_trash_box_for_burnable_living = makePoseStamped(3.8, -1.5, -1.57);
    auto LayoutC_trash_box_for_burnable_kitchen = makePoseStamped(8.3, 2.3, 0);
    auto LayoutC_trash_box_for_bottle_can = makePoseStamped(6.4587, 4.6449, 1.57);
    auto LayoutC_trash_box_for_bottle_can2 = makePoseStamped(8.3, 3.0, 0);
    (void)LayoutC_trash_box_for_bottle_can2;
    auto LayoutC_white_side_table_living_room = makePoseStamped(-0.4, 3.5, 3.14);
    auto LayoutC_round_low_table = makePoseStamped(-0.2, 8.4, 1.57);
    (void)LayoutC_round_low_table;
    auto LayoutC_dining_table = makePoseStamped(3.2, 0.1, 1.57);
    auto LayoutC_dining_table_2 = makePoseStamped(3.9, 0.78, 1.57);
    (void)LayoutC_dining_table_2;
    auto LayoutC_wooden_side_table = makePoseStamped(2.7, -1.6, -1.57);
    auto LayoutC_white_side_table_bedroom = makePoseStamped(2.8, 7.2, -1.57);
    auto LayoutC_iron_bed = makePoseStamped(3.24, 8.2, 0);
    auto LayoutC_wooden_shelf = makePoseStamped(2.4, 4.8, 1.57);
    auto LayoutC_cardboard = makePoseStamped(0.95, 8.5, 1.57);

    //2021HM01
    auto LayoutD_white_side_table_living = makePoseStamped(2.15, 0.7, 1.57);
    auto LayoutD_corner_sofa = makePoseStamped(-5.3, -10.2, -1.57);
    auto LayoutD_arm_chair = makePoseStamped(4.0, -9.86, -1.57);
    auto LayoutD_trash_box_for_burnable_living = makePoseStamped(-1.58, -1.05, 3.14);
    auto LayoutD_trash_box_for_burnable_lobby = makePoseStamped(-3.17, -4.8, 1.57);
    auto LayoutD_white_side_table_lobby = makePoseStamped(-3.7, -10.3, -1.57);
    auto LayoutD_dining_table_living = makePoseStamped(1.5, -2.2, 0);
    auto LayoutD_dining_table_lobby = makePoseStamped(-2.4, -8.4, 3.14);
    auto LayoutD_white_side_table_bedroom = makePoseStamped(1.3, -7.5, 1.57);
    auto LayoutD_wooden_bed = makePoseStamped(1.8, -10.6, 3.14);
    auto LayoutD_wooden_shelf = makePoseStamped(-4.13, -4.7, 1.57);
    auto LayoutD_wagon_living = makePoseStamped(-1.6, -2.35, 3.14);
    auto LayoutD_wagon_bedroom = makePoseStamped(0.27, -9.06, 0);
    auto LayoutD_cardboard = makePoseStamped(-6.2965, -6.9433, 3.14);

    geometry_msgs::msg::PoseStamped location;

    if(ENVIRONMENT == "Layout2019HM01"){

      if(dest == "white_side_table"){
        if(room == "living"){
          location = LayoutA_white_side_table_living_room;
        }else if(room == "lobby"){
          location = LayoutA_white_side_table_lobby;
        }else{
          location = LayoutA_white_side_table_bedroom;
        }
      }else if(dest == "corner_sofa"){
        location = LayoutA_corner_sofa;
      }else if(dest == "armchair"){
        location = LayoutA_arm_chair;
      }else if(dest == "trash_box_for_recycle"){
        location = LayoutA_trash_box_for_recycle;
      }else if(dest == "trash_box_for_burnable"){
        location = LayoutA_trash_box_for_burnable;
      }else if(dest == "trash_box_for_bottle_can"){
        location = LayoutA_trash_box_for_bottle_can;
      }else if(dest == "square_low_table"){
        location = LayoutA_square_low_table;
      }else if(dest == "dining_table"){
        location = LayoutA_dining_table;
      }else if(dest == "wooden_side_table"){
        location = LayoutA_wooden_side_table;
      }else if(dest == "wooden_bed"){
        location = LayoutA_wooden_bed;
      }else if(dest == "wagon"){
        location = LayoutA_wagon;
      }else if(dest == "cardboard_box"){
        location = LayoutA_cardboard;
      }

    }else if (ENVIRONMENT == "Layout2019HM02"){

      if(dest == "white_side_table"){
        if(room == "living"){
          location = LayoutB_white_side_table_living_room;
        }else if(room == "lobby"){
          location = LayoutB_white_side_table_lobby;
        }
      }else if(dest == "armchair"){
        location = LayoutB_arm_chair;
      }else if(dest == "trash_box_for_recycle"){
        location = LayoutB_trash_box_for_recycle;
      }else if(dest == "round_low_table"){
        location = LayoutB_round_low_table;
      }else if(dest == "trash_box_for_bottle_can"){
        location = LayoutB_trash_box_for_bottle_can;
      }else if(dest == "square_low_table"){
        location = LayoutB_square_low_table;
      }else if(dest == "dining_table"){
        if(room == "kitchen"){
          location = LayoutB_dining_table_kitchen;
        }else if(room == "lobby"){
          location = LayoutB_dining_table_lobby;
        }
      }else if(dest == "wooden_side_table"){
        if(room == "kitchen"){
          location = LayoutB_wooden_side_table_kitchen;
        }else if(room == "lobby"){
          location = LayoutB_wooden_side_table_lobby;
        }
      }else if(dest == "wagon"){
        location = LayoutB_wagon;
      }else if(dest == "cardboard_box"){
        location = LayoutB_cardboard;
      } else if (dest == "wooden_shelf"){
        location = LayoutB_wooden_shelf;
      }

    }else if(ENVIRONMENT == "Layout2020HM01"){
      if(dest == "white_side_table"){
        if(room == "living"){
          location = LayoutC_white_side_table_living_room;
        }else if(room == "bedroom"){
          location = LayoutC_white_side_table_bedroom;
        }
      }else if(dest == "trash_box_for_recycle"){
        location = LayoutC_trash_box_for_recycle;
      }else if(dest == "trash_box_for_burnable"){
        if(room == "kitchen"){
          location = LayoutC_trash_box_for_burnable_kitchen;
        }else if(room == "living"){
          location = LayoutC_trash_box_for_burnable_living;
        }
      }else if(dest == "trash_box_for_bottle_can"){
        location = LayoutC_trash_box_for_bottle_can;
      }else if(dest == "dining_table"){
        location = LayoutC_dining_table;
      }else if(dest == "wooden_side_table"){
        location = LayoutC_wooden_side_table;
      }else if(dest == "iron_bed"){
        location = LayoutC_iron_bed;
      }else if(dest == "cardboard_box"){
        location = LayoutC_cardboard;
      }else if(dest == "wooden_shelf"){
        location = LayoutC_wooden_shelf;
      }

    }else if(ENVIRONMENT == "Layout2021HM01"){

      if(dest == "white_side_table"){
        if(room == "living"){
          location = LayoutD_white_side_table_living;
        }else if(room == "bedroom"){
          location = LayoutD_white_side_table_bedroom;
        }else if(room == "lobby"){
          location = LayoutD_white_side_table_lobby;
        }
      }else if(dest == "corner_sofa"){
        location = LayoutD_corner_sofa;
      }else if(dest == "armchair"){
        location = LayoutD_arm_chair;
      }else if(dest == "trash_box_for_burnable"){
        if(room == "living"){
          location = LayoutD_trash_box_for_burnable_living;
        }else if(room == "lobby"){
          location = LayoutD_trash_box_for_burnable_lobby;
        }
      }else if(dest == "wooden_shelf"){
        location = LayoutD_wooden_shelf;
      }else if(dest == "dining_table"){
        if(room == "living"){
          location = LayoutD_dining_table_living;
        }else if(room == "lobby"){
          location = LayoutD_dining_table_lobby;
        }
      }else if(dest == "wooden_bed"){
        location = LayoutD_wooden_bed;
      }else if(dest == "wagon"){
        if(room == "living"){
          location = LayoutD_wagon_living;
        }else if(room == "bedroom"){
          location = LayoutD_wagon_bedroom;
        }
      }else if(dest == "cardboard_box"){
        location = LayoutD_cardboard;
      }

    }
    if (location.pose.orientation.w == 0.0 &&
      location.pose.orientation.x == 0.0 &&
      location.pose.orientation.y == 0.0 &&
      location.pose.orientation.z == 0.0) {
      RCLCPP_ERROR(node_->get_logger(), "Invalid quaternion for dest: %s, room: %s", dest.c_str(), room.c_str());
    }
    return location;
  }

  // Note: Room boundary detection was removed. Robot must ALWAYS patrol to confirm
  // room entry and earn the 20 points. This ensures reliable room detection without
  // relying on uncertain boundary coordinates.

  geometry_msgs::msg::PoseStamped roomLocation(std::string room, int variation){
    // LayoutA
    auto LayoutA_living_room = makePoseStamped(2.0, 1, 1.57);
    auto LayoutA_living_room_2 = makePoseStamped(2.5, 4.08, 3.14);
    auto LayoutA_bedroom = makePoseStamped(2.57, -4.31, 0);
    auto LayoutA_bedroom2 = makePoseStamped(8.64, -5.7, 0);
    auto LayoutA_lobby2 = makePoseStamped(1.0, -3.6, 0);
    auto LayoutA_lobby = makePoseStamped(1.2, -6.16, -1.57);
    auto LayoutA_kitchen = makePoseStamped(8.5, 2.8, 3.14);
    auto LayoutA_kitchen2 = makePoseStamped(5.0, 4.2, -0.7);
    (void)LayoutA_kitchen2;

    // LayoutB
    auto LayoutB_living_room = makePoseStamped(3.5, 9.6, 2.4);
    auto LayoutB_living_room2 = makePoseStamped(1.84, 10.2, 0);
    auto LayoutB_lobby = makePoseStamped(1.0, 0, 0);
    auto LayoutB_lobby2 = makePoseStamped(2.5, 2.0, 0);
    auto LayoutB_kitchen = makePoseStamped(5.5, -1.13, 0);
    auto LayoutB_kitchen2 = makePoseStamped(8.42, -1.13, 0);

    // LayoutC
    auto LayoutC_living_room = makePoseStamped(0.5, 2.0, 1.57);
    auto LayoutC_living_room2 = makePoseStamped(0.42, 3.48, 2.355);
    auto LayoutC_living_room3 = makePoseStamped(4.5, 3.48, 0.0);
    auto LayoutC_living_room4 = makePoseStamped(4.5, -0.65, 0.0);
    auto LayoutC_bedroom = makePoseStamped(0.1, 6.9, 0.0);
    auto LayoutC_bedroom2 = makePoseStamped(3.0, 8.0, 0.0);
    auto LayoutC_kitchen = makePoseStamped(6.5, -1.2, 0);
    auto LayoutC_kitchen2 = makePoseStamped(7.8, 1.2, 0);
    auto LayoutC_kitchen3 = makePoseStamped(6.5, 3.9, 0);

    // LayoutD
    auto LayoutD_living_room = makePoseStamped(1.0, 0.0, 0);
    auto LayoutD_living_room_2 = makePoseStamped(3.5, 0.0, 0);
    auto LayoutD_bedroom = makePoseStamped(4.0, -8.5, 0);
    auto LayoutD_bedroom2 = makePoseStamped(1.69, -8.0, 0.0);
    auto LayoutD_lobby = makePoseStamped(-1.86, -8.38, 0.0);
    auto LayoutD_lobby2 = makePoseStamped(-4.86, -8.7, 0.0);

    geometry_msgs::msg::PoseStamped location;
    std::string env_lower = lower(ENVIRONMENT);
    if (env_lower.compare("layout2019hm01") == 0){
        if(room == "living"){
          max_patrol = 2;
          if(variation == 0)
            location = LayoutA_living_room;
          else if (variation == 1)
            location = LayoutA_living_room_2;
          }
        else if(room == "bedroom"){
          max_patrol = 2;
          if(variation == 0)
            location = LayoutA_bedroom;
          else if (variation == 1)
            location = LayoutA_bedroom2;
          }
        else if(room == "lobby"){
          max_patrol = 2;
          if(variation == 0)
            location = LayoutA_lobby;
          else if (variation == 1)
            location = LayoutA_lobby2;
           }
        else if(room == "kitchen"){
          max_patrol = 2;
          if(variation == 0)
            location = LayoutA_kitchen;
          }
    }
    else if (env_lower.compare("layout2019hm02") == 0){
         if(room == "living"){
          max_patrol = 2;
          if(variation == 0)
            location = LayoutB_living_room;
          else if (variation == 1)
            location = LayoutB_living_room2;
            }
        else if(room == "lobby"){
          max_patrol = 2;
          if(variation == 0)
            location = LayoutB_lobby;
          else if (variation == 1)
            location = LayoutB_lobby2;
            }
        else if(room == "kitchen"){
          max_patrol = 2;
          if(variation == 0)
            location = LayoutB_kitchen;
          else if (variation == 1)
            location = LayoutB_kitchen2;
             }
    }
    else if (env_lower.compare("layout2020hm01") == 0){
      if(room == "living"){
          max_patrol = 4;
          if(variation == 0)
            location = LayoutC_living_room;
          else if (variation == 1)
            location = LayoutC_living_room2;
          else if(variation == 2 )
            location = LayoutC_living_room3;
          else if (variation == 3)
            location = LayoutC_living_room4;
           }
        else if(room == "bedroom"){
          max_patrol = 2;
          if( variation == 0)
            location = LayoutC_bedroom;
          else if(variation == 1)
            location = LayoutC_bedroom2;
          }
        else if(room == "kitchen"){
          max_patrol = 3;
          if( variation == 0)
            location = LayoutC_kitchen;
          else if(variation == 1)
            location = LayoutC_kitchen2;
          else if(variation == 2)
            location = LayoutC_kitchen3;
          }
    }
    else if (env_lower.compare("layout2021hm01") == 0){
        if(room == "living"){
          max_patrol = 2;
          if(variation == 0)
            location = LayoutD_living_room;
          else if (variation == 1)
            location = LayoutD_living_room_2;
        }
        else if(room == "bedroom"){
          max_patrol = 2;
          if(variation == 0)
            location = LayoutD_bedroom;
          else if (variation == 1)
            location = LayoutD_bedroom2;
        }
        else if(room == "lobby"){
          max_patrol = 2;
          if(variation == 0)
            location = LayoutD_lobby;
          else if (variation == 1)
            location = LayoutD_lobby2;
        }
    }

    else{
      location = LayoutC_living_room;
      RCLCPP_INFO(node_->get_logger(), "$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$WELP STH UNDESIRABLE HAPPENED");
    }

    return location;
  }

  // FIX: Check if robot is in the target room using rectangular boundary.
  // Rectangle is centered at waypoint, extends 1m in all directions from center.
  // Returns: {is_in_room, distance_to_rectangle}
  // Distance is 0 if robot is inside the rectangle.
  std::pair<bool, double> getRobotRoomDistance(const std::string& target_room) {
    static constexpr double ROOM_HALF_SIZE_M = 1.0;  // Rectangle extends 1m from center point

    try {
      // Get robot pose in map frame
      geometry_msgs::msg::TransformStamped tf_stamped = tf_buffer_->lookupTransform(
        "map", "base_footprint", tf2::TimePointZero, tf2::durationFromSec(1.0));
      double robot_x = tf_stamped.transform.translation.x;
      double robot_y = tf_stamped.transform.translation.y;

      std::string env_lower = lower(ENVIRONMENT);

      // Helper: calculate distance from point to rectangle
      // Rectangle defined by center (cx, cy) and half-size (h)
      // Returns 0 if inside, otherwise returns minimum horizontal distance to edge
      auto distToRect = [](double px, double py, double cx, double cy, double h) -> double {
        // Check if point is inside rectangle
        if (px >= cx - h && px <= cx + h && py >= cy - h && py <= cy + h) {
          return 0.0;  // Inside rectangle
        }
        // Calculate minimum distance to rectangle edge (horizontal/vertical distance)
        double dx = 0.0, dy = 0.0;
        if (px < cx - h) dx = (cx - h) - px;
        else if (px > cx + h) dx = px - (cx + h);
        if (py < cy - h) dy = (cy - h) - py;
        else if (py > cy + h) dy = py - (cy + h);
        return std::sqrt(dx * dx + dy * dy);
      };

      // Check all waypoints for this room
      // LayoutA waypoints
      if (env_lower == "layout2019hm01") {
        if (target_room == "living") {
          double waypoints[2][2] = {{2.0, 1.0}, {2.5, 4.08}};
          for (int i = 0; i < 2; i++) {
            double dist = distToRect(robot_x, robot_y, waypoints[i][0], waypoints[i][1], ROOM_HALF_SIZE_M);
            if (dist < 0.01) {  // Inside rectangle
              return {true, dist};
            }
          }
        } else if (target_room == "bedroom") {
          double waypoints[2][2] = {{2.57, -4.31}, {8.64, -5.7}};
          for (int i = 0; i < 2; i++) {
            double dist = distToRect(robot_x, robot_y, waypoints[i][0], waypoints[i][1], ROOM_HALF_SIZE_M);
            if (dist < 0.01) return {true, dist};
          }
        } else if (target_room == "lobby") {
          double waypoints[2][2] = {{1.2, -6.16}, {1.0, -3.6}};
          for (int i = 0; i < 2; i++) {
            double dist = distToRect(robot_x, robot_y, waypoints[i][0], waypoints[i][1], ROOM_HALF_SIZE_M);
            if (dist < 0.01) return {true, dist};
          }
        } else if (target_room == "kitchen") {
          double waypoints[2][2] = {{8.5, 2.8}, {5.0, 4.2}};
          for (int i = 0; i < 2; i++) {
            double dist = distToRect(robot_x, robot_y, waypoints[i][0], waypoints[i][1], ROOM_HALF_SIZE_M);
            if (dist < 0.01) return {true, dist};
          }
        }
      }
      // LayoutB waypoints
      else if (env_lower == "layout2019hm02") {
        if (target_room == "living") {
          double waypoints[2][2] = {{3.5, 9.6}, {1.84, 10.2}};
          for (int i = 0; i < 2; i++) {
            double dist = distToRect(robot_x, robot_y, waypoints[i][0], waypoints[i][1], ROOM_HALF_SIZE_M);
            if (dist < 0.01) return {true, dist};
          }
        } else if (target_room == "lobby") {
          double waypoints[2][2] = {{1.0, 0.0}, {2.5, 2.0}};
          for (int i = 0; i < 2; i++) {
            double dist = distToRect(robot_x, robot_y, waypoints[i][0], waypoints[i][1], ROOM_HALF_SIZE_M);
            if (dist < 0.01) return {true, dist};
          }
        } else if (target_room == "kitchen") {
          double waypoints[2][2] = {{5.5, -1.13}, {8.42, -1.13}};
          for (int i = 0; i < 2; i++) {
            double dist = distToRect(robot_x, robot_y, waypoints[i][0], waypoints[i][1], ROOM_HALF_SIZE_M);
            if (dist < 0.01) return {true, dist};
          }
        }
      }
      // LayoutC waypoints
      else if (env_lower == "layout2020hm01") {
        if (target_room == "living") {
          double waypoints[4][2] = {{0.5, 2.0}, {0.42, 3.48}, {4.5, 3.48}, {4.5, -0.65}};
          for (int i = 0; i < 4; i++) {
            double dist = distToRect(robot_x, robot_y, waypoints[i][0], waypoints[i][1], ROOM_HALF_SIZE_M);
            if (dist < 0.01) return {true, dist};
          }
        } else if (target_room == "bedroom") {
          double waypoints[2][2] = {{0.1, 6.9}, {3.0, 8.0}};
          for (int i = 0; i < 2; i++) {
            double dist = distToRect(robot_x, robot_y, waypoints[i][0], waypoints[i][1], ROOM_HALF_SIZE_M);
            if (dist < 0.01) return {true, dist};
          }
        } else if (target_room == "kitchen") {
          double waypoints[3][2] = {{6.5, -1.2}, {7.8, 1.2}, {6.5, 3.9}};
          for (int i = 0; i < 3; i++) {
            double dist = distToRect(robot_x, robot_y, waypoints[i][0], waypoints[i][1], ROOM_HALF_SIZE_M);
            if (dist < 0.01) return {true, dist};
          }
        }
      }
      // LayoutD waypoints
      else if (env_lower == "layout2021hm01") {
        if (target_room == "living") {
          double waypoints[2][2] = {{1.0, 0.0}, {3.5, 0.0}};
          for (int i = 0; i < 2; i++) {
            double dist = distToRect(robot_x, robot_y, waypoints[i][0], waypoints[i][1], ROOM_HALF_SIZE_M);
            if (dist < 0.01) return {true, dist};
          }
        } else if (target_room == "bedroom") {
          double waypoints[2][2] = {{4.0, -8.5}, {1.69, -8.0}};
          for (int i = 0; i < 2; i++) {
            double dist = distToRect(robot_x, robot_y, waypoints[i][0], waypoints[i][1], ROOM_HALF_SIZE_M);
            if (dist < 0.01) return {true, dist};
          }
        } else if (target_room == "lobby") {
          double waypoints[2][2] = {{-1.86, -8.38}, {-4.86, -8.7}};
          for (int i = 0; i < 2; i++) {
            double dist = distToRect(robot_x, robot_y, waypoints[i][0], waypoints[i][1], ROOM_HALF_SIZE_M);
            if (dist < 0.01) return {true, dist};
          }
        }
      }

      // Not inside any rectangle - calculate minimum distance to all rectangles
      // For simplicity, return distance to first waypoint's rectangle
      // (GoToRoom1 loop will handle finding the closest one)
      return {false, 0.0};  // Caller should re-check with actual waypoint

    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN(node_->get_logger(), "getRobotRoomDistance: TF error: %s", ex.what());
      return {false, -1.0};  // Unknown
    }
  }

  // Simplified check: returns true if robot is in target room
  bool isRobotInTargetRoom(const std::string& target_room) {
    auto [in_room, dist] = getRobotRoomDistance(target_room);
    return in_room;
  }

  tf2::Transform getTfBase(tf2_ros::Buffer &tf_buffer)
  {
    tf2::Transform tf_transform;

    try
    {
      geometry_msgs::msg::TransformStamped tf_stamped = tf_buffer.lookupTransform("odom", "base_footprint", tf2::TimePointZero);
      tf2::fromMsg(tf_stamped.transform, tf_transform);
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_ERROR(node_->get_logger(), "%s",ex.what());
    }
    return tf_transform;
  }

  void moveBase(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr &publisher, double linear_x, double linear_y, double angular_z)
  {
    geometry_msgs::msg::Twist twist;

    twist.linear.x  = linear_x;
    twist.linear.y  = linear_y;
    twist.angular.z = angular_z;
    publisher->publish(twist);
  }

  void stopBase(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr &publisher)
  {
    moveBase(publisher, 0.0, 0.0, 0.0);
  }

  void resetAMCL()
  {
      RCLCPP_INFO(node_->get_logger(), "Resetting AMCL for new map...");

      if (!map_manager_) {
          RCLCPP_ERROR(node_->get_logger(), "LoadMapManager not initialized");
          return;
      }

      geometry_msgs::msg::Pose initial_pose;
      initial_pose.position.x = 0.0;
      initial_pose.position.y = 0.0;
      initial_pose.position.z = 0.0;
      initial_pose.orientation.x = 0.0;
      initial_pose.orientation.y = 0.0;
      initial_pose.orientation.z = 0.0;
      initial_pose.orientation.w = 1.0;

      if (map_manager_->resetAMCL(initial_pose)) {
          RCLCPP_INFO(node_->get_logger(), "AMCL reset completed successfully.");
      } else {
          RCLCPP_ERROR(node_->get_logger(), "AMCL reset failed.");
      }
  }

  bool loadMap()
  {
      RCLCPP_INFO(node_->get_logger(), "Loading map for ENVIRONMENT: %s", ENVIRONMENT.c_str());

      if (!map_manager_) {
          RCLCPP_ERROR(node_->get_logger(), "LoadMapManager not initialized");
          return false;
      }

      if (map_manager_->switchEnvironment(ENVIRONMENT)) {
          RCLCPP_INFO(node_->get_logger(), "Successfully switched to environment: %s", ENVIRONMENT.c_str());
          return true;
      } else {
          RCLCPP_ERROR(node_->get_logger(), "Failed to switch to environment: %s", ENVIRONMENT.c_str());
          return false;
      }
  }

  void moveArm(rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr &publisher, const std::vector<double> &positions, rclcpp::Duration &duration)
  {
    arm_joint_trajectory_.points[0].positions = positions;
    arm_joint_trajectory_.points[0].time_from_start = duration;

    publisher->publish(arm_joint_trajectory_);
  }

  void operateHand(rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr &publisher, bool should_grasp)
  {
    std::vector<std::string> joint_names {"hand_motor_joint"};
    std::vector<double> positions;

    if(should_grasp)
    {
      RCLCPP_DEBUG(node_->get_logger(), "Grasp");
      positions.push_back(-0.105);
    }
    else
    {
      RCLCPP_DEBUG(node_->get_logger(), "Open hand");
      positions.push_back(+1.239);
    }

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = positions;
    point.time_from_start = rclcpp::Duration::from_seconds(2);

    trajectory_msgs::msg::JointTrajectory joint_trajectory;
    joint_trajectory.joint_names = joint_names;
    joint_trajectory.points.push_back(point);
    publisher->publish(joint_trajectory);
  }

  void sendNavGoal(const geometry_msgs::msg::PoseStamped& target_pose) {
    if (!nav_action_client_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(node_->get_logger(), "Nav2 action server not available after 10s");
      nav_goal_failed_ = true;
      return;
    }

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = target_pose;

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    send_goal_options.goal_response_callback =
      [this](const GoalHandleNavigate::SharedPtr & goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(node_->get_logger(), "Navigation goal was REJECTED by server");
          nav_goal_active_ = false;
          nav_goal_accepted_ = false;
          nav_goal_failed_ = true;
          nav_goal_sent_ = false;  // FIX: Must reset sent flag so we can retry
          nav_goal_handle_.reset();  // FIX: prevent stale handle from causing UnknownGoalHandleError
        } else {
          RCLCPP_INFO(node_->get_logger(), "Navigation goal ACCEPTED by server");
          nav_goal_accepted_ = true;
          nav_goal_handle_ = goal_handle;
        }
      };

    send_goal_options.result_callback =
      [this](const GoalHandleNavigate::WrappedResult & result) {
        nav_goal_active_ = false;
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            // Only mark as reached if goal was not cancelled by timeout
            // This prevents race condition where SUCCEEDED arrives after we've
            // already moved to next waypoint due to timeout
            if (!nav_goal_cancelled_) {
              RCLCPP_INFO(node_->get_logger(), "Navigation goal SUCCEEDED");
              nav_goal_reached_ = true;
            } else {
              RCLCPP_INFO(node_->get_logger(), "Navigation goal SUCCEEDED (stale, goal was cancelled by timeout)");
            }
            nav_goal_handle_.reset();  // FIX: prevent stale handle from causing UnknownGoalHandleError
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(node_->get_logger(), "Navigation goal ABORTED by server");
            nav_goal_failed_ = true;
            nav_goal_sent_ = false;  // FIX: Must reset sent flag so we can retry after ABORTED
            nav_goal_handle_.reset();  // FIX: prevent stale handle from causing UnknownGoalHandleError
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(node_->get_logger(), "Navigation goal was CANCELED");
            nav_goal_failed_ = true;
            nav_goal_sent_ = false;  // FIX: Must reset sent flag so we can retry after CANCELED
            nav_goal_handle_.reset();  // FIX: prevent stale handle from causing UnknownGoalHandleError
            break;
          default:
            RCLCPP_ERROR(node_->get_logger(), "Navigation goal returned UNKNOWN result code: %d",
                        static_cast<int>(result.code));
            nav_goal_failed_ = true;
            nav_goal_sent_ = false;  // FIX: Must reset sent flag so we can retry
            nav_goal_handle_.reset();  // FIX: prevent stale handle from causing UnknownGoalHandleError
            break;
        }
      };

    send_goal_options.feedback_callback =
      [this](GoalHandleNavigate::SharedPtr,
        const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
          "Navigation progress - distance remaining: %.2f m",
          feedback->distance_remaining);
      };

    nav_goal_active_ = true;
    nav_goal_reached_ = false;
    nav_goal_failed_ = false;
    nav_goal_cancelled_ = false;  // Reset cancellation flag when sending new goal
    nav_goal_accepted_ = false;
    nav_goal_send_time_ = node_->now();
    nav_action_client_->async_send_goal(goal_msg, send_goal_options);
    RCLCPP_INFO(node_->get_logger(), "Navigation goal sent to (%.2f, %.2f)",
                target_pose.pose.position.x, target_pose.pose.position.y);
  }

  std::mutex state_mutex_;

public:
  int run([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
  {
    node_ = std::make_shared<rclcpp::Node>("handyman_sample");

    nav_action_client_ = rclcpp_action::create_client<NavigateToPose>(node_, "navigate_to_pose");
    nav_goal_send_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());

    rclcpp::Rate loop_rate(10.0);
    std::vector<std::string> data;

    init();

    last_detect_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
    rclcpp::Time waiting_start_time(0, 0, node_->get_clock()->get_clock_type());

    RCLCPP_INFO(node_->get_logger(), "Handyman sample start!");

    auto sub_msg = node_->create_subscription<handyman_msgs::msg::HandymanMsg>("/handyman/message/to_robot", 100, std::bind(&HandymanSample::messageCallback, this, std::placeholders::_1));
    auto sub_msg_vision = node_->create_subscription<geometry_msgs::msg::PoseStamped>("/vision", 1000, std::bind(&HandymanSample::visionCallback, this, std::placeholders::_1));
    auto sub_msg_hand = node_->create_subscription<std_msgs::msg::Int32MultiArray>("/hand_detection", 1000, std::bind(&HandymanSample::graspvisioncallback, this, std::placeholders::_1));
    auto pub_target_object = node_->create_publisher<std_msgs::msg::String>("/detection_target", 10);
    auto pub_msg = node_->create_publisher<handyman_msgs::msg::HandymanMsg>("/handyman/message/to_moderator", 10);
    auto pub_base_twist = node_->create_publisher<geometry_msgs::msg::Twist>("/hsrb/command_velocity", 10);
    auto pub_arm_trajectory = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>("/hsrb/arm_trajectory_controller/command", 10);
    auto pub_gripper_trajectory = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>("/hsrb/gripper_controller/command", 10);

    // Suppress unused variable warnings for subscriptions
    (void)sub_msg;
    (void)sub_msg_vision;
    (void)sub_msg_hand;

    while (rclcpp::ok())
    {
      if(is_failed_)
      {
        RCLCPP_INFO(node_->get_logger(), "Task failed!");
        step_ = Initialize;
      }

      // DEBUG: periodic state heartbeat (every 3 seconds)
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
        "DEBUG heartbeat: step=%d, is_failed_=%d, is_started_=%d, ENVIRONMENT='%s'",
        step_, is_failed_, is_started_, ENVIRONMENT.c_str());

      switch(step_)
      {
        case Initialize:
        {
          reset();
          ENVIRONMENT = "None";
          step_++;
          break;
        }
        case Ready:
        {
          if (is_started_)
          {
            if (ENVIRONMENT == "None")
            {
                RCLCPP_WARN(node_->get_logger(), "Environment not set. Waiting for layout message...");
                break;
            }

            RCLCPP_INFO(node_->get_logger(), "############ Before loadmap, the map is %s", ENVIRONMENT.c_str());
            if (!loadMap())
            {
                RCLCPP_ERROR(node_->get_logger(), "Failed to load map, returning to Initialize");
                step_ = Initialize;
                break;
            }

            {
              RCLCPP_INFO(node_->get_logger(), "Waiting for navigate_to_pose action server...");
              int wait_attempts = 0;
              while (!nav_action_client_->wait_for_action_server(std::chrono::seconds(5)) && rclcpp::ok()) {
                wait_attempts++;
                RCLCPP_WARN(node_->get_logger(), "navigate_to_pose action server not available, retrying... (attempt %d)", wait_attempts);
                rclcpp::spin_some(node_);
                if (wait_attempts >= 12) {
                  RCLCPP_ERROR(node_->get_logger(), "Failed to connect to navigate_to_pose after 60s");
                  break;
                }
              }
              if (nav_action_client_->action_server_is_ready()) {
                RCLCPP_INFO(node_->get_logger(), "navigate_to_pose action server is ready");
              }
            }

            RCLCPP_INFO(node_->get_logger(), "############ The current map is %s", ENVIRONMENT.c_str());
            RCLCPP_INFO(node_->get_logger(), "System is ready, sending I_am_ready message");
            sendMessage(pub_msg, MSG_I_AM_READY);
            RCLCPP_INFO(node_->get_logger(), "Task start!");
            step_++;
          }
          break;
        }
        case WaitForInstruction:
        {
          // DEBUG: log entry to diagnose robot not moving
          RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
            "DEBUG WaitForInstruction: instruction_msg_='%s', step=%d",
            instruction_msg_.c_str(), step_);
          if(instruction_msg_!="")
          {
            RCLCPP_INFO(node_->get_logger(), "%s", instruction_msg_.c_str());
            data = extractInfo(instruction_msg_.c_str(),room_list,object_list,dest_list);
            std_msgs::msg::String target_object;
            target_object.data = object_list[0];
            pub_target_object->publish(target_object);
            std::vector<double> positions { 0.1, 0.0, 0.0, -1.57, 0.0 };
            rclcpp::Duration duration = rclcpp::Duration::from_seconds(1.0);

            moveArm(pub_arm_trajectory, positions, duration);

            operateHand(pub_gripper_trajectory, false);
            room_reached= false;
            found_object = false;
            patrol_step = 0;
            nav_goal_sent_ = false;
            step_= GoToRoom1;
          }
          break;
        }
        case GoToRoom1:
        {
          // FIX: If max_patrol is 0 (not initialized), call roomLocation() first to set it.
          if (max_patrol == 0) {
            RCLCPP_INFO(node_->get_logger(), "GoToRoom1: max_patrol is 0, initializing with roomLocation()...");
            auto dummy_pose = roomLocation(room_list.empty() ? "living" : room_list[0], 0);
            RCLCPP_INFO(node_->get_logger(), "GoToRoom1: max_patrol initialized to %d", max_patrol);
          }

          // FIX: On first entry to GoToRoom1 (patrol_step == 0 and no goal sent yet),
          // check if robot is already in the target room.
          // If yes, immediately send Room_reached and enter scan.
          // If no, proceed with navigation.
          if (patrol_step == 0 && !nav_goal_sent_ && !room_reached) {
            RCLCPP_INFO(node_->get_logger(), "GoToRoom1: Checking if robot is already in target room '%s'...",
                        room_list.empty() ? "unknown" : room_list[0].c_str());

            if (isRobotInTargetRoom(room_list.empty() ? "living" : room_list[0])) {
              // Robot is already in target room!
              RCLCPP_INFO(node_->get_logger(), "GoToRoom1: Robot is ALREADY in target room! Sending Room_reached immediately.");
              sendMessage(pub_msg, MSG_ROOM_REACHED);
              room_reached = true;

              // Enter scan state
              stopBase(pub_base_twist);
              nav_goal_active_ = false;
              nav_goal_reached_ = false;
              nav_goal_failed_ = false;
              ready_to_grasp = false;
              aligned_x = false;
              aligned_y = false;
              arm_height = 0.0;
              x_adjust = 0.0;
              y_adjust = 0.0;
              body_height = 0.0;
              patrol_step = 0;
              scan_started_ = false;
              step_ = MoveToInFrontOfTarget;
              RCLCPP_INFO(node_->get_logger(), "GoToRoom1: Entering MoveToInFrontOfTarget (robot was already in room)");
              break;
            } else {
              RCLCPP_INFO(node_->get_logger(), "GoToRoom1: Robot is NOT in target room. Starting navigation...");
            }
          }

          // DEBUG: log entry every frame (throttled)
          RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
            "DEBUG GoToRoom1: patrol_step=%d, max_patrol=%d, nav_goal_sent_=%d, room_reached=%d, target='%s'",
            patrol_step, max_patrol, nav_goal_sent_, room_reached,
            room_list.empty() ? "(empty)" : room_list[0].c_str());

          // ---- Dynamic room distance check during navigation ----
          // Check more frequently as robot gets closer to room
          if (nav_goal_sent_ && !room_reached) {
            auto now = node_->now();
            double time_since_last_check = (now - last_room_check_time_).seconds();

            // Calculate distance to target room
            auto [in_room, dist_to_room] = getRobotRoomDistance(room_list.empty() ? "living" : room_list[0]);

            // Determine check interval based on distance
            double check_interval = ROOM_CHECK_INTERVAL_FAR;
            if (dist_to_room >= 0) {
              if (dist_to_room > ROOM_DIST_FAR_THRESHOLD) {
                check_interval = ROOM_CHECK_INTERVAL_FAR;  // Far: slow check
              } else if (dist_to_room > ROOM_DIST_NEAR_THRESHOLD) {
                check_interval = ROOM_CHECK_INTERVAL_MID;  // Mid: medium check
              } else {
                check_interval = ROOM_CHECK_INTERVAL_NEAR;  // Near: fast check
              }
            }

            // Log periodically based on distance (throttled)
            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
              "GoToRoom1: Room dist=%.2f m, check_interval=%.2f s, in_room=%d",
              dist_to_room >= 0 ? dist_to_room : -1.0, check_interval, in_room);

            // Check if we should update room status this frame
            if (time_since_last_check >= check_interval) {
              last_room_check_time_ = now;

              // Check if robot has entered the room
              if (in_room && !room_reached) {
                RCLCPP_INFO(node_->get_logger(), "GoToRoom1: Robot ENTERED target room! Sending Room_reached!");
                sendMessage(pub_msg, MSG_ROOM_REACHED);
                room_reached = true;

                // Cancel navigation and enter scan immediately
                nav_action_client_->async_cancel_all_goals();
                auto cancel_start = node_->now();
                while(rclcpp::ok() && (node_->now() - cancel_start).seconds() < 1.0) {
                  stopBase(pub_base_twist);
                  rclcpp::spin_some(node_);
                  std::this_thread::sleep_for(std::chrono::milliseconds(50));
                }
                nav_goal_sent_ = false;
                nav_goal_active_ = false;
                nav_goal_reached_ = false;
                nav_goal_failed_ = false;
                ready_to_grasp = false;
                aligned_x = false;
                aligned_y = false;
                arm_height = 0.0;
                x_adjust = 0.0;
                y_adjust = 0.0;
                body_height = 0.0;
                patrol_step = 0;
                scan_started_ = false;
                step_ = MoveToInFrontOfTarget;
                RCLCPP_INFO(node_->get_logger(), "GoToRoom1: Entering MoveToInFrontOfTarget (room reached during navigation)");
                break;
              }
            }
          }

          // ---- CRITICAL: Only send goal if patrol_step is valid ----
          if (!nav_goal_sent_ && patrol_step < max_patrol) {
            // Initialize last_room_check_time_ on first goal send
            last_room_check_time_ = node_->now();

            // Wait for map TF to be available before sending goal
            static bool tf_wait_logged = false;
            bool tf_available = false;
            auto tf_check_start = node_->now();
            while (rclcpp::ok() && !tf_available &&
                   (node_->now() - tf_check_start).seconds() < 30.0) {
              try {
                tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero,
                                           tf2::durationFromSec(1.0));
                tf_available = true;
              } catch (const tf2::TransformException &) {
                if (!tf_wait_logged) {
                  RCLCPP_WARN(node_->get_logger(), "GoToRoom1: Waiting for map TF to become available...");
                  tf_wait_logged = true;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                rclcpp::spin_some(node_);
              }
            }
            if (!tf_available) {
              RCLCPP_ERROR(node_->get_logger(), "GoToRoom1: Map TF not available after 30s, will retry...");
              tf_wait_logged = false;
              break;
            }
            tf_wait_logged = false;

            // Check if robot is already in room before sending goal
            if (isRobotInTargetRoom(room_list.empty() ? "living" : room_list[0])) {
              RCLCPP_INFO(node_->get_logger(), "GoToRoom1: Robot is already in target room! Sending Room_reached immediately.");
              sendMessage(pub_msg, MSG_ROOM_REACHED);
              room_reached = true;
              stopBase(pub_base_twist);
              nav_goal_active_ = false;
              nav_goal_reached_ = false;
              nav_goal_failed_ = false;
              ready_to_grasp = false;
              aligned_x = false;
              aligned_y = false;
              arm_height = 0.0;
              x_adjust = 0.0;
              y_adjust = 0.0;
              body_height = 0.0;
              patrol_step = 0;
              scan_started_ = false;
              step_ = MoveToInFrontOfTarget;
              RCLCPP_INFO(node_->get_logger(), "GoToRoom1: Entering MoveToInFrontOfTarget (robot was already in room)");
              break;
            }

            found_object = false;

            auto goal_pose = roomLocation(room_list[0], patrol_step);
            RCLCPP_INFO(node_->get_logger(), "GoToRoom1: Sending patrol goal %d/%d to (%.2f, %.2f)",
                       patrol_step + 1, max_patrol, goal_pose.pose.position.x, goal_pose.pose.position.y);
            goal_pose.header.frame_id = "map";
            goal_pose.header.stamp = node_->now();
            sendNavGoal(goal_pose);
            nav_goal_sent_ = true;
          }

          // ---- Navigation succeeded ----
          if (nav_goal_reached_.load()){
              RCLCPP_INFO(node_->get_logger(), "GoToRoom1: Patrol point %d/%d reached", patrol_step + 1, max_patrol);
              nav_goal_reached_ = false;
              nav_goal_sent_ = false;

              // FIX: After reaching waypoint, check if robot is now in target room
              if (!room_reached) {
                if (isRobotInTargetRoom(room_list.empty() ? "living" : room_list[0])) {
                  RCLCPP_INFO(node_->get_logger(), "GoToRoom1: Robot arrived in target room! Sending Room_reached.");
                  sendMessage(pub_msg, MSG_ROOM_REACHED);
                  room_reached = true;
                } else {
                  RCLCPP_INFO(node_->get_logger(), "GoToRoom1: Robot NOT in room after reaching waypoint, continuing patrol...");
                }
              }

              patrol_step++;
              if (patrol_step >= max_patrol) {
                // All waypoints done
                if (!room_reached) {
                  // This should not happen if isRobotInTargetRoom works correctly
                  // But as safeguard: check again and send Room_reached if now in room
                  RCLCPP_WARN(node_->get_logger(), "GoToRoom1: All waypoints done but room_reached is false! Checking...");
                  if (isRobotInTargetRoom(room_list.empty() ? "living" : room_list[0])) {
                    sendMessage(pub_msg, MSG_ROOM_REACHED);
                    room_reached = true;
                  }
                }

                if (room_reached) {
                  RCLCPP_INFO(node_->get_logger(), "GoToRoom1: All waypoints done, entering scan.");
                  stopBase(pub_base_twist);
                  if (nav_goal_handle_) {
                    nav_action_client_->async_cancel_goal(nav_goal_handle_);
                  }
                  nav_goal_active_ = false;
                  nav_goal_reached_ = false;
                  nav_goal_failed_ = false;
                  ready_to_grasp = false;
                  aligned_x = false;
                  aligned_y = false;
                  arm_height = 0.0;
                  x_adjust = 0.0;
                  y_adjust = 0.0;
                  body_height = 0.0;
                  patrol_step = 0;
                  scan_started_ = false;
                  step_ = MoveToInFrontOfTarget;
                  RCLCPP_INFO(node_->get_logger(), "Transitioning to MoveToInFrontOfTarget");
                } else {
                  // CRITICAL FIX: If not in room after all waypoints, keep trying last waypoint
                  // DO NOT enter scan without Room_reached!
                  RCLCPP_WARN(node_->get_logger(),
                    "GoToRoom1: NOT in room after all waypoints! Retrying last waypoint. DO NOT enter scan!");
                  patrol_step = max_patrol - 1;  // Go back to last waypoint
                  nav_goal_sent_ = false;
                }
                break;
              }
              // Continue to next patrol waypoint
          }
          // ---- Navigation failed (ABORTED) ----
          else if (nav_goal_failed_.load()) {
              RCLCPP_WARN(node_->get_logger(), "GoToRoom1: Navigation to patrol point %d/%d failed (ABORTED). Retrying...",
                         patrol_step + 1, max_patrol);
              nav_goal_failed_ = false;
              nav_goal_sent_ = false;
              // CRITICAL: DO NOT increment patrol_step or enter scan. Keep retrying!
              // Robot is NOT in target room, entering scan would be pointless.
          }
          // ---- Navigation timeout ----
          else if (nav_goal_sent_ &&
                   (node_->now() - nav_goal_send_time_).seconds() > NAV_GOAL_TIMEOUT_SEC) {
              RCLCPP_WARN(node_->get_logger(), "GoToRoom1: Navigation goal timed out (%.0f s). Retrying...",
                         NAV_GOAL_TIMEOUT_SEC);
              if (nav_goal_handle_) {
                nav_action_client_->async_cancel_goal(nav_goal_handle_);
              }
              nav_goal_sent_ = false;
              nav_goal_cancelled_ = true;
              nav_goal_failed_ = false;
              nav_goal_reached_ = false;
              // CRITICAL: DO NOT increment patrol_step or enter scan. Keep retrying!
              // Robot is NOT in target room, entering scan would be pointless.
          }

          break;
        }
        case GoToRoom2:
        {
          if(!room_reached){
            if (!nav_goal_sent_) {
              geometry_msgs::msg::PoseStamped goal_pose;
              if(room_list.size()>1){
                 goal_pose = destLocation(dest_list[dest_list.size() - 1],room_list[1]);
              }else{
                 goal_pose = destLocation(dest_list[dest_list.size() - 1],room_list[0]);
              }
              RCLCPP_INFO(node_->get_logger(), "dest = %s", dest_list[dest_list.size() - 1].c_str());
              RCLCPP_INFO(node_->get_logger(), "room = %s", room_list[0].c_str());
              goal_pose.header.frame_id = "map";
              goal_pose.header.stamp = node_->now();

              std::vector<double> positions { 0.2, -0.3, 0.0, -1.57, 0.0 };
              rclcpp::Duration duration = rclcpp::Duration::from_seconds(1.0);
              moveArm(pub_arm_trajectory, positions, duration);

              sendNavGoal(goal_pose);
              nav_goal_sent_ = true;
            }

            if(nav_goal_reached_.load()){
              nav_goal_reached_ = false;
              nav_goal_sent_ = false;
              RCLCPP_INFO(node_->get_logger(), "Destination Reached");
              room_reached = true;
            } else if (nav_goal_failed_.load()) {
              RCLCPP_WARN(node_->get_logger(), "Navigation to destination failed, retrying...");
              nav_goal_failed_ = false;
              nav_goal_sent_ = false;
            } else if (nav_goal_sent_ &&
                       (node_->now() - nav_goal_send_time_).seconds() > NAV_GOAL_TIMEOUT_SEC) {
              RCLCPP_WARN(node_->get_logger(), "Navigation goal timed out, retrying...");
              if (nav_goal_handle_) {
                nav_action_client_->async_cancel_goal(nav_goal_handle_);
              }
              nav_goal_sent_ = false;
              nav_goal_failed_ = false;
            }
          }else{
            std::vector<double> positions { 0.2, -0.8, 0.0, -1.0, 0.0 };
            rclcpp::Duration duration = rclcpp::Duration::from_seconds(1.0);
            moveArm(pub_arm_trajectory, positions, duration);
            waiting_start_time = node_->now();
            extend_before_release = false;
            step_ = Release;
          }
          break;
        }
        case MoveToInFrontOfTarget:
        {
          if(nav_goal_active_.load() || nav_goal_sent_) {
            RCLCPP_WARN(node_->get_logger(), "Stale navigation detected in MoveToInFrontOfTarget, cancelling...");
            nav_action_client_->async_cancel_all_goals();
            stopBase(pub_base_twist);
            nav_goal_active_ = false;
            nav_goal_sent_ = false;
            break;
          }

          // Phase 1: Initialize room scan on first entry
          if (!scan_started_) {
            RCLCPP_INFO(node_->get_logger(), "MoveToInFrontOfTarget: Starting systematic room scan...");
            scan_started_ = true;
            scan_total_rotation_ = 0.0;
            scan_start_time_ = node_->now();
            aligned_x = false;
            aligned_y = false;
            // Raise arm to scanning position
            std::vector<double> scan_positions { 0.2, 0.0, 0.0, -1.57, 0.0 };
            rclcpp::Duration scan_duration = rclcpp::Duration::from_seconds(1.0);
            moveArm(pub_arm_trajectory, scan_positions, scan_duration);
          }

          // Phase 1: Systematic room scan when object not detected
          if((node_->now() - last_detect_) < rclcpp::Duration::from_seconds(1.0)){
            // Object found during scan!
            RCLCPP_INFO(node_->get_logger(), "MoveToInFrontOfTarget: Object detected during room scan!");
            scan_started_ = false;  // Reset for next time we enter

            // Track rotation during alignment
            if(x_det < 240 - 25*tol_multiplier){
              x_adjust = 0.03;
              aligned_x = false;
            }else if (x_det > 240 + 25*tol_multiplier){
              x_adjust = -0.03;
               aligned_x = false;
            }else{
              x_adjust = 0.0;
              aligned_x = true;
            }

            if(y_det < 360 - 25*tol_multiplier){
              arm_height+= 0.004;
              if(arm_height>0.0){
                arm_height = 0.0;
              }
              y_adjust = 0.01;
              aligned_y = false;
            }else if (y_det > 360 + 25*tol_multiplier){
              arm_height-= 0.004;
              y_adjust = -0.01;
              aligned_y = false;
            }else{
              y_adjust = 0.0;
              aligned_y = true;
            }

            std::vector<double> positions { 0.2, arm_height, 0.0, -1.57 - arm_height*0.4, 0.0 };
            rclcpp::Duration duration = rclcpp::Duration::from_seconds(1.0);
            moveArm(pub_arm_trajectory, positions, duration);

            moveBase(pub_base_twist,y_adjust, x_adjust*0.5 ,x_adjust);

            if(aligned_x && aligned_y){
              std::cout << "ready = " << std::to_string(ready_to_grasp) << std::endl;
              if(ready_to_grasp == false){
                moveBase(pub_base_twist,0.02,0.0,0.0);
              }else{
                std::cout << "Gripping" << std::endl;
                moveBase(pub_base_twist,0.0,0.0,0.0);
                waiting_start_time = node_->now();
                step_ = Grasp;
              }
            }
          } else {
            // Phase 1: Systematic room scan - rotate and track total rotation
            aligned_x = false;
            aligned_y = false;
            double rotation_delta = SCAN_ROTATION_SPEED * 0.1;  // 0.1s per loop iteration
            scan_total_rotation_ += rotation_delta;

            // Rotate base to scan room
            moveBase(pub_base_twist, 0.0, 0.0, SCAN_ROTATION_SPEED);

            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
              "MoveToInFrontOfTarget: Systematic room scan in progress... (%.1f / %.1f rad, %.1f deg)",
              scan_total_rotation_, MAX_SCAN_ROTATION,
              scan_total_rotation_ * 180.0 / M_PI);

            // Phase 1: Check if full scan completed without finding object
            if (scan_total_rotation_ >= MAX_SCAN_ROTATION) {
              RCLCPP_WARN(node_->get_logger(),
                "MoveToInFrontOfTarget: Full room scan complete (360 deg), object NOT found. Sending Object_not_found.");
              scan_started_ = false;
              // Stop base before sending message
              stopBase(pub_base_twist);
              sendMessage(pub_msg, "Object_not_found");
              // Stay in this state, allow retry or next patrol point
              // Could transition back to GoToRoom1 for next patrol waypoint if available
            }
          }

          break;
        }

        case Grasp:
        {
          operateHand(pub_gripper_trajectory, true);

          if((node_->now() - waiting_start_time) > rclcpp::Duration::from_seconds(8.0))
          {
            sendMessage(pub_msg, MSG_OBJECT_GRASPED);
            room_reached = false;
            found_object = false;
            nav_goal_sent_ = false;
            step_= GoToRoom2;
          }

          break;
        }
        case Release:
        {
          if(!extend_before_release){
            if((node_->now() - waiting_start_time) > rclcpp::Duration::from_seconds(4.0))
              {
                operateHand(pub_gripper_trajectory, false);
                waiting_start_time = node_->now();
                extend_before_release = true;
              }
          }else{
            if((node_->now() - waiting_start_time) > rclcpp::Duration::from_seconds(8.0))
            {
              sendMessage(pub_msg, MSG_TASK_FINISHED);
              step_= TaskFinished;
            }
          }

          break;
        }
        case ComeBack:
        {

          break;
        }
        case TaskFinished:
        {
          if(is_finished_)
          {
            RCLCPP_INFO(node_->get_logger(), "Task finished!");
            step_ = Initialize;
          }

          break;
        }
      }

      rclcpp::spin_some(node_);

      loop_rate.sleep();
    }

    return EXIT_SUCCESS;
  }
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  HandymanSample handyman_sample;
  int ret = handyman_sample.run(argc, argv);

  rclcpp::shutdown();
  return ret;
}
