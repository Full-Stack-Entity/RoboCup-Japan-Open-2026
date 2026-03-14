#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib_msgs/GoalStatus.h>
#include <nav_msgs/OccupancyGrid.h>
#include <handyman/HandymanMsg.h>
#include <nodelet/nodelet.h>
// #include "handyman/infoExtractor.h"
#include <iostream>
#include <cstring>
#include <vector>
#include <thread>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include <algorithm>
#include <cctype>
#include <map>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <mutex>


// Utility function to convert a string to lowercase
std::string lower(const std::string &str) {
  std::string lower_str = str;
  std::transform(lower_str.begin(), lower_str.end(), lower_str.begin(), ::tolower);
  return lower_str;
}


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class LoadMapManager {
private:
    ros::NodeHandle nh_;
    std::string current_environment_;
    std::map<std::string, std::string> environment_to_map_;
    std::map<std::string, geometry_msgs::Pose> environment_to_initial_pose_;
    
    // 节点管理相关
    ros::ServiceClient map_server_client_;
    ros::ServiceClient amcl_client_;
    ros::ServiceClient global_costmap_client_;
    ros::ServiceClient local_costmap_client_;
    ros::Timer status_check_timer_;
    
    // move_base状态监控相关
    ros::Subscriber move_base_feedback_sub_;
    bool move_base_active_;
    bool navigation_cancelled_;
    ros::Time last_feedback_time_;
    std::mutex state_mutex_;
    
public:
    LoadMapManager(ros::NodeHandle& nh) : nh_(nh), current_environment_("None"), move_base_active_(false), navigation_cancelled_(false) {
        // 初始化服务客户端
        map_server_client_ = nh_.serviceClient<std_srvs::Empty>("/map_server/reload");
        amcl_client_ = nh_.serviceClient<std_srvs::Empty>("/amcl/set_parameters");
        global_costmap_client_ = nh_.serviceClient<std_srvs::Empty>("/move_base/global_costmap/clear_costmaps");
        local_costmap_client_ = nh_.serviceClient<std_srvs::Empty>("/move_base/local_costmap/clear_costmaps");
        
        // 初始化move_base状态监控
        move_base_feedback_sub_ = nh_.subscribe("/move_base/feedback", 1, &LoadMapManager::moveBaseFeedbackCallback, this);
        
        // 注册默认环境配置
        registerDefaultEnvironments();
        
    }
    
    // 主要功能接口
    // move_base反馈回调函数
    void moveBaseFeedbackCallback(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& feedback) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        last_feedback_time_ = ros::Time::now();
        move_base_active_ = true;
    }
    
    // 检查move_base是否正在执行导航任务
    bool isMoveBaseActive() {
        std::lock_guard<std::mutex> lock(state_mutex_);
        
        // 检查最近是否有反馈
        if (last_feedback_time_.isValid()) {
            ros::Duration time_since_feedback = ros::Time::now() - last_feedback_time_;
            if (time_since_feedback > ros::Duration(2.0)) {
                move_base_active_ = false;
            }
        }
        
        // 检查move_base action客户端状态
        try {
            MoveBaseClient ac("move_base", true);
            if (ac.isServerConnected()) {
                actionlib::SimpleClientGoalState state = ac.getState();
                if (state == actionlib::SimpleClientGoalState::ACTIVE || 
                    state == actionlib::SimpleClientGoalState::PENDING) {
                    move_base_active_ = true;
                }
            }
        } catch (...) {
            // 如果无法连接，假设move_base不活跃
            move_base_active_ = false;
        }
        
        return move_base_active_;
    }
    
    // 取消当前导航目标
    bool cancelCurrentNavigationGoal() {
        ROS_INFO("Cancelling current navigation goal...");
        
        try {
            MoveBaseClient ac("move_base", true);
            if (ac.isServerConnected()) {
                ac.cancelAllGoals();
                navigation_cancelled_ = true;
                ROS_INFO("Navigation goal cancelled successfully");
                return true;
            } else {
                ROS_WARN("Move_base action server not connected, cannot cancel goal");
                return false;
            }
        } catch (const std::exception& e) {
            ROS_ERROR("Exception while cancelling navigation goal: %s", e.what());
            return false;
        }
    }
    
    // 等待导航停止
    bool waitForNavigationStop(double timeout_seconds = 10.0) {
        ROS_INFO("Waiting for navigation to stop...");
        
        ros::Time start_time = ros::Time::now();
        ros::Rate check_rate(10.0); // 10Hz检查频率
        
        while (ros::ok()) {
            if (!isMoveBaseActive()) {
                ROS_INFO("Navigation has stopped");
                return true;
            }
            
            if ((ros::Time::now() - start_time).toSec() > timeout_seconds) {
                ROS_WARN("Timeout waiting for navigation to stop");
                return false;
            }
            
            check_rate.sleep();
            ros::spinOnce();
        }
        
        return false;
    }
    
    bool switchEnvironment(const std::string& environment) {
        ROS_INFO("Switching to environment: %s", environment.c_str());
        
        // 添加环境格式映射
        std::string mapped_env = mapEnvironmentFormat(environment);
        
        // 检查环境是否已注册
        if (environment_to_map_.find(mapped_env) == environment_to_map_.end()) {
            ROS_ERROR("Environment %s (mapped to %s) not registered", environment.c_str(), mapped_env.c_str());
            return false;
        }
        
        // 优雅停止导航系统
        if (!gracefulStopNavigation()) {
            ROS_ERROR("Failed to gracefully stop navigation");
            return false;
        }
        
        // 启动新地图服务器
        std::string map_file = environment_to_map_[mapped_env];
        if (!startMapServer(map_file)) {
            ROS_ERROR("Failed to start map server with map: %s", map_file.c_str());
            return false;
        }
        
        // 等待地图服务就绪（基础检测）
        if (!waitForMapService()) {
            ROS_ERROR("Map service not ready after switching");
            return false;
        }
        
        // 环境切换后的地图检测（带时间戳验证）
        if (!waitForMapAfterEnvironmentSwitch(mapped_env)) {
            ROS_ERROR("Fresh map not available after environment switch");
            return false;
        }
        
        // 启动AMCL
        geometry_msgs::Pose initial_pose = environment_to_initial_pose_[mapped_env];
        if (!startAMCL(initial_pose)) {
            ROS_ERROR("Failed to start AMCL");
            return false;
        }
        
        // 启动move_base
        if (!startMoveBase()) {
            ROS_ERROR("Failed to start move_base");
            return false;
        }
        
        // 强制清除代价地图
        if (!forceClearCostmaps()) {
            ROS_WARN("Failed to clear costmaps, continuing anyway...");
        }
        
        // 等待系统就绪
        if (!waitForSystemReady()) {
            ROS_ERROR("System not ready after switching");
            return false;
        }
        
        current_environment_ = mapped_env;
        ROS_INFO("Successfully switched to environment: %s (original: %s)", mapped_env.c_str(), environment.c_str());
        return true;
    }
    
    bool loadMap(const std::string& map_file) {
        ROS_INFO("Loading map: %s", map_file.c_str());
        return startMapServer(map_file);
    }
    
    bool resetAMCL(const geometry_msgs::Pose& initial_pose) {
        ROS_INFO("Resetting AMCL with initial pose");
        return startAMCL(initial_pose);
    }
    
    // 状态检查
    bool isMapServerReady() {
        return ros::service::exists("/static_map", false);
    }
    
    bool isAMCLReady() {
        return ros::service::exists("/amcl/set_parameters", false);
    }
    
        
    // 环境配置
    void registerEnvironment(const std::string& env, 
                           const std::string& map_file,
                           const geometry_msgs::Pose& initial_pose) {
        environment_to_map_[env] = map_file;
        environment_to_initial_pose_[env] = initial_pose;
        ROS_INFO("Registered environment: %s -> %s", env.c_str(), map_file.c_str());
    }
    
    // 环境格式映射函数
    std::string mapEnvironmentFormat(const std::string& input_env) {
        ROS_INFO("Mapping environment format: %s", input_env.c_str());
        
        // 如果输入格式是 "Layout2019HM01"，映射为 "2019HM01"
        if (input_env.find("Layout") == 0) {
            std::string mapped_env = input_env.substr(6); // 去掉 "Layout" 前缀
            ROS_INFO("Environment format mapped: %s -> %s", input_env.c_str(), mapped_env.c_str());
            return mapped_env;
        }
        
        // 如果输入格式已经是 "2019HM01"，直接返回
        ROS_INFO("Environment format already correct: %s", input_env.c_str());
        return input_env;
    }
    
    std::string getCurrentEnvironment() const {
        return current_environment_;
    }
    
private:
    void registerDefaultEnvironments() {
        // 设置默认的初始位置
        geometry_msgs::Pose default_pose;
        default_pose.position.x = 0.0;
        default_pose.position.y = 0.0;
        default_pose.position.z = 0.0;
        default_pose.orientation.x = 0.0;
        default_pose.orientation.y = 0.0;
        default_pose.orientation.z = 0.0;
        default_pose.orientation.w = 1.0;
        
        // 注册默认环境
        registerEnvironment("2019HM01", "/home/robot/catkin_ws/src/handyman-ros/maps/2019HM01.yaml", default_pose);
        registerEnvironment("2019HM02", "/home/robot/catkin_ws/src/handyman-ros/maps/2019HM02.yaml", default_pose);
        registerEnvironment("2020HM01", "/home/robot/catkin_ws/src/handyman-ros/maps/2020HM01.yaml", default_pose);
        registerEnvironment("2021HM01", "/home/robot/catkin_ws/src/handyman-ros/maps/2021HM01.yaml", default_pose);
    }
    
    bool stopMapServer() {
        ROS_INFO("Stopping all map server instances...");
        
        // 终止所有map_server相关进程
        system("pkill -f \"map_server\" 2>/dev/null || true");
        
        // 等待进程完全终止
        ros::Duration(1.5).sleep();
        
        // 验证所有map_server进程是否都已终止
        int result = system("pgrep -f \"map_server\" > /dev/null 2>&1");
        if (result == 0) {
            ROS_WARN("Some map_server processes are still running, forcing cleanup...");
            system("pkill -9 -f \"map_server\" 2>/dev/null || true");
            ros::Duration(0.5).sleep();
        }
        
        ROS_INFO("All map server instances stopped");
        return true;
    }
    
    
    bool startMapServer(const std::string& map_file) {
        ROS_INFO("Starting map server with map: %s", map_file.c_str());
        
        // 确保之前的map_server都已终止
        stopMapServer();
        
        // 使用固定的节点名称启动map_server
        std::string cmd = "rosrun map_server map_server _name:=map_server " + map_file + " &";
        int result = system(cmd.c_str());
        
        if (result != 0) {
            ROS_ERROR("Failed to start map server");
            return false;
        }
        
        // 等待map_server启动
        ros::Duration(1.0).sleep();
        
        // 验证map_server是否正常运行
        int check_result = system("rosnode info /map_server > /dev/null 2>&1");
        if (check_result != 0) {
            ROS_ERROR("Map server node not responding");
            return false;
        }
        
        // 等待地图服务启动
        bool map_ready = waitForMapService();
        if (!map_ready) {
            ROS_ERROR("Map service not ready after starting map server");
            return false;
        }
        
        ROS_INFO("Map server started successfully with fixed name: /map_server");
        return true;
    }
    
    bool stopAMCL() {
        ROS_INFO("Stopping AMCL...");
        system("rosnode kill /amcl 2>/dev/null || true");
        ros::Duration(1.0).sleep();
        return true;
    }
    
    // 优雅停止导航系统
    bool gracefulStopNavigation() {
        ROS_INFO("Gracefully stopping navigation system...");
        
        // 步骤1: 检查move_base状态
        if (isMoveBaseActive()) {
            ROS_INFO("Move_base is active, cancelling current goal...");
            
            // 步骤2: 取消当前导航目标
            if (!cancelCurrentNavigationGoal()) {
                ROS_WARN("Failed to cancel navigation goal, continuing with shutdown");
            }
            
            // 步骤3: 等待导航停止
            if (!waitForNavigationStop(15.0)) {
                ROS_WARN("Navigation did not stop gracefully, forcing shutdown");
            }
        } else {
            ROS_INFO("Move_base is not active, proceeding with shutdown");
        }
        
        // 步骤4: 按正确顺序停止节点
        ROS_INFO("Stopping navigation nodes in sequence...");
        
        // 先停止move_base (防止继续发送速度命令)
        if (!stopMoveBase()) {
            ROS_WARN("Failed to stop move_base, continuing anyway");
        }
        
        // 等待move_base完全停止
        ros::Duration(2.0).sleep();
        
        // 再停止AMCL
        if (!stopAMCL()) {
            ROS_WARN("Failed to stop AMCL, continuing anyway");
        }
        
        // 最后停止map_server
        if (!stopMapServer()) {
            ROS_WARN("Failed to stop map server, continuing anyway");
        }
        
        // 重置状态
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            move_base_active_ = false;
            navigation_cancelled_ = false;
        }
        
        ROS_INFO("Navigation system stopped gracefully");
        return true;
    }
    
    bool stopMoveBase() {
        ROS_INFO("Stopping move_base...");
        system("rosnode kill /move_base 2>/dev/null || true");
        ros::Duration(1.0).sleep();
        return true;
    }
    
    bool startAMCL(const geometry_msgs::Pose& initial_pose) {
        ROS_INFO("Starting AMCL...");
        
        // 使用launch文件启动AMCL
        int result = system("roslaunch handyman amcl.launch &");
        
        if (result != 0) {
            ROS_ERROR("Failed to start AMCL");
            return false;
        }
        
        // 等待AMCL服务启动
        if (!ros::service::waitForService("/amcl/set_parameters", ros::Duration(5.0))) {
            ROS_ERROR("AMCL service not available");
            return false;
        }

        // 新增：发布初始位姿给AMCL
        if (!publishInitialPose(initial_pose)) {
            ROS_WARN("Failed to publish initial pose, continuing anyway");
        }
        
        // 设置初始位置（这里需要根据实际AMCL服务进行调整）
        ROS_INFO("AMCL started successfully");
        return true;
    }


    // 发布初始位姿给AMCL
    bool publishInitialPose(const geometry_msgs::Pose& initial_pose) {
        ROS_INFO("Publishing initial pose to AMCL...");
        
        try {
            // 创建发布者
            ros::Publisher initial_pose_pub = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
            
            // 等待发布者就绪
            ros::Duration(0.5).sleep();
            
            // 创建初始位姿消息
            geometry_msgs::PoseWithCovarianceStamped initial_pose_msg;
            initial_pose_msg.header.stamp = ros::Time::now();
            initial_pose_msg.header.frame_id = "map";
            initial_pose_msg.pose.pose = initial_pose;
            
            // 设置协方差矩阵（6x6，只有位置和旋转的不确定性）
            initial_pose_msg.pose.covariance = {
                0.05, 0.0, 0.0, 0.0, 0.0, 0.0,    // x方差
                0.0, 0.05, 0.0, 0.0, 0.0, 0.0,    // y方差
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,    // z方差（设为0）
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,    // 旋转x方差（设为0）
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,    // 旋转y方差（设为0）
                0.0, 0.0, 0.0, 0.0, 0.0, 0.01   // 旋转z方差
            };
            
            // 发布初始位姿（发布3次确保AMCL接收到）
            for (int i = 0; i < 3; i++) {
                initial_pose_pub.publish(initial_pose_msg);
                ros::Duration(0.1).sleep();
            }
            
            ROS_INFO("Initial pose published successfully");
            return true;
            
        } catch (const std::exception& ex) {
            ROS_ERROR("Exception while publishing initial pose: %s", ex.what());
            return false;
        } catch (...) {
            ROS_ERROR("Unknown exception while publishing initial pose");
            return false;
        }
    }
    
    bool startMoveBase() {
        ROS_INFO("Starting move_base...");
        
        // 使用launch文件启动move_base
        int result = system("roslaunch handyman move_base.launch &");
        
        if (result != 0) {
            ROS_ERROR("Failed to start move_base");
            return false;
        }
        
        // 等待move_base服务启动
        if (!ros::service::waitForService("/move_base/make_plan", ros::Duration(5.0))) {
            ROS_ERROR("move_base service not available");
            return false;
        }
        
        ROS_INFO("move_base started successfully");
        return true;
    }
    
    // 强制清除代价地图
    bool forceClearCostmaps() {
        ROS_INFO("Force clearing costmaps...");
        
        std_srvs::Empty srv;
        bool global_cleared = false;
        bool local_cleared = false;
        
        // 清除全局代价地图
        if (global_costmap_client_.exists()) {
            if (global_costmap_client_.call(srv)) {
                ROS_INFO("Global costmap cleared successfully");
                global_cleared = true;
            } else {
                ROS_ERROR("Failed to clear global costmap");
            }
        } else {
            ROS_WARN("Global costmap service not available");
        }
        
        // 清除局部代价地图
        if (local_costmap_client_.exists()) {
            if (local_costmap_client_.call(srv)) {
                ROS_INFO("Local costmap cleared successfully");
                local_cleared = true;
            } else {
                ROS_ERROR("Failed to clear local costmap");
            }
        } else {
            ROS_WARN("Local costmap service not available");
        }
        
        // 等待一下确保清除完成
        ros::Duration(1.0).sleep();
        
        // 再次清除以确保完全清除
        if (global_costmap_client_.exists()) {
            global_costmap_client_.call(srv);
        }
        if (local_costmap_client_.exists()) {
            local_costmap_client_.call(srv);
        }
        
        ROS_INFO("Costmap clearing completed - Global: %s, Local: %s", 
                 global_cleared ? "SUCCESS" : "FAILED", 
                 local_cleared ? "SUCCESS" : "FAILED");
        
        return global_cleared && local_cleared;
    }
    
    
    bool waitForMapService() {
        ROS_INFO("Waiting for map topic with valid data...");
        
        const double max_wait_time = 30.0; // 30秒超时
        const double max_freshness_age = 5.0; // 5秒内算新鲜
        ros::Time start_time = ros::Time::now();
        
        while (ros::Time::now() - start_time < ros::Duration(max_wait_time)) {
            // 尝试获取地图消息
            nav_msgs::OccupancyGrid::ConstPtr map_msg = 
                ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/map", ros::Duration(2.0));
            
            if (map_msg) {
                ros::Time now = ros::Time::now();
                ros::Duration age = now - map_msg->header.stamp;
                
                // 检查地图数据有效性
                if (map_msg->info.width > 0 && map_msg->info.height > 0 && !map_msg->data.empty()) {
                    // 检查时间戳新鲜度
                    if (age < ros::Duration(max_freshness_age)) {
                        ROS_INFO("✅ Valid fresh map received! Size: %dx%d, Age: %.2f seconds", 
                                map_msg->info.width, map_msg->info.height, age.toSec());
                        return true;
                    } else {
                        ROS_WARN("Map received but not fresh enough (age: %.2f seconds, max allowed: %.2f)", 
                                age.toSec(), max_freshness_age);
                    }
                } else {
                    ROS_WARN("Map received but data invalid (size: %dx%d, data length: %zu)", 
                            map_msg->info.width, map_msg->info.height, map_msg->data.size());
                }
            } else {
                ROS_DEBUG("No map message received in this attempt");
            }
            
            // 短暂等待后重试
            ros::Duration(0.5).sleep();
        }
        
        ROS_ERROR("Failed to get valid fresh map within %.2f seconds", max_wait_time);
        return false;
    }
    
    // 环境切换后的地图检测
    bool waitForMapAfterEnvironmentSwitch(const std::string& environment_name) {
        ROS_INFO("Waiting for fresh map after switching to environment: %s", environment_name.c_str());
        
        const double max_wait_time = 30.0; // 30秒超时
        const double max_freshness_age = 10.0; // 10秒内算新鲜（放宽要求）
        ros::Time switch_start_time = ros::Time::now();
        
        while (ros::Time::now() - switch_start_time < ros::Duration(max_wait_time)) {
            // 尝试获取地图消息
            nav_msgs::OccupancyGrid::ConstPtr map_msg = 
                ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/map", ros::Duration(3.0));
            
            if (map_msg) {
                ros::Time now = ros::Time::now();
                ros::Duration age = now - map_msg->header.stamp;
                
                // 检查地图数据有效性
                if (map_msg->info.width > 0 && map_msg->info.height > 0 && !map_msg->data.empty()) {
                    // 检查是否在环境切换后发布
                    if (map_msg->header.stamp > switch_start_time - ros::Duration(2.0)) {
                        // 检查新鲜度
                        if (age < ros::Duration(max_freshness_age)) {
                            ROS_INFO("✅ Fresh map received after environment switch! Size: %dx%d, Age: %.2f seconds", 
                                    map_msg->info.width, map_msg->info.height, age.toSec());
                            return true;
                        } else {
                            ROS_WARN("Map received after switch but not fresh enough (age: %.2f seconds)", age.toSec());
                        }
                    } else {
                        ROS_WARN("Map received but timestamp (%f) is before switch time (%f)", 
                                map_msg->header.stamp.toSec(), switch_start_time.toSec());
                    }
                } else {
                    ROS_WARN("Map received but data invalid (size: %dx%d, data length: %zu)", 
                            map_msg->info.width, map_msg->info.height, map_msg->data.size());
                }
            } else {
                ROS_DEBUG("No map message received in this attempt for environment: %s", environment_name.c_str());
            }
            
            // 短暂等待后重试
            ros::Duration(0.5).sleep();
        }
        
        ROS_ERROR("Failed to get fresh map for environment %s within %.2f seconds", 
                 environment_name.c_str(), max_wait_time);
        return false;
    }
    
    // 等待系统完全就绪
    bool waitForSystemReady() {
        ROS_INFO("Waiting for navigation system to be ready...");
        
        // 等待所有关键节点启动
        ros::Time start_time = ros::Time::now();
        double timeout = 20.0; // 20秒超时
        
        while (ros::ok()) {
            bool all_ready = true;
            
            // 检查map_server
            if (!ros::service::exists("/static_map", false)) {
                ROS_WARN("Map server not ready");
                all_ready = false;
            }
            
            // 检查AMCL
            if (!ros::service::exists("/amcl/set_parameters", false)) {
                ROS_WARN("AMCL not ready");
                all_ready = false;
            }
            
            // 检查move_base
            if (!ros::service::exists("/move_base/make_plan", false)) {
                ROS_WARN("Move_base not ready");
                all_ready = false;
            }
            
            // 检查TF树是否稳定
            if (!checkTFReady()) {
                ROS_WARN("TF tree not ready");
                all_ready = false;
            }
            
            if (all_ready) {
                ROS_INFO("All navigation services are ready");
                return true;
            }
            
            if ((ros::Time::now() - start_time).toSec() > timeout) {
                ROS_ERROR("Timeout waiting for system to be ready");
                return false;
            }
            
            ros::Duration(0.5).sleep();
            ros::spinOnce();
        }
        
        return false;
    }
    
    // 检查TF树是否就绪
    bool checkTFReady() {
        try {
            // 使用静态的TransformListener，避免重复创建
            static tf::TransformListener listener;
            std::string error_msg;
            
            // 给TransformListener一些时间来接收TF消息
            ros::Duration(0.1).sleep();
            
            // 尝试获取关键TF变换
            if (listener.canTransform("map", "base_footprint", ros::Time::now() - ros::Duration(1.0), &error_msg)) {
                ROS_DEBUG("TF transform map->base_footprint is available");
                return true;
            }
            
            ROS_DEBUG("Waiting for map->base_footprint transform: %s", error_msg.c_str());
            
            // 如果map变换不可用，检查odom变换作为备用
            if (listener.canTransform("odom", "base_footprint", ros::Time::now() - ros::Duration(1.0), &error_msg)) {
                ROS_DEBUG("TF transform odom->base_footprint is available (fallback)");
                return true;
            }
            
            ROS_DEBUG("Waiting for odom->base_footprint transform: %s", error_msg.c_str());
            return false;
            
        } catch (const tf::TransformException& ex) {
            ROS_DEBUG("TF transform exception: %s", ex.what());
            return false;
        } catch (const std::exception& ex) {
            ROS_DEBUG("TF check exception: %s", ex.what());
            return false;
        } catch (...) {
            ROS_DEBUG("Unknown TF check exception");
            return false;
        }
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
  
  // LoadMapManager instance
  std::unique_ptr<LoadMapManager> map_manager_;

  trajectory_msgs::JointTrajectory arm_joint_trajectory_;

  int step_;
  int patrol_step;
  int max_patrol;
  bool room_reached;
  bool found_object,found_dest;
  double init_yaw;
  double x_det, y_det;
  bool ready_to_grasp;
  double arm_height,body_height;
  bool aligned_x, aligned_y;
  geometry_msgs::PoseStamped object_pose;
  geometry_msgs::Pose dest_pose;
  bool extend_before_release;

  std::string instruction_msg_;

  bool is_started_;
  bool is_finished_;
  bool is_failed_;
  ros::Time last_detect;
  double tol_multiplier;
  double x_adjust,y_adjust;

  std::vector<std::string> object_list,room_list,dest_list;
  
  // 消息去重机制
  ros::Time last_are_you_ready_time_;
  const double MESSAGE_DEDUPE_INTERVAL = 0.1; // 100ms去重间隔

  void init(ros::NodeHandle& nh)
  {
    // Initialize LoadMapManager
    map_manager_ = std::make_unique<LoadMapManager>(nh);
    
    // Arm Joint Trajectory
    std::vector<std::string> arm_joint_names {"arm_lift_joint", "arm_flex_joint", "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"};

    trajectory_msgs::JointTrajectoryPoint arm_joint_point;

    arm_joint_trajectory_.joint_names = arm_joint_names;
    arm_joint_trajectory_.points.push_back(arm_joint_point);

    step_ = Initialize;

    reset();
    last_are_you_ready_time_ = ros::Time(0);
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
    std::vector<double> arm_positions { 0.0, 0.0, 0.0, 0.0, 0.0 };
    arm_joint_trajectory_.points[0].positions = arm_positions;
  }


  void visionCallback(const geometry_msgs::PoseStamped::ConstPtr& pose)
  {
    // Process the pose as needed
    object_pose = *pose; // Assuming object_pose is used elsewhere
  }
  
  
 

  void graspvisioncallback(const std_msgs::Int32MultiArray::ConstPtr& posearray){

    if(!found_object){
      found_object = true;
    }
    
    x_det = posearray->data[0];
    y_det = posearray->data[1];
    std::cout << "X = " << x_det << " Y = " << y_det << " W = " << posearray->data[2] << " H = " << posearray->data[3] << std::endl;
    if(posearray->data[2] >= 450){
      ready_to_grasp = true;
      tol_multiplier = 1.5;
      // std::cout << "Ready to Grasp!"  << std::endl;
    }else{
      ready_to_grasp = false;
      tol_multiplier = 1.0;
    }
    last_detect = ros::Time::now(); 
  }

  void messageCallback(const handyman::HandymanMsg::ConstPtr& message)
  {
    ROS_INFO("Subscribe message:%s, %s", message->message.c_str(), message->detail.c_str());

    if(message->message.c_str()==MSG_ENVIRONMENT)
    {
      ENVIRONMENT = message->detail.c_str();
    }
    ROS_INFO("######The environment is %s", ENVIRONMENT.c_str());
    if(ENVIRONMENT != "None"){
      if(message->message.c_str()==MSG_ARE_YOU_READY)
      {
        if(step_==Ready)
        {
          is_started_ = true;
        }else if(step_ != Initialize){
          // 关键状态：允许无条件中断（导航和抓取后的关键操作）
          if(step_ == GoToRoom2 || step_ == GoToRoom1 || step_ == MoveToInFrontOfTarget || step_ == Grasp || step_ == Release)
          {
            ROS_WARN("Critical state interrupted by Are_you_ready? message. Current step: %d", step_);
            step_ = Initialize;
          }
          // WaitForInstruction状态下忽略Are_you_ready?消息（防止重复中断循环）
          else if(step_ == WaitForInstruction)
          {
            ROS_DEBUG("Ignoring Are_you_ready? message in WaitForInstruction state - robot is ready for instructions");
          }
          // 其他状态：使用条件中断（防止重复消息干扰）
          else if(step_ == ComeBack || step_ == TaskFinished)
          {
            ROS_INFO("Conditional interrupt in state: %d", step_);
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
      // step_ = Initialize;
    }
    if(message->message.c_str()==MSG_TASK_FAILED)
    {
      is_failed_ = true;
      // step_ = Initialize;
    }
    if(message->message.c_str()==MSG_MISSION_COMPLETE)
    {
      exit(EXIT_SUCCESS);
    }
  }

  void sendMessage(ros::Publisher &publisher, const std::string &message)
  {
    ROS_INFO("Send message:%s", message.c_str());

    handyman::HandymanMsg handyman_msg;
    handyman_msg.message = message;
    publisher.publish(handyman_msg);
  }

  void tokenize(std::string const &str, const char delim, std::vector<std::string> &out) 
  { 
    // construct a stream from the string 
    std::stringstream ss(str); 
 
    std::string s; 
    while (std::getline(ss, s, delim)) { 
        out.push_back(s); 
    } 
  } 

  std::vector<std::string> extractInfo(std::string msg,std::vector<std::string>& room_array,std::vector<std::string>& object_array,std::vector<std::string>& dest_array){ 
    std::vector<std::string> info;
    tokenize(msg,' ',info);

    for(int i=0; i<info.size();i++){
      
    for(int j=0;j<rooms.size();j++){
        size_t index = info[i].find(rooms[j]);
        if (index != std::string::npos){
            // info.push_back(rooms[j]);
            room_array.push_back(rooms[j]);
        }
    }

    for(int j=0;j<objects.size();j++){
        size_t index = info[i].find(objects[j]);
        if (index != std::string::npos){
            // info.push_back(objects[j]);
            object_array.push_back(objects[j]);
        }
    }

    for(int j=0;j<dests.size();j++){
        size_t index = info[i].find(dests[j]);
        if (index != std::string::npos){
            // info.push_back(dests[j]);
            dest_array.push_back(dests[j]);
        }
    }

    }

    return info;
  }

  move_base_msgs::MoveBaseGoal destLocation(std::string dest,std::string room){
    
    ROS_INFO("destLocation called with dest=%s, room=%s, ENVIRONMENT=%s",
      dest.c_str(), room.c_str(), ENVIRONMENT.c_str());

    tf2::Quaternion target;

    // 2019HM01
    move_base_msgs::MoveBaseGoal LayoutA_white_side_table_lobby;
    LayoutA_white_side_table_lobby.target_pose.pose.position.x=1.3;
    LayoutA_white_side_table_lobby.target_pose.pose.position.y=-6.1;
    target.setRPY(0,0,-1.57);
    target.normalize();
    ROS_INFO("Quaternion: w=%f, x=%f, y=%f, z=%f", target.w(), target.x(), target.y(), target.z());
    LayoutA_white_side_table_lobby.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutA_corner_sofa;
    LayoutA_corner_sofa.target_pose.pose.position.x=-0.15;
    LayoutA_corner_sofa.target_pose.pose.position.y=-6.0;
    target.setRPY(0,0,-1.57);
    target.normalize();
    LayoutA_corner_sofa.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutA_arm_chair;
    LayoutA_arm_chair.target_pose.pose.position.x=2.08;
    LayoutA_arm_chair.target_pose.pose.position.y=-3.2916;
    target.setRPY(0,0,1.57);
    target.normalize();
    LayoutA_arm_chair.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutA_trash_box_for_recycle;
    LayoutA_trash_box_for_recycle.target_pose.pose.position.x=-0.5;
    LayoutA_trash_box_for_recycle.target_pose.pose.position.y=1.28;
    target.setRPY(0,0,3.14);
    target.normalize();
    LayoutA_trash_box_for_recycle.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutA_trash_box_for_burnable;
    LayoutA_trash_box_for_burnable.target_pose.pose.position.x=-0.5;
    LayoutA_trash_box_for_burnable.target_pose.pose.position.y=2.4;
    target.setRPY(0,0,3.14);
    target.normalize();
    LayoutA_trash_box_for_burnable.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutA_trash_box_for_bottle_can;
    LayoutA_trash_box_for_bottle_can.target_pose.pose.position.x=-0.8;
    LayoutA_trash_box_for_bottle_can.target_pose.pose.position.y=-2.0;
    target.setRPY(0,0,3.14);
    target.normalize();
    LayoutA_trash_box_for_bottle_can.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutA_white_side_table_living_room;
    LayoutA_white_side_table_living_room.target_pose.pose.position.x=2.13;
    LayoutA_white_side_table_living_room.target_pose.pose.position.y=-0.003;
    target.setRPY(0,0,-1.57);
    target.normalize();
    LayoutA_white_side_table_living_room.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutA_square_low_table;
    LayoutA_square_low_table.target_pose.pose.position.x=0.774;
    LayoutA_square_low_table.target_pose.pose.position.y=2.79;
    target.setRPY(0,0,0.8);
    target.normalize();
    LayoutA_square_low_table.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutA_dining_table;//坐标已修改
    LayoutA_dining_table.target_pose.pose.position.x=7.8;
    LayoutA_dining_table.target_pose.pose.position.y=3.07;
    target.setRPY(0,0,0);
    target.normalize();
    LayoutA_dining_table.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutA_wooden_side_table;
    LayoutA_wooden_side_table.target_pose.pose.position.x=4.8;
    LayoutA_wooden_side_table.target_pose.pose.position.y=1.9;
    target.setRPY(0,0,-1.8);
    target.normalize();
    LayoutA_wooden_side_table.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutA_white_side_table_bedroom;
    LayoutA_white_side_table_bedroom.target_pose.pose.position.x=9.3;
    LayoutA_white_side_table_bedroom.target_pose.pose.position.y=-6.4;
    target.setRPY(0,0,0);
    target.normalize();
    LayoutA_white_side_table_bedroom.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutA_white_side_table_bedroom_2;
    LayoutA_white_side_table_bedroom_2.target_pose.pose.position.x=9.37;
    LayoutA_white_side_table_bedroom_2.target_pose.pose.position.y=-2.6;
    target.setRPY(0,0,1.57);
    target.normalize();
    LayoutA_white_side_table_bedroom_2.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutA_wooden_bed;
    LayoutA_wooden_bed.target_pose.pose.position.x=6.8;
    LayoutA_wooden_bed.target_pose.pose.position.y=-4.9;
    target.setRPY(0,0,1.57);
    target.normalize();
    LayoutA_wooden_bed.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutA_wagon;
    LayoutA_wagon.target_pose.pose.position.x=8.12;
    LayoutA_wagon.target_pose.pose.position.y=-2.7;
    target.setRPY(0,0,1.57);
    target.normalize();
    LayoutA_wagon.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutA_cardboard;
    LayoutA_cardboard.target_pose.pose.position.x=9.2;
    LayoutA_cardboard.target_pose.pose.position.y=-4.85;
    target.setRPY(0,0,0);
    target.normalize();
    LayoutA_cardboard.target_pose.pose.orientation = tf2::toMsg(target);

    //2019HM02
    move_base_msgs::MoveBaseGoal LayoutB_white_side_table_living_room;
    LayoutB_white_side_table_living_room.target_pose.pose.position.x=1.45;
    LayoutB_white_side_table_living_room.target_pose.pose.position.y=7.6;
    target.setRPY(0,0,-1.57);
    target.normalize();
    LayoutB_white_side_table_living_room.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutB_arm_chair;
    LayoutB_arm_chair.target_pose.pose.position.x=-0.19;
    LayoutB_arm_chair.target_pose.pose.position.y=4.45;
    target.setRPY(0,0,3.14);
    target.normalize();
    LayoutB_arm_chair.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutB_trash_box_for_recycle;
    LayoutB_trash_box_for_recycle.target_pose.pose.position.x=6.7;
    LayoutB_trash_box_for_recycle.target_pose.pose.position.y=3.8;
    target.setRPY(0,0,1.57);
    target.normalize();
    LayoutB_trash_box_for_recycle.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutB_trash_box_for_bottle_can;
    LayoutB_trash_box_for_bottle_can.target_pose.pose.position.x=7.56;
    LayoutB_trash_box_for_bottle_can.target_pose.pose.position.y=3.7;
    target.setRPY(0,0,1.57);
    target.normalize();
    LayoutB_trash_box_for_bottle_can.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutB_white_side_table_lobby;
    LayoutB_white_side_table_lobby.target_pose.pose.position.x=1.4;
    LayoutB_white_side_table_lobby.target_pose.pose.position.y=-0.5;
    target.setRPY(0,0,-1.57);
    target.normalize();
    LayoutB_white_side_table_lobby.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutB_square_low_table;
    LayoutB_square_low_table.target_pose.pose.position.x=1.28;
    LayoutB_square_low_table.target_pose.pose.position.y=10.5;
    target.setRPY(0,0,1.57);
    target.normalize();
    LayoutB_square_low_table.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutB_round_low_table;
    LayoutB_round_low_table.target_pose.pose.position.x=1.9;
    LayoutB_round_low_table.target_pose.pose.position.y=8.3;
    target.setRPY(0,0,1.57);
    target.normalize();
    LayoutB_round_low_table.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutB_dining_table_kitchen;
    LayoutB_dining_table_kitchen.target_pose.pose.position.x=7.85;
    LayoutB_dining_table_kitchen.target_pose.pose.position.y=-1.1;
    target.setRPY(0,0,3.14);
    target.normalize();
    LayoutB_dining_table_kitchen.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutB_dining_table_lobby;
    LayoutB_dining_table_lobby.target_pose.pose.position.x=2.2;
    LayoutB_dining_table_lobby.target_pose.pose.position.y=2.0;
    target.setRPY(0,0,3.14);
    target.normalize();
    LayoutB_dining_table_lobby.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutB_wooden_side_table_lobby;
    LayoutB_wooden_side_table_lobby.target_pose.pose.position.x=1.00;
    LayoutB_wooden_side_table_lobby.target_pose.pose.position.y=5.25;
    target.setRPY(0,0,1.57);
    target.normalize();
    LayoutB_wooden_side_table_lobby.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutB_wooden_side_table_kitchen;
    LayoutB_wooden_side_table_kitchen.target_pose.pose.position.x=9.0;
    LayoutB_wooden_side_table_kitchen.target_pose.pose.position.y=3.0;
    target.setRPY(0,0,0);
    target.normalize();
    LayoutB_wooden_side_table_kitchen.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutB_wooden_shelf;
    LayoutB_wooden_shelf.target_pose.pose.position.x=2.0;
    LayoutB_wooden_shelf.target_pose.pose.position.y=5.0;
    target.setRPY(0,0,1.57);
    target.normalize();
    LayoutB_wooden_shelf.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutB_wagon;
    LayoutB_wagon.target_pose.pose.position.x=3.14;
    LayoutB_wagon.target_pose.pose.position.y=0.75;
    target.setRPY(0,0,0);
    target.normalize();
    LayoutB_wagon.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutB_cardboard;
    LayoutB_cardboard.target_pose.pose.position.x=2.9;
    LayoutB_cardboard.target_pose.pose.position.y=7.6;
    target.setRPY(0,0,-1.57);
    target.normalize();
    LayoutB_cardboard.target_pose.pose.orientation = tf2::toMsg(target);

    //2020HM01
    move_base_msgs::MoveBaseGoal LayoutC_trash_box_for_recycle;
    LayoutC_trash_box_for_recycle.target_pose.pose.position.x=7.2;
    LayoutC_trash_box_for_recycle.target_pose.pose.position.y=4.55;
    target.setRPY(0,0,1.57);
    target.normalize();
    LayoutC_trash_box_for_recycle.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutC_trash_box_for_burnable_living;
    LayoutC_trash_box_for_burnable_living.target_pose.pose.position.x=3.8;
    LayoutC_trash_box_for_burnable_living.target_pose.pose.position.y=-1.5;
    target.setRPY(0,0,-1.57);
    target.normalize();
    LayoutC_trash_box_for_burnable_living.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutC_trash_box_for_burnable_kitchen;
    LayoutC_trash_box_for_burnable_kitchen.target_pose.pose.position.x=8.3;
    LayoutC_trash_box_for_burnable_kitchen.target_pose.pose.position.y=2.3;
    target.setRPY(0,0,0);
    target.normalize();
    LayoutC_trash_box_for_burnable_kitchen.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutC_trash_box_for_bottle_can;
    LayoutC_trash_box_for_bottle_can.target_pose.pose.position.x=6.4587;
    LayoutC_trash_box_for_bottle_can.target_pose.pose.position.y=4.6449;
    target.setRPY(0,0,1.57);
    target.normalize();
    LayoutC_trash_box_for_bottle_can.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutC_trash_box_for_bottle_can2;
    LayoutC_trash_box_for_bottle_can2.target_pose.pose.position.x=8.3;
    LayoutC_trash_box_for_bottle_can2.target_pose.pose.position.y=3.0;
    target.setRPY(0,0,0);
    target.normalize();
    LayoutC_trash_box_for_bottle_can2.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutC_white_side_table_living_room;
    LayoutC_white_side_table_living_room.target_pose.pose.position.x=-0.4;
    LayoutC_white_side_table_living_room.target_pose.pose.position.y=3.5;
    target.setRPY(0,0,3.14);
    target.normalize();
    LayoutC_white_side_table_living_room.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutC_round_low_table;
    LayoutC_round_low_table.target_pose.pose.position.x=-0.2;
    LayoutC_round_low_table.target_pose.pose.position.y=8.4;
    target.setRPY(0,0,1.57);
    target.normalize();
    LayoutC_round_low_table.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutC_dining_table;
    LayoutC_dining_table.target_pose.pose.position.x=3.2;
    LayoutC_dining_table.target_pose.pose.position.y=0.1;
    target.setRPY(0,0,1.57);
    target.normalize();
    LayoutC_dining_table.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutC_dining_table_2;
    LayoutC_dining_table_2.target_pose.pose.position.x=3.9;
    LayoutC_dining_table_2.target_pose.pose.position.y=0.78;
    target.setRPY(0,0,1.57);
    target.normalize();
    LayoutC_dining_table_2.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutC_wooden_side_table;
    LayoutC_wooden_side_table.target_pose.pose.position.x=2.7;
    LayoutC_wooden_side_table.target_pose.pose.position.y=-1.6;
    target.setRPY(0,0,-1.57);
    target.normalize();
    LayoutC_wooden_side_table.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutC_white_side_table_bedroom;
    LayoutC_white_side_table_bedroom.target_pose.pose.position.x=2.8;
    LayoutC_white_side_table_bedroom.target_pose.pose.position.y=7.2;
    target.setRPY(0,0,-1.57);
    target.normalize();
    LayoutC_white_side_table_bedroom.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutC_iron_bed;
    LayoutC_iron_bed.target_pose.pose.position.x=3.24;
    LayoutC_iron_bed.target_pose.pose.position.y=8.2;
    target.setRPY(0,0,0);
    target.normalize();
    LayoutC_iron_bed.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutC_wooden_shelf;
    LayoutC_wooden_shelf.target_pose.pose.position.x=2.4;
    LayoutC_wooden_shelf.target_pose.pose.position.y=4.8;
    target.setRPY(0,0,1.57);
    target.normalize();
    LayoutC_wooden_shelf.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutC_cardboard;
    LayoutC_cardboard.target_pose.pose.position.x=0.95;
    LayoutC_cardboard.target_pose.pose.position.y=8.5;
    target.setRPY(0,0,1.57);
    target.normalize();
    LayoutC_cardboard.target_pose.pose.orientation = tf2::toMsg(target);

    //2021HM01
    move_base_msgs::MoveBaseGoal LayoutD_white_side_table_living;
    LayoutD_white_side_table_living.target_pose.pose.position.x=2.15;
    LayoutD_white_side_table_living.target_pose.pose.position.y=0.7;
    target.setRPY(0,0,1.57);
    target.normalize();
    LayoutD_white_side_table_living.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutD_corner_sofa;
    LayoutD_corner_sofa.target_pose.pose.position.x=-5.3;
    LayoutD_corner_sofa.target_pose.pose.position.y=-10.2;
    target.setRPY(0,0,-1.57);
    target.normalize();
    LayoutD_corner_sofa.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutD_arm_chair;
    LayoutD_arm_chair.target_pose.pose.position.x=4.0;
    LayoutD_arm_chair.target_pose.pose.position.y=-9.86;
    target.setRPY(0,0,-1.57);
    target.normalize();
    LayoutD_arm_chair.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutD_trash_box_for_burnable_living;
    LayoutD_trash_box_for_burnable_living.target_pose.pose.position.x=-1.58;
    LayoutD_trash_box_for_burnable_living.target_pose.pose.position.y=-1.05;
    target.setRPY(0,0,3.14);
    target.normalize();
    LayoutD_trash_box_for_burnable_living.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutD_trash_box_for_burnable_lobby;
    LayoutD_trash_box_for_burnable_lobby.target_pose.pose.position.x=-3.17;
    LayoutD_trash_box_for_burnable_lobby.target_pose.pose.position.y=-4.8;
    target.setRPY(0,0,1.57);
    target.normalize();
    LayoutD_trash_box_for_burnable_lobby.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutD_white_side_table_lobby;
    LayoutD_white_side_table_lobby.target_pose.pose.position.x=-3.7;
    LayoutD_white_side_table_lobby.target_pose.pose.position.y=-10.3;
    target.setRPY(0,0,-1.57);
    target.normalize();
    LayoutD_white_side_table_lobby.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutD_dining_table_living;
    LayoutD_dining_table_living.target_pose.pose.position.x=1.5;
    LayoutD_dining_table_living.target_pose.pose.position.y=-2.2;
    target.setRPY(0,0,0);
    target.normalize();
    LayoutD_dining_table_living.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutD_dining_table_lobby;
    LayoutD_dining_table_lobby.target_pose.pose.position.x=-2.4;
    LayoutD_dining_table_lobby.target_pose.pose.position.y=-8.4;
    target.setRPY(0,0,3.14);
    target.normalize();
    LayoutD_dining_table_lobby.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutD_white_side_table_bedroom;
    LayoutD_white_side_table_bedroom.target_pose.pose.position.x=1.3;
    LayoutD_white_side_table_bedroom.target_pose.pose.position.y=-7.5;
    target.setRPY(0,0,1.57);
    target.normalize();
    LayoutD_white_side_table_bedroom.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutD_wooden_bed;
    LayoutD_wooden_bed.target_pose.pose.position.x=1.8;
    LayoutD_wooden_bed.target_pose.pose.position.y=-10.6;
    target.setRPY(0,0,3.14);
    target.normalize();
    LayoutD_wooden_bed.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutD_wooden_shelf;
    LayoutD_wooden_shelf.target_pose.pose.position.x=-4.13;
    LayoutD_wooden_shelf.target_pose.pose.position.y=-4.7;
    target.setRPY(0,0,1.57);
    target.normalize();
    LayoutD_wooden_shelf.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutD_wagon_living;
    LayoutD_wagon_living.target_pose.pose.position.x=-1.6;
    LayoutD_wagon_living.target_pose.pose.position.y=-2.35;
    target.setRPY(0,0,3.14);
    target.normalize();
    LayoutD_wagon_living.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutD_wagon_bedroom;
    LayoutD_wagon_bedroom.target_pose.pose.position.x=0.27;
    LayoutD_wagon_bedroom.target_pose.pose.position.y=-9.06;
    target.setRPY(0,0,0);
    target.normalize();
    LayoutD_wagon_bedroom.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutD_cardboard;
    LayoutD_cardboard.target_pose.pose.position.x=-6.2965;
    LayoutD_cardboard.target_pose.pose.position.y=-6.9433;
    target.setRPY(0,0,3.14);
    target.normalize();
    LayoutD_cardboard.target_pose.pose.orientation = tf2::toMsg(target);


    move_base_msgs::MoveBaseGoal location;

    if(ENVIRONMENT == "Layout2019HM01"){ //这里原本为Layout2019HM01，此处修改解决了机器人抓取就放的问题，以下Environment同理

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
    if (location.target_pose.pose.orientation.w == 0.0 &&
      location.target_pose.pose.orientation.x == 0.0 &&
      location.target_pose.pose.orientation.y == 0.0 &&
      location.target_pose.pose.orientation.z == 0.0) {
      ROS_ERROR("Invalid quaternion for dest: %s, room: %s", dest.c_str(), room.c_str());
      // 返回默认目标或抛出错误
  }
    return location;

  }

  move_base_msgs::MoveBaseGoal roomLocation(std::string room, int variation){
    // LayoutA
    tf2::Quaternion target;
    move_base_msgs::MoveBaseGoal LayoutA_living_room;
    LayoutA_living_room.target_pose.pose.position.x=2.0;
    LayoutA_living_room.target_pose.pose.position.y=1;
    target.setRPY(0,0,1.57);
    target.normalize();
    LayoutA_living_room.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutA_living_room_2;
    LayoutA_living_room_2.target_pose.pose.position.x=2.5;
    LayoutA_living_room_2.target_pose.pose.position.y=4.08;
    target.setRPY(0,0,3.14);
    target.normalize();
    LayoutA_living_room_2.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutA_bedroom;
    LayoutA_bedroom.target_pose.pose.position.x=2.57;
    LayoutA_bedroom.target_pose.pose.position.y=-4.31;
    target.setRPY(0,0,0);
    target.normalize();
    LayoutA_bedroom.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutA_bedroom2;
    LayoutA_bedroom2.target_pose.pose.position.x=8.64;
    LayoutA_bedroom2.target_pose.pose.position.y=-5.7;
    target.setRPY(0,0,0);
    target.normalize();
    LayoutA_bedroom2.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutA_lobby2;
    LayoutA_lobby2.target_pose.pose.position.x=1.0;
    LayoutA_lobby2.target_pose.pose.position.y=-3.6;
    target.setRPY(0,0,0);
    target.normalize();
    LayoutA_lobby2.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutA_lobby;
    LayoutA_lobby.target_pose.pose.position.x=1.2;
    LayoutA_lobby.target_pose.pose.position.y=-6.16;
    target.setRPY(0,0,-1.57);
    target.normalize();
    LayoutA_lobby.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutA_kitchen;
    LayoutA_kitchen.target_pose.pose.position.x=8.5;
    LayoutA_kitchen.target_pose.pose.position.y=2.8;
    target.setRPY(0,0,3.14);
    target.normalize();
    LayoutA_kitchen.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutA_kitchen2;
    LayoutA_kitchen2.target_pose.pose.position.x=5.0;
    LayoutA_kitchen2.target_pose.pose.position.y=4.2;
    target.setRPY(0,0,-0.7);
    target.normalize();
    LayoutA_kitchen2.target_pose.pose.orientation = tf2::toMsg(target);

    // LayoutB
    move_base_msgs::MoveBaseGoal LayoutB_living_room;
    LayoutB_living_room.target_pose.pose.position.x=3.5;
    LayoutB_living_room.target_pose.pose.position.y=9.6;
    target.setRPY(0,0,2.4);
    target.normalize();
    LayoutB_living_room.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutB_living_room2;
    LayoutB_living_room2.target_pose.pose.position.x=1.84;
    LayoutB_living_room2.target_pose.pose.position.y=10.2;
    target.setRPY(0,0,0);
    target.normalize();
    LayoutB_living_room2.target_pose.pose.orientation = tf2::toMsg(target);

    // move_base_msgs::MoveBaseGoal LayoutB_bedroom;
    // LayoutB_bedroom.target_pose.pose.position.x=0;
    // LayoutB_bedroom.target_pose.pose.position.y=0;
    // target.setRPY(0,0,0);
    // target.normalize();
    // LayoutB_bedroom.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutB_lobby;
    LayoutB_lobby.target_pose.pose.position.x=1.0;
    LayoutB_lobby.target_pose.pose.position.y=0;
    target.setRPY(0,0,0);
    target.normalize();
    LayoutB_lobby.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutB_lobby2;
    LayoutB_lobby2.target_pose.pose.position.x=2.5;
    LayoutB_lobby2.target_pose.pose.position.y=2.0;
    target.setRPY(0,0,0);
    target.normalize();
    LayoutB_lobby2.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutB_kitchen;
    LayoutB_kitchen.target_pose.pose.position.x=5.5;
    LayoutB_kitchen.target_pose.pose.position.y=-1.13;
    target.setRPY(0,0,0);
    target.normalize();
    LayoutB_kitchen.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutB_kitchen2;
    LayoutB_kitchen2.target_pose.pose.position.x=8.42;
    LayoutB_kitchen2.target_pose.pose.position.y=-1.13;
    target.setRPY(0,0,0);
    target.normalize();
    LayoutB_kitchen2.target_pose.pose.orientation = tf2::toMsg(target);

    // LayoutC
    move_base_msgs::MoveBaseGoal LayoutC_living_room;
    LayoutC_living_room.target_pose.pose.position.x=0.5;
    LayoutC_living_room.target_pose.pose.position.y=2.0;
    target.setRPY(0,0,1.57);
    target.normalize();
    LayoutC_living_room.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutC_living_room2;
    LayoutC_living_room2.target_pose.pose.position.x=0.42;
    LayoutC_living_room2.target_pose.pose.position.y=3.48;
    target.setRPY(0,0,2.355);
    target.normalize();
    LayoutC_living_room2.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutC_living_room3;
    LayoutC_living_room3.target_pose.pose.position.x=4.5;
    LayoutC_living_room3.target_pose.pose.position.y=3.48;
    target.setRPY(0,0,0.0);
    target.normalize();
    LayoutC_living_room3.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutC_living_room4;
    LayoutC_living_room4.target_pose.pose.position.x=4.5;
    LayoutC_living_room4.target_pose.pose.position.y=-0.65;
    target.setRPY(0,0,0.0);
    target.normalize();
    LayoutC_living_room4.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutC_bedroom;
    LayoutC_bedroom.target_pose.pose.position.x=0.1;
    LayoutC_bedroom.target_pose.pose.position.y=6.9;
    target.setRPY(0,0,0.0);
    target.normalize();
    LayoutC_bedroom.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutC_bedroom2;
    LayoutC_bedroom2.target_pose.pose.position.x=3.0;
    LayoutC_bedroom2.target_pose.pose.position.y=8.0;
    target.setRPY(0,0,0.0);
    target.normalize();
    LayoutC_bedroom2.target_pose.pose.orientation = tf2::toMsg(target);

    // move_base_msgs::MoveBaseGoal LayoutC_lobby;
    // LayoutC_lobby.target_pose.pose.position.x=0;
    // LayoutC_lobby.target_pose.pose.position.y=6.9;
    // target.setRPY(0,0,0);
    // LayoutC_lobby.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutC_kitchen;
    LayoutC_kitchen.target_pose.pose.position.x=6.5;
    LayoutC_kitchen.target_pose.pose.position.y=-1.2;
    target.setRPY(0,0,0);
    target.normalize();
    LayoutC_kitchen.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutC_kitchen2;
    LayoutC_kitchen2.target_pose.pose.position.x=7.8;
    LayoutC_kitchen2.target_pose.pose.position.y=1.2;
    target.setRPY(0,0,0);
    target.normalize();
    LayoutC_kitchen2.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutC_kitchen3;
    LayoutC_kitchen3.target_pose.pose.position.x=6.5;
    LayoutC_kitchen3.target_pose.pose.position.y=3.9;
    target.setRPY(0,0,0);
    target.normalize();
    LayoutC_kitchen3.target_pose.pose.orientation = tf2::toMsg(target);

    // LayoutD
    move_base_msgs::MoveBaseGoal LayoutD_living_room;
    LayoutD_living_room.target_pose.pose.position.x=1.0;
    LayoutD_living_room.target_pose.pose.position.y=0.0;
    target.setRPY(0,0,0);
    target.normalize();
    LayoutD_living_room.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutD_living_room_2;
    LayoutD_living_room_2.target_pose.pose.position.x=3.5;
    LayoutD_living_room_2.target_pose.pose.position.y=0.0;
    target.setRPY(0,0,0);
    target.normalize();
    LayoutD_living_room_2.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutD_bedroom;
    LayoutD_bedroom.target_pose.pose.position.x=4.0;
    LayoutD_bedroom.target_pose.pose.position.y=-8.5;
    target.setRPY(0,0,0);
    target.normalize();
    LayoutD_bedroom.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutD_bedroom2;
    LayoutD_bedroom2.target_pose.pose.position.x=1.69;
    LayoutD_bedroom2.target_pose.pose.position.y=-8.0;
    target.setRPY(0.0,0.0,0.0);
    target.normalize();
    LayoutD_bedroom2.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutD_lobby;
    LayoutD_lobby.target_pose.pose.position.x=-1.86;
    LayoutD_lobby.target_pose.pose.position.x=-8.38;
    target.setRPY(0.0,0.0,0.0);
    target.normalize();
    LayoutD_lobby.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal LayoutD_lobby2;
    LayoutD_lobby2.target_pose.pose.position.x=-4.86;
    LayoutD_lobby2.target_pose.pose.position.x=-8.7;
    target.setRPY(0.0,0.0,0.0);
    target.normalize();
    LayoutD_lobby2.target_pose.pose.orientation = tf2::toMsg(target);

    move_base_msgs::MoveBaseGoal location;
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
          //else if (variation == 1)
          //  location = LayoutA_kitchen2; 
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
        // else if(room == "bedroom"){ location = LayoutB_bedroom; }
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
        // else if(room == "lobby"){ location = LayoutC_lobby; }
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
        // else if(room == "kitchen"){ location = LayoutD_kitchen; }
    }

    else{ 
      location = LayoutC_living_room; 
      ROS_INFO("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$WELP STH UNDESIRABLE HAPPENED");
    }

    return location;
  }

  tf::StampedTransform getTfBase(tf::TransformListener &tf_listener)
  {
    tf::StampedTransform tf_transform;

    try
    {
      tf_listener.lookupTransform("/odom", "/base_footprint", ros::Time(0), tf_transform);
    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR("%s",ex.what());
    }
    return tf_transform;
  }

  void moveBase(ros::Publisher &publisher, double linear_x, double linear_y, double angular_z)
  {
    geometry_msgs::Twist twist;

    twist.linear.x  = linear_x;
    twist.linear.y  = linear_y;
    twist.angular.z = angular_z;
    publisher.publish(twist);
  }

  void stopBase(ros::Publisher &publisher)
  {
    moveBase(publisher, 0.0, 0.0, 0.0);
  }
  
  void resetAMCL()
  {
      ROS_INFO("Resetting AMCL for new map...");
      
      if (!map_manager_) {
          ROS_ERROR("LoadMapManager not initialized");
          return;
      }
      
      // 设置默认的初始位置
      geometry_msgs::Pose initial_pose;
      initial_pose.position.x = 0.0;
      initial_pose.position.y = 0.0;
      initial_pose.position.z = 0.0;
      initial_pose.orientation.x = 0.0;
      initial_pose.orientation.y = 0.0;
      initial_pose.orientation.z = 0.0;
      initial_pose.orientation.w = 1.0;
      
      // 使用LoadMapManager重置AMCL
      if (map_manager_->resetAMCL(initial_pose)) {
          ROS_INFO("AMCL reset completed successfully.");
      } else {
          ROS_ERROR("AMCL reset failed.");
      }
  }

  bool loadMap()
  {
      ROS_INFO("Loading map for ENVIRONMENT: %s", ENVIRONMENT.c_str());

      if (!map_manager_) {
          ROS_ERROR("LoadMapManager not initialized");
          return false;
      }

      // Use LoadMapManager to switch environment (mapping is handled internally)
      if (map_manager_->switchEnvironment(ENVIRONMENT)) {
          ROS_INFO("Successfully switched to environment: %s", ENVIRONMENT.c_str());
          return true;
      } else {
          ROS_ERROR("Failed to switch to environment: %s", ENVIRONMENT.c_str());
          return false;
      }
  }

  void moveArm(ros::Publisher &publisher, const std::vector<double> &positions, ros::Duration &duration)
  {
    arm_joint_trajectory_.points[0].positions = positions;
    arm_joint_trajectory_.points[0].time_from_start = duration;

    publisher.publish(arm_joint_trajectory_);
  }

  void operateHand(ros::Publisher &publisher, bool should_grasp)
  {
    std::vector<std::string> joint_names {"hand_motor_joint"};
    std::vector<double> positions;

    if(should_grasp)
    {
      ROS_DEBUG("Grasp");
      positions.push_back(-0.105);
    }
    else
    {
      ROS_DEBUG("Open hand");
      positions.push_back(+1.239);
    }

    trajectory_msgs::JointTrajectoryPoint point;
    point.positions = positions;
    point.time_from_start = ros::Duration(2);

    trajectory_msgs::JointTrajectory joint_trajectory;
    joint_trajectory.joint_names = joint_names;
    joint_trajectory.points.push_back(point);
    publisher.publish(joint_trajectory);
  }


public:
  int run(int argc, char **argv)
  {
    ros::NodeHandle node_handle;

    ros::Rate loop_rate(10);
    std::vector<std::string> data;
    std::string sub_msg_to_robot_topic_name;
    std::string pub_msg_to_moderator_topic_name;
    std::string pub_base_twist_topic_name;
    std::string pub_arm_trajectory_topic_name;
    std::string pub_gripper_trajectory_topic_name;

    node_handle.param<std::string>("sub_msg_to_robot_topic_name",       sub_msg_to_robot_topic_name,       "/handyman/message/to_robot");
    node_handle.param<std::string>("pub_msg_to_moderator_topic_name",   pub_msg_to_moderator_topic_name,   "/handyman/message/to_moderator");
    node_handle.param<std::string>("pub_base_twist_topic_name",         pub_base_twist_topic_name,         "/hsrb/command_velocity");
    node_handle.param<std::string>("pub_arm_trajectory_topic_name",     pub_arm_trajectory_topic_name,     "/hsrb/arm_trajectory_controller/command");
    node_handle.param<std::string>("pub_gripper_trajectory_topic_name", pub_gripper_trajectory_topic_name, "/hsrb/gripper_controller/command");

    init(node_handle);

    ros::Time waiting_start_time;

    ROS_INFO("Handyman sample start!");

    ros::Subscriber sub_msg                = node_handle.subscribe<handyman::HandymanMsg>(sub_msg_to_robot_topic_name, 100, &HandymanSample::messageCallback, this);\
    ros::Subscriber sub_msg_vision         = node_handle.subscribe<geometry_msgs::PoseStamped>("/vision", 1000, &HandymanSample::visionCallback, this);   
    ros::Subscriber sub_msg_hand           = node_handle.subscribe<std_msgs::Int32MultiArray>("/hand_detection", 1000, &HandymanSample::graspvisioncallback, this);
    ros::Publisher  pub_target_object      = node_handle.advertise<std_msgs::String>("/detection_target", 10);
    ros::Publisher  pub_msg                = node_handle.advertise<handyman::HandymanMsg>(pub_msg_to_moderator_topic_name, 10);
    ros::Publisher  pub_base_twist         = node_handle.advertise<geometry_msgs::Twist>            (pub_base_twist_topic_name, 10);
    ros::Publisher  pub_arm_trajectory     = node_handle.advertise<trajectory_msgs::JointTrajectory>(pub_arm_trajectory_topic_name, 10);
    ros::Publisher  pub_gripper_trajectory = node_handle.advertise<trajectory_msgs::JointTrajectory>(pub_gripper_trajectory_topic_name, 10);
    tf::TransformListener tf_listener;

    MoveBaseClient ac("move_base", true);
    

    while (ros::ok())
    {
      if(is_failed_)
      {
        ROS_INFO("Task failed!");
        // tf::StampedTransform tf_transform = getTfBase(tf_listener);
        step_ = Initialize;
      }

      switch(step_)
      {
        case Initialize:
        {
          reset();
          // moveBase(pub_base_twist, +0.1, 0.0, 0.0);
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
                ROS_WARN("Environment not set. Waiting for layout message...");
                break;
            }
            
            // Load the map based on the current environment first
            ROS_INFO("############ Before loadmap, the map is %s", ENVIRONMENT.c_str());
            if (!loadMap())
            {
                ROS_ERROR("Failed to load map, returning to Initialize");
                step_ = Initialize;
                break;
            }

            while (!ac.waitForServer(ros::Duration(5.0)))
            {
              ROS_INFO("Waiting for the move_base action server to come up");
            }

            // Only send ready message after system is truly ready
            ROS_INFO("############ The current map is %s", ENVIRONMENT.c_str());
            ROS_INFO("System is ready, sending I_am_ready message");
            sendMessage(pub_msg, MSG_I_AM_READY);
            ROS_INFO("Task start!");
            step_++;
          }
          break;
        }
        case WaitForInstruction:
        {
          if(instruction_msg_!="")
          {
            ROS_INFO("%s", instruction_msg_.c_str());
            data = extractInfo(instruction_msg_.c_str(),room_list,object_list,dest_list);
            std_msgs::String target_object;
            target_object.data = object_list[0];
            pub_target_object.publish(target_object);
            std::vector<double> positions { 0.1, 0.0, 0.0, -1.57, 0.0 };
            ros::Duration duration;
            duration.sec = 1;

            moveArm(pub_arm_trajectory, positions, duration);

            operateHand(pub_gripper_trajectory, false);
            room_reached= false;
            found_object = false;
            patrol_step = 0;
            step_= GoToRoom1;
          }
          break;
        }
        case GoToRoom1:
        {
          if(!found_object || !room_reached){
            move_base_msgs::MoveBaseGoal goal = roomLocation(room_list[0],patrol_step);
            ROS_INFO("Assigned goal for room: %s, x: %.3f, y: %.3f", room_list[0].c_str(), goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
            std::cout << "room = " << room_list[0] << std::endl;
            ROS_INFO("room = %s", room_list[0].c_str());
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            // goal.target_pose.pose = roomLocation(data, ENVIRONMENT);

            ROS_INFO("Sending goal");
            ac.sendGoal(goal);

            // std::cout << "Angle = " << std::to_string(yaw) << std::endl;
            if(ac.waitForResult(ros::Duration(0))){
              //  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                patrol_step++;
                if(patrol_step >= max_patrol){
                  patrol_step = 0;
                }
              //  }
            }
            // ros::Duration(120).sleep();
            if(!room_reached){
            ROS_INFO("Room Reached");
            sendMessage(pub_msg, MSG_ROOM_REACHED);
            room_reached = true;
            // found_object = false;
            }

          }else{
            ready_to_grasp = false;
            aligned_x = false;
            aligned_y = false;
            arm_height = 0.0;
            x_adjust = 0.0;
            y_adjust = 0.0;
            body_height = 0.0;
            step_ = MoveToInFrontOfTarget;
          }


          break;
        }
        case GoToRoom2:
        {
          if(!room_reached){
            move_base_msgs::MoveBaseGoal goal;
            if(room_list.size()>1){
               goal = destLocation(dest_list[dest_list.size() - 1],room_list[1]);
            }else{
               goal = destLocation(dest_list[dest_list.size() - 1],room_list[0]);
            }
            std::cout << "dest = " << dest_list[dest_list.size() - 1] << std::endl;
            ROS_INFO("dest = %s", dest_list[dest_list.size() - 1].c_str());
            ROS_INFO("room = %s", room_list[0].c_str());
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            // goal.target_pose.pose = roomLocation(data, ENVIRONMENT);
            
            std::vector<double> positions { 0.2, -0.3, 0.0, -1.57, 0.0 };
            ros::Duration duration;
            duration.sec = 1;
            moveArm(pub_arm_trajectory, positions, duration);

            ROS_INFO("Sending goal");
            ac.sendGoal(goal);


            ac.waitForResult(ros::Duration(0));

            // ros::Duration(120).sleep();
            if(!room_reached){
            ROS_INFO("Room Reached");
            // sendMessage(pub_msg, MSG_ROOM_REACHED);
            room_reached = true;
            }
          }else{
            std::vector<double> positions { 0.2, -0.8, 0.0, -1.0, 0.0 };
            ros::Duration duration;
            duration.sec = 1;
            moveArm(pub_arm_trajectory, positions, duration);
            waiting_start_time = ros::Time::now();
            extend_before_release = false;
            step_ = Release;
          }
          break;
        }
        case MoveToInFrontOfTarget:
        {
       

          // if(!aligned_x){
          if(ros::Time::now() - last_detect < ros::Duration(1)){

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
          // }

          // if(!aligned_y){
          
            if(y_det < 360 - 25*tol_multiplier){
              arm_height+= 0.004;
              if(arm_height>0.0){
                // body_height+=0.005;
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

            // if(aligned_y){
            //   if(arm_height > -0.2){
            //     body_height+=0.005;
            //   }
            // }

            std::vector<double> positions { 0.2, arm_height, 0.0, -1.57 - arm_height*0.4, 0.0 };
            ros::Duration duration;
            duration.sec = 1;
            moveArm(pub_arm_trajectory, positions, duration);

            moveBase(pub_base_twist,y_adjust, x_adjust*0.5 ,x_adjust);
          }

          
          // }
          
          if(aligned_x && aligned_y){
            std::cout << "ready = " << std::to_string(ready_to_grasp) << std::endl;
            if(ready_to_grasp == false){
              moveBase(pub_base_twist,0.02,0.0,0.0);
            }else{
              std::cout << "Gripping" << std::endl;
              moveBase(pub_base_twist,0.0,0.0,0.0);
              waiting_start_time = ros::Time::now();
              step_ = Grasp;
            }
          }

          break;
        }

        case Grasp:
        {
  
          // tf::StampedTransform tf_transform = getTfBase(tf_listener);

          // std::vector<double> positions { object_pose.position.z - 0.15 - 0.34, 0.0, 0.0, 0.0 , 0.0 };
          // ros::Duration duration;
          // duration.sec = 1;
          // moveArm(pub_arm_trajectory, positions, duration);
          
          // sleep(1);

          // tf::StampedTransform tf_transform = getTfBase(tf_listener);
          // float hyp = pow(pow(object_pose.position.x - tf_transform.getOrigin().getX() - 0.145,2.0) + pow(0.15,2.0),0.5);
          
          // float lower_angle_1 = atan2f(0.15,object_pose.position.x - tf_transform.getOrigin().getX() - 0.145);
          // float upper_angle = acos((pow(0.345,2.0) + pow(0.140,2.0) - pow(hyp,2.0))/(2.0*0.345*0.140));
          // float lower_angle_2 = acos((pow(0.345,2) - pow(0.140,2) + pow(hyp,2.0)) /(2.0*0.345*hyp));

          // std::vector<double> positions { object_pose.position.z - 0.15 - 0.34, -1.57-(lower_angle_1 + lower_angle_2), 0.0, upper_angle - 1.57 , 0.0 };
          // ros::Duration duration;
          // duration.sec = 3;
          // moveArm(pub_arm_trajectory, positions, duration);
          
  

          operateHand(pub_gripper_trajectory, true);

          if(ros::Time::now() - waiting_start_time > ros::Duration(8, 0))
          {
            sendMessage(pub_msg, MSG_OBJECT_GRASPED);
            room_reached = false;
            found_object = false;
            step_= GoToRoom2;
          }
          
          break;
        }
        case Release:
        {

          if(!extend_before_release){
            if(ros::Time::now() - waiting_start_time > ros::Duration(4, 0))
              {
                operateHand(pub_gripper_trajectory, false);
                waiting_start_time = ros::Time::now();
                extend_before_release = true;
              }
          }else{
            if(ros::Time::now() - waiting_start_time > ros::Duration(8, 0))
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
            ROS_INFO("Task finished!");
            step_ = Initialize;
          }

          break;
        }
      }

      ros::spinOnce();

      loop_rate.sleep();
    }

    return EXIT_SUCCESS;
  }
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "handyman_sample");

  HandymanSample handyman_sample;
  return handyman_sample.run(argc, argv);
};

