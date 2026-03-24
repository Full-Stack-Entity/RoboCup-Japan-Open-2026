#include <rclcpp/rclcpp.hpp>  // 原 ROS1: ros/ros.h -> ROS2: rclcpp 节点与日志接口
#include <geometry_msgs/msg/twist.hpp>  // 原 ROS1: geometry_msgs/Twist.h
#include <trajectory_msgs/msg/joint_trajectory.hpp>  // 原 ROS1: trajectory_msgs/JointTrajectory.h
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>  // 原 ROS1: trajectory_msgs/JointTrajectoryPoint.h

// 原 ROS1: <human_navigation/...> -> ROS2: human_navigation_msgs/msg/...
// 消息定义来自 ros2-competition-msgs 仓库中的 human_navigation_msgs 包
#include <human_navigation_msgs/msg/human_navi_object_info.hpp>
#include <human_navigation_msgs/msg/human_navi_destination.hpp>
#include <human_navigation_msgs/msg/human_navi_task_info.hpp>
#include <human_navigation_msgs/msg/human_navi_msg.hpp>
#include <human_navigation_msgs/msg/human_navi_guidance_msg.hpp>
#include <human_navigation_msgs/msg/human_navi_avatar_status.hpp>
#include <human_navigation_msgs/msg/human_navi_object_status.hpp>

#include "human_nav_llm_ros2/srv/rewrite_guidance.hpp"

// 原 ROS1: tf/transform_listener.h -> ROS2: tf2_ros + tf2_geometry_msgs
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <cmath>
#include <iomanip>
#include <string>
#include <algorithm>
#include <sstream>
#include <chrono>
#include <cctype>
#include <vector>

class HumanNavigationSample
{
private:
  // 与 ROS1 版本保持相同的状态机定义
  enum Step
  {
    Initialize,
    Ready,
    WaitTaskInfo,
    GuideForTakingObject,
    GuideForPlacement,
    // WaitTaskFinished,  // 在 2025 版本中未使用，保持注释
    TaskFinished
  };

  enum class SpeechState
  {
    None,
    WaitingState,
    Speaking,
    Speakable
  };

  // human navigation message from/to the moderator
  // 与 ROS1 字符串常量完全一致，保证与 Unity 侧协议兼容
  const std::string MSG_ARE_YOU_READY      = "Are_you_ready?";
  const std::string MSG_TASK_SUCCEEDED     = "Task_succeeded";
  const std::string MSG_TASK_FAILED        = "Task_failed";
  const std::string MSG_TASK_FINISHED      = "Task_finished";
  const std::string MSG_GO_TO_NEXT_SESSION = "Go_to_next_session";
  const std::string MSG_MISSION_COMPLETE   = "Mission_complete";
  const std::string MSG_REQUEST            = "Guidance_request";
  const std::string MSG_SPEECH_STATE       = "Speech_state";
  const std::string MSG_SPEECH_RESULT      = "Speech_result";

  const std::string MSG_I_AM_READY        = "I_am_ready";
  const std::string MSG_GET_AVATAR_STATUS = "Get_avatar_status";
  const std::string MSG_GET_OBJECT_STATUS = "Get_object_status";
  const std::string MSG_GET_SPEECH_STATE  = "Get_speech_state";

  // 显示给受试者的引导面板类型，与 ROS1 完全一致
  const std::string DISPLAY_TYPE_ALL         = "All";
  const std::string DISPLAY_TYPE_ROBOT_ONLY  = "RobotOnly";
  const std::string DISPLAY_TYPE_AVATAR_ONLY = "AvatarOnly";
  const std::string DISPLAY_TYPE_NONE        = "None";

  int step{};
  int questcounter{};  // 2025 版本新增：用于控制距离提示频率，ROS2 保留此逻辑
  SpeechState speechState{SpeechState::None};

  // 原 ROS1: tf::TransformListener listener_;
  // ROS2 中使用 tf2_ros::Buffer + TransformListener
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  bool isStarted{false};
  bool isFinished{false};

  bool isTaskInfoReceived{false};
  bool isRequestReceived{false};

  // 原 ROS1: ros::Time -> ROS2: rclcpp::Time
  rclcpp::Time timePrevSpeechStateConfirmed;

  bool isSentGetAvatarStatus{false};
  bool isSentGetObjectStatus{false};

  human_navigation_msgs::msg::HumanNaviTaskInfo taskInfo;

  std::string guideMsg;

  human_navigation_msgs::msg::HumanNaviAvatarStatus avatarStatus;
  human_navigation_msgs::msg::HumanNaviObjectStatus objectStatus;

  std::string targetObjectName;

  double direction_target_object{0.0};
  double direction_target_direction{0.0};
  double avatar_timer{0.0};

  trajectory_msgs::msg::JointTrajectory arm_joint_trajectory_;

  int requestTime{0};
  bool request{false};
  std::string final_location;
  std::string initial_location;

  // 原 ROS1: ros::NodeHandle nodeHandle; -> ROS2: rclcpp::Node
  std::shared_ptr<rclcpp::Node> nodeHandle;

  // 原 ROS1: NodeHandle 成员上直接调用 subscribe/advertise
  // ROS2: 使用 rclcpp::Subscription/Publisher 成员
  rclcpp::Subscription<human_navigation_msgs::msg::HumanNaviMsg>::SharedPtr subHumanNaviMsg;
  rclcpp::Subscription<human_navigation_msgs::msg::HumanNaviTaskInfo>::SharedPtr subTaskInfoMsg;
  rclcpp::Subscription<human_navigation_msgs::msg::HumanNaviAvatarStatus>::SharedPtr subAvatarStatusMsg;
  rclcpp::Subscription<human_navigation_msgs::msg::HumanNaviObjectStatus>::SharedPtr subObjectStatusMsg;

  rclcpp::Publisher<human_navigation_msgs::msg::HumanNaviMsg>::SharedPtr pubHumanNaviMsg;
  rclcpp::Publisher<human_navigation_msgs::msg::HumanNaviGuidanceMsg>::SharedPtr pubGuidanceMsg;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_base_twist_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_base_trajectory_;

  // Subtask C: optional local LLM rewrite (see docs/SUBTASK2_LLM_方案分析.md)
  bool use_llm_rewrite_{false};
  double llm_timeout_sec_{2.0};
  std::string llm_service_name_{"/rewrite_guidance"};
  bool strict_template_mode_{true};
  rclcpp::Client<human_nav_llm_ros2::srv::RewriteGuidance>::SharedPtr llm_client_;
  /// 抓取阶段骨架句（已 polish 或与原文一致）；周期性距离行在此基础上拼接，不再调用 LLM
  std::string polished_pick_base_;
  /// 放置阶段骨架句
  std::string polished_place_base_;
  /// TaskInfo 到达后待主循环中 polish（避免在订阅回调里 spin_until_future_complete 死锁）
  bool pending_polish_{false};

  /// 方位提示节流：上次发送时间(秒)、间隔秒数（规则要求≤15条，不再实时距离）
  double last_direction_hint_sec_{0.0};
  double direction_hint_interval_sec_{7.0};
  /// 抓取正确提示后，字幕保持到该时间点（秒）；期间禁止新字幕覆盖
  double subtitle_hold_until_sec_{0.0};
  std::string last_wrong_object_name_;

  std::string getDirectionHint(
    double av_x, double av_y,
    double target_x, double target_y,
    const geometry_msgs::msg::Quaternion & body_orient) const;
  static std::string toReadableName(const std::string & name);
  static std::string toLowerAscii(std::string s);
  static bool isLargeLandmarkName(const std::string & readable_name_lower);
  static bool isRefrigeratorName(const std::string & readable_name_lower);
  /// 基于 getFurnitureRelation 的空间判断，返回 "on/in/near the X"（仅来自 furniture，不掺入 non_target）
  std::string getRelationPhraseForPosition(double dx, double dy, double dz,
                                          double size_x, double size_y, double size_z) const;
  std::string nearestFurnitureName(double x, double y, double z) const;
  std::string nearestLargeLandmarkName(
    double x, double y, double z,
    double min_distance_m,
    double max_distance_m,
    const std::string & avoid_name_lower = "") const;
  std::string nearestNearLandmarkName(
    double x, double y, double z,
    double min_distance_m,
    double max_distance_m,
    const std::string & avoid_name_lower = "") const;
  std::string buildStrictPickTemplate() const;
  std::string buildStrictPlaceTemplate() const;

  static std::string truncateUtf8(const std::string & s, size_t max_chars);
  static std::string escapeJsonString(const std::string & s);
  std::string buildContextJson() const;
  std::string polishGuidance(const std::string & draft, const std::string & phase, const std::string & context_json);

  void init()
  {
    // 原 ROS1: step = Initialize; speechState = SpeechState::None; reset();
    step = Initialize;
    speechState = SpeechState::None;
    reset();
  }

  void reset()
  {
    // 保持与 ROS1 2025 版本相同的重置逻辑
    isStarted             = false;
    isFinished            = false;
    isTaskInfoReceived    = false;
    isRequestReceived     = false;
    isSentGetAvatarStatus = false;
    isSentGetObjectStatus = false;
    questcounter          = 0;

    // 重置 avatar 状态
    avatarStatus.object_in_left_hand = "";
    avatarStatus.object_in_right_hand = "";
    avatarStatus.is_target_object_in_left_hand = false;
    avatarStatus.is_target_object_in_right_hand = false;

    polished_pick_base_.clear();
    polished_place_base_.clear();
    pending_polish_ = false;
    last_direction_hint_sec_ = 0.0;
    subtitle_hold_until_sec_ = 0.0;
    last_wrong_object_name_.clear();
  }

  // send humanNaviMsg to the moderator (Unity)
  // 原 ROS1: void sendMessage(ros::Publisher&, const std::string&) -> ROS2: 使用 rclcpp::Publisher SharedPtr
  void sendMessage(const rclcpp::Publisher<human_navigation_msgs::msg::HumanNaviMsg>::SharedPtr & publisher,
                   const std::string & message)
  {
    human_navigation_msgs::msg::HumanNaviMsg human_navi_msg;
    human_navi_msg.message = message;
    human_navi_msg.detail  = "";

    publisher->publish(human_navi_msg);

    RCLCPP_INFO(nodeHandle->get_logger(), "Send message:%s", message.c_str());
  }

  // send guidance_message only
  // 原 ROS1: void sendGuidanceMessage(ros::Publisher&, ...) -> ROS2: 使用 rclcpp::Publisher SharedPtr
  void sendGuidanceMessage(
    const rclcpp::Publisher<human_navigation_msgs::msg::HumanNaviGuidanceMsg>::SharedPtr & publisher,
    const std::string & message,
    const std::string & displayType)
  {
    const std::string safe_message = truncateUtf8(message, 400);

    human_navigation_msgs::msg::HumanNaviGuidanceMsg guidanceMessage;
    guidanceMessage.message         = safe_message;
    guidanceMessage.display_type    = displayType;
    guidanceMessage.source_language = "";  // Blank or ISO-639-1 language code
    guidanceMessage.target_language = "";

    publisher->publish(guidanceMessage);

    speechState = SpeechState::Speaking;

    RCLCPP_INFO(
      nodeHandle->get_logger(),
      "Send guide message: %s : %s",
      guidanceMessage.message.c_str(),
      guidanceMessage.display_type.c_str());
    if (message != safe_message) {
      RCLCPP_WARN(
        nodeHandle->get_logger(),
        "Guidance truncated to <=400 Unicode scalars for competition limit");
    }
  }

  // receive humanNaviMsg from the moderator (Unity)
  // 原 ROS1: void messageCallback(const HumanNaviMsg::ConstPtr&) -> ROS2: ConstSharedPtr
  void messageCallback(const human_navigation_msgs::msg::HumanNaviMsg::ConstSharedPtr message)
  {
    RCLCPP_INFO(
      nodeHandle->get_logger(),
      "Subscribe message: %s : %s",
      message->message.c_str(),
      message->detail.c_str());

    if (message->message == MSG_ARE_YOU_READY)
    {
      isStarted = true;
      step      = Ready;
      sendMessage(pubHumanNaviMsg, MSG_I_AM_READY);
    }
    else if (message->message == MSG_REQUEST)  // Guidance_request
    {
      if (isTaskInfoReceived && !isFinished)
      {
        isRequestReceived = true;
      }
    }
    else if (message->message == MSG_TASK_SUCCEEDED)
    {
      // 与 ROS1 一致，暂不处理
    }
    else if (message->message == MSG_TASK_FAILED)
    {
      init();
    }
    else if (message->message == MSG_TASK_FINISHED)
    {
      isFinished = true;
    }
    else if (message->message == MSG_GO_TO_NEXT_SESSION)
    {
      RCLCPP_INFO(nodeHandle->get_logger(), "Go to next session");
      init();
    }
    else if (message->message == MSG_MISSION_COMPLETE)
    {
      // ROS1 中注释掉 exit，这里同样保持不退出进程
    }
    else if (message->message == MSG_SPEECH_STATE)
    {
      if (message->detail == "Is_speaking")
      {
        speechState = SpeechState::Speaking;
      }
      else
      {
        speechState = SpeechState::Speakable;
      }
    }
    else if (message->message == MSG_SPEECH_RESULT)
    {
      RCLCPP_INFO(
        nodeHandle->get_logger(),
        "Speech result: %s",
        message->detail.c_str());
    }
  }

  // 原 ROS1: moveBaseTwist(ros::Publisher&, double) -> ROS2: 使用 geometry_msgs::msg::Twist
  void moveBaseTwist(const rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr & publisher,
                     double rot)
  {
    geometry_msgs::msg::Twist twist;
    twist.linear.x  = 0.0;
    twist.linear.y  = 0.0;
    twist.angular.z = rot;
    publisher->publish(twist);
  }

  // 原 ROS1: tf::TransformListener + geometry_msgs::PointStamped -> ROS2: tf2_ros::Buffer
  void moveBaseJointTrajectory(
    const rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr & publisher,
    double linear_x,
    double linear_y,
    double theta,
    double duration_sec)
  {
    if (!tf_buffer_)
    {
      return;
    }
    try
    {
      // 使用 TimePointZero 取“最新可用”的 TF，避免 nodeHandle->now() 领先于仿真 TF 导致 ExtrapolationException
      if (!tf_buffer_->canTransform("odom", "base_footprint", tf2::TimePointZero))
      {
        return;
      }

      geometry_msgs::msg::TransformStamped transform =
        tf_buffer_->lookupTransform("odom", "base_footprint", tf2::TimePointZero);

      geometry_msgs::msg::PointStamped basefootprint_2_target;
      geometry_msgs::msg::PointStamped odom_2_target;
      basefootprint_2_target.header.frame_id = "base_footprint";
      basefootprint_2_target.header.stamp    = transform.header.stamp;  // 使用 TF 时间，避免外推
      basefootprint_2_target.point.x         = linear_x;
      basefootprint_2_target.point.y         = linear_y;

      tf_buffer_->transform(basefootprint_2_target, odom_2_target, "odom");

      trajectory_msgs::msg::JointTrajectory joint_trajectory;
    joint_trajectory.joint_names.push_back("odom_x");
    joint_trajectory.joint_names.push_back("odom_y");
    joint_trajectory.joint_names.push_back("odom_t");

    trajectory_msgs::msg::JointTrajectoryPoint omni_joint_point;
    omni_joint_point.positions = {
      odom_2_target.point.x,
      odom_2_target.point.y,
      theta  // 原 ROS1: 使用 theta（而不是 yaw+theta），保持行为一致
    };
    omni_joint_point.time_from_start = rclcpp::Duration::from_seconds(duration_sec);

    joint_trajectory.points.push_back(omni_joint_point);
    publisher->publish(joint_trajectory);
    }
    catch (const tf2::TransformException & e)
    {
      RCLCPP_DEBUG(
        nodeHandle->get_logger(),
        "moveBaseJointTrajectory: TF not available (%s), skipping.",
        e.what());
    }
  }

  // 将 target_object.name（如 empty_plastic_bottle_0）转为给受试者看的自然语言
  static std::string getHumanReadableTargetName(const std::string & name)
  {
    if (name.find("empty_plastic_bottle") != std::string::npos)
    {
      return "an empty plastic bottle";
    }
    if (name.find("cup") != std::string::npos)
    {
      return "a cup";
    }
    // 默认：去掉末尾 _0/_1 等，下划线改空格，前加 "the "
    std::string s = name;
    while (s.size() > 0 && std::isdigit(static_cast<unsigned char>(s.back())))
    {
      s.pop_back();
    }
    if (s.size() > 0 && s.back() == '_')
    {
      s.pop_back();
    }
    std::replace(s.begin(), s.end(), '_', ' ');
    return "the " + s;
  }

  // 下方 getFinalDestination/getInitialDestination/getFurnitureRelation 基本照搬 2025 逻辑
  std::string getFurnitureRelation(const std::string & furniture_name,
                                   double dx, double dy, double dz,
                                   double fx_min, double fx_max,
                                   double fy_min, double fy_max,
                                   double fz_min, double fz_max) const
  {
    // 判断家具类型，与 ROS1 2025 版完全一致
    if (furniture_name.find("table") != std::string::npos)
    {
      if (dz >= fz_min && dz <= fz_max)
      {
        if (dx >= fx_min && dx <= fx_max && dy >= fy_min && dy <= fy_max)
        {
          return "on the " + furniture_name;
        }
      }
      else if (dz < fz_min)
      {
        return "under the " + furniture_name;
      }
    }
    else if (furniture_name.find("trash") != std::string::npos ||
             furniture_name.find("bin")   != std::string::npos)
    {
      if (dz >= fz_min && dz <= fz_max)
      {
        if (dx >= fx_min && dx <= fx_max && dy >= fy_min && dy <= fy_max)
        {
          return "inside the " + furniture_name;
        }
        else
        {
          return "on the " + furniture_name;
        }
      }
    }
    else if (furniture_name.find("cabinet") != std::string::npos ||
             furniture_name.find("shelf")   != std::string::npos)
    {
      if (dz >= fz_min && dz <= fz_max)
      {
        if (dx >= fx_min && dx <= fx_max && dy >= fy_min && dy <= fy_max)
        {
          return "inside the " + furniture_name;
        }
        else
        {
          return "on the " + furniture_name;
        }
      }
    }

    return "near the " + furniture_name;
  }

  std::string getFinalDestination(
    const human_navigation_msgs::msg::HumanNaviTaskInfo::ConstSharedPtr & message)
  {
    taskInfo = *message;

    double dx = taskInfo.destination.position.x;
    double dy = taskInfo.destination.position.y;
    double dz = taskInfo.destination.position.z;

    int   i                        = static_cast<int>(taskInfo.furniture.size());
    int   index_of_nearest_furniture = 0;
    float min_furniture_distance   = 999999999.0f;

    if (i <= 0)
    {
      RCLCPP_WARN(nodeHandle->get_logger(), "getFinalDestination: no furniture, using default");
      return "Place it at the designated destination.";
    }

    for (int j = 0; j < i; ++j)
    {
      float distance =
        std::sqrt(
          std::pow(taskInfo.destination.position.x - taskInfo.furniture[j].position.x, 2) +
          std::pow(std::abs(taskInfo.destination.position.y) - std::abs(taskInfo.furniture[j].position.y), 2) +
          std::pow(taskInfo.destination.position.z - taskInfo.furniture[j].position.z, 2));

      if (distance < min_furniture_distance)
      {
        index_of_nearest_furniture = j;
        min_furniture_distance     = distance;
      }

      double fx    = taskInfo.furniture[index_of_nearest_furniture].position.x;
      double fy    = taskInfo.furniture[index_of_nearest_furniture].position.y;
      double fz    = taskInfo.furniture[index_of_nearest_furniture].position.z;
      double fx_min = fx - std::abs(taskInfo.destination.size.x) / 2.0;
      double fx_max = fx + std::abs(taskInfo.destination.size.x) / 2.0;
      double fy_min = fy - std::abs(taskInfo.destination.size.y) / 2.0;
      double fy_max = fy + std::abs(taskInfo.destination.size.y) / 2.0;
      double fz_min = fz - std::abs(taskInfo.destination.size.z);
      double fz_max = fz + std::abs(taskInfo.destination.size.z);

      if (j == 0)
      {
        RCLCPP_INFO(
          nodeHandle->get_logger(),
          "Destination.size: %f, %f, %f",
          taskInfo.destination.size.x,
          std::abs(taskInfo.destination.size.y),
          taskInfo.destination.size.z);
        RCLCPP_INFO(
          nodeHandle->get_logger(),
          "dx: %f, dy: %f, dz: %f",
          dx, dy, dz);
        RCLCPP_INFO(
          nodeHandle->get_logger(),
          "furniture_amount: %d.",
          i);
      }
      RCLCPP_INFO(
        nodeHandle->get_logger(),
        "furniture: %s, index: %d, fx: %f, fy: %f, fz: %f",
        taskInfo.furniture[index_of_nearest_furniture].name.c_str(),
        j, fx, fy, fz);
      RCLCPP_INFO(
        nodeHandle->get_logger(),
        "Min_Distance: %f",
        min_furniture_distance);

      bool within = (dx <= fx_max && dx >= fx_min && dy <= fy_max && dy >= fy_min);
      if (within)
      {
        RCLCPP_INFO(nodeHandle->get_logger(), "Within the furniture");
        std::string furniture_name = taskInfo.furniture[index_of_nearest_furniture].name;
        std::replace(furniture_name.begin(), furniture_name.end(), '_', ' ');

        std::string relation = getFurnitureRelation(
          furniture_name,
          dx, dy, dz,
          fx_min, fx_max,
          fy_min, fy_max,
          fz_min, fz_max);

        return "Place it " + relation;
      }
    }

    RCLCPP_INFO(nodeHandle->get_logger(), "Not within the furniture");
    std::string nearest_furniture_name = taskInfo.furniture[index_of_nearest_furniture].name;
    std::replace(nearest_furniture_name.begin(), nearest_furniture_name.end(), '_', ' ');

    return "Place it in/on the " + nearest_furniture_name;
  }

  std::string getInitialDestination(
    const human_navigation_msgs::msg::HumanNaviTaskInfo::ConstSharedPtr & message)
  {
    taskInfo = *message;

    double dx = taskInfo.target_object.position.x;
    double dy = taskInfo.target_object.position.y;
    double dz = taskInfo.target_object.position.z;

    int   index_of_nearest_furniture = 0;
    float min_distance               = 999999999.0f;

    int i = static_cast<int>(taskInfo.furniture.size());
    if (i <= 0)
    {
      RCLCPP_WARN(nodeHandle->get_logger(), "getInitialDestination: no furniture");
      return "Get " + getHumanReadableTargetName(targetObjectName) + ".";
    }
    for (int j = 0; j < i; ++j)
    {
      double fx = taskInfo.furniture[j].position.x;
      double fy = taskInfo.furniture[j].position.y;
      double fz = taskInfo.furniture[j].position.z;

      float distance = std::sqrt(
        std::pow(dx - fx, 2) +
        std::pow(dy - fy, 2) +
        std::pow(dz - fz, 2));

      if (distance < min_distance)
      {
        index_of_nearest_furniture = j;
        min_distance               = distance;
      }
    }

    std::string nearest_furniture = taskInfo.furniture[index_of_nearest_furniture].name;
    std::replace(nearest_furniture.begin(), nearest_furniture.end(), '_', ' ');

    int   index_of_nearest_non = 0;
    float min_distance_non     = 999999999.0f;

    int k = static_cast<int>(taskInfo.non_target_objects.size());
    for (int j = 0; j < k; ++j)
    {
      double fx = taskInfo.non_target_objects[j].position.x;
      double fy = taskInfo.non_target_objects[j].position.y;
      double fz = taskInfo.non_target_objects[j].position.z;

      float distance = std::sqrt(
        std::pow(dx - fx, 2) +
        std::pow(dy - fy, 2) +
        std::pow(dz - fz, 2));

      if (distance < min_distance_non)
      {
        index_of_nearest_non = j;
        min_distance_non     = distance;
      }
    }

    std::string readableTarget = getHumanReadableTargetName(targetObjectName);
    std::string nearest_non_target =
      (k > 0) ? taskInfo.non_target_objects[index_of_nearest_non].name : "";
    if (!nearest_non_target.empty())
    {
      std::replace(nearest_non_target.begin(), nearest_non_target.end(), '_', ' ');
    }

    if (k > 0 && min_distance_non < 0.4)
    {
      return "Get " + readableTarget + ", close to the " + nearest_furniture;
    }
    else
    {
      return "Get " + readableTarget + ", close to the " + nearest_furniture;
    }
  }

  // receive taskInfo from the moderator (Unity)
  // 原 ROS1: taskInfoMessageCallback(const HumanNaviTaskInfo::ConstPtr&) -> ROS2: ConstSharedPtr
  void taskInfoMessageCallback(
    const human_navigation_msgs::msg::HumanNaviTaskInfo::ConstSharedPtr message)
  {
    taskInfo = *message;

    RCLCPP_INFO_STREAM(
      nodeHandle->get_logger(),
      "Subscribe task info message:" << std::endl <<
      "Environment ID: " << taskInfo.environment_id);

    targetObjectName          = taskInfo.target_object.name;

    // 防止除零错误
    if (taskInfo.target_object.position.x != 0)
    {
      direction_target_object =
        std::atan(taskInfo.target_object.position.y / taskInfo.target_object.position.x);
    }
    else
    {
      direction_target_object = M_PI / 2.0;  // x=0时，默认90度
    }

    if (taskInfo.destination.position.x != 0)
    {
      direction_target_direction =
        std::atan(taskInfo.destination.position.y / taskInfo.destination.position.x);
    }
    else
    {
      direction_target_direction = M_PI / 2.0;  // x=0时，默认90度
    }

    final_location   = getFinalDestination(message);
    initial_location = getInitialDestination(message);

    // 在主循环中 polish（见 run()），避免在回调内嵌套 spin
    pending_polish_ = true;

    isTaskInfoReceived = true;
  }

  // receive avatar status
  void avatarStatusMessageCallback(
    const human_navigation_msgs::msg::HumanNaviAvatarStatus::ConstSharedPtr message)
  {
    avatarStatus = *message;

    RCLCPP_INFO_STREAM(
      nodeHandle->get_logger(),
      "Subscribe avatar status message:" << std::endl <<
      "objctInLeftHand: " << avatarStatus.object_in_left_hand << std::endl <<
      "objectInRightHand: " << avatarStatus.object_in_right_hand << std::endl <<
      "isTargetObjectInLeftHand: " << std::boolalpha
        << static_cast<bool>(avatarStatus.is_target_object_in_left_hand) << std::endl <<
      "isTargetObjectInRightHand: " << std::boolalpha
        << static_cast<bool>(avatarStatus.is_target_object_in_right_hand) << std::endl);

    isSentGetAvatarStatus = false;

    // 规则≤15条：取消实时距离，改为每隔 N 秒在骨架句后追加方位提示（Forward/Left/Right/Backward/Right here）
    const double now_sec = nodeHandle->now().seconds();
    // 抓取正确后保留字幕3秒，避免被新的周期字幕立刻覆盖
    if (subtitle_hold_until_sec_ > 0.0) {
      if (now_sec < subtitle_hold_until_sec_) {
        return;
      }
      // 保持窗口结束：重置计时起点，确保下一条周期字幕按完整间隔触发
      subtitle_hold_until_sec_ = 0.0;
      last_direction_hint_sec_ = now_sec;
    }
    const bool interval_elapsed = (now_sec - last_direction_hint_sec_) >= direction_hint_interval_sec_;

    if (step == GuideForTakingObject && interval_elapsed)
    {
      sendMessage(pubHumanNaviMsg, MSG_GET_AVATAR_STATUS);
      const std::string base = polished_pick_base_.empty() ? truncateUtf8(initial_location, 400)
                                                          : polished_pick_base_;
      const double av_x = avatarStatus.body.position.x;
      const double av_y = avatarStatus.body.position.y;
      const double av_z = avatarStatus.body.position.z;
      const double obj_x = taskInfo.target_object.position.x;
      const double obj_y = taskInfo.target_object.position.y;
      const double obj_z = taskInfo.target_object.position.z;
      const double dist_obj = std::sqrt(
        std::pow(av_x - obj_x, 2) + std::pow(av_y - obj_y, 2) + std::pow(av_z - obj_z, 2));
      std::string hint = getDirectionHint(av_x, av_y, obj_x, obj_y, avatarStatus.body.orientation);
      std::ostringstream dist_ss;
      dist_ss << std::fixed << std::setprecision(2) << dist_obj;
      guideMsg = base + "\n" + hint + "\nDistance to object: " + dist_ss.str() + " meters";
      sendGuidanceMessage(pubGuidanceMsg, guideMsg, DISPLAY_TYPE_ALL);
      last_direction_hint_sec_ = now_sec;
    }
    else if (step == GuideForPlacement && interval_elapsed)
    {
      sendMessage(pubHumanNaviMsg, MSG_GET_AVATAR_STATUS);
      const std::string base = polished_place_base_.empty() ? truncateUtf8(final_location, 400)
                                                           : polished_place_base_;
      const double av_x = avatarStatus.body.position.x;
      const double av_y = avatarStatus.body.position.y;
      const double av_z = avatarStatus.body.position.z;
      const double dest_x = taskInfo.destination.position.x;
      const double dest_y = taskInfo.destination.position.y;
      const double dest_z = taskInfo.destination.position.z;
      const double dist_dest = std::sqrt(
        std::pow(av_x - dest_x, 2) + std::pow(av_y - dest_y, 2) + std::pow(av_z - dest_z, 2));
      std::string hint = getDirectionHint(av_x, av_y, dest_x, dest_y, avatarStatus.body.orientation);
      std::ostringstream dist_ss;
      dist_ss << std::fixed << std::setprecision(2) << dist_dest;
      guideMsg = base + "\n" + hint + "\nDistance to destination: " + dist_ss.str() + " meters";
      sendGuidanceMessage(pubGuidanceMsg, guideMsg, DISPLAY_TYPE_ALL);
      last_direction_hint_sec_ = now_sec;
    }
  }

  void objectStatusMessageCallback(
    const human_navigation_msgs::msg::HumanNaviObjectStatus::ConstSharedPtr message)
  {
    objectStatus = *message;

    RCLCPP_INFO_STREAM(
      nodeHandle->get_logger(),
      "Subscribe object status message:" << std::endl <<
      "Target object size of non_target_objects: " <<
      taskInfo.non_target_objects.size());

    int numOfNonTargetObjects = static_cast<int>(taskInfo.non_target_objects.size());
    std::cout << "Number of non-target objects: " << numOfNonTargetObjects << std::endl;
    std::cout << "Non-target objects:" << std::endl;
    for (int i = 0; i < numOfNonTargetObjects; ++i)
    {
      // 原 ROS1 打印对象，ROS2 中简单输出名称或使用 to_yaml 也可，这里只输出 name 以保持简洁
      std::cout << taskInfo.non_target_objects[i].name << std::endl;
    }

    isSentGetObjectStatus = false;
  }

  // 原 ROS1: bool speakGuidanceMessage(ros::Publisher, ros::Publisher, ...) ->
  // ROS2: 使用 Publisher SharedPtr，时间改为 nodeHandle->now()
  bool speakGuidanceMessage(
    const rclcpp::Publisher<human_navigation_msgs::msg::HumanNaviMsg>::SharedPtr & pubHumanNaviMsgLocal,
    const rclcpp::Publisher<human_navigation_msgs::msg::HumanNaviGuidanceMsg>::SharedPtr & pubGuidanceMsgLocal,
    const std::string & message,
    int interval = 1)
  {
    if (speechState == SpeechState::Speakable)
    {
      sendGuidanceMessage(pubGuidanceMsgLocal, message, DISPLAY_TYPE_ALL);
      speechState = SpeechState::None;
      return true;
    }
    else if (speechState == SpeechState::None || speechState == SpeechState::Speaking)
    {
      if ((timePrevSpeechStateConfirmed.seconds() + interval) < nodeHandle->now().seconds())
      {
        sendMessage(pubHumanNaviMsgLocal, MSG_GET_SPEECH_STATE);
        timePrevSpeechStateConfirmed = nodeHandle->now();
        speechState = SpeechState::Speakable;
      }
    }

    return false;
  }

public:
  int run(int argc, char ** argv)
  {
    (void)argc;
    (void)argv;

    // 原 ROS1: ros::init + ros::NodeHandle -> ROS2: rclcpp::init 在 main 中，类内部创建 Node
    nodeHandle = std::make_shared<rclcpp::Node>("human_navi_sample");

    // 初始化 tf2 Buffer/Listener（替代 ROS1 的 tf::TransformListener）
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(nodeHandle->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    rclcpp::Rate loopRate(10.0);  // 原 ROS1: ros::Rate loopRate(10);

    init();

    RCLCPP_INFO(nodeHandle->get_logger(), "Human Navi sample start!");

    // ROS2 订阅/发布器创建，对应 ROS1 NodeHandle.subscribe/advertise
    subHumanNaviMsg = nodeHandle->create_subscription<human_navigation_msgs::msg::HumanNaviMsg>(
      "/human_navigation/message/to_robot",
      100,
      std::bind(&HumanNavigationSample::messageCallback, this, std::placeholders::_1));
    subTaskInfoMsg = nodeHandle->create_subscription<human_navigation_msgs::msg::HumanNaviTaskInfo>(
      "/human_navigation/message/task_info",
      1,
      std::bind(&HumanNavigationSample::taskInfoMessageCallback, this, std::placeholders::_1));
    subAvatarStatusMsg = nodeHandle->create_subscription<human_navigation_msgs::msg::HumanNaviAvatarStatus>(
      "/human_navigation/message/avatar_status",
      1,
      std::bind(&HumanNavigationSample::avatarStatusMessageCallback, this, std::placeholders::_1));
    subObjectStatusMsg = nodeHandle->create_subscription<human_navigation_msgs::msg::HumanNaviObjectStatus>(
      "/human_navigation/message/object_status",
      1,
      std::bind(&HumanNavigationSample::objectStatusMessageCallback, this, std::placeholders::_1));

    pubHumanNaviMsg = nodeHandle->create_publisher<human_navigation_msgs::msg::HumanNaviMsg>(
      "/human_navigation/message/to_moderator",
      10);
    pubGuidanceMsg = nodeHandle->create_publisher<human_navigation_msgs::msg::HumanNaviGuidanceMsg>(
      "/human_navigation/message/guidance_message",
      10);
    pub_base_twist_ = nodeHandle->create_publisher<geometry_msgs::msg::Twist>(
      "/hsrb/command_velocity",
      10);
    pub_base_trajectory_ = nodeHandle->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/hsrb/omni_base_controller/command",
      10);

    nodeHandle->declare_parameter("use_llm_rewrite", false);
    nodeHandle->declare_parameter("llm_timeout_sec", 2.0);
    nodeHandle->declare_parameter("llm_service_name", "/rewrite_guidance");
    nodeHandle->declare_parameter("strict_template_mode", true);
    nodeHandle->declare_parameter("direction_hint_interval_sec", 7.0);
    use_llm_rewrite_ = nodeHandle->get_parameter("use_llm_rewrite").as_bool();
    llm_timeout_sec_ = nodeHandle->get_parameter("llm_timeout_sec").as_double();
    llm_service_name_ = nodeHandle->get_parameter("llm_service_name").as_string();
    strict_template_mode_ = nodeHandle->get_parameter("strict_template_mode").as_bool();
    direction_hint_interval_sec_ = nodeHandle->get_parameter("direction_hint_interval_sec").as_double();
    if (use_llm_rewrite_) {
      llm_client_ = nodeHandle->create_client<human_nav_llm_ros2::srv::RewriteGuidance>(
        llm_service_name_);
      RCLCPP_INFO(
        nodeHandle->get_logger(),
        "use_llm_rewrite=true, strict_template_mode=%s, service=%s timeout=%.2fs",
        strict_template_mode_ ? "true" : "false",
        llm_service_name_.c_str(),
        llm_timeout_sec_);
    } else {
      RCLCPP_INFO(nodeHandle->get_logger(), "use_llm_rewrite=false (guidance text unchanged except truncation)");
    }

    timePrevSpeechStateConfirmed = nodeHandle->now();

    rclcpp::Time time = nodeHandle->now();

    while (rclcpp::ok())
    {
      rclcpp::spin_some(nodeHandle);

      if (pending_polish_) {
        const std::string ctx = buildContextJson();
        const std::string draft_pick  = buildStrictPickTemplate();
        const std::string draft_place = buildStrictPlaceTemplate();
        if (use_llm_rewrite_ && !strict_template_mode_) {
          polished_pick_base_  = polishGuidance(draft_pick, "pick", ctx);
          polished_place_base_ = polishGuidance(draft_place, "place", ctx);
        } else {
          polished_pick_base_  = truncateUtf8(draft_pick, 400);
          polished_place_base_ = truncateUtf8(draft_place, 400);
        }
        pending_polish_ = false;
        RCLCPP_INFO(
          nodeHandle->get_logger(),
          "Polished guidance bases (pick len=%zu, place len=%zu)",
          polished_pick_base_.size(),
          polished_place_base_.size());
      }

      switch (step)
      {
        case Initialize:
        {
          reset();
          RCLCPP_INFO(nodeHandle->get_logger(), "##### Initialized ######");
          step++;
          break;
        }
        case Ready:
        {
          if (isStarted)
          {
            step++;
            RCLCPP_INFO(nodeHandle->get_logger(), "Task start");
          }
          break;
        }
        case WaitTaskInfo:
        {
          if (isTaskInfoReceived)
          {
            RCLCPP_INFO(nodeHandle->get_logger(), "Task Received");
            // session start 后立即朝目标物品方向转向
            moveBaseJointTrajectory(pub_base_trajectory_, 0.0, 0.0, direction_target_object, 5.0);
            isRequestReceived = true;
            avatar_timer      = nodeHandle->now().seconds();
            step++;
          }
          break;
        }
        case GuideForTakingObject:
        {
          RCLCPP_INFO(nodeHandle->get_logger(), "GuideForTakingObject");

          if (nodeHandle->now().seconds() - avatar_timer > 0.100)
          {
            sendMessage(pubHumanNaviMsg, MSG_GET_AVATAR_STATUS);
            avatar_timer = nodeHandle->now().seconds();

            if (static_cast<bool>(avatarStatus.is_target_object_in_left_hand) ||
                static_cast<bool>(avatarStatus.is_target_object_in_right_hand))
            {
              guideMsg = "Please Keep holding the Object";
              while (!speakGuidanceMessage(pubHumanNaviMsg, pubGuidanceMsg, guideMsg))
              {
                RCLCPP_INFO(nodeHandle->get_logger(), "still waiting");
                rclcpp::spin_some(nodeHandle);
              }
              // 该提示保留3秒，期间字幕更新计时器视为0，防止立即被覆盖
              last_direction_hint_sec_ = 0.0;
              subtitle_hold_until_sec_ = nodeHandle->now().seconds() + 3.0;

              // 抓取正确后立即朝第三阶段目的地方向转向，给受试者即时方向提示
              moveBaseJointTrajectory(
                pub_base_trajectory_,
                0.0, 0.0,
                direction_target_direction,
                5.0);

              step             = GuideForPlacement;
              isRequestReceived = true;
              last_wrong_object_name_.clear();
              break;
            }
            else if (!avatarStatus.object_in_left_hand.empty() ||
                     !avatarStatus.object_in_right_hand.empty())
            {
              const std::string wrong_object = !avatarStatus.object_in_left_hand.empty()
                                             ? avatarStatus.object_in_left_hand
                                             : avatarStatus.object_in_right_hand;
              if (wrong_object != last_wrong_object_name_)
              {
                guideMsg = "This is not the object, Please try again!";
                while (!speakGuidanceMessage(pubHumanNaviMsg, pubGuidanceMsg, guideMsg))
                {
                  RCLCPP_INFO(nodeHandle->get_logger(), "still waiting_2");
                  rclcpp::spin_some(nodeHandle);
                }
                last_wrong_object_name_ = wrong_object;
              }
              break;
            }
          }

          if (isRequestReceived)
          {
            moveBaseJointTrajectory(pub_base_trajectory_, 0.0, 0.0, direction_target_object, 5.0);

            guideMsg = polished_pick_base_.empty() ? truncateUtf8(initial_location, 400)
                                                  : polished_pick_base_;

            if (static_cast<bool>(avatarStatus.is_target_object_in_left_hand) ||
                static_cast<bool>(avatarStatus.is_target_object_in_right_hand))
            {
              double obj_x  = taskInfo.target_object.position.x;
              double obj_y  = taskInfo.target_object.position.y;
              double obj_z  = taskInfo.target_object.position.z;
              double dest_x = taskInfo.destination.position.x;
              double dest_y = taskInfo.destination.position.y;
              double dest_z = taskInfo.destination.position.z;

              double distance = std::sqrt(
                std::pow(obj_x - dest_x, 2) +
                std::pow(obj_y - dest_y, 2) +
                std::pow(obj_z - dest_z, 2));

              (void)distance;  // 原 ROS1 中只是计算并打印标记，这里保持逻辑但不额外输出
              RCLCPP_INFO(nodeHandle->get_logger(), "Distance obj->dest computed");
            }

            if (speakGuidanceMessage(pubHumanNaviMsg, pubGuidanceMsg, guideMsg))
            {
              time            = nodeHandle->now();
              isRequestReceived = false;
            }
          }
          break;
        }
        case GuideForPlacement:
        {
          RCLCPP_INFO(nodeHandle->get_logger(), "GuideForPlacement");

          // A: 放置阶段也持续请求 avatar status，保证方位/距离提示可持续更新
          if (nodeHandle->now().seconds() - avatar_timer > 0.100)
          {
            sendMessage(pubHumanNaviMsg, MSG_GET_AVATAR_STATUS);
            avatar_timer = nodeHandle->now().seconds();
          }

          if (isRequestReceived)
          {
            moveBaseJointTrajectory(
              pub_base_trajectory_,
              0.0, 0.0,
              direction_target_direction,
              5.0);

            // 放置阶段发送“放到哪里”的指令（不能沿用 "Please Keep holding the Object"）
            guideMsg = polished_place_base_.empty() ? truncateUtf8(final_location, 400)
                                                   : polished_place_base_;
            // 直接发送一次，确保 Unity 立即显示，不依赖 TTS 状态
            sendGuidanceMessage(pubGuidanceMsg, guideMsg, DISPLAY_TYPE_ALL);
            isRequestReceived = false;
          }

          break;
        }
        case TaskFinished:
        {
          reset();
          break;
        }
      }

      loopRate.sleep();
    }

    return 0;
  }
};

std::string HumanNavigationSample::truncateUtf8(const std::string & s, size_t max_chars)
{
  size_t n = 0;
  size_t i = 0;
  while (i < s.size() && n < max_chars) {
    const unsigned char c = static_cast<unsigned char>(s[i]);
    size_t w = 1;
    if (c >= 0xF0U) {
      w = 4;
    } else if (c >= 0xE0U) {
      w = 3;
    } else if (c >= 0xC0U) {
      w = 2;
    } else if (c >= 0x80U) {
      ++i;
      continue;
    }
    if (i + w > s.size()) {
      break;
    }
    i += w;
    ++n;
  }
  return s.substr(0, i);
}

std::string HumanNavigationSample::escapeJsonString(const std::string & s)
{
  std::string o;
  o.reserve(s.size() + 8);
  for (const char ch : s) {
    if (ch == '"' || ch == '\\') {
      o += '\\';
    }
    o += ch;
  }
  return o;
}

std::string HumanNavigationSample::toReadableName(const std::string & name)
{
  std::string s = name;
  while (!s.empty() && std::isdigit(static_cast<unsigned char>(s.back())))
  {
    s.pop_back();
  }
  if (!s.empty() && s.back() == '_')
  {
    s.pop_back();
  }
  std::replace(s.begin(), s.end(), '_', ' ');
  return s;
}

std::string HumanNavigationSample::toLowerAscii(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return s;
}

bool HumanNavigationSample::isLargeLandmarkName(const std::string & readable_name_lower)
{
  static const std::vector<std::string> kKeywords = {
    "table", "desk", "counter", "cabinet", "cupboard",
    "drawer", "shelf", "wardrobe", "refrigerator", "fridge"};
  for (const auto & kw : kKeywords) {
    if (readable_name_lower.find(kw) != std::string::npos) {
      return true;
    }
  }
  return false;
}

bool HumanNavigationSample::isRefrigeratorName(const std::string & readable_name_lower)
{
  return readable_name_lower.find("refrigerator") != std::string::npos ||
         readable_name_lower.find("fridge") != std::string::npos;
}

std::string HumanNavigationSample::getRelationPhraseForPosition(
  double dx, double dy, double dz,
  double size_x, double size_y, double size_z) const
{
  if (taskInfo.furniture.empty()) {
    return "";
  }
  int idx = 0;
  float best = 999999999.0f;
  for (size_t i = 0; i < taskInfo.furniture.size(); ++i) {
    const auto & f = taskInfo.furniture[i];
    float d = static_cast<float>(std::sqrt(
      std::pow(dx - f.position.x, 2) +
      std::pow(dy - f.position.y, 2) +
      std::pow(dz - f.position.z, 2)));
    if (d < best) {
      best = d;
      idx = static_cast<int>(i);
    }
  }
  const auto & fn = taskInfo.furniture[idx];
  double fx = fn.position.x, fy = fn.position.y, fz = fn.position.z;
  double fx_min = fx - std::abs(size_x) / 2.0;
  double fx_max = fx + std::abs(size_x) / 2.0;
  double fy_min = fy - std::abs(size_y) / 2.0;
  double fy_max = fy + std::abs(size_y) / 2.0;
  double fz_min = fz - std::abs(size_z);
  double fz_max = fz + std::abs(size_z);

  std::string furniture_name = fn.name;
  std::replace(furniture_name.begin(), furniture_name.end(), '_', ' ');

  bool within = (dx <= fx_max && dx >= fx_min && dy <= fy_max && dy >= fy_min);
  if (within) {
    return getFurnitureRelation(
      furniture_name, dx, dy, dz,
      fx_min, fx_max, fy_min, fy_max, fz_min, fz_max);
  }
  return "near the " + furniture_name;
}

std::string HumanNavigationSample::nearestFurnitureName(double x, double y, double z) const
{
  if (taskInfo.furniture.empty()) {
    return "";
  }
  size_t idx = 0;
  double best = 1e12;
  for (size_t i = 0; i < taskInfo.furniture.size(); ++i) {
    const auto & f = taskInfo.furniture[i];
    const double d = std::sqrt(
      std::pow(x - f.position.x, 2) +
      std::pow(y - f.position.y, 2) +
      std::pow(z - f.position.z, 2));
    if (d < best) {
      best = d;
      idx = i;
    }
  }
  return "the " + toReadableName(taskInfo.furniture[idx].name);
}

std::string HumanNavigationSample::nearestLargeLandmarkName(
  double x, double y, double z,
  double min_distance_m,
  double max_distance_m,
  const std::string & avoid_name_lower) const
{
  std::string best_name;
  double best = 1e12;

  auto consider_name = [&](const std::string & raw_name, double ox, double oy, double oz) {
    const double d = std::sqrt(
      std::pow(x - ox, 2) +
      std::pow(y - oy, 2) +
      std::pow(z - oz, 2));
    if (d < min_distance_m || d > max_distance_m) {
      return;
    }
    const std::string readable = toReadableName(raw_name);
    const std::string readable_lower = toLowerAscii(readable);
    if (!isLargeLandmarkName(readable_lower)) {
      return;
    }
    const std::string cand = "the " + readable;
    const std::string cand_lower = toLowerAscii(cand);
    if (!avoid_name_lower.empty() && cand_lower == avoid_name_lower) {
      return;
    }
    if (d < best) {
      best = d;
      best_name = cand;
    }
  };

  for (const auto & f : taskInfo.furniture) {
    consider_name(f.name, f.position.x, f.position.y, f.position.z);
  }
  for (const auto & o : taskInfo.non_target_objects) {
    consider_name(o.name, o.position.x, o.position.y, o.position.z);
  }
  return best_name;
}

// near sth：以中心点(x,y,z)为基准，在 min~max 米内扫描所有大型不可交互物品，
// 记录最近的两项；若最近的一项与 on 后的物品重合，则用第二近的
std::string HumanNavigationSample::nearestNearLandmarkName(
  double x, double y, double z,
  double min_distance_m,
  double max_distance_m,
  const std::string & avoid_name_lower) const
{
  double d1 = 1e12, d2 = 1e12;
  std::string name1, name2;

  auto consider = [&](const std::string & raw_name, double ox, double oy, double oz) {
    const double d = std::sqrt(
      std::pow(x - ox, 2) + std::pow(y - oy, 2) + std::pow(z - oz, 2));
    if (d < min_distance_m || d > max_distance_m) {
      return;
    }
    const std::string readable = toReadableName(raw_name);
    const std::string cand = "the " + readable;
    const std::string cand_lower = toLowerAscii(cand);
    if (!isLargeLandmarkName(cand_lower)) {
      return;  // 仅大型不可交互：table, cabinet, shelf, refrigerator 等
    }
    // 只记录“不同物品名”的前两近，避免同名占据第1/第2导致误选
    if (!name1.empty() && cand_lower == toLowerAscii(name1)) {
      return;
    }
    if (d < d1) {
      d2 = d1;
      name2 = name1;
      d1 = d;
      name1 = cand;
    } else if (d < d2) {
      d2 = d;
      name2 = cand;
    }
  };

  for (const auto & f : taskInfo.furniture) {
    consider(f.name, f.position.x, f.position.y, f.position.z);
  }
  for (const auto & o : taskInfo.non_target_objects) {
    consider(o.name, o.position.x, o.position.y, o.position.z);
  }

  if (name1.empty()) {
    return "";
  }
  if (avoid_name_lower.empty() || toLowerAscii(name1) != avoid_name_lower) {
    return name1;  // 最近的一项不与 on 重合，用最近的
  }
  if (!name2.empty() && toLowerAscii(name2) != avoid_name_lower) {
    return name2;  // 最近的一项与 on 重合，用第二近的（且第二近也不与 on 重合）
  }
  return "";  // 第二近也为空或与 on 重合，不加 near
}

std::string HumanNavigationSample::buildStrictPickTemplate() const
{
  const std::string target = "the " + toReadableName(taskInfo.target_object.name);
  const double tx = taskInfo.target_object.position.x;
  const double ty = taskInfo.target_object.position.y;
  const double tz = taskInfo.target_object.position.z;
  // 抓取模板严格固定为: "Please grab <target> on <A>, near <B>..."
  // 因此这里强制主介词为 on，仅取 furniture 名词位，避免出现 "near ..., near ..."
  std::string on_target = nearestFurnitureName(tx, ty, tz);  // "the <furniture>"
  std::string on_phrase = on_target.empty() ? "" : ("on " + on_target);
  std::string avoid;
  if (!on_phrase.empty()) {
    const size_t pos = on_phrase.find("the ");
    avoid = (pos != std::string::npos) ? toLowerAscii(on_phrase.substr(pos)) : "";
  }
  std::string nearby = nearestNearLandmarkName(tx, ty, tz, 0.0, 3.0, avoid);

  if (!on_phrase.empty() && !nearby.empty()) {
    return "Please grab " + target + " " + on_phrase + ", near " + nearby +
           ",where the robot points.";
  }
  if (!on_phrase.empty()) {
    return "Please grab " + target + " " + on_phrase + ",where the robot points.";
  }
  if (!nearby.empty()) {
    return "Please grab " + target + " near " + nearby + ",where the robot points.";
  }
  return "Please grab " + target + ",where the robot points.";
}

std::string HumanNavigationSample::buildStrictPlaceTemplate() const
{
  const double dx = taskInfo.destination.position.x;
  const double dy = taskInfo.destination.position.y;
  const double dz = taskInfo.destination.position.z;
  // 放置模板严格固定为: "Please place it on <A>, near <B>..."
  // 强制主介词为 on，仅取 furniture 名词位，避免出现 "place it near ..."
  std::string on_target = nearestFurnitureName(dx, dy, dz);  // "the <furniture>"
  std::string on_phrase = on_target.empty() ? "" : ("on " + on_target);
  std::string avoid;
  if (!on_phrase.empty()) {
    const size_t pos = on_phrase.find("the ");
    avoid = (pos != std::string::npos) ? toLowerAscii(on_phrase.substr(pos)) : "";
  }
  std::string nearby = nearestNearLandmarkName(dx, dy, dz, 0.0, 3.0, avoid);

  if (!on_phrase.empty() && !nearby.empty()) {
    return "Please place it " + on_phrase + ", near " + nearby + ",where the robot points.";
  }
  if (!on_phrase.empty()) {
    return "Please place it " + on_phrase + ",where the robot points.";
  }
  if (!nearby.empty()) {
    return "Please place it near " + nearby + ",where the robot points.";
  }
  return "Please place it where the robot points.";
}

std::string HumanNavigationSample::getDirectionHint(
  double av_x, double av_y,
  double target_x, double target_y,
  const geometry_msgs::msg::Quaternion & body_orient) const
{
  const double dx = target_x - av_x;
  const double dy = target_y - av_y;
  const double dist = std::sqrt(dx * dx + dy * dy);

  constexpr double NEAR_THRESHOLD = 0.5;
  if (dist < NEAR_THRESHOLD) {
    return "It's right here.";
  }

  tf2::Quaternion tf_q;
  tf2::fromMsg(body_orient, tf_q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);

  const double forward_x = std::cos(yaw);
  const double forward_y = std::sin(yaw);
  const double to_target_angle = std::atan2(dy, dx);
  double rel = to_target_angle - std::atan2(forward_y, forward_x);
  while (rel > M_PI) rel -= 2.0 * M_PI;
  while (rel < -M_PI) rel += 2.0 * M_PI;

  const double deg45 = M_PI / 4.0;
  const double deg135 = 3.0 * M_PI / 4.0;
  if (rel >= -deg45 && rel <= deg45) return "Go forward.";
  // 坐标系与受试者体感方向相反，左右提示在此交换
  if (rel > deg45 && rel <= deg135) return "Go left.";
  if (rel < -deg45 && rel >= -deg135) return "Go right.";
  return "Go backward.";
}

std::string HumanNavigationSample::buildContextJson() const
{
  const double tx = taskInfo.target_object.position.x;
  const double ty = taskInfo.target_object.position.y;
  const double tz = taskInfo.target_object.position.z;
  const double dx = taskInfo.destination.position.x;
  const double dy = taskInfo.destination.position.y;
  const double dz = taskInfo.destination.position.z;

  std::string pick_on = nearestFurnitureName(tx, ty, tz);   // "the <furniture>"
  std::string place_on = nearestFurnitureName(dx, dy, dz);

  std::string pick_near = nearestNearLandmarkName(tx, ty, tz, 0.0, 3.0, toLowerAscii(pick_on));
  std::string place_near = nearestNearLandmarkName(dx, dy, dz, 0.0, 3.0, toLowerAscii(place_on));

  std::ostringstream oss;
  oss << "{\"environment_id\":\"" << escapeJsonString(taskInfo.environment_id) << "\""
      << ",\"target_prefab\":\"" << escapeJsonString(taskInfo.target_object.name) << "\""
      << ",\"pick_on\":\"" << escapeJsonString(pick_on) << "\""
      << ",\"pick_near\":\"" << escapeJsonString(pick_near) << "\""
      << ",\"place_on\":\"" << escapeJsonString(place_on) << "\""
      << ",\"place_near\":\"" << escapeJsonString(place_near) << "\"}";
  return oss.str();
}

std::string HumanNavigationSample::polishGuidance(
  const std::string & draft, const std::string & phase, const std::string & context_json)
{
  if (draft.empty()) {
    return draft;
  }
  if (!use_llm_rewrite_ || !llm_client_) {
    return truncateUtf8(draft, 400);
  }
  if (!llm_client_->wait_for_service(std::chrono::milliseconds(500))) {
    RCLCPP_WARN(
      nodeHandle->get_logger(),
      "LLM service not available within 500ms, using draft");
    return truncateUtf8(draft, 400);
  }

  auto request = std::make_shared<human_nav_llm_ros2::srv::RewriteGuidance::Request>();
  request->draft = draft;
  request->phase = phase;
  request->context_json = context_json.empty() ? "{}" : context_json;

  auto future = llm_client_->async_send_request(request);
  const std::chrono::duration<double> timeout(llm_timeout_sec_);
  const auto ret = rclcpp::spin_until_future_complete(nodeHandle, future, timeout);
  if (ret != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_WARN(nodeHandle->get_logger(), "LLM rewrite future failed or timed out, using draft");
    return truncateUtf8(draft, 400);
  }

  const human_nav_llm_ros2::srv::RewriteGuidance::Response::SharedPtr response = future.get();
  if (!response || !response->success || response->rewritten.empty()) {
    RCLCPP_WARN(nodeHandle->get_logger(), "LLM rewrite returned failure or empty, using draft");
    return truncateUtf8(draft, 400);
  }

  const std::string rewritten = truncateUtf8(response->rewritten, 400);

  // D: output guardrail. If rewritten text does not match the expected stage pattern, fallback to draft.
  auto toLower = [](std::string s) {
      std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
      });
      return s;
    };
  const std::string low = toLower(rewritten);
  const std::string phase_low = toLower(phase);

  bool valid = true;
  // 禁止两物品拼接：如 cafe set + mota table -> "cafe set mota table"
  const bool fused_items = (low.find("set mota") != std::string::npos) ||
                          (low.find("coffee set mota") != std::string::npos) ||
                          (low.find("cafe set mota") != std::string::npos) ||
                          (low.find("desk lamp") != std::string::npos);
  if (fused_items) {
    valid = false;
  } else if (phase_low == "pick") {
    const bool has_prefix = low.find("please grab the") != std::string::npos;
    const bool has_on = low.find(" on ") != std::string::npos;  // 主位必须 on
    const bool near_clause_ok = (low.find(" near ") == std::string::npos) ||
                                (low.find(", near ") != std::string::npos);
    valid = has_prefix && has_on && near_clause_ok;
  } else if (phase_low == "place") {
    const bool has_prefix = low.find("please place it") != std::string::npos;
    const bool has_on = low.find(" on ") != std::string::npos;  // 主位必须 on
    const bool has_pointing = low.find("where the robot points") != std::string::npos;
    const bool near_clause_ok = (low.find(" near ") == std::string::npos) ||
                                (low.find(", near ") != std::string::npos);
    valid = has_prefix && has_on && has_pointing && near_clause_ok;
  }

  if (!valid) {
    RCLCPP_WARN(
      nodeHandle->get_logger(),
      "LLM rewrite rejected by stage guardrail (phase=%s), using draft",
      phase.c_str());
    return truncateUtf8(draft, 400);
  }

  return rewritten;
}

int main(int argc, char ** argv)
{
  // 原 ROS1: ros::init(argc, argv, "human_navi_sample");
  // ROS2: 使用 rclcpp::init / shutdown
  rclcpp::init(argc, argv);

  HumanNavigationSample humanNaviSample;
  int ret = humanNaviSample.run(argc, argv);

  rclcpp::shutdown();
  return ret;
}

