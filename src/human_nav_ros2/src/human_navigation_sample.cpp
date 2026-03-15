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

// 原 ROS1: tf/transform_listener.h -> ROS2: tf2_ros + tf2_geometry_msgs
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>
#include <string>
#include <algorithm>
#include <sstream>

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
    human_navigation_msgs::msg::HumanNaviGuidanceMsg guidanceMessage;
    guidanceMessage.message         = message;
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
                                   double fz_min, double fz_max)
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

    // 2025 版本新增逻辑：在 GuideForTakingObject/GuideForPlacement 阶段周期性给出距离反馈
    if (step == GuideForTakingObject)
    {
      questcounter++;
      if (questcounter > 10)
      {
        sendMessage(pubHumanNaviMsg, MSG_GET_AVATAR_STATUS);
        double avatar_x = avatarStatus.body.position.x;
        double avatar_y = avatarStatus.body.position.y;
        double avatar_z = avatarStatus.body.position.z;
        double obj_x    = taskInfo.target_object.position.x;
        double obj_y    = taskInfo.target_object.position.y;
        double obj_z    = taskInfo.target_object.position.z;
        double dist_avatar_object = std::sqrt(
          std::pow(avatar_x - obj_x, 2) +
          std::pow(avatar_y - obj_y, 2) +
          std::pow(avatar_z - obj_z, 2));

        guideMsg = initial_location + "\n" +
                   "Distance to object :" + std::to_string(dist_avatar_object) + " meters";
        sendGuidanceMessage(pubGuidanceMsg, guideMsg, DISPLAY_TYPE_ALL);
        questcounter = 0;
      }
    }
    else if (step == GuideForPlacement)
    {
      questcounter++;
      if (questcounter > 10)
      {
        sendMessage(pubHumanNaviMsg, MSG_GET_AVATAR_STATUS);
        double avatar_x = avatarStatus.body.position.x;
        double avatar_y = avatarStatus.body.position.y;
        double avatar_z = avatarStatus.body.position.z;
        double dest_x   = taskInfo.destination.position.x;
        double dest_y   = taskInfo.destination.position.y;
        double dest_z   = taskInfo.destination.position.z;
        double dist_avatar_dest = std::sqrt(
          std::pow(avatar_x - dest_x, 2) +
          std::pow(avatar_y - dest_y, 2) +
          std::pow(avatar_z - dest_z, 2));

        std::ostringstream oss;
        oss << "Distance to destination: " << dist_avatar_dest << " meters\n";
        oss << "body: (" << avatarStatus.body.position.x << ", "
            << avatarStatus.body.position.y << ", "
            << avatarStatus.body.position.z << ")\n";

        guideMsg = final_location + "\n" + oss.str();
        sendGuidanceMessage(pubGuidanceMsg, guideMsg, DISPLAY_TYPE_ALL);
        questcounter = 0;
      }
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

    timePrevSpeechStateConfirmed = nodeHandle->now();

    rclcpp::Time time = nodeHandle->now();

    while (rclcpp::ok())
    {
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

              step             = GuideForPlacement;
              isRequestReceived = true;
              break;
            }
            else if (!avatarStatus.object_in_left_hand.empty() ||
                     !avatarStatus.object_in_right_hand.empty())
            {
              guideMsg = "This is not the object, Please try again!";
              while (!speakGuidanceMessage(pubHumanNaviMsg, pubGuidanceMsg, guideMsg))
              {
                RCLCPP_INFO(nodeHandle->get_logger(), "still waiting_2");
                rclcpp::spin_some(nodeHandle);
              }
              break;
            }
          }

          if (isRequestReceived)
          {
            moveBaseJointTrajectory(pub_base_trajectory_, 0.0, 0.0, direction_target_object, 5.0);

            guideMsg = initial_location + final_location;

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

          if (isRequestReceived)
          {
            moveBaseJointTrajectory(
              pub_base_trajectory_,
              0.0, 0.0,
              direction_target_direction,
              5.0);

            // 放置阶段发送“放到哪里”的指令（不能沿用 "Please Keep holding the Object"）
            guideMsg = final_location;
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

      rclcpp::spin_some(nodeHandle);
      loopRate.sleep();
    }

    return 0;
  }
};

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

