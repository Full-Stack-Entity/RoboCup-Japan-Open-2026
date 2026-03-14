/**
 * teleop_key_handyman.cpp  —  ROS 2 Humble
 *
 * Complete migration from ROS1. All original keyboard controls preserved:
 *   arrow keys  : rotate/move base (Twist)
 *   u/i/o/j/k/l/m/,/. : omni base trajectory moves
 *   y/h/n       : torso up/stop/down
 *   a/b/c/d     : arm presets (vertical/upward/horizontal/downward)
 *   g           : toggle grasp/open hand
 *   q/z         : speed up/down
 *   0-3,6,9     : send protocol messages
 *
 * ROS2 changes:
 *   ros::NodeHandle  -> rclcpp::Node
 *   ros::Publisher   -> rclcpp::Publisher
 *   ros::Subscriber  -> rclcpp::Subscription
 *   tf::TransformListener -> tf2_ros::TransformListener
 *   handyman::HandymanMsg -> handyman::msg::HandymanMsg
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <handyman_ros2/msg/handyman_msg.hpp>

#include <cmath>
#include <cstring>
#include <signal.h>
#include <stdio.h>
#include <termios.h>

using HandymanMsg = handyman_ros2::msg::HandymanMsg;

// ---------------------------------------------------------------------------
class HandymanTeleopKey : public rclcpp::Node
{
public:
  HandymanTeleopKey();
  int run();

private:
  // Key codes (same as original)
  static const char KEYCODE_0 = 0x30;
  static const char KEYCODE_1 = 0x31;
  static const char KEYCODE_2 = 0x32;
  static const char KEYCODE_3 = 0x33;
  static const char KEYCODE_6 = 0x36;
  static const char KEYCODE_9 = 0x39;
  static const char KEYCODE_UP    = 0x41;
  static const char KEYCODE_DOWN  = 0x42;
  static const char KEYCODE_RIGHT = 0x43;
  static const char KEYCODE_LEFT  = 0x44;
  static const char KEYCODE_A = 0x61;
  static const char KEYCODE_B = 0x62;
  static const char KEYCODE_C = 0x63;
  static const char KEYCODE_D = 0x64;
  static const char KEYCODE_G = 0x67;
  static const char KEYCODE_H = 0x68;
  static const char KEYCODE_I = 0x69;
  static const char KEYCODE_J = 0x6a;
  static const char KEYCODE_K = 0x6b;
  static const char KEYCODE_L = 0x6c;
  static const char KEYCODE_M = 0x6d;
  static const char KEYCODE_N = 0x6e;
  static const char KEYCODE_O = 0x6f;
  static const char KEYCODE_Q = 0x71;
  static const char KEYCODE_U = 0x75;
  static const char KEYCODE_Y = 0x79;
  static const char KEYCODE_Z = 0x7a;
  static const char KEYCODE_COMMA  = 0x2c;
  static const char KEYCODE_PERIOD = 0x2e;
  static const char KEYCODE_SPACE  = 0x20;

  // Protocol messages
  const std::string MSG_ARE_YOU_READY    = "Are_you_ready?";
  const std::string MSG_ENVIRONMENT      = "Environment";
  const std::string MSG_TASK_SUCCEEDED   = "Task_succeeded";
  const std::string MSG_TASK_FAILED      = "Task_failed";
  const std::string MSG_MISSION_COMPLETE = "Mission_complete";
  const std::string MSG_I_AM_READY       = "I_am_ready";
  const std::string MSG_ROOM_REACHED     = "Room_reached";
  const std::string MSG_DOES_NOT_EXIST   = "Does_not_exist";
  const std::string MSG_OBJECT_GRASPED   = "Object_grasped";
  const std::string MSG_TASK_FINISHED    = "Task_finished";
  const std::string MSG_GIVE_UP          = "Give_up";

  bool is_received_are_you_ready_;
  bool is_received_environment_;

  double arm_lift_joint_pos1_;
  double arm_lift_joint_pos2_;
  double arm_flex_joint_pos_;
  double wrist_flex_joint_pos_;

  rclcpp::Subscription<HandymanMsg>::SharedPtr       sub_msg_;
  rclcpp::Publisher<HandymanMsg>::SharedPtr          pub_msg_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_state_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr       pub_base_twist_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_base_trajectory_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_arm_trajectory_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_gripper_trajectory_;

  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  static int  canReceive(int fd);
  void messageCallback(const HandymanMsg::SharedPtr message);
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr joint_state);
  void sendMessage(const std::string &message);
  void moveBaseTwist(double linear_x, double linear_y, double angular_z);
  void moveBaseJointTrajectory(double linear_x, double linear_y, double theta, double duration_sec);
  void operateArm(double arm_lift_pos, double arm_flex_pos, double wrist_flex_pos, double duration_sec);
  void operateArmByName(const std::string &name, double position, double duration_sec);
  void operateArmFlex(double arm_flex_pos, double wrist_flex_pos);
  double getDurationRot(double next_pos, double current_pos);
  void operateHand(bool grasp);
  void showHelp();
};

// ---------------------------------------------------------------------------
HandymanTeleopKey::HandymanTeleopKey()
: rclcpp::Node("handyman_teleop_key"),
  is_received_are_you_ready_(false),
  is_received_environment_(false),
  arm_lift_joint_pos1_(0.0), arm_lift_joint_pos2_(0.0),
  arm_flex_joint_pos_(0.0),  wrist_flex_joint_pos_(0.0)
{
  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // 对应原稿 node_handle_.param<std::string>(...) — 从launch文件参数读取topic名
  this->declare_parameter<std::string>("sub_msg_to_robot_topic_name",       "/handyman/message/to_robot");
  this->declare_parameter<std::string>("pub_msg_to_moderator_topic_name",   "/handyman/message/to_moderator");
  this->declare_parameter<std::string>("sub_joint_state_topic_name",        "/hsrb/joint_states");
  this->declare_parameter<std::string>("pub_base_twist_topic_name",         "/hsrb/command_velocity");
  this->declare_parameter<std::string>("pub_base_trajectory_topic_name",    "/hsrb/omni_base_controller/command");
  this->declare_parameter<std::string>("pub_arm_trajectory_topic_name",     "/hsrb/arm_trajectory_controller/command");
  this->declare_parameter<std::string>("pub_gripper_trajectory_topic_name", "/hsrb/gripper_controller/command");

  const auto sub_msg_topic   = this->get_parameter("sub_msg_to_robot_topic_name").as_string();
  const auto pub_msg_topic   = this->get_parameter("pub_msg_to_moderator_topic_name").as_string();
  const auto sub_js_topic    = this->get_parameter("sub_joint_state_topic_name").as_string();
  const auto pub_twist_topic = this->get_parameter("pub_base_twist_topic_name").as_string();
  const auto pub_traj_topic  = this->get_parameter("pub_base_trajectory_topic_name").as_string();
  const auto pub_arm_topic   = this->get_parameter("pub_arm_trajectory_topic_name").as_string();
  const auto pub_grip_topic  = this->get_parameter("pub_gripper_trajectory_topic_name").as_string();

  pub_msg_             = create_publisher<HandymanMsg>(pub_msg_topic, 10);
  pub_base_twist_      = create_publisher<geometry_msgs::msg::Twist>(pub_twist_topic, 10);
  pub_base_trajectory_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(pub_traj_topic, 10);
  pub_arm_trajectory_  = create_publisher<trajectory_msgs::msg::JointTrajectory>(pub_arm_topic, 10);
  pub_gripper_trajectory_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(pub_grip_topic, 10);

  sub_msg_ = create_subscription<HandymanMsg>(
    sub_msg_topic, 100,
    std::bind(&HandymanTeleopKey::messageCallback, this, std::placeholders::_1));
  sub_joint_state_ = create_subscription<sensor_msgs::msg::JointState>(
    sub_js_topic, 10,
    std::bind(&HandymanTeleopKey::jointStateCallback, this, std::placeholders::_1));
}

// ---------------------------------------------------------------------------
int HandymanTeleopKey::canReceive(int fd)
{
  fd_set fdset;
  struct timeval timeout;
  FD_ZERO(&fdset);
  FD_SET(fd, &fdset);
  timeout.tv_sec = 0;
  timeout.tv_usec = 0;
  return select(fd+1, &fdset, NULL, NULL, &timeout);
}

void HandymanTeleopKey::messageCallback(const HandymanMsg::SharedPtr message)
{
  if (message->message == MSG_ARE_YOU_READY && is_received_are_you_ready_) return;
  if (message->message == MSG_ENVIRONMENT   && is_received_environment_)   return;

  RCLCPP_INFO(get_logger(), "Subscribe message:%s, %s",
              message->message.c_str(), message->detail.c_str());

  if (message->message == MSG_ARE_YOU_READY) is_received_are_you_ready_ = true;
  if (message->message == MSG_ENVIRONMENT)   is_received_environment_   = true;

  if (message->message == MSG_TASK_SUCCEEDED ||
      message->message == MSG_TASK_FAILED ||
      message->message == MSG_MISSION_COMPLETE) {
    is_received_are_you_ready_ = false;
    is_received_environment_   = false;
  }
}

void HandymanTeleopKey::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr js)
{
  for (size_t i = 0; i < js->name.size(); ++i) {
    if (js->name[i] == "arm_lift_joint") {
      arm_lift_joint_pos2_ = arm_lift_joint_pos1_;
      arm_lift_joint_pos1_ = js->position[i];
    }
    if (js->name[i] == "arm_flex_joint")  arm_flex_joint_pos_   = js->position[i];
    if (js->name[i] == "wrist_flex_joint") wrist_flex_joint_pos_ = js->position[i];
  }
}

void HandymanTeleopKey::sendMessage(const std::string &message)
{
  RCLCPP_INFO(get_logger(), "Send message:%s", message.c_str());
  HandymanMsg msg;
  msg.message = message;
  pub_msg_->publish(msg);
}

void HandymanTeleopKey::moveBaseTwist(double linear_x, double linear_y, double angular_z)
{
  geometry_msgs::msg::Twist twist;
  twist.linear.x  = linear_x;
  twist.linear.y  = linear_y;
  twist.angular.z = angular_z;
  pub_base_twist_->publish(twist);
}

void HandymanTeleopKey::moveBaseJointTrajectory(
  double linear_x, double linear_y, double theta, double duration_sec)
{
  if (!tf_buffer_->canTransform("odom", "base_footprint", tf2::TimePointZero)) return;

  geometry_msgs::msg::PointStamped bf_target, odom_target;
  bf_target.header.frame_id = "base_footprint";
  bf_target.header.stamp    = rclcpp::Time(0);
  bf_target.point.x = linear_x;
  bf_target.point.y = linear_y;
  tf_buffer_->transform(bf_target, odom_target, "odom");

  geometry_msgs::msg::TransformStamped ts =
    tf_buffer_->lookupTransform("odom", "base_footprint", tf2::TimePointZero);
  double roll, pitch, yaw;
  tf2::Quaternion q;
  tf2::fromMsg(ts.transform.rotation, q);
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  trajectory_msgs::msg::JointTrajectory jt;
  jt.joint_names = {"odom_x", "odom_y", "odom_t"};
  trajectory_msgs::msg::JointTrajectoryPoint pt;
  pt.positions       = {odom_target.point.x, odom_target.point.y, yaw + theta};
  pt.time_from_start = rclcpp::Duration::from_seconds(duration_sec);
  jt.points.push_back(pt);
  pub_base_trajectory_->publish(jt);
}

void HandymanTeleopKey::operateArm(
  double arm_lift_pos, double arm_flex_pos, double wrist_flex_pos, double duration_sec)
{
  trajectory_msgs::msg::JointTrajectory jt;
  jt.joint_names = {"arm_lift_joint","arm_flex_joint","arm_roll_joint",
                    "wrist_flex_joint","wrist_roll_joint"};
  trajectory_msgs::msg::JointTrajectoryPoint pt;
  pt.positions       = {arm_lift_pos, arm_flex_pos, 0.0f, wrist_flex_pos, 0.0f};
  pt.time_from_start = rclcpp::Duration::from_seconds(duration_sec);
  jt.points.push_back(pt);
  pub_arm_trajectory_->publish(jt);
}

void HandymanTeleopKey::operateArmByName(
  const std::string &name, double position, double duration_sec)
{
  if (name == "arm_lift_joint")
    operateArm(position, arm_flex_joint_pos_, wrist_flex_joint_pos_, duration_sec);
  else if (name == "arm_flex_joint")
    operateArm(2.0*arm_lift_joint_pos1_-arm_lift_joint_pos2_, position,
               wrist_flex_joint_pos_, duration_sec);
  else if (name == "wrist_flex_joint")
    operateArm(2.0*arm_lift_joint_pos1_-arm_lift_joint_pos2_,
               arm_flex_joint_pos_, position, duration_sec);
}

void HandymanTeleopKey::operateArmFlex(double arm_flex_pos, double wrist_flex_pos)
{
  double duration = std::max(
    getDurationRot(arm_flex_pos,   arm_flex_joint_pos_),
    getDurationRot(wrist_flex_pos, wrist_flex_joint_pos_));
  operateArm(2.0*arm_lift_joint_pos1_-arm_lift_joint_pos2_,
             arm_flex_pos, wrist_flex_pos, duration);
}

double HandymanTeleopKey::getDurationRot(double next_pos, double current_pos)
{
  return std::max((std::abs(next_pos - current_pos) * 1.2), 1.0);
}

void HandymanTeleopKey::operateHand(bool grasp)
{
  trajectory_msgs::msg::JointTrajectory jt;
  jt.joint_names = {"hand_motor_joint"};
  trajectory_msgs::msg::JointTrajectoryPoint pt;
  pt.positions       = {grasp ? -0.105 : +1.239};
  pt.time_from_start = rclcpp::Duration::from_seconds(2.0);
  jt.points.push_back(pt);
  pub_gripper_trajectory_->publish(jt);
}

void HandymanTeleopKey::showHelp()
{
  puts("Operate by Keyboard");
  puts("---------------------------");
  puts("arrow keys : Move HSR (rotate)");
  puts("space      : Stop HSR");
  puts("---------------------------");
  puts("Move HSR Linearly (1m)");
  puts("  u   i   o  ");
  puts("  j   k   l  ");
  puts("  m   ,   .  ");
  puts("---------------------------");
  puts("q/z : Increase/Decrease Moving Speed");
  puts("---------------------------");
  puts("y : Up   Torso");
  puts("h : Stop Torso");
  puts("n : Down Torso");
  puts("---------------------------");
  puts("a : Rotate Arm - Vertical");
  puts("b : Rotate Arm - Upward");
  puts("c : Rotate Arm - Horizontal");
  puts("d : Rotate Arm - Downward");
  puts("---------------------------");
  puts("g : Grasp/Open Hand");
  puts("---------------------------");
  puts(("0 : Send "+MSG_I_AM_READY).c_str());
  puts(("1 : Send "+MSG_ROOM_REACHED).c_str());
  puts(("2 : Send "+MSG_OBJECT_GRASPED).c_str());
  puts(("3 : Send "+MSG_TASK_FINISHED).c_str());
  puts(("6 : Send "+MSG_DOES_NOT_EXIST).c_str());
  puts(("9 : Send "+MSG_GIVE_UP).c_str());
}

// ---------------------------------------------------------------------------
int HandymanTeleopKey::run()
{
  // Put terminal into raw mode
  int kfd = 0;
  struct termios cooked, raw;
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  showHelp();

  rclcpp::Rate loop_rate(40);

  const float linear_coef  = 0.2f;
  const float angular_coef = 0.5f;
  float move_speed = 1.0f;
  bool  is_hand_open = false;

  const std::string arm_lift_joint_name   = "arm_lift_joint";
  const std::string arm_flex_joint_name   = "arm_flex_joint";
  const std::string wrist_flex_joint_name = "wrist_flex_joint";

  while (rclcpp::ok()) {
    if (canReceive(kfd)) {
      char c;
      if (read(kfd, &c, 1) < 0) {
        perror("read():");
        tcsetattr(kfd, TCSANOW, &cooked);
        return EXIT_FAILURE;
      }

      switch (c) {
        case KEYCODE_0: sendMessage(MSG_I_AM_READY);     break;
        case KEYCODE_1: sendMessage(MSG_ROOM_REACHED);   break;
        case KEYCODE_2: sendMessage(MSG_OBJECT_GRASPED); break;
        case KEYCODE_3: sendMessage(MSG_TASK_FINISHED);  break;
        case KEYCODE_6: sendMessage(MSG_DOES_NOT_EXIST); break;
        case KEYCODE_9: sendMessage(MSG_GIVE_UP);        break;

        case KEYCODE_UP:
          RCLCPP_DEBUG(get_logger(), "Go Forward");
          moveBaseTwist(+linear_coef*move_speed, 0.0, 0.0);
          break;
        case KEYCODE_DOWN:
          RCLCPP_DEBUG(get_logger(), "Go Backward");
          moveBaseTwist(-linear_coef*move_speed, 0.0, 0.0);
          break;
        case KEYCODE_RIGHT:
          RCLCPP_DEBUG(get_logger(), "Turn Right");
          moveBaseTwist(0.0, 0.0, -angular_coef*move_speed);
          break;
        case KEYCODE_LEFT:
          RCLCPP_DEBUG(get_logger(), "Turn Left");
          moveBaseTwist(0.0, 0.0, +angular_coef*move_speed);
          break;
        case KEYCODE_SPACE:
          RCLCPP_DEBUG(get_logger(), "Stop");
          moveBaseTwist(0.0, 0.0, 0.0);
          break;

        case KEYCODE_U:
          RCLCPP_DEBUG(get_logger(), "Move Left Forward");
          moveBaseJointTrajectory(+1.0, +1.0, +M_PI_4, 10);
          break;
        case KEYCODE_I:
          RCLCPP_DEBUG(get_logger(), "Move Forward");
          moveBaseJointTrajectory(+1.0, 0.0, 0.0, 10);
          break;
        case KEYCODE_O:
          RCLCPP_DEBUG(get_logger(), "Move Right Forward");
          moveBaseJointTrajectory(+1.0, -1.0, -M_PI_4, 10);
          break;
        case KEYCODE_J:
          RCLCPP_DEBUG(get_logger(), "Move Left");
          moveBaseJointTrajectory(0.0, +1.0, +M_PI_2, 10);
          break;
        case KEYCODE_K:
          RCLCPP_DEBUG(get_logger(), "Stop");
          moveBaseJointTrajectory(0.0, 0.0, 0.0, 0.5);
          break;
        case KEYCODE_L:
          RCLCPP_DEBUG(get_logger(), "Move Right");
          moveBaseJointTrajectory(0.0, -1.0, -M_PI_2, 10);
          break;
        case KEYCODE_M:
          RCLCPP_DEBUG(get_logger(), "Move Left Backward");
          moveBaseJointTrajectory(-1.0, +1.0, +M_PI_2+M_PI_4, 10);
          break;
        case KEYCODE_COMMA:
          RCLCPP_DEBUG(get_logger(), "Move Backward");
          moveBaseJointTrajectory(-1.0, 0.0, +M_PI, 10);
          break;
        case KEYCODE_PERIOD:
          RCLCPP_DEBUG(get_logger(), "Move Right Backward");
          moveBaseJointTrajectory(-1.0, -1.0, -M_PI_2-M_PI_4, 10);
          break;

        case KEYCODE_Q:
          RCLCPP_DEBUG(get_logger(), "Speed Up");
          move_speed *= 2.0f;
          if (move_speed > 2.0f) move_speed = 2.0f;
          break;
        case KEYCODE_Z:
          RCLCPP_DEBUG(get_logger(), "Speed Down");
          move_speed /= 2.0f;
          if (move_speed < 0.125f) move_speed = 0.125f;
          break;

        case KEYCODE_Y:
          RCLCPP_DEBUG(get_logger(), "Up Torso");
          operateArmByName(arm_lift_joint_name, 0.69,
            std::max((int)(std::abs(0.69 - arm_lift_joint_pos1_) / 0.05), 1));
          break;
        case KEYCODE_H:
          RCLCPP_DEBUG(get_logger(), "Stop Torso");
          operateArmByName(arm_lift_joint_name,
            2.0*arm_lift_joint_pos1_-arm_lift_joint_pos2_, 0.5);
          break;
        case KEYCODE_N:
          RCLCPP_DEBUG(get_logger(), "Down Torso");
          operateArmByName(arm_lift_joint_name, 0.0,
            std::max((int)(std::abs(0.0 - arm_lift_joint_pos1_) / 0.05), 1));
          break;

        case KEYCODE_A:
          RCLCPP_DEBUG(get_logger(), "Rotate Arm - Vertical");
          operateArmFlex(0.0, -1.57);
          break;
        case KEYCODE_B:
          RCLCPP_DEBUG(get_logger(), "Rotate Arm - Upward");
          operateArmFlex(-0.785, -0.785);
          break;
        case KEYCODE_C:
          RCLCPP_DEBUG(get_logger(), "Rotate Arm - Horizontal");
          operateArmFlex(-1.57, 0.0);
          break;
        case KEYCODE_D:
          RCLCPP_DEBUG(get_logger(), "Rotate Arm - Downward");
          operateArmFlex(-2.2, 0.35);
          break;

        case KEYCODE_G:
          operateHand(is_hand_open);
          is_hand_open = !is_hand_open;
          break;

        default:
          break;
      }
    }

    rclcpp::spin_some(shared_from_this());
    loop_rate.sleep();
  }

  // Restore terminal
  tcsetattr(kfd, TCSANOW, &cooked);
  return EXIT_SUCCESS;
}

// ---------------------------------------------------------------------------
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HandymanTeleopKey>();
  int ret = node->run();
  rclcpp::shutdown();
  return ret;
}
