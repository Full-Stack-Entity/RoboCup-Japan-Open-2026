#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <interactive_cleanup/msg/interactive_cleanup_msg.hpp>

#include <string>
#include <vector>
#include <map>
#include <mutex>
#include <memory>
#include <functional>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class InteractiveCleanupSample : public rclcpp::Node
{
public:
  InteractiveCleanupSample()
  : rclcpp::Node("interactive_cleanup_sample")
  {
  }

  int run()
  {
    // ----- Publishers -----
    pub_msg_ = this->create_publisher<interactive_cleanup::msg::InteractiveCleanupMsg>(
        "/interactive_cleanup/message/to_moderator", 10);
    pub_base_twist_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/hsrb/command_velocity", 10);
    pub_arm_trajectory_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/hsrb/arm_trajectory_controller/command", 10);
    pub_gripper_trajectory_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/hsrb/gripper_controller/command", 10);
    pub_head_trajectory_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/hsrb/head_trajectory_controller/command", 10);

    // ----- Subscribers -----
    sub_msg_ = this->create_subscription<interactive_cleanup::msg::InteractiveCleanupMsg>(
        "/interactive_cleanup/message/to_robot", 100,
        std::bind(&InteractiveCleanupSample::messageCallback, this, std::placeholders::_1));
    sub_joint_state_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/hsrb/joint_states", 10,
        std::bind(&InteractiveCleanupSample::jointStateCallback, this, std::placeholders::_1));

    // ----- TF -----
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ----- Nav2 Action Client -----
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    // ----- Init arm trajectory structure -----
    arm_joint_trajectory_.joint_names = {
        "arm_lift_joint", "arm_flex_joint", "arm_roll_joint",
        "wrist_flex_joint", "wrist_roll_joint"};
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions = {0.0, 0.0, 0.0, 0.0, 0.0};
    arm_joint_trajectory_.points.push_back(pt);

    step_ = Initialize;

    RCLCPP_INFO(this->get_logger(), "Interactive Cleanup sample started!");

    rclcpp::Rate loop_rate(10);
    while (rclcpp::ok()) {
      rclcpp::spin_some(shared_from_this());
      runStateMachine();
      loop_rate.sleep();
    }
    return EXIT_SUCCESS;
  }

private:
  // =========================================================================
  // State definitions
  // =========================================================================
  enum Step {
    Initialize,
    Ready,
    WaitForPickCommand,
    WaitForCleanCommand,
    MoveToObject,
    GraspObject,
    SendObjectGrasped,
    MoveToDestination,
    ReleaseObject,
    SendTaskFinished,
    WaitForResult
  };

  // =========================================================================
  // Event message constants
  // =========================================================================
  const std::string MSG_ARE_YOU_READY    = "Are_you_ready?";
  const std::string MSG_I_AM_READY       = "I_am_ready";
  const std::string MSG_PICK_IT_UP       = "Pick_it_up!";
  const std::string MSG_CLEAN_UP         = "Clean_up!";
  const std::string MSG_IS_THIS_CORRECT  = "Is_this_correct?";
  const std::string MSG_POINT_IT_AGAIN   = "Point_it_again";
  const std::string MSG_YES              = "Yes";
  const std::string MSG_NO               = "No";
  const std::string MSG_OBJECT_GRASPED   = "Object_grasped";
  const std::string MSG_TASK_FINISHED    = "Task_finished";
  const std::string MSG_TASK_SUCCEEDED   = "Task_succeeded";
  const std::string MSG_TASK_FAILED      = "Task_failed";
  const std::string MSG_MISSION_COMPLETE = "Mission_complete";
  const std::string MSG_GIVE_UP          = "Give_up";

  // =========================================================================
  // Named arm poses: {arm_lift, arm_flex, arm_roll, wrist_flex, wrist_roll}
  // =========================================================================
  const std::vector<double> ARM_POSE_INITIAL      = {0.0,   0.0,   0.0,  0.0,   0.0};
  const std::vector<double> ARM_POSE_OBSERVE      = {0.1,   0.0,   0.0, -1.57,  0.0};
  const std::vector<double> ARM_POSE_GRASP_READY  = {0.1,  -0.5,   0.0, -1.0,   0.0};
  const std::vector<double> ARM_POSE_GRASP_LOW    = {0.05, -0.8,   0.0, -0.7,   0.0};
  const std::vector<double> ARM_POSE_CARRY        = {0.3,   0.0,   0.0, -1.57,  0.0};
  const std::vector<double> ARM_POSE_RELEASE_READY = {0.2, -0.8,   0.0, -1.0,   0.0};
  const std::vector<double> ARM_POSE_RELEASE_LOW  = {0.05, -1.0,   0.0, -0.5,   0.0};

  // =========================================================================
  // Head scan pattern: {pan, tilt} pairs for lookAround
  // =========================================================================
  struct HeadPose { double pan; double tilt; };
  const std::vector<HeadPose> SCAN_PATTERN = {
      { 0.0, -0.3},   // center-down (look at floor/table level)
      { 0.8, -0.2},   // left
      { 0.0, -0.5},   // center-lower
      {-0.8, -0.2},   // right
      { 0.0,  0.0},   // center-straight
  };

  // =========================================================================
  // Timeout constants (seconds)
  // =========================================================================
  static constexpr double TIMEOUT_WAIT_COMMAND = 120.0;//超时保护时间常量定义
  static constexpr double TIMEOUT_MOVE         = 60.0;
  static constexpr double TIMEOUT_GRASP        = 30.0;
  static constexpr double TIMEOUT_RELEASE      = 30.0;
  static constexpr double TIMEOUT_WAIT_RESULT  = 60.0;
  static constexpr int    MAX_POINT_AGAIN      = 2;

  // =========================================================================
  // Member variables
  // =========================================================================
  int step_;
  bool is_started_  = false;
  bool is_finished_ = false;
  bool is_failed_   = false;
  std::string failed_detail_;
  bool received_pick_command_  = false;
  bool received_clean_command_ = false;
  bool received_yes_ = false;
  bool received_no_  = false;

  // Timing
  rclcpp::Time state_enter_time_;
  rclcpp::Time global_task_start_time_;
  bool state_timer_initialized_ = false;

  // Sub-step tracking inside a state
  int sub_step_ = 0;

  // Head scan
  int scan_index_ = 0;

  // Point-it-again counter
  int point_again_count_ = 0;

  // Joint state storage
  std::mutex joint_mutex_;
  std::map<std::string, double> joint_positions_;

  // Nav2
  bool nav_goal_sent_    = false;
  bool nav_goal_reached_ = false;
  bool nav_goal_failed_  = false;

  // Arm
  trajectory_msgs::msg::JointTrajectory arm_joint_trajectory_;

  // Publishers
  rclcpp::Publisher<interactive_cleanup::msg::InteractiveCleanupMsg>::SharedPtr pub_msg_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_base_twist_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_arm_trajectory_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_gripper_trajectory_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_head_trajectory_;

  // Subscribers
  rclcpp::Subscription<interactive_cleanup::msg::InteractiveCleanupMsg>::SharedPtr sub_msg_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_state_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Nav2
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;

  // =========================================================================
  // Callbacks
  // =========================================================================
  void messageCallback(const interactive_cleanup::msg::InteractiveCleanupMsg::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received: message='%s', detail='%s'",
                msg->message.c_str(), msg->detail.c_str());

    if (msg->message == MSG_ARE_YOU_READY) {
      if (step_ == Ready) {
        is_started_ = true;
      } else {
        RCLCPP_WARN(this->get_logger(),
            "Unexpected Are_you_ready? in state %d, forcing reset.", step_);
        emergencyStop();
        step_ = Initialize;
      }
    }
    else if (msg->message == MSG_PICK_IT_UP)  { received_pick_command_ = true; }
    else if (msg->message == MSG_CLEAN_UP)    { received_clean_command_ = true; }
    else if (msg->message == MSG_YES)         { received_yes_ = true; }
    else if (msg->message == MSG_NO)          { received_no_ = true; }
    else if (msg->message == MSG_TASK_SUCCEEDED) { is_finished_ = true; }
    else if (msg->message == MSG_TASK_FAILED) {
      is_failed_ = true;
      failed_detail_ = msg->detail;
      RCLCPP_WARN(this->get_logger(), "Task failed! detail='%s'", msg->detail.c_str());
    }
    else if (msg->message == MSG_MISSION_COMPLETE) {
      RCLCPP_INFO(this->get_logger(), "Mission complete! Shutting down.");
      emergencyStop();
      rclcpp::shutdown();
    }
    else {
      RCLCPP_WARN(this->get_logger(), "Unknown message: '%s'", msg->message.c_str());
    }
  }

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(joint_mutex_);
    for (size_t i = 0; i < msg->name.size(); ++i) {
      joint_positions_[msg->name[i]] = msg->position[i];
    }
  }

  // =========================================================================
  // Joint state query
  // =========================================================================
  double getJointPosition(const std::string &name, double default_val = 0.0)
  {
    std::lock_guard<std::mutex> lock(joint_mutex_);
    auto it = joint_positions_.find(name);
    return (it != joint_positions_.end()) ? it->second : default_val;
  }

  // =========================================================================
  // Message helpers
  // =========================================================================
  void sendMessage(const std::string &message)
  {
    RCLCPP_INFO(this->get_logger(), "Send: message='%s'", message.c_str());
    interactive_cleanup::msg::InteractiveCleanupMsg msg;
    msg.message = message;
    pub_msg_->publish(msg);
  }

  // =========================================================================
  // Base motion control
  // =========================================================================
  void moveBase(double linear_x, double linear_y, double angular_z)
  {
    geometry_msgs::msg::Twist twist;
    twist.linear.x  = linear_x;
    twist.linear.y  = linear_y;
    twist.angular.z = angular_z;
    pub_base_twist_->publish(twist);
  }

  void stopBase() { moveBase(0.0, 0.0, 0.0); }

  void turnBase(double angular_z) { moveBase(0.0, 0.0, angular_z); }

  void driveForward(double speed) { moveBase(speed, 0.0, 0.0); }

  // =========================================================================
  // Arm control
  // =========================================================================
  void moveArm(const std::vector<double> &positions, double duration_sec)
  {
    arm_joint_trajectory_.points[0].positions = positions;
    arm_joint_trajectory_.points[0].time_from_start =
        rclcpp::Duration::from_seconds(duration_sec);
    pub_arm_trajectory_->publish(arm_joint_trajectory_);
  }

  void armInitial()      { moveArm(ARM_POSE_INITIAL, 2.0); }
  void armObserve()      { moveArm(ARM_POSE_OBSERVE, 1.5); }
  void armGraspReady()   { moveArm(ARM_POSE_GRASP_READY, 1.5); }
  void armGraspLow()     { moveArm(ARM_POSE_GRASP_LOW, 1.5); }
  void armCarry()        { moveArm(ARM_POSE_CARRY, 1.5); }
  void armReleaseReady() { moveArm(ARM_POSE_RELEASE_READY, 1.5); }
  void armReleaseLow()   { moveArm(ARM_POSE_RELEASE_LOW, 1.5); }

  // =========================================================================
  // Head control
  // =========================================================================
  void moveHead(double pan, double tilt, double duration_sec)
  {
    trajectory_msgs::msg::JointTrajectory jt;
    jt.joint_names = {"head_pan_joint", "head_tilt_joint"};
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions = {pan, tilt};
    pt.time_from_start = rclcpp::Duration::from_seconds(duration_sec);
    jt.points.push_back(pt);
    pub_head_trajectory_->publish(jt);
  }

  void headCenter()    { moveHead(0.0, 0.0, 1.0); }
  void headLookDown()  { moveHead(0.0, -0.5, 1.0); }
  void headLookLeft()  { moveHead(0.8, -0.2, 1.0); }
  void headLookRight() { moveHead(-0.8, -0.2, 1.0); }

  void lookAtScanStep(int index)
  {
    int i = index % static_cast<int>(SCAN_PATTERN.size());
    moveHead(SCAN_PATTERN[i].pan, SCAN_PATTERN[i].tilt, 1.2);
  }

  // =========================================================================
  // Gripper control
  // =========================================================================
  void operateHand(bool should_grasp)
  {
    trajectory_msgs::msg::JointTrajectory jt;
    jt.joint_names = {"hand_motor_joint"};
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions = {should_grasp ? -0.105 : 1.239};
    pt.time_from_start = rclcpp::Duration::from_seconds(2.0);
    jt.points.push_back(pt);
    pub_gripper_trajectory_->publish(jt);
  }

  void gripperOpen()  { operateHand(false); }
  void gripperClose() { operateHand(true); }

  // =========================================================================
  // Emergency & exception actions
  // =========================================================================
  void emergencyStop()
  {
    stopBase();
    armInitial();
    headCenter();
    RCLCPP_WARN(this->get_logger(), "Emergency stop executed.");
  }

  void giveUp()
  {
    RCLCPP_WARN(this->get_logger(), "Giving up current task.");
    emergencyStop();
    sendMessage(MSG_GIVE_UP);
  }

  void requestPointAgain()
  {
    if (point_again_count_ < MAX_POINT_AGAIN) {
      point_again_count_++;
      RCLCPP_INFO(this->get_logger(),
          "Requesting re-pointing (%d/%d)", point_again_count_, MAX_POINT_AGAIN);
      sendMessage(MSG_POINT_IT_AGAIN);
    } else {
      RCLCPP_WARN(this->get_logger(),
          "Max re-pointing attempts reached (%d), giving up.", MAX_POINT_AGAIN);
      giveUp();
    }
  }

  void askConfirmation()
  {
    RCLCPP_INFO(this->get_logger(), "Asking: Is this correct?");
    received_yes_ = false;
    received_no_  = false;
    sendMessage(MSG_IS_THIS_CORRECT);
  }

  // =========================================================================
  // Nav2 navigation
  // =========================================================================
  void sendNavGoal(double x, double y, double yaw)
  {
    if (nav_goal_sent_) return;

    if (!nav_client_->wait_for_action_server(3s)) {
      RCLCPP_WARN(this->get_logger(), "Nav2 action server not available");
      nav_goal_failed_ = true;
      return;
    }

    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    q.normalize();

    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp    = this->now();
    goal.pose.pose.position.x = x;
    goal.pose.pose.position.y = y;
    goal.pose.pose.orientation = tf2::toMsg(q);

    auto send_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_options.result_callback =
        [this](const GoalHandleNav::WrappedResult &result) {
          nav_goal_sent_ = false;
          if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "Nav goal reached.");
            nav_goal_reached_ = true;
          } else {
            RCLCPP_WARN(this->get_logger(), "Nav goal failed (code=%d).",
                        static_cast<int>(result.code));
            nav_goal_failed_ = true;
          }
        };

    nav_client_->async_send_goal(goal, send_options);
    nav_goal_sent_    = true;
    nav_goal_reached_ = false;
    nav_goal_failed_  = false;
    RCLCPP_INFO(this->get_logger(), "Sending nav goal: (%.2f, %.2f, yaw=%.2f)", x, y, yaw);
  }

  void resetNavState()
  {
    nav_goal_sent_    = false;
    nav_goal_reached_ = false;
    nav_goal_failed_  = false;
  }

  // =========================================================================
  // Timing helpers
  // =========================================================================
  void reset()
  {
    is_started_  = false;
    is_finished_ = false;
    is_failed_   = false;
    failed_detail_ = "";
    received_pick_command_  = false;
    received_clean_command_ = false;
    received_yes_ = false;
    received_no_  = false;
    state_timer_initialized_ = false;
    sub_step_ = 0;
    scan_index_ = 0;
    point_again_count_ = 0;
    resetNavState();
  }

  std::string stepName(int s)
  {
    switch (s) {
      case Initialize:       return "Initialize";
      case Ready:            return "Ready";
      case WaitForPickCommand:  return "WaitForPickCommand";
      case WaitForCleanCommand: return "WaitForCleanCommand";
      case MoveToObject:     return "MoveToObject";
      case GraspObject:      return "GraspObject";
      case SendObjectGrasped: return "SendObjectGrasped";
      case MoveToDestination: return "MoveToDestination";
      case ReleaseObject:    return "ReleaseObject";
      case SendTaskFinished: return "SendTaskFinished";
      case WaitForResult:    return "WaitForResult";
      default:               return "Unknown";
    }
  }

  void enterTimedState()
  {
    state_enter_time_ = this->now();
    state_timer_initialized_ = true;
  }

  void enterNewState()
  {
    enterTimedState();
    sub_step_ = 0;
  }

  double elapsedInState()
  {
    if (!state_timer_initialized_) return 0.0;
    return (this->now() - state_enter_time_).seconds();
  }

  // =========================================================================
  // State machine
  // =========================================================================
  void runStateMachine()
  {
    // =====================================================================
    // Global failure handler — any state
    // =====================================================================
    if (is_failed_) {
      RCLCPP_WARN(this->get_logger(),
          "Task failed in state [%s], detail='%s'. Resetting.",
          stepName(step_).c_str(), failed_detail_.c_str());
      emergencyStop();
      step_ = Initialize;
      is_failed_ = false;
      return;
    }

    switch (step_) {

      // -------------------------------------------------------------------
      case Initialize: {
        reset();
        armInitial();
        headCenter();
        gripperOpen();
        RCLCPP_INFO(this->get_logger(), "[Initialize] Robot reset. Waiting for Are_you_ready?...");
        step_ = Ready;
        break;
      }

      // -------------------------------------------------------------------
      case Ready: {
        if (is_started_) {
          RCLCPP_INFO(this->get_logger(), "[Ready] Got Are_you_ready? -> sending I_am_ready");
          armObserve();
          headCenter();
          sendMessage(MSG_I_AM_READY);
          enterNewState();
          global_task_start_time_ = this->now();
          step_ = WaitForPickCommand;
        }
        break;
      }

      // -------------------------------------------------------------------
      case WaitForPickCommand: {
        if (received_pick_command_) {
          RCLCPP_INFO(this->get_logger(),
              "[WaitForPickCommand] Got Pick_it_up! Avatar is pointing at object.");
          received_pick_command_ = false;
          enterNewState();
          scan_index_ = 0;
          step_ = WaitForCleanCommand;
        }
        else if (state_timer_initialized_ && elapsedInState() > TIMEOUT_WAIT_COMMAND) {
          RCLCPP_WARN(this->get_logger(),
              "[WaitForPickCommand] Timeout (%.0fs), giving up.", TIMEOUT_WAIT_COMMAND);
          giveUp();
          step_ = WaitForResult;
        }
        break;
      }

      // -------------------------------------------------------------------
      case WaitForCleanCommand: {
        if (elapsedInState() > 1.5 * (scan_index_ + 1) &&
            scan_index_ < static_cast<int>(SCAN_PATTERN.size())) {
          lookAtScanStep(scan_index_);
          RCLCPP_INFO(this->get_logger(),
              "[WaitForCleanCommand] Scanning step %d ...", scan_index_);
          scan_index_++;
        }

        if (received_clean_command_) {
          RCLCPP_INFO(this->get_logger(),
              "[WaitForCleanCommand] Got Clean_up! Avatar is pointing at destination.");
          received_clean_command_ = false;
          headCenter();
          enterNewState();
          step_ = MoveToObject;
        }
        else if (elapsedInState() > TIMEOUT_WAIT_COMMAND) {
          RCLCPP_WARN(this->get_logger(),
              "[WaitForCleanCommand] Timeout (%.0fs), giving up.", TIMEOUT_WAIT_COMMAND);
          giveUp();
          step_ = WaitForResult;
        }
        break;
      }

      // -------------------------------------------------------------------
      case MoveToObject: {
        if (elapsedInState() > TIMEOUT_MOVE && sub_step_ > 0) {
          RCLCPP_WARN(this->get_logger(),
              "[MoveToObject] Timeout (%.0fs), giving up.", TIMEOUT_MOVE);
          emergencyStop();
          giveUp();
          step_ = WaitForResult;
          break;
        }

        switch (sub_step_) {
          case 0: {
            armObserve();
            headLookDown();
            gripperOpen();
            RCLCPP_INFO(this->get_logger(), "[MoveToObject] Preparing to approach object...");
            sub_step_ = 1;
            enterTimedState();
            break;
          }
          case 1: {
            if (elapsedInState() < 4.0) {
              RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                  "[MoveToObject] Moving forward... (%.1fs)", elapsedInState());
              driveForward(0.15);
            } else {
              stopBase();
              RCLCPP_INFO(this->get_logger(), "[MoveToObject] Reached object vicinity.");
              sub_step_ = 2;
            }
            break;
          }
          case 2: {
            enterNewState();
            step_ = GraspObject;
            RCLCPP_INFO(this->get_logger(), "[MoveToObject] -> GraspObject");
            break;
          }
        }
        break;
      }

      // -------------------------------------------------------------------
      case GraspObject: {
        if (elapsedInState() > TIMEOUT_GRASP && sub_step_ > 0) {
          RCLCPP_WARN(this->get_logger(),
              "[GraspObject] Timeout (%.0fs), giving up.", TIMEOUT_GRASP);
          emergencyStop();
          giveUp();
          step_ = WaitForResult;
          break;
        }

        switch (sub_step_) {
          case 0: {
            armGraspReady();
            headLookDown();
            RCLCPP_INFO(this->get_logger(), "[GraspObject] Arm -> grasp ready");
            sub_step_ = 1;
            enterTimedState();
            break;
          }
          case 1: {
            if (elapsedInState() > 2.0) {
              armGraspLow();
              RCLCPP_INFO(this->get_logger(), "[GraspObject] Arm -> grasp low");
              sub_step_ = 2;
              enterTimedState();
            }
            break;
          }
          case 2: {
            if (elapsedInState() > 2.0) {
              gripperClose();
              RCLCPP_INFO(this->get_logger(), "[GraspObject] Gripper closing");
              sub_step_ = 3;
              enterTimedState();
            }
            break;
          }
          case 3: {
            if (elapsedInState() > 3.0) {
              armCarry();
              RCLCPP_INFO(this->get_logger(), "[GraspObject] Arm -> carry position");
              sub_step_ = 4;
              enterTimedState();
            }
            break;
          }
          case 4: {
            if (elapsedInState() > 2.0) {
              RCLCPP_INFO(this->get_logger(), "[GraspObject] Grasp complete.");
              step_ = SendObjectGrasped;
            }
            break;
          }
        }
        break;
      }

      // -------------------------------------------------------------------
      case SendObjectGrasped: {
        RCLCPP_INFO(this->get_logger(), "[SendObjectGrasped] Sending Object_grasped");
        sendMessage(MSG_OBJECT_GRASPED);
        enterNewState();
        step_ = MoveToDestination;
        break;
      }

      // -------------------------------------------------------------------
      case MoveToDestination: {
        if (elapsedInState() > TIMEOUT_MOVE && sub_step_ > 0) {
          RCLCPP_WARN(this->get_logger(),
              "[MoveToDestination] Timeout (%.0fs), giving up.", TIMEOUT_MOVE);
          emergencyStop();
          giveUp();
          step_ = WaitForResult;
          break;
        }

        switch (sub_step_) {
          case 0: {
            headCenter();
            RCLCPP_INFO(this->get_logger(), "[MoveToDestination] Turning toward destination...");
            sub_step_ = 1;
            enterTimedState();
            break;
          }
          case 1: {
            if (elapsedInState() < 2.0) {
              turnBase(0.5);
            } else if (elapsedInState() < 5.0) {
              RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                  "[MoveToDestination] Moving to destination... (%.1fs)", elapsedInState());
              driveForward(0.15);
            } else {
              stopBase();
              RCLCPP_INFO(this->get_logger(), "[MoveToDestination] Reached destination.");
              sub_step_ = 2;
            }
            break;
          }
          case 2: {
            enterNewState();
            step_ = ReleaseObject;
            RCLCPP_INFO(this->get_logger(), "[MoveToDestination] -> ReleaseObject");
            break;
          }
        }
        break;
      }

      // -------------------------------------------------------------------
      case ReleaseObject: {
        if (elapsedInState() > TIMEOUT_RELEASE && sub_step_ > 0) {
          RCLCPP_WARN(this->get_logger(),
              "[ReleaseObject] Timeout (%.0fs), giving up.", TIMEOUT_RELEASE);
          emergencyStop();
          giveUp();
          step_ = WaitForResult;
          break;
        }

        switch (sub_step_) {
          case 0: {
            armReleaseReady();
            headLookDown();
            RCLCPP_INFO(this->get_logger(), "[ReleaseObject] Arm -> release ready");
            sub_step_ = 1;
            enterTimedState();
            break;
          }
          case 1: {
            if (elapsedInState() > 2.0) {
              armReleaseLow();
              RCLCPP_INFO(this->get_logger(), "[ReleaseObject] Arm -> release low");
              sub_step_ = 2;
              enterTimedState();
            }
            break;
          }
          case 2: {
            if (elapsedInState() > 2.0) {
              gripperOpen();
              RCLCPP_INFO(this->get_logger(), "[ReleaseObject] Gripper opening");
              sub_step_ = 3;
              enterTimedState();
            }
            break;
          }
          case 3: {
            if (elapsedInState() > 3.0) {
              armInitial();
              headCenter();
              RCLCPP_INFO(this->get_logger(), "[ReleaseObject] Arm -> initial, backing up");
              sub_step_ = 4;
              enterTimedState();
            }
            break;
          }
          case 4: {
            if (elapsedInState() < 2.0) {
              driveForward(-0.1);
            } else {
              stopBase();
              RCLCPP_INFO(this->get_logger(), "[ReleaseObject] Release complete.");
              step_ = SendTaskFinished;
            }
            break;
          }
        }
        break;
      }

      // -------------------------------------------------------------------
      case SendTaskFinished: {
        RCLCPP_INFO(this->get_logger(), "[SendTaskFinished] Sending Task_finished");
        sendMessage(MSG_TASK_FINISHED);
        enterNewState();
        step_ = WaitForResult;
        break;
      }

      // -------------------------------------------------------------------
      case WaitForResult: {
        if (is_finished_) {
          RCLCPP_INFO(this->get_logger(),
              "[WaitForResult] Task succeeded! Resetting for next task.");
          step_ = Initialize;
        }
        else if (elapsedInState() > TIMEOUT_WAIT_RESULT) {
          RCLCPP_WARN(this->get_logger(),
              "[WaitForResult] Timeout waiting for result (%.0fs), resetting.",
              TIMEOUT_WAIT_RESULT);
          step_ = Initialize;
        }
        break;
      }
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<InteractiveCleanupSample>();
  int ret = node->run();
  rclcpp::shutdown();
  return ret;
}
