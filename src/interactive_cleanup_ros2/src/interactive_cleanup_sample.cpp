#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <interactive_cleanup/msg/interactive_cleanup_msg.hpp>
#include <cleanup_vision_ros2/msg/detected_object_array.hpp>
#include <cleanup_vision_ros2/msg/pointing_direction.hpp>

#include <string>
#include <vector>
#include <map>
#include <mutex>
#include <memory>
#include <functional>
#include <chrono>
#include <cmath>
#include <algorithm>

using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;
using DetectedObjectArray = cleanup_vision_ros2::msg::DetectedObjectArray;
using DetectedObject = cleanup_vision_ros2::msg::DetectedObject;
using PointingDirection = cleanup_vision_ros2::msg::PointingDirection;

class InteractiveCleanupSample : public rclcpp::Node
{
public:
  InteractiveCleanupSample()
  : rclcpp::Node("interactive_cleanup_sample")
  {
  }

  int run()
  {
    // ---- Publishers ----
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
    pub_vision_enable_ = this->create_publisher<std_msgs::msg::Bool>(
        "/cleanup_vision/enable", 10);

    // ---- Subscribers ----
    sub_msg_ = this->create_subscription<interactive_cleanup::msg::InteractiveCleanupMsg>(
        "/interactive_cleanup/message/to_robot", 100,
        std::bind(&InteractiveCleanupSample::messageCallback, this, std::placeholders::_1));
    sub_joint_state_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/hsrb/joint_states", 10,
        std::bind(&InteractiveCleanupSample::jointStateCallback, this, std::placeholders::_1));
    sub_detected_objects_ = this->create_subscription<DetectedObjectArray>(
        "/cleanup_vision/detected_objects", 10,
        std::bind(&InteractiveCleanupSample::objectsCallback, this, std::placeholders::_1));
    sub_pointing_ = this->create_subscription<PointingDirection>(
        "/cleanup_vision/pointing_direction", 10,
        std::bind(&InteractiveCleanupSample::pointingCallback, this, std::placeholders::_1));

    // ---- TF ----
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ---- Nav2 ----
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    // ---- Arm trajectory template ----
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
    ObserveTarget,
    WaitForCleanCommand,
    ObserveDestination,
    MoveToObject,
    GraspObject,
    SendObjectGrasped,
    MoveToDestination,
    ReleaseObject,
    SendTaskFinished,
    WaitForResult
  };

  // =========================================================================
  // Message constants
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
  // Arm poses: {arm_lift, arm_flex, arm_roll, wrist_flex, wrist_roll}
  // =========================================================================
  const std::vector<double> ARM_INITIAL       = {0.0,   0.0,   0.0,  0.0,   0.0};
  const std::vector<double> ARM_OBSERVE       = {0.1,   0.0,   0.0, -1.57,  0.0};
  const std::vector<double> ARM_GRASP_READY   = {0.1,  -0.5,   0.0, -1.0,   0.0};
  const std::vector<double> ARM_GRASP_LOW     = {0.05, -0.8,   0.0, -0.7,   0.0};
  const std::vector<double> ARM_CARRY         = {0.3,   0.0,   0.0, -1.57,  0.0};
  const std::vector<double> ARM_RELEASE_READY = {0.2,  -0.8,   0.0, -1.0,   0.0};
  const std::vector<double> ARM_RELEASE_LOW   = {0.05, -1.0,   0.0, -0.5,   0.0};

  // =========================================================================
  // Head scan patterns
  // =========================================================================
  struct HeadPose { double pan; double tilt; };
  const std::vector<HeadPose> SCAN_WIDE = {
      { 0.0, -0.3}, { 0.6, -0.2}, {-0.6, -0.2},
      { 0.0, -0.5}, { 0.8, -0.3}, {-0.8, -0.3}, { 0.0,  0.0},
  };
  const std::vector<HeadPose> SCAN_FORWARD = {
      { 0.0, -0.3}, { 0.0, -0.5}, { 0.2, -0.4}, {-0.2, -0.4}, { 0.0, 0.0},
  };

  // =========================================================================
  // Timeouts
  // =========================================================================
  static constexpr double TIMEOUT_WAIT_COMMAND = 120.0;
  static constexpr double TIMEOUT_OBSERVE      =   5.0;
  static constexpr double TIMEOUT_MOVE         =  60.0;
  static constexpr double TIMEOUT_GRASP        =  30.0;
  static constexpr double TIMEOUT_RELEASE      =  30.0;
  static constexpr double TIMEOUT_WAIT_RESULT  =  60.0;
  static constexpr int    MAX_POINT_AGAIN      =   2;

  static constexpr double APPROACH_STOP_DIST   = 0.45;
  static constexpr double TURN_THRESHOLD       = 0.15;
  static constexpr double VISUAL_SERVO_FWD     = 0.08;

  // =========================================================================
  // State variables
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

  rclcpp::Time state_enter_time_;
  bool state_timer_initialized_ = false;
  int sub_step_ = 0;
  int scan_index_ = 0;
  int point_again_count_ = 0;

  // Joint state
  std::mutex joint_mutex_;
  std::map<std::string, double> joint_positions_;

  // Nav2
  bool nav_goal_sent_    = false;
  bool nav_goal_reached_ = false;
  bool nav_goal_failed_  = false;
  bool nav2_available_   = false;

  trajectory_msgs::msg::JointTrajectory arm_joint_trajectory_;

  // ---- Vision state ----
  DetectedObjectArray latest_objects_;
  PointingDirection   latest_pointing_;
  bool has_new_objects_  = false;
  bool has_new_pointing_ = false;

  // Accumulated observations for voting
  std::vector<DetectedObjectArray> obs_objects_;
  std::vector<PointingDirection>   obs_pointings_;

  // Selected targets
  geometry_msgs::msg::Point target_position_;
  bool   target_valid_ = false;
  std::string target_class_;

  geometry_msgs::msg::Point dest_position_;
  bool   dest_valid_ = false;
  std::string dest_class_;

  // =========================================================================
  // ROS handles
  // =========================================================================
  rclcpp::Publisher<interactive_cleanup::msg::InteractiveCleanupMsg>::SharedPtr pub_msg_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_base_twist_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_arm_trajectory_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_gripper_trajectory_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_head_trajectory_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_vision_enable_;

  rclcpp::Subscription<interactive_cleanup::msg::InteractiveCleanupMsg>::SharedPtr sub_msg_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_state_;
  rclcpp::Subscription<DetectedObjectArray>::SharedPtr sub_detected_objects_;
  rclcpp::Subscription<PointingDirection>::SharedPtr   sub_pointing_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;

  // =========================================================================
  // Callbacks
  // =========================================================================
  void messageCallback(const interactive_cleanup::msg::InteractiveCleanupMsg::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Recv: '%s' detail='%s'",
                msg->message.c_str(), msg->detail.c_str());

    if (msg->message == MSG_ARE_YOU_READY) {
      if (step_ == Ready) { is_started_ = true; }
      else { emergencyStop(); step_ = Initialize; }
    }
    else if (msg->message == MSG_PICK_IT_UP)     { received_pick_command_ = true; }
    else if (msg->message == MSG_CLEAN_UP)       { received_clean_command_ = true; }
    else if (msg->message == MSG_YES)            { received_yes_ = true; }
    else if (msg->message == MSG_NO)             { received_no_ = true; }
    else if (msg->message == MSG_TASK_SUCCEEDED) { is_finished_ = true; }
    else if (msg->message == MSG_TASK_FAILED) {
      is_failed_ = true;
      failed_detail_ = msg->detail;
      RCLCPP_WARN(this->get_logger(), "Task failed: '%s'", msg->detail.c_str());
    }
    else if (msg->message == MSG_MISSION_COMPLETE) {
      RCLCPP_INFO(this->get_logger(), "Mission complete!");
      emergencyStop();
      rclcpp::shutdown();
    }
  }

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(joint_mutex_);
    for (size_t i = 0; i < msg->name.size(); ++i)
      joint_positions_[msg->name[i]] = msg->position[i];
  }

  void objectsCallback(const DetectedObjectArray::SharedPtr msg)
  {
    latest_objects_ = *msg;
    has_new_objects_ = true;
    if (step_ == ObserveTarget || step_ == ObserveDestination)
      obs_objects_.push_back(*msg);
  }

  void pointingCallback(const PointingDirection::SharedPtr msg)
  {
    latest_pointing_ = *msg;
    has_new_pointing_ = true;
    if (step_ == ObserveTarget || step_ == ObserveDestination)
      obs_pointings_.push_back(*msg);
  }

  // =========================================================================
  // Joint helpers
  // =========================================================================
  double getJointPosition(const std::string &name, double def = 0.0)
  {
    std::lock_guard<std::mutex> lk(joint_mutex_);
    auto it = joint_positions_.find(name);
    return (it != joint_positions_.end()) ? it->second : def;
  }

  // =========================================================================
  // Message helpers
  // =========================================================================
  void sendMessage(const std::string &message)
  {
    RCLCPP_INFO(this->get_logger(), "Send: '%s'", message.c_str());
    interactive_cleanup::msg::InteractiveCleanupMsg m;
    m.message = message;
    pub_msg_->publish(m);
  }

  // =========================================================================
  // Base motion
  // =========================================================================
  void moveBase(double lx, double ly, double az)
  {
    geometry_msgs::msg::Twist t;
    t.linear.x = lx; t.linear.y = ly; t.angular.z = az;
    pub_base_twist_->publish(t);
  }
  void stopBase()            { moveBase(0, 0, 0); }
  void turnBase(double az)   { moveBase(0, 0, az); }
  void driveForward(double v){ moveBase(v, 0, 0); }

  // =========================================================================
  // Arm control
  // =========================================================================
  void moveArm(const std::vector<double> &pos, double dur)
  {
    arm_joint_trajectory_.points[0].positions = pos;
    arm_joint_trajectory_.points[0].time_from_start =
        rclcpp::Duration::from_seconds(dur);
    pub_arm_trajectory_->publish(arm_joint_trajectory_);
  }
  void armInitial()      { moveArm(ARM_INITIAL, 2.0); }
  void armObserve()      { moveArm(ARM_OBSERVE, 1.5); }
  void armGraspReady()   { moveArm(ARM_GRASP_READY, 1.5); }
  void armGraspLow()     { moveArm(ARM_GRASP_LOW, 1.5); }
  void armCarry()        { moveArm(ARM_CARRY, 1.5); }
  void armReleaseReady() { moveArm(ARM_RELEASE_READY, 1.5); }
  void armReleaseLow()   { moveArm(ARM_RELEASE_LOW, 1.5); }

  // =========================================================================
  // Head control
  // =========================================================================
  void moveHead(double pan, double tilt, double dur)
  {
    trajectory_msgs::msg::JointTrajectory jt;
    jt.joint_names = {"head_pan_joint", "head_tilt_joint"};
    trajectory_msgs::msg::JointTrajectoryPoint p;
    p.positions = {pan, tilt};
    p.time_from_start = rclcpp::Duration::from_seconds(dur);
    jt.points.push_back(p);
    pub_head_trajectory_->publish(jt);
  }
  void headCenter()    { moveHead(0.0,  0.0, 1.0); }
  void headLookDown()  { moveHead(0.0, -0.5, 1.0); }

  void headScanStep(const std::vector<HeadPose> &pattern, int idx)
  {
    int i = idx % static_cast<int>(pattern.size());
    moveHead(pattern[i].pan, pattern[i].tilt, 1.0);
  }

  // =========================================================================
  // Gripper
  // =========================================================================
  void operateHand(bool grasp)
  {
    trajectory_msgs::msg::JointTrajectory jt;
    jt.joint_names = {"hand_motor_joint"};
    trajectory_msgs::msg::JointTrajectoryPoint p;
    p.positions = {grasp ? -0.105 : 1.239};
    p.time_from_start = rclcpp::Duration::from_seconds(2.0);
    jt.points.push_back(p);
    pub_gripper_trajectory_->publish(jt);
  }
  void gripperOpen()  { operateHand(false); }
  void gripperClose() { operateHand(true); }

  // =========================================================================
  // Safety
  // =========================================================================
  void emergencyStop()
  {
    stopBase();
    armInitial();
    headCenter();
    enableVision(false);
    RCLCPP_WARN(this->get_logger(), "Emergency stop");
  }

  void giveUp()
  {
    RCLCPP_WARN(this->get_logger(), "Giving up");
    emergencyStop();
    sendMessage(MSG_GIVE_UP);
  }

  // =========================================================================
  // Vision helpers
  // =========================================================================
  void enableVision(bool on)
  {
    std_msgs::msg::Bool m;
    m.data = on;
    pub_vision_enable_->publish(m);
  }

  /**
   * Compute perpendicular distance from point (px,py) to a ray
   * starting at (ox,oy) with direction (dx,dy).
   * Returns large value if behind the ray origin.
   */
  static double pointRayDist2D(double px, double py,
                               double ox, double oy,
                               double dx, double dy)
  {
    double len = std::hypot(dx, dy);
    if (len < 1e-6) return 9999.0;
    dx /= len; dy /= len;
    double vx = px - ox, vy = py - oy;
    double proj = vx * dx + vy * dy;
    if (proj < 0) return 9999.0;
    double perp_x = vx - proj * dx;
    double perp_y = vy - proj * dy;
    return std::hypot(perp_x, perp_y);
  }

  /**
   * Select the target/destination from accumulated observations.
   * @param is_dest true → picking destination (prefer furniture classes)
   * @return true if a valid target was found
   */
  bool selectFromObservations(bool is_dest)
  {
    if (obs_objects_.empty()) {
      RCLCPP_WARN(this->get_logger(), "No detections during observation");
      return false;
    }

    struct Candidate {
      std::string name;
      double x, y, z;
      int votes;
      double conf_sum;
    };
    std::map<std::string, Candidate> cands;

    for (size_t fi = 0; fi < obs_objects_.size(); ++fi) {
      const auto &frame = obs_objects_[fi];

      // Find the temporally-closest pointing for this frame
      const PointingDirection *pt = nullptr;
      if (!obs_pointings_.empty()) {
        auto frame_t = rclcpp::Time(frame.header.stamp);
        double best_dt = 1e9;
        for (auto &p : obs_pointings_) {
          double dt = std::abs((rclcpp::Time(p.header.stamp) - frame_t).seconds());
          if (dt < best_dt) { best_dt = dt; pt = &p; }
        }
        if (best_dt > 1.0) pt = nullptr;
      }

      for (const auto &obj : frame.objects) {
        if (obj.class_name == "person") continue;
        if (!obj.has_3d_position) continue;

        if (is_dest) {
          // Prefer furniture-like classes for destination
          // (dining table, chair, bench, potted plant, etc.)
          // but accept any non-person if nothing else available
        }

        double ray_d = 9999.0;
        if (pt && pt->is_valid) {
          ray_d = pointRayDist2D(
            obj.bbox_cx, obj.bbox_cy,
            pt->wrist_pixel_x, pt->wrist_pixel_y,
            pt->point_pixel_x - pt->wrist_pixel_x,
            pt->point_pixel_y - pt->wrist_pixel_y);
        }

        // Build a spatial key to avoid merging distant same-class objects
        std::string key = obj.class_name + "_" +
            std::to_string(int(obj.position_3d.x * 2)) + "_" +
            std::to_string(int(obj.position_3d.y * 2));

        if (cands.find(key) == cands.end()) {
          cands[key] = {obj.class_name,
                        obj.position_3d.x, obj.position_3d.y, obj.position_3d.z,
                        0, 0.0};
        }
        auto &c = cands[key];

        double vote_weight = (ray_d < 100.0) ? (1.0 / (1.0 + ray_d / 50.0)) : 0.1;
        c.votes++;
        c.conf_sum += obj.confidence * vote_weight;

        // Running average of position
        double n = static_cast<double>(c.votes);
        c.x = c.x * (n - 1) / n + obj.position_3d.x / n;
        c.y = c.y * (n - 1) / n + obj.position_3d.y / n;
        c.z = c.z * (n - 1) / n + obj.position_3d.z / n;
      }
    }

    if (cands.empty()) {
      RCLCPP_WARN(this->get_logger(), "No valid candidates after filtering");
      return false;
    }

    // Pick the candidate with highest weighted confidence sum
    const Candidate *best = nullptr;
    double best_score = -1.0;
    for (auto &[k, c] : cands) {
      if (c.conf_sum > best_score) { best_score = c.conf_sum; best = &c; }
    }

    if (!best) return false;

    auto &pos   = is_dest ? dest_position_ : target_position_;
    auto &valid = is_dest ? dest_valid_     : target_valid_;
    auto &cls   = is_dest ? dest_class_     : target_class_;

    pos.x = best->x;
    pos.y = best->y;
    pos.z = best->z;
    valid = true;
    cls   = best->name;

    RCLCPP_INFO(this->get_logger(), "Selected %s: class='%s' pos=(%.2f, %.2f, %.2f) votes=%d",
                is_dest ? "DESTINATION" : "TARGET",
                cls.c_str(), pos.x, pos.y, pos.z, best->votes);
    return true;
  }

  // =========================================================================
  // Robot pose helpers (odom frame)
  // =========================================================================
  struct RobotPose { double x, y, yaw; bool valid; };

  RobotPose getRobotPose()
  {
    RobotPose rp{0, 0, 0, false};
    try {
      auto tf = tf_buffer_->lookupTransform("odom", "base_footprint",
                                            tf2::TimePointZero);
      rp.x = tf.transform.translation.x;
      rp.y = tf.transform.translation.y;
      tf2::Quaternion q;
      tf2::fromMsg(tf.transform.rotation, q);
      double r, p, ya;
      tf2::Matrix3x3(q).getRPY(r, p, ya);
      rp.yaw = ya;
      rp.valid = true;
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "TF error: %s", ex.what());
    }
    return rp;
  }

  double angleToPoint(double tx, double ty)
  {
    auto rp = getRobotPose();
    if (!rp.valid) return 0.0;
    double a = std::atan2(ty - rp.y, tx - rp.x) - rp.yaw;
    while (a >  M_PI) a -= 2 * M_PI;
    while (a < -M_PI) a += 2 * M_PI;
    return a;
  }

  double distToPoint(double tx, double ty)
  {
    auto rp = getRobotPose();
    if (!rp.valid) return 0.0;
    return std::hypot(tx - rp.x, ty - rp.y);
  }

  // =========================================================================
  // Nav2
  // =========================================================================
  void sendNavGoal(double x, double y, double yaw)
  {
    if (nav_goal_sent_) return;
    if (!nav_client_->wait_for_action_server(2s)) {
      RCLCPP_WARN(this->get_logger(), "Nav2 not available, using direct drive");
      nav2_available_ = false;
      nav_goal_failed_ = true;
      return;
    }
    nav2_available_ = true;

    tf2::Quaternion q; q.setRPY(0, 0, yaw); q.normalize();
    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp    = this->now();
    goal.pose.pose.position.x = x;
    goal.pose.pose.position.y = y;
    goal.pose.pose.orientation = tf2::toMsg(q);

    auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    opts.result_callback = [this](const GoalHandleNav::WrappedResult &res) {
      nav_goal_sent_ = false;
      if (res.code == rclcpp_action::ResultCode::SUCCEEDED) {
        nav_goal_reached_ = true;
      } else {
        nav_goal_failed_ = true;
      }
    };

    nav_client_->async_send_goal(goal, opts);
    nav_goal_sent_ = true;
    nav_goal_reached_ = false;
    nav_goal_failed_  = false;
    RCLCPP_INFO(this->get_logger(), "Nav goal: (%.2f, %.2f, yaw=%.2f)", x, y, yaw);
  }

  void resetNavState()
  {
    nav_goal_sent_ = false;
    nav_goal_reached_ = false;
    nav_goal_failed_  = false;
  }

  // =========================================================================
  // Timing
  // =========================================================================
  void reset()
  {
    is_started_ = false;
    is_finished_ = false;
    is_failed_ = false;
    failed_detail_ = "";
    received_pick_command_ = false;
    received_clean_command_ = false;
    received_yes_ = false;
    received_no_ = false;
    state_timer_initialized_ = false;
    sub_step_ = 0;
    scan_index_ = 0;
    point_again_count_ = 0;
    has_new_objects_ = false;
    has_new_pointing_ = false;
    target_valid_ = false;
    dest_valid_ = false;
    obs_objects_.clear();
    obs_pointings_.clear();
    resetNavState();
  }

  std::string stepName(int s)
  {
    switch (s) {
      case Initialize:       return "Initialize";
      case Ready:            return "Ready";
      case WaitForPickCommand:  return "WaitForPick";
      case ObserveTarget:    return "ObserveTarget";
      case WaitForCleanCommand: return "WaitForClean";
      case ObserveDestination: return "ObserveDest";
      case MoveToObject:     return "MoveToObject";
      case GraspObject:      return "GraspObject";
      case SendObjectGrasped: return "SendGrasped";
      case MoveToDestination: return "MoveToDest";
      case ReleaseObject:    return "ReleaseObj";
      case SendTaskFinished: return "SendFinished";
      case WaitForResult:    return "WaitResult";
      default:               return "?";
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

  double elapsed()
  {
    return state_timer_initialized_
      ? (this->now() - state_enter_time_).seconds() : 0.0;
  }

  void changeStep(int next)
  {
    RCLCPP_INFO(this->get_logger(), "[%s] -> [%s]",
                stepName(step_).c_str(), stepName(next).c_str());
    step_ = next;
    enterNewState();
  }

  // =========================================================================
  // State machine
  // =========================================================================
  void runStateMachine()
  {
    // Global failure handler
    if (is_failed_) {
      RCLCPP_WARN(this->get_logger(), "FAILED in [%s] detail='%s'",
                  stepName(step_).c_str(), failed_detail_.c_str());
      emergencyStop();
      step_ = Initialize;
      is_failed_ = false;
      return;
    }

    switch (step_) {

    // -----------------------------------------------------------------
    case Initialize: {
      reset();
      armInitial();
      headCenter();
      gripperOpen();
      enableVision(false);
      RCLCPP_INFO(this->get_logger(), "[Init] Waiting for Are_you_ready?");
      step_ = Ready;
      break;
    }

    // -----------------------------------------------------------------
    case Ready: {
      if (is_started_) {
        armObserve();
        headCenter();
        sendMessage(MSG_I_AM_READY);
        enableVision(true);
        changeStep(WaitForPickCommand);
      }
      break;
    }

    // -----------------------------------------------------------------
    case WaitForPickCommand: {
      if (received_pick_command_) {
        RCLCPP_INFO(this->get_logger(), "[WaitForPick] Got Pick_it_up! → observing target");
        received_pick_command_ = false;
        obs_objects_.clear();
        obs_pointings_.clear();
        scan_index_ = 0;
        changeStep(ObserveTarget);
      } else if (state_timer_initialized_ && elapsed() > TIMEOUT_WAIT_COMMAND) {
        giveUp();
        changeStep(WaitForResult);
      }
      break;
    }

    // -----------------------------------------------------------------
    case ObserveTarget: {
      // Scan the scene for TIMEOUT_OBSERVE seconds, accumulating detections
      double t = elapsed();
      int expected_scan = static_cast<int>(t / 1.2);
      if (expected_scan > scan_index_ &&
          scan_index_ < static_cast<int>(SCAN_WIDE.size())) {
        headScanStep(SCAN_WIDE, scan_index_);
        scan_index_++;
      }

      if (t >= TIMEOUT_OBSERVE) {
        headCenter();
        bool found = selectFromObservations(false);
        if (!found && point_again_count_ < MAX_POINT_AGAIN) {
          RCLCPP_WARN(this->get_logger(), "[ObserveTarget] Target not found, requesting re-point");
          point_again_count_++;
          sendMessage(MSG_POINT_IT_AGAIN);
          obs_objects_.clear();
          obs_pointings_.clear();
          changeStep(WaitForPickCommand);
        } else if (!found) {
          RCLCPP_WARN(this->get_logger(), "[ObserveTarget] No target after max re-points, giving up");
          giveUp();
          changeStep(WaitForResult);
        } else {
          changeStep(WaitForCleanCommand);
        }
      }
      break;
    }

    // -----------------------------------------------------------------
    case WaitForCleanCommand: {
      // Continue scanning while waiting
      double t = elapsed();
      int expected = static_cast<int>(t / 1.5);
      if (expected > scan_index_ && scan_index_ < static_cast<int>(SCAN_WIDE.size())) {
        headScanStep(SCAN_WIDE, scan_index_ % SCAN_WIDE.size());
        scan_index_++;
      }

      if (received_clean_command_) {
        RCLCPP_INFO(this->get_logger(), "[WaitForClean] Got Clean_up! → observing destination");
        received_clean_command_ = false;
        obs_objects_.clear();
        obs_pointings_.clear();
        scan_index_ = 0;
        changeStep(ObserveDestination);
      } else if (elapsed() > TIMEOUT_WAIT_COMMAND) {
        giveUp();
        changeStep(WaitForResult);
      }
      break;
    }

    // -----------------------------------------------------------------
    case ObserveDestination: {
      double t = elapsed();
      int expected = static_cast<int>(t / 1.2);
      if (expected > scan_index_ &&
          scan_index_ < static_cast<int>(SCAN_WIDE.size())) {
        headScanStep(SCAN_WIDE, scan_index_);
        scan_index_++;
      }

      if (t >= TIMEOUT_OBSERVE) {
        headCenter();
        bool found = selectFromObservations(true);
        if (!found && point_again_count_ < MAX_POINT_AGAIN) {
          RCLCPP_WARN(this->get_logger(), "[ObserveDest] Dest not found, requesting re-point");
          point_again_count_++;
          sendMessage(MSG_POINT_IT_AGAIN);
          obs_objects_.clear();
          obs_pointings_.clear();
          changeStep(WaitForPickCommand);
        } else if (!found) {
          RCLCPP_WARN(this->get_logger(), "[ObserveDest] No destination, giving up");
          giveUp();
          changeStep(WaitForResult);
        } else {
          enableVision(false);
          changeStep(MoveToObject);
        }
      }
      break;
    }

    // -----------------------------------------------------------------
    case MoveToObject: {
      if (elapsed() > TIMEOUT_MOVE && sub_step_ > 0) {
        emergencyStop(); giveUp(); changeStep(WaitForResult); break;
      }

      switch (sub_step_) {
        case 0: {
          armObserve();
          headLookDown();
          gripperOpen();
          RCLCPP_INFO(this->get_logger(), "[MoveToObj] Preparing approach...");

          if (target_valid_) {
            double yaw = angleToPoint(target_position_.x, target_position_.y);
            sendNavGoal(target_position_.x, target_position_.y, yaw);
          }
          sub_step_ = 1;
          enterTimedState();
          break;
        }
        case 1: {
          // Navigate — Nav2 or direct drive
          if (target_valid_ && nav2_available_ && nav_goal_sent_) {
            if (nav_goal_reached_) {
              RCLCPP_INFO(this->get_logger(), "[MoveToObj] Nav2 goal reached");
              stopBase();
              sub_step_ = 3;
              enterTimedState();
            } else if (nav_goal_failed_) {
              RCLCPP_WARN(this->get_logger(), "[MoveToObj] Nav2 failed, fallback to direct");
              resetNavState();
              sub_step_ = 2;
              enterTimedState();
            }
          } else {
            sub_step_ = 2;
            enterTimedState();
          }
          break;
        }
        case 2: {
          // Direct drive toward target
          if (!target_valid_) {
            // No vision target — drive forward blindly for a few seconds (legacy)
            if (elapsed() < 4.0) {
              driveForward(0.15);
            } else {
              stopBase();
              sub_step_ = 4;
            }
            break;
          }

          double dist = distToPoint(target_position_.x, target_position_.y);
          double angle = angleToPoint(target_position_.x, target_position_.y);

          if (dist > APPROACH_STOP_DIST) {
            if (std::abs(angle) > TURN_THRESHOLD) {
              double az = std::clamp(angle * 1.5, -0.5, 0.5);
              moveBase(0.05, 0.0, az);
            } else {
              double speed = std::min(0.2, dist * 0.4);
              moveBase(speed, 0.0, angle * 0.3);
            }
          } else {
            stopBase();
            RCLCPP_INFO(this->get_logger(), "[MoveToObj] Reached vicinity (%.2fm)", dist);
            sub_step_ = 3;
            enterTimedState();
          }
          break;
        }
        case 3: {
          // Fine approach: enable vision and use detections to center on target
          enableVision(true);
          headLookDown();

          if (elapsed() < 1.0) break; // wait for detections

          if (has_new_objects_) {
            has_new_objects_ = false;
            // Find matching object in current view
            const DetectedObject *match = nullptr;
            double best_dist = 9999.0;
            for (const auto &o : latest_objects_.objects) {
              if (o.class_name == "person") continue;
              // Prefer same class; otherwise any close object
              double d = std::hypot(o.bbox_cx - 320.0, o.bbox_cy - 240.0);
              if (o.class_name == target_class_) d *= 0.5;
              if (d < best_dist) { best_dist = d; match = &o; }
            }

            if (match) {
              double dx = (match->bbox_cx - 320.0) / 320.0;
              int area = match->bbox_w * match->bbox_h;

              if (area > 40000) {
                // Object is large enough in view → close enough
                stopBase();
                enableVision(false);
                sub_step_ = 4;
                break;
              }
              moveBase(VISUAL_SERVO_FWD, 0.0, -dx * 0.4);
            } else {
              driveForward(VISUAL_SERVO_FWD);
            }
          }

          if (elapsed() > 12.0) {
            stopBase();
            enableVision(false);
            sub_step_ = 4;
          }
          break;
        }
        case 4: {
          enableVision(false);
          changeStep(GraspObject);
          break;
        }
      }
      break;
    }

    // -----------------------------------------------------------------
    case GraspObject: {
      if (elapsed() > TIMEOUT_GRASP && sub_step_ > 0) {
        emergencyStop(); giveUp(); changeStep(WaitForResult); break;
      }

      switch (sub_step_) {
        case 0:
          armGraspReady(); headLookDown();
          sub_step_ = 1; enterTimedState(); break;
        case 1:
          if (elapsed() > 2.0) { armGraspLow(); sub_step_ = 2; enterTimedState(); }
          break;
        case 2:
          if (elapsed() > 2.0) { gripperClose(); sub_step_ = 3; enterTimedState(); }
          break;
        case 3:
          if (elapsed() > 3.0) { armCarry(); sub_step_ = 4; enterTimedState(); }
          break;
        case 4:
          if (elapsed() > 2.0) { changeStep(SendObjectGrasped); }
          break;
      }
      break;
    }

    // -----------------------------------------------------------------
    case SendObjectGrasped: {
      sendMessage(MSG_OBJECT_GRASPED);
      changeStep(MoveToDestination);
      break;
    }

    // -----------------------------------------------------------------
    case MoveToDestination: {
      if (elapsed() > TIMEOUT_MOVE && sub_step_ > 0) {
        emergencyStop(); giveUp(); changeStep(WaitForResult); break;
      }

      switch (sub_step_) {
        case 0: {
          headCenter();
          RCLCPP_INFO(this->get_logger(), "[MoveToDest] Heading to destination...");

          if (dest_valid_) {
            double yaw = angleToPoint(dest_position_.x, dest_position_.y);
            sendNavGoal(dest_position_.x, dest_position_.y, yaw);
          }
          sub_step_ = 1;
          enterTimedState();
          break;
        }
        case 1: {
          if (dest_valid_ && nav2_available_ && nav_goal_sent_) {
            if (nav_goal_reached_) {
              stopBase();
              sub_step_ = 3;
              enterTimedState();
            } else if (nav_goal_failed_) {
              resetNavState();
              sub_step_ = 2;
              enterTimedState();
            }
          } else {
            sub_step_ = 2;
            enterTimedState();
          }
          break;
        }
        case 2: {
          // Direct drive
          if (!dest_valid_) {
            if (elapsed() < 2.0)     { turnBase(0.5); }
            else if (elapsed() < 5.0){ driveForward(0.15); }
            else { stopBase(); sub_step_ = 3; }
            break;
          }

          double dist = distToPoint(dest_position_.x, dest_position_.y);
          double angle = angleToPoint(dest_position_.x, dest_position_.y);

          if (dist > APPROACH_STOP_DIST) {
            if (std::abs(angle) > TURN_THRESHOLD) {
              moveBase(0.05, 0.0, std::clamp(angle * 1.5, -0.5, 0.5));
            } else {
              moveBase(std::min(0.2, dist * 0.4), 0.0, angle * 0.3);
            }
          } else {
            stopBase();
            sub_step_ = 3;
            enterTimedState();
          }
          break;
        }
        case 3: {
          changeStep(ReleaseObject);
          break;
        }
      }
      break;
    }

    // -----------------------------------------------------------------
    case ReleaseObject: {
      if (elapsed() > TIMEOUT_RELEASE && sub_step_ > 0) {
        emergencyStop(); giveUp(); changeStep(WaitForResult); break;
      }

      switch (sub_step_) {
        case 0:
          armReleaseReady(); headLookDown();
          sub_step_ = 1; enterTimedState(); break;
        case 1:
          if (elapsed() > 2.0) { armReleaseLow(); sub_step_ = 2; enterTimedState(); }
          break;
        case 2:
          if (elapsed() > 2.0) { gripperOpen(); sub_step_ = 3; enterTimedState(); }
          break;
        case 3:
          if (elapsed() > 3.0) {
            armInitial(); headCenter();
            sub_step_ = 4; enterTimedState();
          }
          break;
        case 4:
          if (elapsed() < 2.0) { driveForward(-0.1); }
          else { stopBase(); changeStep(SendTaskFinished); }
          break;
      }
      break;
    }

    // -----------------------------------------------------------------
    case SendTaskFinished: {
      sendMessage(MSG_TASK_FINISHED);
      changeStep(WaitForResult);
      break;
    }

    // -----------------------------------------------------------------
    case WaitForResult: {
      enableVision(false);
      if (is_finished_) {
        RCLCPP_INFO(this->get_logger(), "[WaitResult] Task succeeded!");
        step_ = Initialize;
      } else if (elapsed() > TIMEOUT_WAIT_RESULT) {
        RCLCPP_WARN(this->get_logger(), "[WaitResult] Timeout, resetting");
        step_ = Initialize;
      }
      break;
    }

    } // switch
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
