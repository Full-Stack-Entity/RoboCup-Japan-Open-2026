/**
 * @file vision_manipulation_controller.cpp
 * @brief 视觉对准与机械臂控制层实现
 */

#include "handyman_ros2/vision_manipulation_controller.hpp"

namespace handyman_ros2 {

// ---------------------------------------------------------------------------
// 构造函数
// ---------------------------------------------------------------------------

VisionManipulationController::VisionManipulationController(rclcpp::Node::SharedPtr node)
  : node_(node),
    last_detection_time_(0, 0, node_->get_clock()->get_clock_type())
{
    // 订阅手部检测（YOLO bbox 结果）
    sub_hand_detection_ = node_->create_subscription<std_msgs::msg::Int32MultiArray>(
        "/hand_detection", 1000,
        std::bind(&VisionManipulationController::handDetectionCallback, this, std::placeholders::_1));

    // 订阅目标位姿（/vision）
    sub_vision_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/vision", 1000,
        std::bind(&VisionManipulationController::visionCallback, this, std::placeholders::_1));

    // 订阅关节状态（用于抓取验证）
    sub_joint_states_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/hsrb/joint_states", 10,
        std::bind(&VisionManipulationController::jointStatesCallback, this, std::placeholders::_1));

    RCLCPP_INFO(node_->get_logger(), "VisionManipulationController initialized");
}

// ---------------------------------------------------------------------------
// 状态转换
// ---------------------------------------------------------------------------

void VisionManipulationController::startRoomScan()
{
    RCLCPP_INFO(node_->get_logger(),
                "VisionManipulationController: Starting room scan");
    state_ = State::ROOM_SCAN;
    scan_total_rotation_ = 0.0;
    scan_start_time_ = node_->now();
    scan_complete_ = false;
    aligned_x_ = false;
    aligned_y_ = false;
    grasp_ready_ = false;
    x_adjust_ = 0.0;
    y_adjust_ = 0.0;
    arm_height_ = 0.0;
    object_found_ = false;

    // 设置扫描姿态
    setInitialScanPose();
}

void VisionManipulationController::stopScan()
{
    if (state_ == State::ROOM_SCAN || state_ == State::ALIGNING) {
        RCLCPP_INFO(node_->get_logger(),
                    "VisionManipulationController: Stopping room scan");
        state_ = State::IDLE;
        stopBase();
    }
}

void VisionManipulationController::reset()
{
    std::lock_guard<std::mutex> lock(gripper_mutex_);
    state_ = State::IDLE;
    scan_complete_ = false;
    grasp_ready_ = false;
    aligned_x_ = false;
    aligned_y_ = false;
    grasp_retries_ = 0;
    extend_before_release_ = false;
    release_complete_ = false;
    grasping_ = false;
    scan_total_rotation_ = 0.0;
    arm_height_ = 0.0;
    x_adjust_ = 0.0;
    y_adjust_ = 0.0;
    tol_multiplier_ = 1.0;
    object_found_ = false;
    gripper_actual_position_ = 0.0;
}

// ---------------------------------------------------------------------------
// 每循环调用
// ---------------------------------------------------------------------------

void VisionManipulationController::update()
{
    // 主要逻辑由 TaskOrchestrator 在主循环中驱动
    // 此处仅处理状态相关的清理
}

void VisionManipulationController::publishBaseCommand(
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr& pub_base_twist)
{
    if (state_ == State::ROOM_SCAN) {
        // 房间扫描模式：持续旋转
        double rotation_delta = SCAN_ROTATION_SPEED * 0.1;  // 约 0.1s 一次循环
        scan_total_rotation_ += rotation_delta;

        // 直接发布 Twist 命令
        geometry_msgs::msg::Twist twist;
        twist.linear.x = 0.0;
        twist.linear.y = 0.0;
        twist.angular.z = SCAN_ROTATION_SPEED;
        pub_base_twist->publish(twist);

        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
            "VisionManipulationController: Scanning... (%.1f / %.1f rad, %.1f deg)",
            scan_total_rotation_, MAX_SCAN_ROTATION,
            scan_total_rotation_ * 180.0 / M_PI);

        // 检查是否完成 360° 扫描
        if (scan_total_rotation_ >= MAX_SCAN_ROTATION) {
            RCLCPP_WARN(node_->get_logger(),
                "VisionManipulationController: Room scan complete (360 deg), object NOT found");
            scan_complete_ = true;
            stopBase();
            state_ = State::IDLE;
        }
    } else if (state_ == State::ALIGNING) {
        // 对准模式：根据检测结果调整
        if (objectDetected()) {
            // 计算调整量
            if (det_x_ < 240 - ALIGN_TOL_BASE * tol_multiplier_) {
                x_adjust_ = 0.03;
                aligned_x_ = false;
            } else if (det_x_ > 240 + ALIGN_TOL_BASE * tol_multiplier_) {
                x_adjust_ = -0.03;
                aligned_x_ = false;
            } else {
                x_adjust_ = 0.0;
                aligned_x_ = true;
            }

            if (det_y_ < 360 - ALIGN_TOL_BASE * tol_multiplier_) {
                arm_height_ += 0.004;
                if (arm_height_ > 0.0) arm_height_ = 0.0;
                y_adjust_ = 0.01;
                aligned_y_ = false;
            } else if (det_y_ > 360 + ALIGN_TOL_BASE * tol_multiplier_) {
                arm_height_ -= 0.004;
                y_adjust_ = -0.01;
                aligned_y_ = false;
            } else {
                y_adjust_ = 0.0;
                aligned_y_ = true;
            }

            // 发布手臂调整
            std::vector<double> positions = { 0.2, arm_height_, 0.0, -1.57 - arm_height_ * 0.4, 0.0 };
            moveArmInternal(positions, 1.0);

            // 发布底盘调整（直接发布，不调用 moveBase）
            geometry_msgs::msg::Twist twist;
            twist.linear.x = y_adjust_;
            twist.linear.y = 0.0;
            twist.angular.z = x_adjust_ * 0.5;
            pub_base_twist->publish(twist);

            // 检查是否对准
            if (aligned_x_ && aligned_y_) {
                if (!grasp_ready_) {
                    // 还不够近，微微前进
                    geometry_msgs::msg::Twist twist2;
                    twist2.linear.x = 0.02;
                    twist2.linear.y = 0.0;
                    twist2.angular.z = 0.0;
                    pub_base_twist->publish(twist2);
                } else {
                    // 完全对准且足够近，停止底盘
                    stopBase();
                    // 通知 TaskOrchestrator 可以进入抓取了
                    state_ = State::IDLE;
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// 对准结果
// ---------------------------------------------------------------------------

VisionManipulationController::AlignmentResult VisionManipulationController::getAlignment() const
{
    AlignmentResult result;
    result.base_angular = x_adjust_;
    result.base_linear_x = y_adjust_;
    result.arm_lift = arm_height_;
    result.aligned_x = aligned_x_;
    result.aligned_y = aligned_y_;
    result.aligned = aligned_x_ && aligned_y_;
    result.grasp_ready = grasp_ready_;
    return result;
}

void VisionManipulationController::getDetection(int& x, int& y, int& w, int& h) const
{
    x = det_x_;
    y = det_y_;
    w = det_w_;
    h = det_h_;
}

// ---------------------------------------------------------------------------
// 回调
// ---------------------------------------------------------------------------

void VisionManipulationController::handDetectionCallback(
    const std_msgs::msg::Int32MultiArray::ConstSharedPtr& msg)
{
    if (msg->data.size() < 4) return;

    det_x_ = msg->data[0];
    det_y_ = msg->data[1];
    det_w_ = msg->data[2];
    det_h_ = msg->data[3];
    last_detection_time_ = node_->now();
    detection_fresh_ = true;
    object_found_ = true;

    // 判断是否足够近（可以抓取）
    if (det_w_ >= GRASP_READY_WIDTH) {
        grasp_ready_ = true;
        tol_multiplier_ = ALIGN_TOL_FACTOR_CLOSE;
    } else {
        grasp_ready_ = false;
        tol_multiplier_ = 1.0;
    }

    // 检测到物体时，如果正在扫描，切换到对准模式
    if (state_ == State::ROOM_SCAN) {
        RCLCPP_INFO(node_->get_logger(),
                    "VisionManipulationController: Object detected during scan! Switching to alignment.");
        state_ = State::ALIGNING;
        aligned_x_ = false;
        aligned_y_ = false;
        stopBase();
    }

    RCLCPP_DEBUG(node_->get_logger(),
                 "VisionManipulationController: Detection -> x=%d, y=%d, w=%d, h=%d, grasp_ready=%d",
                 det_x_.load(), det_y_.load(), det_w_.load(), det_h_.load(), grasp_ready_.load());
}

void VisionManipulationController::visionCallback(
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg)
{
    // 当前主流程不使用 /vision，此处仅记录日志
    RCLCPP_DEBUG(node_->get_logger(),
                 "VisionManipulationController: Received vision pose (%.3f, %.3f, %.3f)",
                 msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
}

void VisionManipulationController::jointStatesCallback(
    const sensor_msgs::msg::JointState::ConstSharedPtr& msg)
{
    std::lock_guard<std::mutex> lock(gripper_mutex_);

    // 查找 hand_motor_joint 的位置
    for (size_t i = 0; i < msg->name.size(); ++i) {
        if (msg->name[i] == HAND_JOINT_NAME) {
            if (i < msg->position.size()) {
                gripper_actual_position_ = msg->position[i];
            }
            break;
        }
    }
}

// ---------------------------------------------------------------------------
// 底盘控制
// ---------------------------------------------------------------------------

void VisionManipulationController::moveBase(double linear_x, double linear_y, double angular_z)
{
    // 这个函数需要 publisher，实际由 TaskOrchestrator 调用
    // 此处仅记录 debug 信息
    (void)linear_x;
    (void)linear_y;
    (void)angular_z;
}

void VisionManipulationController::stopBase()
{
    // 底盘停止由 TaskOrchestrator 直接调用
}

// ---------------------------------------------------------------------------
// 手臂控制
// ---------------------------------------------------------------------------

void VisionManipulationController::moveArm(const std::vector<double>& positions, double duration_sec)
{
    moveArmInternal(positions, duration_sec);
}

void VisionManipulationController::moveArmInternal(const std::vector<double>& positions, double duration_sec)
{
    if (!pub_arm_trajectory_) {
        RCLCPP_WARN(node_->get_logger(),
                    "VisionManipulationController: arm publisher not set");
        return;
    }

    trajectory_msgs::msg::JointTrajectory traj;
    traj.joint_names = {
        "arm_lift_joint", "arm_flex_joint", "arm_roll_joint",
        "wrist_flex_joint", "wrist_roll_joint"
    };

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = positions;
    point.time_from_start = rclcpp::Duration::from_seconds(duration_sec);
    traj.points.push_back(point);

    pub_arm_trajectory_->publish(traj);
}

void VisionManipulationController::setScanningPose()
{
    std::vector<double> positions = { 0.2, 0.0, 0.0, -1.57, 0.0 };
    moveArmInternal(positions, 1.0);
}

void VisionManipulationController::setCarryPose()
{
    std::vector<double> positions = { 0.2, -0.3, 0.0, -1.57, 0.0 };
    moveArmInternal(positions, 1.0);
}

void VisionManipulationController::setReleasePose()
{
    std::vector<double> positions = { 0.2, -0.8, 0.0, -1.0, 0.0 };
    moveArmInternal(positions, 1.0);
}

void VisionManipulationController::setInitialScanPose()
{
    std::vector<double> positions = { 0.2, 0.0, 0.0, -1.57, 0.0 };
    moveArmInternal(positions, 1.0);
}

// ---------------------------------------------------------------------------
// 夹爪控制
// ---------------------------------------------------------------------------

void VisionManipulationController::openGripper()
{
    operateHandInternal(false);
}

void VisionManipulationController::closeGripper()
{
    operateHandInternal(true);
}

void VisionManipulationController::operateHandInternal(bool should_grasp)
{
    if (!pub_gripper_trajectory_) {
        RCLCPP_WARN(node_->get_logger(),
                    "VisionManipulationController: gripper publisher not set");
        return;
    }

    std::vector<std::string> joint_names = { HAND_JOINT_NAME };
    std::vector<double> positions;

    if (should_grasp) {
        positions.push_back(HAND_JOINT_GRASP_POSITION);
    } else {
        positions.push_back(HAND_JOINT_OPEN_POSITION);
    }

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = positions;
    point.time_from_start = rclcpp::Duration::from_seconds(2.0);

    trajectory_msgs::msg::JointTrajectory joint_trajectory;
    joint_trajectory.joint_names = joint_names;
    joint_trajectory.points.push_back(point);

    pub_gripper_trajectory_->publish(joint_trajectory);

    RCLCPP_DEBUG(node_->get_logger(), "VisionManipulationController: Gripper %s",
                 should_grasp ? "CLOSED (grasp)" : "OPENED");
}

// ---------------------------------------------------------------------------
// 抓取验证
// ---------------------------------------------------------------------------

VisionManipulationController::GraspResult VisionManipulationController::verifyGrasp()
{
    std::lock_guard<std::mutex> lock(gripper_mutex_);

    double pos = gripper_actual_position_;
    RCLCPP_INFO(node_->get_logger(),
                "VisionManipulationController: Grasp verify: gripper_pos=%.4f (target=%.3f, tolerance=%.3f)",
                pos, HAND_JOINT_GRASP_POSITION, GRASP_TOLERANCE);

    if (std::abs(pos - HAND_JOINT_GRASP_POSITION) < GRASP_TOLERANCE) {
        RCLCPP_INFO(node_->get_logger(),
                    "VisionManipulationController: Grasp SUCCESS");
        return GraspResult::SUCCESS;
    }

    if (pos > HAND_JOINT_GRASP_POSITION - 0.05) {
        // 夹爪没有闭合到目标位置，说明打滑
        RCLCPP_WARN(node_->get_logger(),
                    "VisionManipulationController: Grasp SLIPPED (gripper at %.4f, expected %.3f)",
                    pos, HAND_JOINT_GRASP_POSITION);
        return GraspResult::SLIPPED;
    }

    RCLCPP_ERROR(node_->get_logger(),
                  "VisionManipulationController: Grasp MISSED");
    return GraspResult::MISSED;
}

double VisionManipulationController::getGripperPosition() const
{
    std::lock_guard<std::mutex> lock(gripper_mutex_);
    return gripper_actual_position_;
}

// ---------------------------------------------------------------------------
// 放置序列
// ---------------------------------------------------------------------------

bool VisionManipulationController::startReleaseSequence()
{
    if (!extend_before_release_) {
        release_start_time_ = node_->now();
        extend_before_release_ = true;
        RCLCPP_INFO(node_->get_logger(),
                    "VisionManipulationController: Release sequence started");
        return false;  // 还未完成
    }

    double elapsed = (node_->now() - release_start_time_).seconds();

    if (elapsed < RELEASE_WAIT_A) {
        // 张开前等待
        return false;
    } else if (elapsed < RELEASE_WAIT_A + RELEASE_WAIT_B) {
        // 张开后等待
        return false;
    }

    RCLCPP_INFO(node_->get_logger(),
                "VisionManipulationController: Release sequence complete");
    release_complete_ = true;
    return true;
}

}  // namespace handyman_ros2
