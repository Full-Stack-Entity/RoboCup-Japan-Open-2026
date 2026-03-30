#pragma once

/**
 * @file vision_manipulation_controller.hpp
 * @brief 视觉对准与机械臂控制层头文件
 *
 * 模块3：视觉对准与机械臂控制层
 * 功能范围：
 * - 订阅 /hand_detection / /vision，处理目标检测结果
 * - 提供房间扫描（原地旋转搜索物体）
 * - 提供近距离对准控制（bbox → 底盘 + 手臂调整量）
 * - 提供手臂关节控制和夹爪控制
 * - 提供抓取验证（订阅 /hsrb/joint_states）
 * - 提供放置动作
 *
 * 被 TaskOrchestrator 持有和调用。
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <memory>
#include <atomic>
#include <mutex>

namespace handyman_ros2 {

/**
 * @brief 视觉对准与机械臂控制器
 *
 * 职责：
 * - 订阅 YOLO 检测结果（/hand_detection），提供对准状态
 * - 提供房间扫描模式（原地旋转搜索）
 * - 封装手臂关节控制 /hsrb/arm_trajectory_controller/command
 * - 封装夹爪控制 /hsrb/gripper_controller/command
 * - 提供抓取验证（读取 hand_motor_joint 位置）
 * - 提供放置动作序列
 *
 * 不包含任务逻辑，只负责"对准目标"和"抓取/放置动作"。
 */
class VisionManipulationController {
public:
    // ========================================================================
    // 常量
    // ========================================================================

    // 扫描参数
    static constexpr double MAX_SCAN_ROTATION = 6.28;     // 360° 扫描上限
    static constexpr double SCAN_ROTATION_SPEED = 0.3;    // rad/s
    static constexpr double DETECTION_TIMEOUT_SEC = 1.0;  // 检测超时（秒）

    // 对准参数
    static constexpr double ALIGN_TOL_BASE = 25.0;       // 像素容差（图像中心）
    static constexpr double ALIGN_TOL_FACTOR_CLOSE = 1.5; // 检测框足够大时放宽容差

    // 抓取验证参数
    static constexpr double HAND_JOINT_GRASP_POSITION = -0.105;
    static constexpr double HAND_JOINT_OPEN_POSITION = 1.239;
    static constexpr double GRASP_TOLERANCE = 0.03;       // 抓取验证容差
    static constexpr int MAX_GRASP_RETRIES = 2;

    // 放置参数
    static constexpr double RELEASE_WAIT_A = 4.0;  // 张开前等待（秒）
    static constexpr double RELEASE_WAIT_B = 8.0;  // 张开后等待（秒）

    // 检测阈值
    static constexpr int GRASP_READY_WIDTH = 450;  // 检测框宽度达到此值认为可以抓取

    // ========================================================================
    // 对准结果
    // ========================================================================

    struct AlignmentResult {
        double base_angular;   // 底盘角速度（rad/s）
        double base_linear_x;  // 底盘前进速度（m/s）
        double arm_lift;       // 手臂升降调整量
        bool aligned_x;       // X方向（左右）对准
        bool aligned_y;       // Y方向（上下）对准
        bool aligned;         // 完全对准（aligned_x && aligned_y）
        bool grasp_ready;     // 可以执行抓取
    };

    // ========================================================================
    // 抓取验证结果
    // ========================================================================

    enum class GraspResult {
        SUCCESS,   // 抓取成功
        SLIPPED,  // 抓取打滑，需重试
        MISSED    // 抓取失败
    };

    // ========================================================================
    // 状态枚举
    // ========================================================================

    enum class State {
        IDLE,       // 空闲，不做任何动作
        ROOM_SCAN,  // 房间扫描（原地旋转）
        ALIGNING,   // 对准中（根据 bbox 调整）
        GRASPING,   // 抓取中
        RELEASING   // 放置中
    };

    // ========================================================================
    // 构造与初始化
    // ========================================================================

    explicit VisionManipulationController(rclcpp::Node::SharedPtr node);

    // ========================================================================
    // 状态转换
    // ========================================================================

    // 进入房间扫描模式（进入目标房间后调用）
    void startRoomScan();

    // 停止扫描（检测到物体后调用）
    void stopScan();

    // 重置所有状态
    void reset();

    // ========================================================================
    // 每循环调用（由 TaskOrchestrator 在主循环中调用）
    // ========================================================================

    // 更新状态（处理检测回调、更新扫描等）
    void update();

    // 发布底盘控制命令（调用者需持有 publisher）
    void publishBaseCommand(rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr& pub_base_twist);

    // ========================================================================
    // 状态查询
    // ========================================================================

    // 是否正在扫描
    bool isScanning() const { return state_ == State::ROOM_SCAN; }

    // 扫描是否完成（360° 未找到物体）
    bool isScanComplete() const { return scan_complete_; }

    // 最近 1 秒内有检测到物体
    bool objectDetected() const {
        return (node_->now() - last_detection_time_).seconds() < DETECTION_TIMEOUT_SEC;
    }

    // 获取对准结果
    AlignmentResult getAlignment() const;

    // 获取当前检测框
    void getDetection(int& x, int& y, int& w, int& h) const;

    // ========================================================================
    // 手臂控制
    // ========================================================================

    // 移动手臂到指定关节位置
    void moveArm(const std::vector<double>& positions, double duration_sec);

    // 发布扫描姿态（高位置，手臂伸展）
    void setScanningPose();

    // 发布搬运姿态（低位置，手臂收回）
    void setCarryPose();

    // 发布放置姿态（伸直位置）
    void setReleasePose();

    // 发布扫描姿态（room scan 开始时）
    void setInitialScanPose();

    // ========================================================================
    // 夹爪控制
    // ========================================================================

    // 张开夹爪
    void openGripper();

    // 闭合夹爪（抓取）
    void closeGripper();

    // ========================================================================
    // 抓取验证（订阅 /hsrb/joint_states）
    // ========================================================================

    // 验证抓取是否成功
    // 返回 SUCCESS = 成功，SLIPPED = 打滑需重试，MISSED = 完全失败
    GraspResult verifyGrasp();

    // 获取当前夹爪实际位置（用于调试）
    double getGripperPosition() const;

    // 获取重试次数
    int getGripRetries() const { return grasp_retries_; }

    // ========================================================================
    // 放置序列（Release）
    // ========================================================================

    // 开始放置序列（返回 true = 放置完成）
    bool startReleaseSequence();

    // 查询放置是否完成
    bool isReleaseComplete() const { return release_complete_; }

    // ========================================================================
    // 内部成员
    // ========================================================================

private:
    rclcpp::Node::SharedPtr node_;

    // 订阅
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_hand_detection_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_vision_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_states_;

    // 发布（由 TaskOrchestrator 在 run() 中传递）
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_arm_trajectory_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_gripper_trajectory_;

    // 发布者设置（由 TaskOrchestrator 调用）
public:
    void setArmPublisher(rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub) {
        pub_arm_trajectory_ = pub;
    }
    void setGripperPublisher(rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub) {
        pub_gripper_trajectory_ = pub;
    }

private:
    // 检测结果
    std::atomic<bool> object_found_{false};
    std::atomic<int> det_x_{0}, det_y_{0}, det_w_{0}, det_h_{0};
    rclcpp::Time last_detection_time_;
    std::atomic<bool> detection_fresh_{false};  // 最近 1s 内有检测

    // 对准状态
    std::atomic<bool> aligned_x_{false}, aligned_y_{false};
    double x_adjust_ = 0.0;
    double y_adjust_ = 0.0;
    double arm_height_ = 0.0;
    double tol_multiplier_ = 1.0;
    std::atomic<bool> grasp_ready_{false};

    // 扫描状态
    std::atomic<State> state_{State::IDLE};
    double scan_total_rotation_ = 0.0;
    rclcpp::Time scan_start_time_;
    std::atomic<bool> scan_complete_{false};

    // 抓取状态
    int grasp_retries_ = 0;
    rclcpp::Time grasp_start_time_;
    std::atomic<bool> grasping_{false};

    // 放置状态
    bool extend_before_release_ = false;
    rclcpp::Time release_start_time_;
    std::atomic<bool> release_complete_{false};

    // 夹爪实际位置
    double gripper_actual_position_ = 0.0;
    mutable std::mutex gripper_mutex_;

    // 关节状态订阅回调
    void handDetectionCallback(const std_msgs::msg::Int32MultiArray::ConstSharedPtr& msg);
    void visionCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg);
    void jointStatesCallback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg);

    // 内部工具
    void moveBase(double linear_x, double linear_y, double angular_z);
    void stopBase();
    void moveArmInternal(const std::vector<double>& positions, double duration_sec);
    void operateHandInternal(bool should_grasp);

    // 关节名称
    static constexpr const char* ARM_JOINT_NAMES[] = {
        "arm_lift_joint", "arm_flex_joint", "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"
    };
    static constexpr const char* HAND_JOINT_NAME = "hand_motor_joint";
};

}  // namespace handyman_ros2
