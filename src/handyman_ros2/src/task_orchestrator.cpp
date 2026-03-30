/**
 * @file task_orchestrator.cpp
 * @brief 任务编排层实现
 *
 * 模块4：任务编排层
 * 持有并协调 ProtocolHandler、MapNavController、VisionManipulationController 三个模块。
 * 实现状态机主循环，串联各模块调用。
 */

#include "handyman_ros2/task_orchestrator.hpp"
#include "handyman_ros2/protocol_handler.hpp"
#include "handyman_ros2/map_nav_controller.hpp"
#include "handyman_ros2/vision_manipulation_controller.hpp"

namespace handyman_ros2 {

// ---------------------------------------------------------------------------
// 构造函数
// ---------------------------------------------------------------------------

TaskOrchestrator::TaskOrchestrator(rclcpp::Node::SharedPtr node)
  : node_(node)
{
}

// ---------------------------------------------------------------------------
// ProtocolHandler::Listener 实现
// ---------------------------------------------------------------------------

void TaskOrchestrator::onEnvironment(const std::string& unity_env)
{
    RCLCPP_INFO(node_->get_logger(), "TaskOrchestrator: Received Environment '%s'", unity_env.c_str());
    task_.environment = ProtocolHandler::InstructionParser::mapUnityEnvironmentName(unity_env);
    task_.mapped_env = task_.environment;
}

void TaskOrchestrator::onAreYouReady()
{
    RCLCPP_INFO(node_->get_logger(), "TaskOrchestrator: Received Are_you_ready?");

    if (step_ == Step::Ready) {
        is_started_ = true;
    } else if (step_ == Step::GoToRoom1 || step_ == Step::MoveToInFrontOfTarget ||
               step_ == Step::Grasp || step_ == Step::GoToRoom2 || step_ == Step::Release) {
        // 关键状态被中断，跳转到 GiveUp
        RCLCPP_WARN(node_->get_logger(),
                    "TaskOrchestrator: Critical state interrupted by Are_you_ready?. Transitioning to GiveUp.");
        is_failed_ = true;
    } else if (step_ == Step::WaitForInstruction) {
        // 在等待指令状态，忽略
        RCLCPP_DEBUG(node_->get_logger(),
                     "TaskOrchestrator: Ignoring Are_you_ready? in WaitForInstruction");
    }
}

void TaskOrchestrator::onInstruction(const std::string& detail)
{
    RCLCPP_INFO(node_->get_logger(), "TaskOrchestrator: Received Instruction '%s'", detail.c_str());

    if (step_ == Step::WaitForInstruction) {
        task_.instruction = detail;

        // 使用 InstructionParser 解析指令
        std::vector<std::string> rooms_kw = {"living", "bedroom", "lobby", "kitchen"};
        std::vector<std::string> objects_kw = {
            "apple", "toy_penguin", "rabbit_doll", "bear_doll", "dog_doll",
            "canned_juice", "sugar", "soysauce", "sauce", "ketchup",
            "tumbler", "white_cup", "pink_cup", "empty_ketchup", "filled_ketchup",
            "ground_pepper", "salt", "empty_plastic_bottle", "filled_plastic_bottle",
            "cubic_clock", "toy_car", "toy_duck", "nursing_bottle",
            "cigarette", "hourglass", "camera", "rubik's_cube",
            "spray_bottle", "matryoshka", "game_controller", "piggy_bank"
        };
        std::vector<std::string> dests_kw = {
            "white_side_table", "corner_sofa", "round_low_table", "square_low_table",
            "wooden_shelf", "armchair", "dining_table", "wooden_side_table",
            "wooden_bed", "iron_bed", "wagon", "trash_box_for_recycle",
            "trash_box_for_burnable", "trash_box_for_bottle_can",
            "cardboard_box", "Avatar"
        };

        ProtocolHandler::InstructionParser::parse(
            detail, rooms_kw, objects_kw, dests_kw, task_);

        // 发布目标物体给视觉节点
        if (!task_.objects.empty()) {
            publishDetectionTarget();
        }

        // 初始化手臂和夹爪
        std::vector<double> positions = { 0.1, 0.0, 0.0, -1.57, 0.0 };
        rclcpp::Duration duration = rclcpp::Duration::from_seconds(1.0);
        vision_arm_->moveArm(positions, 1.0);
        vision_arm_->openGripper();

        // 重置导航状态
        nav_->resetTaskState();
        vision_arm_->reset();
        give_up_sent_ = false;
        is_failed_ = false;

        // 进入 GoToRoom1
        step_ = Step::GoToRoom1;
    }
}

void TaskOrchestrator::onCorrectedInstruction(const std::string& detail)
{
    RCLCPP_INFO(node_->get_logger(), "TaskOrchestrator: Received Corrected_instruction '%s'", detail.c_str());

    if (step_ == Step::MoveToInFrontOfTarget) {
        // Avatar 纠正了指令，重新解析并搜索新物体
        task_.instruction = detail;
        std::vector<std::string> rooms_kw = {"living", "bedroom", "lobby", "kitchen"};
        std::vector<std::string> objects_kw = {
            "apple", "toy_penguin", "rabbit_doll", "bear_doll", "dog_doll",
            "canned_juice", "sugar", "soysauce", "sauce", "ketchup",
            "tumbler", "white_cup", "pink_cup", "empty_ketchup", "filled_ketchup",
            "ground_pepper", "salt", "empty_plastic_bottle", "filled_plastic_bottle",
            "cubic_clock", "toy_car", "toy_duck", "nursing_bottle",
            "cigarette", "hourglass", "camera", "rubik's_cube",
            "spray_bottle", "matryoshka", "game_controller", "piggy_bank"
        };
        std::vector<std::string> dests_kw = {
            "white_side_table", "corner_sofa", "round_low_table", "square_low_table",
            "wooden_shelf", "armchair", "dining_table", "wooden_side_table",
            "wooden_bed", "iron_bed", "wagon", "trash_box_for_recycle",
            "trash_box_for_burnable", "trash_box_for_bottle_can",
            "cardboard_box", "Avatar"
        };
        ProtocolHandler::InstructionParser::parse(detail, rooms_kw, objects_kw, dests_kw, task_);

        // 重置扫描状态，重新搜索
        vision_arm_->startRoomScan();
    }
}

void TaskOrchestrator::onTaskSucceeded()
{
    RCLCPP_INFO(node_->get_logger(), "TaskOrchestrator: Received Task_succeeded");
    if (step_ == Step::TaskFinished) {
        is_finished_ = true;
    }
}

void TaskOrchestrator::onTaskFailed(const std::string& /*detail*/)
{
    RCLCPP_WARN(node_->get_logger(), "TaskOrchestrator: Received Task_failed");
    is_failed_ = true;
}

void TaskOrchestrator::onMissionComplete()
{
    RCLCPP_INFO(node_->get_logger(), "TaskOrchestrator: Received Mission_complete");
    RCLCPP_INFO(node_->get_logger(), "All tasks complete. Shutting down.");
    rclcpp::shutdown();
    std::exit(EXIT_SUCCESS);
}

// ---------------------------------------------------------------------------
// 辅助方法
// ---------------------------------------------------------------------------

void TaskOrchestrator::stopBase()
{
    moveBase(0.0, 0.0, 0.0);
}

void TaskOrchestrator::moveBase(double linear_x, double linear_y, double angular_z)
{
    if (!pub_base_twist_) return;
    geometry_msgs::msg::Twist twist;
    twist.linear.x = linear_x;
    twist.linear.y = linear_y;
    twist.angular.z = angular_z;
    pub_base_twist_->publish(twist);
}

void TaskOrchestrator::resetTaskContext()
{
    task_.reset();
    give_up_sent_ = false;
    is_failed_ = false;
    is_started_ = false;
    is_finished_ = false;
    nav_->resetTaskState();
    vision_arm_->reset();
    test_mode_target_valid_ = false;
}

void TaskOrchestrator::publishDetectionTarget()
{
    if (!task_.objects.empty() && pub_detection_target_) {
        std_msgs::msg::String msg;
        msg.data = task_.objects[0];
        pub_detection_target_->publish(msg);
        RCLCPP_INFO(node_->get_logger(), "TaskOrchestrator: Published detection target: '%s'", msg.data.c_str());
    }
}

// ---------------------------------------------------------------------------
// 状态处理
// ---------------------------------------------------------------------------

void TaskOrchestrator::handleInitialize()
{
    resetTaskContext();
    step_ = Step::Ready;
    RCLCPP_INFO(node_->get_logger(), "TaskOrchestrator: Step -> Initialize (done), now Ready");
}

void TaskOrchestrator::handleReady()
{
    if (!is_started_) return;

    if (task_.environment.empty()) {
        RCLCPP_WARN(node_->get_logger(), "TaskOrchestrator: Environment not set. Waiting...");
        return;
    }

    RCLCPP_INFO(node_->get_logger(), "TaskOrchestrator: Loading map for environment '%s'", task_.environment.c_str());

    // 加载环境地图
    if (!nav_->loadEnvironment(task_.environment)) {
        RCLCPP_ERROR(node_->get_logger(), "TaskOrchestrator: Failed to load map, returning to Initialize");
        step_ = Step::Initialize;
        return;
    }

    // 等待 Nav2 就绪
    if (!nav_->waitForNav2Ready(12)) {
        RCLCPP_ERROR(node_->get_logger(), "TaskOrchestrator: Nav2 not ready, returning to Initialize");
        step_ = Step::Initialize;
        return;
    }

    RCLCPP_INFO(node_->get_logger(), "TaskOrchestrator: System ready, sending I_am_ready");
    protocol_->sendIAmReady();

    step_ = Step::WaitForInstruction;
}

void TaskOrchestrator::handleWaitForInstruction()
{
    // 指令通过 ProtocolHandler::Listener.onInstruction 回调处理
    // 此处仅记录日志
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
        "TaskOrchestrator: Waiting for instruction... context: %s", task_.toString().c_str());
}

void TaskOrchestrator::handleGoToRoom1()
{
    const std::string& target_room = task_.rooms.empty() ? "living" : task_.rooms[0];

    // ---- 测试模式：动态房间距离检查提前进入 ----
    if (TEST_MODE_ENABLED) {
        auto [in_room, dist] = nav_->getRobotRoomDistance(target_room);
        if (in_room) {
            RCLCPP_INFO(node_->get_logger(),
                        "TaskOrchestrator: [TEST] Robot already in room '%s', entering scan", target_room.c_str());
            stopBase();
            nav_->cancelNavGoal();
            nav_->resetTaskState();
            protocol_->sendRoomReached();
            vision_arm_->startRoomScan();
            step_ = Step::MoveToInFrontOfTarget;
            return;
        }
    }
    // -------------------------------------------

    // 首次进入：初始化 patrol
    // 注意：patrol 由 nav_->startRoomPatrol / nav_->updateRoomPatrol 管理
    if (!nav_->isGoalActive() && !nav_->isRoomReached()) {
        nav_->startRoomPatrol(target_room);
    }

    // 更新 patrol 状态
    bool patrol_done = nav_->updateRoomPatrol();

    // 检查是否已到达房间（任意时刻）
    if (nav_->isRoomReached()) {
        RCLCPP_INFO(node_->get_logger(),
                    "TaskOrchestrator: Room '%s' reached! Sending Room_reached", target_room.c_str());
        stopBase();
        nav_->cancelNavGoal();
        nav_->resetTaskState();
        protocol_->sendRoomReached();
        vision_arm_->startRoomScan();
        step_ = Step::MoveToInFrontOfTarget;
        return;
    }

    // 所有 waypoint 用完仍未到达房间：进入扫描（兜底）
    if (patrol_done && !nav_->isRoomReached()) {
        RCLCPP_WARN(node_->get_logger(),
                    "TaskOrchestrator: All patrol waypoints done but room not confirmed. Entering scan anyway.");
        protocol_->sendRoomReached();
        vision_arm_->startRoomScan();
        step_ = Step::MoveToInFrontOfTarget;
    }
}

void TaskOrchestrator::handleMoveToInFrontOfTarget()
{
    // ---- 测试模式：5 秒扫描 → 跳转到 GoToRoom2 导航到放置位置 ----
    if (TEST_MODE_ENABLED) {
        if (test_mode_scan_start_time_.seconds() == 0) {
            test_mode_scan_start_time_ = node_->now();
            RCLCPP_WARN(node_->get_logger(),
                "TaskOrchestrator: [TEST MODE] Scan started, will navigate to destination after %.1f seconds",
                TEST_SCAN_DURATION_SEC);
        }
        double scan_elapsed = (node_->now() - test_mode_scan_start_time_).seconds();
        if (scan_elapsed >= TEST_SCAN_DURATION_SEC) {
            RCLCPP_WARN(node_->get_logger(),
                "TaskOrchestrator: [TEST MODE] Scan timeout (%.1f s), jumping to GoToRoom2 (navigate to destination)",
                scan_elapsed);
            vision_arm_->stopScan();
            stopBase();
            nav_->cancelNavGoal();
            nav_->resetTaskState();
            // Reset test mode state for GoToRoom2
            test_mode_target_valid_ = false;
            test_mode_scan_start_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
            step_ = Step::GoToRoom2;
            return;
        }
        // Slow rotation for visual feedback during scan
        moveBase(0.0, 0.0, 0.3);
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
            "TaskOrchestrator: [TEST MODE] Scanning... (%.1f / %.1f s)",
            scan_elapsed, TEST_SCAN_DURATION_SEC);
        return;
    }
    // ------------------------------------------------------

    // 取消残留的导航目标
    if (nav_->isGoalActive()) {
        RCLCPP_WARN(node_->get_logger(), "TaskOrchestrator: Stale nav in MoveToInFrontOfTarget, cancelling");
        nav_->cancelNavGoal();
    }

    // 更新视觉模块
    vision_arm_->publishBaseCommand(pub_base_twist_);

    // 360° 扫描完成仍未找到物体
    if (vision_arm_->isScanComplete()) {
        RCLCPP_WARN(node_->get_logger(),
                     "TaskOrchestrator: Room scan complete (360 deg), object NOT found. Sending Does_not_exist.");
        vision_arm_->stopScan();
        stopBase();
        protocol_->sendDoesNotExist(task_.objects.empty() ? "" : task_.objects[0]);
        step_ = Step::Initialize;  // 重新开始下一张地图
        return;
    }

    // 检测到物体且对准完成 -> 进入抓取
    if (vision_arm_->objectDetected()) {
        auto aln = vision_arm_->getAlignment();
        if (aln.grasp_ready && aln.aligned) {
            RCLCPP_INFO(node_->get_logger(),
                        "TaskOrchestrator: Alignment complete and grasp ready! Entering Grasp");
            vision_arm_->stopScan();
            stopBase();
            step_ = Step::Grasp;
            return;
        }
    }
}

void TaskOrchestrator::handleGrasp()
{
    // 闭合夹爪
    vision_arm_->closeGripper();

    // 等待 8 秒后验证抓取
    if (waiting_start_time_.seconds() == 0) {
        waiting_start_time_ = node_->now();
    }

    if ((node_->now() - waiting_start_time_).seconds() < 8.0) {
        return;  // 等待中
    }

    // 验证抓取
    auto result = vision_arm_->verifyGrasp();

    if (result == VisionManipulationController::GraspResult::SUCCESS) {
        RCLCPP_INFO(node_->get_logger(),
                    "TaskOrchestrator: Grasp SUCCESS, sending Object_grasped");
        protocol_->sendObjectGrasped();
        step_ = Step::GoToRoom2;
    } else if (result == VisionManipulationController::GraspResult::SLIPPED) {
        if (vision_arm_->getGripRetries() < VisionManipulationController::MAX_GRASP_RETRIES) {
            RCLCPP_WARN(node_->get_logger(),
                        "TaskOrchestrator: Grasp slipped, retrying (attempt %d)",
                        vision_arm_->getGripRetries() + 1);
            vision_arm_->openGripper();
            waiting_start_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
            // 重试计数在 vision_arm_->verifyGrasp() 中已更新
        } else {
            RCLCPP_ERROR(node_->get_logger(),
                          "TaskOrchestrator: Grasp failed after %d retries, giving up",
                          vision_arm_->getGripRetries());
            protocol_->sendGiveUp();
            give_up_sent_ = true;
            give_up_send_time_ = node_->now();
            step_ = Step::GiveUp;
        }
    } else {
        RCLCPP_ERROR(node_->get_logger(), "TaskOrchestrator: Grasp MISSED");
        protocol_->sendGiveUp();
        give_up_sent_ = true;
        give_up_send_time_ = node_->now();
        step_ = Step::GiveUp;
    }
}

void TaskOrchestrator::handleGoToRoom2()
{
    // ---- 测试模式：实时检测是否在目标 1.2m 范围内 ----
    if (TEST_MODE_ENABLED) {
        if (!test_mode_target_valid_) {
            // 获取目标位置
            std::string dest_name = task_.destinations.empty() ? "white_side_table" : task_.destinations.back();
            std::string dest_room = task_.rooms.size() > 1 ? task_.rooms[1] : task_.rooms[0];
            dest_pose_ = nav_->destLocation(dest_name, dest_room);
            test_target_x_ = dest_pose_.pose.position.x;
            test_target_y_ = dest_pose_.pose.position.y;
            nav_->setDestinationForDistanceCheck(test_target_x_, test_target_y_);
            test_mode_target_valid_ = true;
            RCLCPP_INFO(node_->get_logger(),
                "TaskOrchestrator: [TEST] GoToRoom2 target: (%.2f, %.2f)",
                test_target_x_, test_target_y_);
        }

        // 计算实时距离
        auto tf_buffer = nav_->getTfBuffer();
        if (tf_buffer) {
            try {
                geometry_msgs::msg::TransformStamped tf_stamped =
                    tf_buffer->lookupTransform("map", "base_footprint", tf2::TimePointZero);
                double dx = tf_stamped.transform.translation.x - test_target_x_;
                double dy = tf_stamped.transform.translation.y - test_target_y_;
                double dist = std::sqrt(dx * dx + dy * dy);

                RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                    "TaskOrchestrator: [TEST] Distance to target: %.2f m", dist);

                if (dist < TEST_DEST_THRESHOLD_M) {
                    RCLCPP_WARN(node_->get_logger(),
                        "TaskOrchestrator: [TEST] Within %.2fm of destination, triggering Give_up", TEST_DEST_THRESHOLD_M);
                    stopBase();
                    nav_->cancelNavGoal();
                    // Send Give_up and jump to GiveUp (correct flow)
                    protocol_->sendGiveUp();
                    give_up_sent_ = true;
                    give_up_send_time_ = node_->now();
                    step_ = Step::GiveUp;
                    return;
                }
            } catch (const tf2::TransformException&) {
                // TF 获取失败，忽略
            }
        }
    }
    // ----------------------------------------------------

    // 首次进入：发送导航目标
    if (!nav_->isGoalActive() && !nav_->isRoomReached()) {
        std::string dest_name = task_.destinations.empty() ? "white_side_table" : task_.destinations.back();
        std::string dest_room = task_.rooms.size() > 1 ? task_.rooms[1] : task_.rooms[0];

        RCLCPP_INFO(node_->get_logger(),
                    "TaskOrchestrator: GoToRoom2: dest='%s', room='%s'",
                    dest_name.c_str(), dest_room.c_str());

        // 发送搬运姿态
        vision_arm_->setCarryPose();

        // 发送导航目标
        nav_->navigateToDestination(dest_name, dest_room);
    }

    // 导航到达
    auto nav_result = nav_->getDestinationNavResult();
    if (nav_result == MapNavController::NavResult::SUCCESS) {
        RCLCPP_INFO(node_->get_logger(), "TaskOrchestrator: Destination reached! Entering Release");
        stopBase();
        nav_->resetTaskState();
        step_ = Step::Release;
    } else if (nav_result == MapNavController::NavResult::FAILED) {
        RCLCPP_WARN(node_->get_logger(), "TaskOrchestrator: Destination nav failed, retrying...");
        nav_->resetTaskState();
    } else if (nav_result == MapNavController::NavResult::TIMEOUT) {
        RCLCPP_WARN(node_->get_logger(), "TaskOrchestrator: Destination nav timed out, retrying...");
        nav_->resetTaskState();
    }
}

void TaskOrchestrator::handleRelease()
{
    bool done = vision_arm_->startReleaseSequence();

    if (done) {
        RCLCPP_INFO(node_->get_logger(),
                    "TaskOrchestrator: Release complete, sending Task_finished");
        protocol_->sendTaskFinished();
        step_ = Step::TaskFinished;
    }
}

void TaskOrchestrator::handleTaskFinished()
{
    if (is_finished_) {
        RCLCPP_INFO(node_->get_logger(), "TaskOrchestrator: Task succeeded! Returning to Initialize");
        step_ = Step::Initialize;
    }
}

void TaskOrchestrator::handleGiveUp()
{
    // 第一次进入：发送 Give_up 消息
    if (!give_up_sent_) {
        RCLCPP_WARN(node_->get_logger(),
                    "TaskOrchestrator: GiveUp: Sending MSG_GIVE_UP to Avatar");
        protocol_->sendGiveUp();
        give_up_sent_ = true;
        give_up_send_time_ = node_->now();
    }

    // 等待 Avatar 回复 Task_failed
    if (is_failed_) {
        RCLCPP_INFO(node_->get_logger(),
                    "TaskOrchestrator: GiveUp: Avatar replied Task_failed. Resetting to Initialize");
        give_up_sent_ = false;
        is_failed_ = false;
        step_ = Step::Initialize;
        return;
    }

    // 超时保护：如果 Avatar 超过 30 秒没有回复，强制重置
    double elapsed = (node_->now() - give_up_send_time_).seconds();
    if (elapsed > GIVE_UP_REPLY_TIMEOUT_SEC) {
        RCLCPP_WARN(node_->get_logger(),
                    "TaskOrchestrator: GiveUp: Timeout (%.1f s) waiting for Avatar, forcing reset",
                    elapsed);
        give_up_sent_ = false;
        step_ = Step::Initialize;
        return;
    }

    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
        "TaskOrchestrator: GiveUp: Waiting for Avatar's Task_failed (elapsed=%.1f/%.1f s)...",
        elapsed, GIVE_UP_REPLY_TIMEOUT_SEC);
}

// ---------------------------------------------------------------------------
// 主循环
// ---------------------------------------------------------------------------

int TaskOrchestrator::run(int argc, char** argv)
{
    (void)argc; (void)argv;

    // 创建并初始化 ProtocolHandler
    protocol_ = std::make_unique<ProtocolHandler>(node_);
    protocol_->setListener(this);
    protocol_->init();

    // 创建并初始化 MapNavController
    nav_ = std::make_unique<MapNavController>(node_);

    // 创建并初始化 VisionManipulationController
    vision_arm_ = std::make_unique<VisionManipulationController>(node_);

    // 创建发布者
    pub_base_twist_ = node_->create_publisher<geometry_msgs::msg::Twist>(
        "/hsrb/command_velocity", 10);
    pub_arm_trajectory_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/hsrb/arm_trajectory_controller/command", 10);
    pub_gripper_trajectory_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/hsrb/gripper_controller/command", 10);
    pub_detection_target_ = node_->create_publisher<std_msgs::msg::String>(
        "/detection_target", 10);

    // 将 publisher 注入到 VisionManipulationController
    vision_arm_->setArmPublisher(pub_arm_trajectory_);
    vision_arm_->setGripperPublisher(pub_gripper_trajectory_);

    // 初始化时间戳
    waiting_start_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
    test_mode_scan_start_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());

    RCLCPP_INFO(node_->get_logger(), "TaskOrchestrator: Starting main loop");

    rclcpp::Rate loop_rate(10.0);  // 10 Hz

    while (rclcpp::ok()) {
        // ---- is_failed_ 必须在所有状态之前处理 ----
        if (is_failed_ && step_ != Step::GiveUp && step_ != Step::Initialize) {
            RCLCPP_WARN(node_->get_logger(),
                        "TaskOrchestrator: is_failed_ triggered, entering GiveUp state");
            stopBase();
            nav_->cancelNavGoal();
            step_ = Step::GiveUp;
        }
        // -----------------------------------------

        // 心跳日志（每 3 秒）
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
            "TaskOrchestrator: [heartbeat] step=%d, is_failed_=%d, is_started_=%d, env='%s'",
            static_cast<int>(step_.load()), is_failed_, is_started_, task_.environment.c_str());

        // 状态分发
        switch (step_) {
            case Step::Initialize:   handleInitialize();            break;
            case Step::Ready:          handleReady();                  break;
            case Step::WaitForInstruction: handleWaitForInstruction(); break;
            case Step::GoToRoom1:       handleGoToRoom1();             break;
            case Step::MoveToInFrontOfTarget: handleMoveToInFrontOfTarget(); break;
            case Step::Grasp:          handleGrasp();                  break;
            case Step::GoToRoom2:      handleGoToRoom2();             break;
            case Step::Release:        handleRelease();              break;
            case Step::ComeBack:       /* 空状态 */                   break;
            case Step::TaskFinished:   handleTaskFinished();          break;
            case Step::GiveUp:         handleGiveUp();               break;
        }

        rclcpp::spin_some(node_);
        loop_rate.sleep();
    }

    return EXIT_SUCCESS;
}

}  // namespace handyman_ros2
