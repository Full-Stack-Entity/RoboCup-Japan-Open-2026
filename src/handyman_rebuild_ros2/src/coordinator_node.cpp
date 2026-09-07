#include "handyman_rebuild_ros2/coordinator_node.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

namespace handyman_rebuild_ros2
{

CoordinatorNode::CoordinatorNode(const rclcpp::NodeOptions & options)
: Node("handyman_coordinator", options)
{
    simulate_modules_ = declare_parameter<bool>("simulate_modules", false);
    simulation_step_ms_ = declare_parameter<int>("simulation_step_ms", 100);

    const std::string config_directory =
      ament_index_cpp::get_package_share_directory("handyman_rebuild_ros2") + "/config";
    std::string config_error;
    NameAliases aliases;
    if (aliases.loadFromFile(config_directory + "/name_aliases.yaml", config_error)) {
      instruction_parser_ = std::make_unique<RuleBasedInstructionParser>(std::move(aliases));
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to load name aliases: %s", config_error.c_str());
    }
    if (!environment_catalog_.loadFromFile(
        config_directory + "/environments.yaml", config_error))
    {
      RCLCPP_ERROR(get_logger(), "Failed to load environments: %s", config_error.c_str());
    }
    NavigationSettings navigation_settings;
    try {
      const YAML::Node recovery = YAML::LoadFile(config_directory + "/recovery.yaml");
      navigation_settings.maximum_attempts =
        recovery["retries"]["navigation"].as<std::size_t>();
      navigation_settings.goal_timeout_sec =
        recovery["timeouts_sec"]["navigation"].as<double>();
    } catch (const YAML::Exception & exception) {
      RCLCPP_WARN(
        get_logger(), "Could not load navigation recovery settings; using defaults: %s",
        exception.what());
    }
    const int maximum_attempts = declare_parameter<int>(
      "navigation.maximum_attempts", static_cast<int>(navigation_settings.maximum_attempts));
    navigation_settings.maximum_attempts = static_cast<std::size_t>(std::max(1, maximum_attempts));
    navigation_settings.goal_timeout_sec = declare_parameter<double>(
      "navigation.goal_timeout_sec", navigation_settings.goal_timeout_sec);
    navigation_settings.goal_timeout_sec = std::max(0.1, navigation_settings.goal_timeout_sec);
    navigation_settings.server_wait_timeout_sec = declare_parameter<double>(
      "navigation.server_wait_timeout_sec", navigation_settings.server_wait_timeout_sec);
    navigation_settings.server_wait_timeout_sec = std::max(
      0.1, navigation_settings.server_wait_timeout_sec);
    navigation_settings.room_boundary_tolerance_m = declare_parameter<double>(
      "navigation.room_boundary_tolerance_m", navigation_settings.room_boundary_tolerance_m);
    navigation_settings.destination_tolerance_m = declare_parameter<double>(
      "navigation.destination_tolerance_m", navigation_settings.destination_tolerance_m);
    navigation_settings.map_frame = declare_parameter<std::string>(
      "navigation.map_frame", navigation_settings.map_frame);
    navigation_settings.robot_frame = declare_parameter<std::string>(
      "navigation.robot_frame", navigation_settings.robot_frame);
    navigation_settings.action_name = declare_parameter<std::string>(
      "navigation.action_name", navigation_settings.action_name);
    navigation_executor_ = std::make_unique<NavigationExecutor>(
      this, &environment_catalog_, navigation_settings);
    using handyman_msgs::msg::HandymanMsg;
    publisher_ = create_publisher<HandymanMsg>(std::string(protocol::kToModeratorTopic), 10);
    subscription_ = create_subscription<HandymanMsg>(
      std::string(protocol::kToRobotTopic), 10,
      [this](const HandymanMsg::ConstSharedPtr message) { onMessage(*message); });

    state_machine_.bootCompleted();
    RCLCPP_INFO(get_logger(), "Handyman coordinator scaffold is waiting for Environment and Are_you_ready?");
    if (simulate_modules_) {
      RCLCPP_WARN(get_logger(), "Module simulation is enabled; no robot commands will be sent");
    }
  }

void CoordinatorNode::stopSimulation()
{
    if (simulation_timer_) {
      simulation_timer_->cancel();
      simulation_timer_.reset();
    }
  }

void CoordinatorNode::startSimulation()
{
    if (!simulate_modules_) {
      return;
    }
    stopSimulation();
    simulation_step_ = 1;
    const auto period = std::chrono::milliseconds(std::max(1, simulation_step_ms_));
    simulation_timer_ = create_wall_timer(period, [this]() { runSimulationStep(); });
  }

void CoordinatorNode::runSimulationStep()
{
    bool transitioned = false;
    switch (simulation_step_++) {
      case 0:
        transitioned = state_machine_.parsingSucceeded();
        break;
      case 1:
        transitioned = state_machine_.roomNavigationSucceeded();
        break;
      case 2:
        transitioned = state_machine_.roomVerified();
        if (transitioned) {
          publishEvent(protocol::CompetitionEvent::kRoomReached);
        }
        break;
      case 3:
        transitioned = state_machine_.objectLocated();
        break;
      case 4:
        transitioned = state_machine_.approachSucceeded();
        break;
      case 5:
        transitioned = state_machine_.graspSucceeded();
        break;
      case 6:
        transitioned = state_machine_.graspVerified();
        if (transitioned) {
          publishEvent(protocol::CompetitionEvent::kObjectGrasped);
        }
        break;
      case 7:
        transitioned = state_machine_.destinationReached();
        break;
      case 8:
        transitioned = state_machine_.placementSucceeded();
        break;
      case 9:
        transitioned = state_machine_.placementVerified();
        if (transitioned) {
          publishEvent(protocol::CompetitionEvent::kTaskFinished);
        }
        stopSimulation();
        break;
      default:
        stopSimulation();
        return;
    }
    if (!transitioned) {
      RCLCPP_ERROR(get_logger(), "Simulated module result was invalid at step %d", simulation_step_ - 1);
      stopSimulation();
    }
  }

bool CoordinatorNode::parseCurrentInstruction(const std::string & instruction)
{
    if (!instruction_parser_) {
      state_machine_.parsingFailed();
      RCLCPP_ERROR(get_logger(), "Instruction parser is unavailable");
      return false;
    }
    HandymanTask parsed_task = state_machine_.task();
    const auto result = instruction_parser_->parse(instruction, parsed_task);
    if (!result.success) {
      state_machine_.parsingFailed();
      RCLCPP_WARN(get_logger(), "Instruction rejected: %s", result.reason.c_str());
      return false;
    }
    std::string environment_error;
    if (!environment_catalog_.resolveTask(parsed_task, environment_error)) {
      state_machine_.parsingFailed();
      RCLCPP_WARN(get_logger(), "Instruction does not match environment: %s", environment_error.c_str());
      return false;
    }
    if (!state_machine_.parsingSucceeded(parsed_task)) {
      RCLCPP_ERROR(get_logger(), "Parsed instruction could not advance the state machine");
      return false;
    }
    RCLCPP_INFO(
      get_logger(),
      "Parsed task: pickup_room=%s object=%s destination_room=%s destination=%s avatar=%s",
      parsed_task.pickup_room.c_str(), parsed_task.target_object.c_str(),
      parsed_task.destination_room.c_str(), parsed_task.destination.c_str(),
      parsed_task.destination_is_avatar ? "true" : "false");
    return true;
  }

void CoordinatorNode::startRoomNavigation()
{
    const HandymanTask task = state_machine_.task();
    const bool started = navigation_executor_ && navigation_executor_->navigateToRoom(
      task.environment, task.pickup_room,
      [this](const NavigationOutcome & outcome) {handleNavigationOutcome(outcome);});
    if (!started) {
      RCLCPP_ERROR(get_logger(), "Could not start room navigation");
      if (state_machine_.giveUp()) {
        publishEvent(protocol::CompetitionEvent::kGiveUp);
      }
    }
  }

void CoordinatorNode::handleNavigationOutcome(const NavigationOutcome & outcome)
{
    if (!outcome.success) {
      RCLCPP_ERROR(
        get_logger(), "Navigation failed after %zu attempt(s): %s",
        outcome.attempts, outcome.reason.c_str());
      if (state_machine_.giveUp()) {
        publishEvent(protocol::CompetitionEvent::kGiveUp);
      }
      return;
    }
    if (outcome.target == NavigationTarget::kRoom &&
      state_machine_.roomNavigationSucceeded() && state_machine_.roomVerified())
    {
      RCLCPP_INFO(
        get_logger(), "Room navigation verified at (%.3f, %.3f) after %zu attempt(s)",
        outcome.robot_pose.x, outcome.robot_pose.y, outcome.attempts);
      publishEvent(protocol::CompetitionEvent::kRoomReached);
      return;
    }
    if (outcome.target == NavigationTarget::kDestination && state_machine_.destinationReached()) {
      RCLCPP_INFO(
        get_logger(), "Destination navigation verified at (%.3f, %.3f)",
        outcome.robot_pose.x, outcome.robot_pose.y);
      return;
    }
    RCLCPP_ERROR(get_logger(), "Navigation result did not match the current task state");
  }

void CoordinatorNode::publishEvent(protocol::CompetitionEvent event)
{
    const auto built = protocol::makeOutgoingMessage(event);
    if (!built.valid) {
      RCLCPP_ERROR(get_logger(), "Refused outgoing event: %s", built.reason.c_str());
      return;
    }
    publisher_->publish(built.message);
    RCLCPP_INFO(get_logger(), "Sent event: %s", built.message.message.c_str());
  }

void CoordinatorNode::handleEvent(const protocol::CompetitionMessage & message)
{
    using protocol::CompetitionEvent;
    switch (message.event) {
      case CompetitionEvent::kEnvironment:
        if (const auto * environment = environment_catalog_.find(message.detail)) {
          state_machine_.setEnvironment(environment->name);
          RCLCPP_INFO(get_logger(), "Loaded environment configuration: %s", environment->name.c_str());
        } else {
          RCLCPP_WARN(get_logger(), "Ignored unknown environment: %s", message.detail.c_str());
        }
        break;
      case CompetitionEvent::kAreYouReady:
        if (state_machine_.acceptReady()) {
          publishEvent(CompetitionEvent::kIAmReady);
        } else {
          RCLCPP_WARN(get_logger(), "Ignored Are_you_ready?: environment missing or state not ready");
        }
        break;
      case CompetitionEvent::kInstruction:
        if (state_machine_.acceptInstruction(message.detail)) {
          if (parseCurrentInstruction(message.detail)) {
            if (simulate_modules_) {
              startSimulation();
            } else {
              startRoomNavigation();
            }
          }
        }
        break;
      case CompetitionEvent::kCorrectedInstruction:
        if (state_machine_.acceptInstruction(message.detail, true)) {
          if (parseCurrentInstruction(message.detail)) {
            if (simulate_modules_) {
              startSimulation();
            } else {
              startRoomNavigation();
            }
          }
        }
        break;
      case CompetitionEvent::kTaskSucceeded:
        if (!state_machine_.moderatorSucceeded()) {
          RCLCPP_WARN(get_logger(), "Ignored Task_succeeded in the current state");
        }
        break;
      case CompetitionEvent::kTaskFailed:
        stopSimulation();
        if (navigation_executor_) {
          navigation_executor_->cancel();
        }
        if (!state_machine_.moderatorFailed()) {
          RCLCPP_WARN(get_logger(), "Ignored Task_failed in the current state");
        }
        break;
      case CompetitionEvent::kMissionComplete:
        stopSimulation();
        if (navigation_executor_) {
          navigation_executor_->cancel();
        }
        state_machine_.missionCompleted();
        RCLCPP_INFO(get_logger(), "Mission complete received; shutting down safely");
        rclcpp::shutdown();
        break;
      default:
        RCLCPP_ERROR(
          get_logger(), "Incoming event reached an impossible dispatch path: %s",
          std::string(protocol::eventName(message.event)).c_str());
        break;
    }
  }

void CoordinatorNode::onMessage(const handyman_msgs::msg::HandymanMsg & ros_message)
{
    const auto parsed = protocol::parseIncomingMessage(ros_message);
    if (!parsed.valid) {
      RCLCPP_WARN(get_logger(), "Rejected competition message: %s", parsed.reason.c_str());
      return;
    }
    RCLCPP_INFO(
      get_logger(), "Received event: %s, detail: %s",
      std::string(protocol::eventName(parsed.message.event)).c_str(),
      parsed.message.detail.c_str());
    handleEvent(parsed.message);
  }

}  // namespace handyman_rebuild_ros2
