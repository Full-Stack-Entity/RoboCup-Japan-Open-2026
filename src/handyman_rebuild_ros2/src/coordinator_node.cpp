#include "handyman_rebuild_ros2/coordinator_node.hpp"

#include <algorithm>
#include <chrono>
#include <string>

namespace handyman_rebuild_ros2
{

CoordinatorNode::CoordinatorNode(const rclcpp::NodeOptions & options)
: Node("handyman_coordinator", options)
{
    simulate_modules_ = declare_parameter<bool>("simulate_modules", false);
    simulation_step_ms_ = declare_parameter<int>("simulation_step_ms", 100);
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
    simulation_step_ = 0;
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
        state_machine_.setEnvironment(message.detail);
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
          RCLCPP_INFO(get_logger(), "Instruction accepted");
          startSimulation();
        }
        break;
      case CompetitionEvent::kCorrectedInstruction:
        if (state_machine_.acceptInstruction(message.detail, true)) {
          RCLCPP_INFO(get_logger(), "Corrected instruction accepted");
          startSimulation();
        }
        break;
      case CompetitionEvent::kTaskSucceeded:
        if (!state_machine_.moderatorSucceeded()) {
          RCLCPP_WARN(get_logger(), "Ignored Task_succeeded in the current state");
        }
        break;
      case CompetitionEvent::kTaskFailed:
        stopSimulation();
        if (!state_machine_.moderatorFailed()) {
          RCLCPP_WARN(get_logger(), "Ignored Task_failed in the current state");
        }
        break;
      case CompetitionEvent::kMissionComplete:
        stopSimulation();
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
