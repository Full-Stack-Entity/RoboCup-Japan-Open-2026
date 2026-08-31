#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <handyman_msgs/msg/handyman_msg.hpp>

#include "handyman_rebuild_ros2/competition_protocol.hpp"
#include "handyman_rebuild_ros2/task_state_machine.hpp"

namespace handyman_rebuild_ros2
{

class CoordinatorNode final : public rclcpp::Node
{
public:
  CoordinatorNode()
  : Node("handyman_coordinator")
  {
    using handyman_msgs::msg::HandymanMsg;
    publisher_ = create_publisher<HandymanMsg>(std::string(protocol::kToModeratorTopic), 10);
    subscription_ = create_subscription<HandymanMsg>(
      std::string(protocol::kToRobotTopic), 10,
      [this](const HandymanMsg::ConstSharedPtr message) { onMessage(*message); });

    state_machine_.bootCompleted();
    RCLCPP_INFO(get_logger(), "Handyman coordinator scaffold is waiting for Environment and Are_you_ready?");
  }

private:
  void publishEvent(std::string_view event)
  {
    handyman_msgs::msg::HandymanMsg message;
    message.message = std::string(event);
    publisher_->publish(message);
    RCLCPP_INFO(get_logger(), "Sent event: %s", message.message.c_str());
  }

  void onMessage(const handyman_msgs::msg::HandymanMsg & message)
  {
    RCLCPP_INFO(
      get_logger(), "Received event: %s, detail: %s",
      message.message.c_str(), message.detail.c_str());

    if (message.message == protocol::kEnvironment) {
      state_machine_.setEnvironment(message.detail);
      return;
    }
    if (message.message == protocol::kAreYouReady) {
      if (state_machine_.acceptReady()) {
        publishEvent(protocol::kIAmReady);
      } else {
        RCLCPP_WARN(get_logger(), "Ignored Are_you_ready?: environment missing or state not ready");
      }
      return;
    }
    if (message.message == protocol::kInstruction) {
      if (state_machine_.acceptInstruction(message.detail)) {
        RCLCPP_INFO(get_logger(), "Instruction accepted; parser module is the next implementation milestone");
      }
      return;
    }
    if (message.message == protocol::kCorrectedInstruction) {
      if (state_machine_.acceptInstruction(message.detail, true)) {
        RCLCPP_INFO(get_logger(), "Corrected instruction accepted");
      }
      return;
    }
    if (message.message == protocol::kTaskSucceeded) {
      state_machine_.moderatorSucceeded();
      return;
    }
    if (message.message == protocol::kTaskFailed) {
      state_machine_.moderatorFailed();
      return;
    }
    if (message.message == protocol::kMissionComplete) {
      state_machine_.missionCompleted();
      RCLCPP_INFO(get_logger(), "Mission complete received; shutting down safely");
      rclcpp::shutdown();
    }
  }

  TaskStateMachine state_machine_;
  rclcpp::Publisher<handyman_msgs::msg::HandymanMsg>::SharedPtr publisher_;
  rclcpp::Subscription<handyman_msgs::msg::HandymanMsg>::SharedPtr subscription_;
};

}  // namespace handyman_rebuild_ros2

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<handyman_rebuild_ros2::CoordinatorNode>());
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
