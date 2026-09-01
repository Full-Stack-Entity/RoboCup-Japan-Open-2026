#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <handyman_msgs/msg/handyman_msg.hpp>

#include "handyman_rebuild_ros2/competition_protocol.hpp"
#include "handyman_rebuild_ros2/environment_config.hpp"
#include "handyman_rebuild_ros2/instruction_parser.hpp"
#include "handyman_rebuild_ros2/task_state_machine.hpp"

namespace handyman_rebuild_ros2
{

class CoordinatorNode final : public rclcpp::Node
{
public:
  explicit CoordinatorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void stopSimulation();
  void startSimulation();
  void runSimulationStep();
  bool parseCurrentInstruction(const std::string & instruction);
  void publishEvent(protocol::CompetitionEvent event);
  void handleEvent(const protocol::CompetitionMessage & message);
  void onMessage(const handyman_msgs::msg::HandymanMsg & ros_message);

  TaskStateMachine state_machine_;
  EnvironmentCatalog environment_catalog_;
  std::unique_ptr<RuleBasedInstructionParser> instruction_parser_;
  rclcpp::Publisher<handyman_msgs::msg::HandymanMsg>::SharedPtr publisher_;
  rclcpp::Subscription<handyman_msgs::msg::HandymanMsg>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr simulation_timer_;
  bool simulate_modules_{false};
  int simulation_step_ms_{100};
  int simulation_step_{0};
};

}  // namespace handyman_rebuild_ros2
