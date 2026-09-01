#pragma once

#include <rclcpp/rclcpp.hpp>
#include <handyman_msgs/msg/handyman_msg.hpp>

#include "handyman_rebuild_ros2/competition_protocol.hpp"
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
  void publishEvent(protocol::CompetitionEvent event);
  void handleEvent(const protocol::CompetitionMessage & message);
  void onMessage(const handyman_msgs::msg::HandymanMsg & ros_message);

  TaskStateMachine state_machine_;
  rclcpp::Publisher<handyman_msgs::msg::HandymanMsg>::SharedPtr publisher_;
  rclcpp::Subscription<handyman_msgs::msg::HandymanMsg>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr simulation_timer_;
  bool simulate_modules_{false};
  int simulation_step_ms_{100};
  int simulation_step_{0};
};

}  // namespace handyman_rebuild_ros2
