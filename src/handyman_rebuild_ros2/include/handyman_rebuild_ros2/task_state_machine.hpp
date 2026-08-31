#pragma once

#include <string>

#include "handyman_rebuild_ros2/task.hpp"

namespace handyman_rebuild_ros2
{

class TaskStateMachine
{
public:
  TaskState state() const noexcept;
  const HandymanTask & task() const noexcept;

  void bootCompleted();
  void setEnvironment(const std::string & environment);
  bool acceptReady();
  bool acceptInstruction(const std::string & instruction, bool corrected = false);
  void parsingSucceeded();
  void moderatorSucceeded();
  void moderatorFailed();
  void missionCompleted();

private:
  TaskState state_{TaskState::kBooting};
  HandymanTask task_{};
};

}  // namespace handyman_rebuild_ros2
