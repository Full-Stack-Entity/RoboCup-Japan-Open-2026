#include "handyman_rebuild_ros2/task_state_machine.hpp"

namespace handyman_rebuild_ros2
{

TaskState TaskStateMachine::state() const noexcept { return state_; }
const HandymanTask & TaskStateMachine::task() const noexcept { return task_; }

void TaskStateMachine::bootCompleted()
{
  if (state_ == TaskState::kBooting) {
    state_ = TaskState::kWaitingReady;
  }
}

void TaskStateMachine::setEnvironment(const std::string & environment)
{
  task_.environment = environment;
}

bool TaskStateMachine::acceptReady()
{
  if (state_ != TaskState::kWaitingReady || task_.environment.empty()) {
    return false;
  }
  state_ = TaskState::kWaitingInstruction;
  return true;
}

bool TaskStateMachine::acceptInstruction(const std::string & instruction, bool corrected)
{
  const bool initial = state_ == TaskState::kWaitingInstruction;
  const bool correction = corrected &&
    (state_ == TaskState::kSearchingObject || state_ == TaskState::kRecovering);
  if (instruction.empty() || (!initial && !correction)) {
    return false;
  }
  task_.raw_instruction = instruction;
  task_.pickup_room.clear();
  task_.target_object.clear();
  task_.destination_room.clear();
  task_.destination.clear();
  state_ = TaskState::kParsingInstruction;
  return true;
}

void TaskStateMachine::parsingSucceeded()
{
  if (state_ == TaskState::kParsingInstruction) {
    state_ = TaskState::kNavigatingToRoom;
  }
}

void TaskStateMachine::moderatorSucceeded()
{
  if (state_ == TaskState::kWaitingModeratorResult) {
    state_ = TaskState::kWaitingReady;
    task_ = HandymanTask{};
  }
}

void TaskStateMachine::moderatorFailed()
{
  state_ = TaskState::kWaitingReady;
  task_ = HandymanTask{};
}

void TaskStateMachine::missionCompleted()
{
  state_ = TaskState::kFinished;
}

}  // namespace handyman_rebuild_ros2
