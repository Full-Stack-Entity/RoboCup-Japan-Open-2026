#include <gtest/gtest.h>

#include "handyman_rebuild_ros2/task_state_machine.hpp"

using handyman_rebuild_ros2::TaskState;
using handyman_rebuild_ros2::TaskStateMachine;

TEST(TaskStateMachineTest, RequiresEnvironmentBeforeReady)
{
  TaskStateMachine machine;
  machine.bootCompleted();
  EXPECT_FALSE(machine.acceptReady());
  EXPECT_EQ(machine.state(), TaskState::kWaitingReady);

  machine.setEnvironment("LayoutA");
  EXPECT_TRUE(machine.acceptReady());
  EXPECT_EQ(machine.state(), TaskState::kWaitingInstruction);
}

TEST(TaskStateMachineTest, AcceptsNonEmptyInstructionOnlyInExpectedState)
{
  TaskStateMachine machine;
  machine.bootCompleted();
  machine.setEnvironment("LayoutA");
  ASSERT_TRUE(machine.acceptReady());

  EXPECT_FALSE(machine.acceptInstruction(""));
  EXPECT_TRUE(machine.acceptInstruction("Go to the kitchen and grasp the apple."));
  EXPECT_EQ(machine.state(), TaskState::kParsingInstruction);
  EXPECT_EQ(machine.task().raw_instruction, "Go to the kitchen and grasp the apple.");
}

TEST(TaskStateMachineTest, FailureResetsTaskSafely)
{
  TaskStateMachine machine;
  machine.bootCompleted();
  machine.setEnvironment("LayoutB");
  ASSERT_TRUE(machine.acceptReady());
  ASSERT_TRUE(machine.acceptInstruction("Bring the cup here."));

  machine.moderatorFailed();
  EXPECT_EQ(machine.state(), TaskState::kWaitingReady);
  EXPECT_TRUE(machine.task().raw_instruction.empty());
  EXPECT_TRUE(machine.task().environment.empty());
}

TEST(TaskStateMachineTest, MissionCompleteIsTerminal)
{
  TaskStateMachine machine;
  machine.bootCompleted();
  machine.missionCompleted();
  EXPECT_EQ(machine.state(), TaskState::kFinished);
}
