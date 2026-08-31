#pragma once

#include <string_view>

namespace handyman_rebuild_ros2::protocol
{

inline constexpr std::string_view kToRobotTopic = "/handyman/message/to_robot";
inline constexpr std::string_view kToModeratorTopic = "/handyman/message/to_moderator";

inline constexpr std::string_view kEnvironment = "Environment";
inline constexpr std::string_view kAreYouReady = "Are_you_ready?";
inline constexpr std::string_view kInstruction = "Instruction";
inline constexpr std::string_view kCorrectedInstruction = "Corrected_instruction";
inline constexpr std::string_view kTaskSucceeded = "Task_succeeded";
inline constexpr std::string_view kTaskFailed = "Task_failed";
inline constexpr std::string_view kMissionComplete = "Mission_complete";

inline constexpr std::string_view kIAmReady = "I_am_ready";
inline constexpr std::string_view kRoomReached = "Room_reached";
inline constexpr std::string_view kDoesNotExist = "Does_not_exist";
inline constexpr std::string_view kObjectGrasped = "Object_grasped";
inline constexpr std::string_view kTaskFinished = "Task_finished";
inline constexpr std::string_view kGiveUp = "Give_up";

}  // namespace handyman_rebuild_ros2::protocol
