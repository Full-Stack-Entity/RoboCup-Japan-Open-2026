/**
 * @file protocol_handler.cpp
 * @brief 通信协议层：负责与 Unity/Avatar 的所有消息收发
 *
 * 功能范围：
 * - 订阅 /handyman/message/to_robot，解析消息并通知上层
 * - 发布 /handyman/message/to_moderator，发送状态消息
 * - 消息去重、时序保证
 *
 * 与其他模块的关系：
 * - 被 TaskOrchestrator 持有，TaskOrchestrator 实现 Listener 接口
 * - 不做任何任务逻辑判断，只负责"收到 X → 通知 Y"
 */

#include "handyman_ros2/protocol_handler.hpp"

namespace handyman_ros2 {

// ---------------------------------------------------------------------------
// ProtocolHandler implementation
// ---------------------------------------------------------------------------

ProtocolHandler::ProtocolHandler(
    rclcpp::Node::SharedPtr node,
    const std::string& subscriber_topic,
    const std::string& publisher_topic)
  : node_(node),
    subscriber_topic_(subscriber_topic),
    publisher_topic_(publisher_topic),
    last_are_you_ready_time_(node->get_clock()->now()),
    message_dedupe_interval_(0.1)  // 100ms 去重间隔
{
}

void ProtocolHandler::init()
{
    // 创建订阅：接收来自 Avatar/Moderator 的消息
    sub_msg_ = node_->create_subscription<handyman_msgs::msg::HandymanMsg>(
        subscriber_topic_, 100,
        std::bind(&ProtocolHandler::messageCallback, this, std::placeholders::_1));

    // 创建发布：发送消息给 Avatar/Moderator
    pub_msg_ = node_->create_publisher<handyman_msgs::msg::HandymanMsg>(publisher_topic_, 10);

    RCLCPP_INFO(node_->get_logger(), "ProtocolHandler initialized. Subscribing to '%s', publishing to '%s'",
                subscriber_topic_.c_str(), publisher_topic_.c_str());
}

void ProtocolHandler::sendMessage(const std::string& message, const std::string& detail)
{
    RCLCPP_INFO(node_->get_logger(), "ProtocolHandler -> Avatar: '%s' (detail: '%s')",
                message.c_str(), detail.c_str());

    handyman_msgs::msg::HandymanMsg msg;
    msg.message = message;
    msg.detail = detail;
    pub_msg_->publish(msg);
}

void ProtocolHandler::sendIAmReady()
{
    sendMessage(MSG_I_AM_READY, "");
}

void ProtocolHandler::sendRoomReached()
{
    sendMessage(MSG_ROOM_REACHED, "");
}

void ProtocolHandler::sendObjectGrasped()
{
    sendMessage(MSG_OBJECT_GRASPED, "");
}

void ProtocolHandler::sendTaskFinished()
{
    sendMessage(MSG_TASK_FINISHED, "");
}

void ProtocolHandler::sendGiveUp()
{
    sendMessage(MSG_GIVE_UP, "");
}

void ProtocolHandler::sendDoesNotExist(const std::string& object_name)
{
    sendMessage(MSG_DOES_NOT_EXIST, object_name);
}

void ProtocolHandler::messageCallback(const handyman_msgs::msg::HandymanMsg::ConstSharedPtr message)
{
    const std::string& msg_text = message->message;
    const std::string& detail   = message->detail;

    RCLCPP_INFO(node_->get_logger(), "ProtocolHandler <- Avatar: '%s', detail: '%s'",
                msg_text.c_str(), detail.c_str());

    // ---- 消息去重：防止重复的 Are_you_ready? 触发多次回调 ----
    if (msg_text == MSG_ARE_YOU_READY) {
        auto now = node_->now();
        double elapsed = (now - last_are_you_ready_time_).seconds();
        if (elapsed < message_dedupe_interval_) {
            RCLCPP_DEBUG(node_->get_logger(),
                        "Ignoring duplicate Are_you_ready? (elapsed=%.3f s < %.3f s)",
                        elapsed, message_dedupe_interval_);
            return;
        }
        last_are_you_ready_time_ = now;
    }

    // ---- Environment ----
    if (msg_text == MSG_ENVIRONMENT) {
        if (listener_) {
            listener_->onEnvironment(detail);
        }
        return;
    }

    // ---- Are_you_ready? ----
    if (msg_text == MSG_ARE_YOU_READY) {
        if (listener_) {
            listener_->onAreYouReady();
        }
        return;
    }

    // ---- Instruction ----
    if (msg_text == MSG_INSTRUCTION) {
        if (listener_) {
            listener_->onInstruction(detail);
        }
        return;
    }

    // ---- Corrected_instruction ----
    if (msg_text == MSG_CORRECTED) {
        if (listener_) {
            listener_->onCorrectedInstruction(detail);
        }
        return;
    }

    // ---- Task_succeeded ----
    if (msg_text == MSG_TASK_SUCCEEDED) {
        if (listener_) {
            listener_->onTaskSucceeded();
        }
        return;
    }

    // ---- Task_failed ----
    if (msg_text == MSG_TASK_FAILED) {
        if (listener_) {
            listener_->onTaskFailed(detail);
        }
        return;
    }

    // ---- Mission_complete ----
    if (msg_text == MSG_MISSION_COMPLETE) {
        if (listener_) {
            listener_->onMissionComplete();
        }
        return;
    }

    // ---- 未知消息 ----
    RCLCPP_WARN(node_->get_logger(), "ProtocolHandler: Unknown message type '%s', ignored", msg_text.c_str());
}

// ---------------------------------------------------------------------------
// ProtocolHandler::TaskContext
// ---------------------------------------------------------------------------

void ProtocolHandler::TaskContext::reset()
{
    instruction.clear();
    environment.clear();
    rooms.clear();
    objects.clear();
    destinations.clear();
}

bool ProtocolHandler::TaskContext::isComplete() const
{
    return !environment.empty() && !objects.empty() && !destinations.empty();
}

std::string ProtocolHandler::TaskContext::toString() const
{
    std::ostringstream oss;
    oss << "env=" << environment
        << ", rooms=" << rooms.size()
        << ", objects=" << objects.size()
        << ", dests=" << destinations.size();
    return oss.str();
}

// ---------------------------------------------------------------------------
// ProtocolHandler::InstructionParser
// ---------------------------------------------------------------------------

void ProtocolHandler::InstructionParser::tokenize(const std::string& str, char delim,
                                                   std::vector<std::string>& out)
{
    std::stringstream ss(str);
    std::string s;
    while (std::getline(ss, s, delim)) {
        if (!s.empty()) {
            out.push_back(s);
        }
    }
}

void ProtocolHandler::InstructionParser::parse(const std::string& instruction,
                                                const std::vector<std::string>& rooms_keywords,
                                                const std::vector<std::string>& objects_keywords,
                                                const std::vector<std::string>& dests_keywords,
                                                TaskContext& ctx)
{
    ctx.reset();
    ctx.instruction = instruction;

    std::vector<std::string> tokens;
    tokenize(instruction, ' ', tokens);

    for (size_t i = 0; i < tokens.size(); ++i) {
        const std::string& token = tokens[i];

        for (size_t j = 0; j < rooms_keywords.size(); ++j) {
            if (token.find(rooms_keywords[j]) != std::string::npos) {
                // 避免重复添加
                if (std::find(ctx.rooms.begin(), ctx.rooms.end(), rooms_keywords[j]) == ctx.rooms.end()) {
                    ctx.rooms.push_back(rooms_keywords[j]);
                }
            }
        }

        for (size_t j = 0; j < objects_keywords.size(); ++j) {
            if (token.find(objects_keywords[j]) != std::string::npos) {
                if (std::find(ctx.objects.begin(), ctx.objects.end(), objects_keywords[j]) == ctx.objects.end()) {
                    ctx.objects.push_back(objects_keywords[j]);
                }
            }
        }

        for (size_t j = 0; j < dests_keywords.size(); ++j) {
            if (token.find(dests_keywords[j]) != std::string::npos) {
                if (std::find(ctx.destinations.begin(), ctx.destinations.end(), dests_keywords[j]) == ctx.destinations.end()) {
                    ctx.destinations.push_back(dests_keywords[j]);
                }
            }
        }
    }
}

std::string ProtocolHandler::InstructionParser::mapUnityEnvironmentName(const std::string& unity_env)
{
    static const std::map<std::string, std::string> unity_to_internal = {
        {"LayoutA", "Layout2019HM01"},
        {"LayoutB", "Layout2019HM02"},
        {"LayoutC", "Layout2020HM01"},
        {"LayoutD", "Layout2021HM01"},
    };

    auto it = unity_to_internal.find(unity_env);
    if (it != unity_to_internal.end()) {
        return it->second;
    }
    return unity_env;  // 未知环境原样返回
}

// ---------------------------------------------------------------------------
// 常量定义
// ---------------------------------------------------------------------------

// 发送消息常量
const std::string ProtocolHandler::MSG_I_AM_READY     = "I_am_ready";
const std::string ProtocolHandler::MSG_ROOM_REACHED    = "Room_reached";
const std::string ProtocolHandler::MSG_OBJECT_GRASPED = "Object_grasped";
const std::string ProtocolHandler::MSG_TASK_FINISHED  = "Task_finished";
const std::string ProtocolHandler::MSG_GIVE_UP        = "Give_up";
const std::string ProtocolHandler::MSG_DOES_NOT_EXIST = "Does_not_exist";

// 接收消息常量
const std::string ProtocolHandler::MSG_ARE_YOU_READY    = "Are_you_ready?";
const std::string ProtocolHandler::MSG_ENVIRONMENT      = "Environment";
const std::string ProtocolHandler::MSG_INSTRUCTION       = "Instruction";
const std::string ProtocolHandler::MSG_CORRECTED        = "Corrected_instruction";
const std::string ProtocolHandler::MSG_TASK_SUCCEEDED   = "Task_succeeded";
const std::string ProtocolHandler::MSG_TASK_FAILED      = "Task_failed";
const std::string ProtocolHandler::MSG_MISSION_COMPLETE = "Mission_complete";

}  // namespace handyman_ros2
