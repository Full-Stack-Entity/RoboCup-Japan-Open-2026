#pragma once

/**
 * @file protocol_handler.hpp
 * @brief 通信协议层头文件
 *
 * 模块1：通信协议层
 * 负责与 Unity/Avatar 的所有消息收发，与任务逻辑无关。
 */

#include <rclcpp/rclcpp.hpp>
#include <handyman_msgs/msg/handyman_msg.hpp>

#include <memory>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <sstream>
#include <algorithm>
#include <functional>

namespace handyman_ros2 {

/**
 * @brief 通信协议处理器
 *
 * 职责：
 * - 订阅 /handyman/message/to_robot，解析消息并通过 Listener 回调通知上层
 * - 发布 /handyman/message/to_moderator，发送状态消息
 * - 消息去重、时序保证
 *
 * 使用方式：
 * @code
 * class MyOrchestrator : public ProtocolHandler::Listener {
 *   void onEnvironment(const std::string& env) override { ... }
 *   ...
 * };
 *
 * ProtocolHandler handler(node);
 * handler.setListener(&myOrchestrator);
 * handler.init();
 * @endcode
 */
class ProtocolHandler {
public:
    // ========================================================================
    // 常量
    // ========================================================================

    // 发送消息（Robot -> Avatar）
    static const std::string MSG_I_AM_READY;
    static const std::string MSG_ROOM_REACHED;
    static const std::string MSG_OBJECT_GRASPED;
    static const std::string MSG_TASK_FINISHED;
    static const std::string MSG_GIVE_UP;
    static const std::string MSG_DOES_NOT_EXIST;

    // 接收消息（Avatar -> Robot）
    static const std::string MSG_ARE_YOU_READY;
    static const std::string MSG_ENVIRONMENT;
    static const std::string MSG_INSTRUCTION;
    static const std::string MSG_CORRECTED;
    static const std::string MSG_TASK_SUCCEEDED;
    static const std::string MSG_TASK_FAILED;
    static const std::string MSG_MISSION_COMPLETE;

    // ========================================================================
    // Listener 接口
    // ========================================================================

    class Listener {
    public:
        virtual ~Listener() = default;

        virtual void onEnvironment(const std::string& unity_env) = 0;
        virtual void onAreYouReady() = 0;
        virtual void onInstruction(const std::string& detail) = 0;
        virtual void onCorrectedInstruction(const std::string& detail) = 0;
        virtual void onTaskSucceeded() = 0;
        virtual void onTaskFailed(const std::string& detail) = 0;
        virtual void onMissionComplete() = 0;
    };

    // ========================================================================
    // TaskContext：当前任务上下文
    // ========================================================================

    struct TaskContext {
        std::string instruction;
        std::string environment;       // Unity 环境名（如 "LayoutA"）
        std::string mapped_env;        // 映射后环境名（如 "Layout2019HM01"）
        std::vector<std::string> rooms;
        std::vector<std::string> objects;
        std::vector<std::string> destinations;

        void reset();
        bool isComplete() const;
        std::string toString() const;
    };

    // ========================================================================
    // InstructionParser：指令解析工具
    // ========================================================================

    class InstructionParser {
    public:
        static void tokenize(const std::string& str, char delim, std::vector<std::string>& out);
        static void parse(const std::string& instruction,
                         const std::vector<std::string>& rooms_keywords,
                         const std::vector<std::string>& objects_keywords,
                         const std::vector<std::string>& dests_keywords,
                         TaskContext& ctx);
        static std::string mapUnityEnvironmentName(const std::string& unity_env);
    };

    // ========================================================================
    // 构造与初始化
    // ========================================================================

    /**
     * @param node ROS2 节点指针
     * @param subscriber_topic 接收消息的 topic（默认 /handyman/message/to_robot）
     * @param publisher_topic 发送消息的 topic（默认 /handyman/message/to_moderator）
     */
    explicit ProtocolHandler(rclcpp::Node::SharedPtr node,
                             const std::string& subscriber_topic = "/handyman/message/to_robot",
                             const std::string& publisher_topic = "/handyman/message/to_moderator");

    void init();

    // ========================================================================
    // Listener 管理
    // ========================================================================

    void setListener(Listener* listener) { listener_ = listener; }

    // ========================================================================
    // 发送消息 API（TaskOrchestrator 调用）
    // ========================================================================

    void sendIAmReady();
    void sendRoomReached();
    void sendObjectGrasped();
    void sendTaskFinished();
    void sendGiveUp();
    void sendDoesNotExist(const std::string& object_name = "");

    // ========================================================================
    // 内部成员
    // ========================================================================

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<handyman_msgs::msg::HandymanMsg>::SharedPtr sub_msg_;
    rclcpp::Publisher<handyman_msgs::msg::HandymanMsg>::SharedPtr pub_msg_;

    const std::string subscriber_topic_;
    const std::string publisher_topic_;

    Listener* listener_ = nullptr;

    // 消息去重
    rclcpp::Time last_are_you_ready_time_;
    double message_dedupe_interval_;

    void messageCallback(const handyman_msgs::msg::HandymanMsg::ConstSharedPtr message);
    void sendMessage(const std::string& message, const std::string& detail = "");
};

}  // namespace handyman_ros2
